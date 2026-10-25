#include "Core/FFNYLab.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <unordered_map>
#include <sstream>
#include <fstream>

#include "Core/Core.h"
#include "Core/HW/Memmap.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/PowerPC/JitInterface.h"
#include "Core/State.h"
#include "Core/HLE/HLE.h"
#include "Core/PowerPC/Gekko.h"
#include "Core/System.h"
#include "Core/HW/ProcessorInterface.h"
#include "Core/HW/VideoInterface.h"
#include "VideoCommon/CommandProcessor.h"
#include "VideoCommon/FrameDumper.h"

namespace FFNYLab
{
static int s_active = -1;
static std::string s_dir;
static FILE* s_cmd = nullptr;
static long s_run_left = 0;
static unsigned long long s_fields = 0;
static bool s_pad_on[4];
static GCPadStatus s_pad[4];
static int s_mode = 0;
static std::string s_pending_state;          // 0: count VI fields, 1: count main-loop passes
static unsigned long long s_loops = 0;
struct Snap
{
  std::vector<u8> ram;
  std::vector<u8> vmem;
  u32 gpr[32];
  PowerPC::PairedSingle ps[32];
  PowerPC::ConditionRegister cr;
  u32 fpscr;
  u8 ca, so;
  u32 lr, ctr, gqr[8];
};
static Snap s_snap[8];

// ---- compact rollback snapshot (prototype of the real per-frame save) ----
struct CSnap
{
  std::vector<std::pair<u32, u32>> ranges;  // physical offsets [lo, hi)
  std::vector<u8> data;
  u32 gpr[32];
  PowerPC::PairedSingle ps[32];
  PowerPC::ConditionRegister cr;
  u32 fpscr;
  u8 ca, so;
  u32 lr, ctr, gqr[8];
};
static CSnap s_csnap[8];
static std::string s_cstatic_path;
static std::vector<std::pair<u32, u32>> s_cstatic;
static bool s_ctag = false;   // true: system blocks are recognised by the DOL tag (rbtag: header flags bit 0x0080), not by rbprobe names
static void CBuildRanges(const u8* ram, u32 size, std::vector<std::pair<u32, u32>>& out, u32* nblocks)
{
  auto rd = [&](u32 a) { a &= 0x01FFFFFF; return (u32)((ram[a] << 24) | (ram[a + 1] << 16) | (ram[a + 2] << 8) | ram[a + 3]); };
  std::vector<std::pair<u32, u32>> inc(s_cstatic), ex;
  // lab stand-in for the DOL-side block tag: names from the rbprobe allocation table
  std::unordered_map<u32, u32> names;
  if (rd(0x80480000) == 0x48454150)
    for (u32 i = 0; i < rd(0x8048000C) && i < 6000; i++)
    { const u32 e = 0x80480010 + 16 * i; if (rd(e)) names[rd(e)] = rd(e + 8); }
  static const char* black[] = {"AUDIO", "Audio", "audio", "GENRSTRM", "SPCHSTRM", "SPCHBNK", "ABKRES", "FIFO", "FrameBuffer",
                                "GCBuffer", "ThreadStack", "File Sys", "bigfile", "Fixed MPF", "ASYNCFILE", "_AsyncLoaderCallback",
                                "Stream", "InputBuffer", "MCThread", "MyThread", "SndHeap"};
  const u32 H0 = 0x8060C050, base = rd(H0 + 8), end = rd(H0 + 0xC);
  inc.push_back({base & 0x01FFFFFF, (base + 0x90) & 0x01FFFFFF});   // heap 0 struct + sentinel area up to the first block
  u32 a = base + 0x90, n = 0;
  while (a >= base && a < end && n < 20000)
  {
    const u32 tag = rd(a) >> 16, sz = rd(a + 4), nx = rd(a + 8);
    inc.push_back({a & 0x01FFFFFF, (a + 16) & 0x01FFFFFF});            // every header (layout)
    if (tag == 0x424D && sz)
    {
      bool keep = true;
      if (s_ctag && (rd(a) & 0x80)) keep = false;
      auto it = s_ctag ? names.end() : names.find(a + 16);
      if (it != names.end() && (it->second & 0xFE000000) == 0x80000000)
      {
        std::string nm;
        for (u32 k = 0; k < 64; k++) { char ch = (char)ram[(it->second + k) & 0x01FFFFFF]; if (!ch) break; nm += ch; }
        for (auto b : black) if (nm.find(b) != std::string::npos) keep = false;
      }
      const u32 mg = rd(a + 16);
      if (mg == 0x4D4F4952 || mg == 0x41424B43 || mg == 0x454D4954) keep = false;   // MOIR / ABKC / EMIT: sound-engine data
      if (keep) { inc.push_back({(a + 16) & 0x01FFFFFF, (a + 16 + sz) & 0x01FFFFFF}); n++; }
    }
    if (nx <= a) break;
    a = nx;
  }
  for (u32 k = 0; k < 64; k++)                                          // allocator mutexes
  { const u32 h = rd(0x8040BD70 + 4 * k); if ((h & 0xFE000000) == 0x80000000) ex.push_back({(h + 0x3C) & 0x01FFFFFF, (h + 0x54) & 0x01FFFFFF}); }
  const u32 pool = rd(0x802E9B00);
  if (pool) ex.push_back({(pool + 0x2A4) & 0x01FFFFFF, (pool + 0x2BC) & 0x01FFFFFF});
  const u32 mainth = 0x80432D10;
  for (u32 t = rd(0x800000DC), k = 0; t && k < 64; t = rd(t + 0x2FC), k++)   // OS threads: structs and (non-main) stacks
  {
    ex.push_back({t & 0x01FFFFFF, (t + 0x318) & 0x01FFFFFF});
    if (t != mainth) ex.push_back({rd(t + 0x308) & 0x01FFFFFF, rd(t + 0x304) & 0x01FFFFFF});
  }
  auto norm = [](std::vector<std::pair<u32, u32>>& v) {
    std::sort(v.begin(), v.end());
    std::vector<std::pair<u32, u32>> r;
    for (auto p : v) { if (p.first >= p.second) continue; if (!r.empty() && p.first <= r.back().second) r.back().second = std::max(r.back().second, p.second); else r.push_back(p); }
    v.swap(r);
  };
  norm(inc); norm(ex);
  out.clear();
  size_t j = 0;
  for (auto [lo, hi] : inc)
  {
    if (hi > size) hi = size;
    while (j < ex.size() && ex[j].second <= lo) j++;
    u32 cur = lo;
    for (size_t k = j; k < ex.size() && ex[k].first < hi; k++)
    {
      if (ex[k].first > cur) out.push_back({cur, ex[k].first});
      cur = std::max(cur, ex[k].second);
    }
    if (cur < hi) out.push_back({cur, hi});
  }
  if (nblocks) *nblocks = n;
}
static Common::UniqueBuffer<u8> s_mstate[8];
static std::size_t s_msize[8];
static std::vector<std::vector<std::string>> s_script;   // per loop pass: pad lines
static unsigned long long s_script_start = 0;
static void Commands(Core::System& system);
static long s_track_left = 0;
bool g_store_trace = false;
static bool s_trace_logic_only = true, s_in_logic = false;
struct WInfo { u32 pc[3]; u32 thread; u32 count; u32 mainw; u32 otherw; u32 threadw; u32 tpc; u32 lr[3]; u32 mainout; };
static u32 s_main_thread = 0x80432D10;
static std::unordered_map<u32, WInfo> s_writes;
static int s_inmode = 0;        // 1 record pad snapshot per update counter, 2 replay
static std::map<u32, std::vector<u8>> s_inputs;
static unsigned s_inplayed = 0, s_inrecorded = 0;
static std::vector<u8> s_tbuf, s_logicw, s_otherw;
static bool s_tbuf_valid = false;
static std::string s_rec_tag;   // per-update recording of chosen ranges: file dir/rec/<tag>_<counter>.bin
static std::vector<std::pair<u32, u32>> s_rec_ranges;
static void TrackMark(Core::System& system, std::vector<u8>& into)
{
  auto& memory = system.GetMemory();
  const u8* ram = memory.GetRAM();
  const u32 size = memory.GetRamSizeReal();
  if (s_logicw.size() != size / 4) { s_logicw.assign(size / 4, 0); s_otherw.assign(size / 4, 0); }
  if (s_tbuf_valid)
  {
    const u32* a = reinterpret_cast<const u32*>(ram);
    const u32* b = reinterpret_cast<const u32*>(s_tbuf.data());
    for (u32 i = 0; i < size / 4; i++)
      if (a[i] != b[i] && into[i] < 255) into[i]++;
  }
  s_tbuf.assign(ram, ram + size);
  s_tbuf_valid = true;
}

bool Active()
{
  if (s_active < 0)
  {
    const char* d = getenv("FFNY_LAB");
    s_active = d && *d;
    if (s_active)
      s_dir = d;
  }
  return s_active == 1;
}

bool GetPad(int port, GCPadStatus* out)
{
  if (!Active() || port < 0 || port > 3 || !s_pad_on[port])
    return false;
  *out = s_pad[port];
  return true;
}

static void Reply(const std::string& s)
{
  FILE* f = fopen((s_dir + "/out").c_str(), "a");
  if (f)
  {
    fprintf(f, "%s\n", s.c_str());
    fclose(f);
  }
}

void OnNewField(Core::System& system)
{
  if (!Active())
    return;
  s_fields++;
  if (s_mode == 1)
  {
    static unsigned long long last_loops = 0, idle = 0;
    if (s_loops != last_loops) { last_loops = s_loops; idle = 0; }
    else if (++idle == 300)
    {
      Reply("! no main-loop pass for 300 fields, back to field mode");
      idle = 0;
      s_mode = 0;
      s_run_left = 0;
      Commands(system);
    }
    return;
  }
  if (s_run_left > 0 && --s_run_left > 0)
    return;
  if (!s_pending_state.empty())
  {
    std::string c = s_pending_state.substr(0, 4), path = s_pending_state.substr(5);
    s_pending_state.clear();
    if (c == "load") State::LoadAs(system, path); else State::SaveAs(system, path);
    Reply("ok " + c + " " + path);
  }
  Commands(system);
}

static void ApplyScript()
{
  if (s_script.empty() || s_loops < s_script_start)
    return;
  const auto i = s_loops - s_script_start;
  if (i >= s_script.size())
    return;
  for (auto& l : s_script[i])
  {
    std::istringstream in(l);
    int p;
    unsigned b, sx, sy;
    in >> p >> std::hex >> b >> std::dec >> sx >> sy;
    GCPadStatus st;
    st.button = b; st.stickX = sx; st.stickY = sy;
    st.analogA = (b & 0x100) ? 0xFF : 0;
    st.analogB = (b & 0x200) ? 0xFF : 0;
    s_pad[p & 3] = st;
    s_pad_on[p & 3] = true;
  }
}

void StoreTrace(Core::System& system, u32 addr, u32 size)
{
  if (s_trace_logic_only && !s_in_logic)
    return;
  const bool isvm = (addr & 0xFE000000) == 0x7E000000;
  if ((addr & 0xFE000000) != 0x80000000 && (addr & 0xFE000000) != 0xC0000000 && !isvm)
    return;
  const u8* ram = system.GetMemory().GetRAM();
  const u32 pc = system.GetPPCState().pc;
  const u32 th = (ram[0xE4] << 24) | (ram[0xE5] << 16) | (ram[0xE6] << 8) | ram[0xE7];
  for (u32 a = addr & ~3u; a < addr + size; a += 4)
  {
    WInfo& w = s_writes[isvm ? (a | 0x7E000000) : ((a & 0x01FFFFFF) | 0x80000000)];
    w.count++;
    w.thread = th;
    if (th == s_main_thread && s_in_logic) w.mainw++; else w.otherw++;
    if (th == s_main_thread && !s_in_logic) w.mainout++;
    if (th != s_main_thread) { w.threadw++; w.tpc = pc; }
    for (int i = 0; i < 3; i++)
    {
      if (w.pc[i] == pc) break;
      if (w.pc[i] == 0) { w.pc[i] = pc; w.lr[i] = system.GetPPCState().spr[SPR_LR]; break; }
    }
  }
}

void OnPreUpdate(const Core::CPUThreadGuard& guard)
{
  s_in_logic = true;
  if (s_inmode)
  {
    auto& system = guard.GetSystem();
    auto& memory = system.GetMemory();
    u8* ram = memory.GetRAM();
    const u32 tick = system.GetPPCState().gpr[28] & 0x01FFFFFF;
    const u32 ctr = (ram[0x440C24] << 24) | (ram[0x440C25] << 16) | (ram[0x440C26] << 8) | ram[0x440C27];
    u8* pads = ram + tick + 0x17CC;
    if (s_inmode == 1)
    {
      s_inputs[ctr].assign(pads, pads + 0x260);
      s_inrecorded++;
    }
    else
    {
      auto it = s_inputs.find(ctr);
      if (it != s_inputs.end())
      {
        memcpy(pads, it->second.data(), 0x260);
        s_inplayed++;
      }
    }
  }
  if (s_track_left > 0)
    TrackMark(guard.GetSystem(), s_otherw);
}

void OnPostUpdate(const Core::CPUThreadGuard& guard)
{
  s_in_logic = false;
  if (!s_rec_tag.empty())
  {
    const u8* ram = guard.GetSystem().GetMemory().GetRAM();
    const u32 ctr = (ram[0x440C24] << 24) | (ram[0x440C25] << 16) | (ram[0x440C26] << 8) | ram[0x440C27];
    FILE* f = fopen((s_dir + "/rec/" + s_rec_tag + "_" + std::to_string(ctr) + ".bin").c_str(), "wb");
    if (f) { for (auto& [lo, hi] : s_rec_ranges) fwrite(ram + (lo & 0x01FFFFFF), 1, hi - lo, f); fclose(f); }
  }
  if (s_track_left > 0)
  {
    TrackMark(guard.GetSystem(), s_logicw);
    if (--s_track_left == 0)
    {
      s_tbuf_valid = false;
      Reply("track done");
    }
  }
}

void OnTrace(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  auto str = [&](u32 a) -> std::string {
    if ((a & 0xFE000000) != 0x80000000) return "";
    std::string r;
    for (u32 i = 0; i < 120; i++) { char ch = ram[(a + i) & 0x01FFFFFF]; if (!ch) break; r += ch; }
    return r;
  };
  char buf[512];
  const u32 th = (ram[0xE4] << 24) | (ram[0xE5] << 16) | (ram[0xE6] << 8) | ram[0xE7];
  snprintf(buf, sizeof buf, "TRACE pc %08x th %08x L%d lr %08x r3 %08x r4 %08x r5 %08x r6 %08x | %s | %s | %s", ppc.pc, th, (int)s_in_logic, ppc.spr[SPR_LR], ppc.gpr[3], ppc.gpr[4], ppc.gpr[5], ppc.gpr[6],
           str(ppc.gpr[3]).c_str(), str(ppc.gpr[4]).c_str(), str(ppc.gpr[5]).c_str());
  std::string bt;
  u32 sp = ppc.gpr[1];
  for (int k = 0; k < 6 && (sp & 0xFE000000) == 0x80000000; k++)
  {
    const u32 o = sp & 0x01FFFFFF;
    const u32 up = (ram[o] << 24) | (ram[o + 1] << 16) | (ram[o + 2] << 8) | ram[o + 3];
    if ((up & 0xFE000000) != 0x80000000) break;
    const u32 uo = up & 0x01FFFFFF;
    const u32 ra = (ram[uo + 4] << 24) | (ram[uo + 5] << 16) | (ram[uo + 6] << 8) | ram[uo + 7];
    char t[16]; snprintf(t, sizeof t, " %08x", ra); bt += t;
    sp = up;
  }
  fprintf(stderr, "%s BT%s\n", buf, bt.c_str());
}

void OnLoopTop(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  s_loops++;
  if (s_loops < 3)
    fprintf(stderr, "FFNYLab: loop hook hit pc=%08x\n", system.GetPPCState().pc);
  ApplyScript();
  if (s_mode != 1)
    return;
  if (s_run_left > 0 && --s_run_left > 0)
    return;
  Commands(system);
}

static void Commands(Core::System& system)
{
  if (!s_cmd)
    s_cmd = fopen((s_dir + "/cmd").c_str(), "r");  // FIFO: blocks until a writer opens it
  auto& memory = system.GetMemory();
  char line[4096];
  while (true)
  {
    if (!fgets(line, sizeof line, s_cmd))
    {
      // writer closed: reopen and wait for the next one
      fclose(s_cmd);
      s_cmd = fopen((s_dir + "/cmd").c_str(), "r");
      continue;
    }
    std::istringstream in(line);
    std::string c;
    in >> c;
    if (c.empty())
      continue;
    if (c == "run")
    {
      long n = 1;
      in >> n;
      s_run_left = n;
      Reply("ok run " + std::to_string(n) + " at " + std::to_string(s_fields));
      return;
    }
    else if (c == "pad")
    {
      int p;
      unsigned b, sx, sy, cx, cy, l, r;
      in >> p >> std::hex >> b >> std::dec >> sx >> sy >> cx >> cy >> l >> r;
      GCPadStatus st;
      st.button = b;
      st.stickX = sx; st.stickY = sy; st.substickX = cx; st.substickY = cy;
      st.triggerLeft = l; st.triggerRight = r;
      st.analogA = (b & 0x100) ? 0xFF : 0;
      st.analogB = (b & 0x200) ? 0xFF : 0;
      s_pad[p & 3] = st;
      s_pad_on[p & 3] = true;
      Reply("ok pad");
    }
    else if (c == "padoff")
    {
      int p;
      in >> p;
      s_pad_on[p & 3] = false;
      Reply("ok padoff");
    }
    else if (c == "shot")
    {
      std::string path;
      in >> path;
      if (g_frame_dumper)
        g_frame_dumper->SaveScreenshot(path);
      Reply("ok shot " + path);
    }
    else if (c == "dump")
    {
      std::string path;
      in >> path;
      FILE* f = fopen(path.c_str(), "wb");
      if (f)
      {
        fwrite(memory.GetRAM(), 1, memory.GetRamSizeReal(), f);
        fclose(f);
      }
      Reply("ok dump " + path);
    }

    else if (c == "load" || c == "save")
    {
      std::string path;
      in >> path;
      if (s_mode == 1)
      {
        // inside a JIT block (loop hook): defer to the next VI field
        s_pending_state = c + " " + path;
        s_mode = 0;
        s_run_left = 1;
        return;
      }
      if (c == "load") State::LoadAs(system, path); else State::SaveAs(system, path);
      Reply("ok " + c + " " + path);
    }
    else if (c == "peek")
    {
      unsigned a, n;
      in >> std::hex >> a >> n;
      std::string s = "peek ";
      char buf[4];
      for (unsigned i = 0; i < n; i++)
      {
        snprintf(buf, sizeof buf, "%02x", memory.GetRAM()[(a + i) & 0x01FFFFFF]);
        s += buf;
      }
      Reply(s);
    }
    else if (c == "poke")
    {
      unsigned a;
      std::string hex;
      in >> std::hex >> a >> hex;
      for (size_t i = 0; i + 1 < hex.size(); i += 2)
        memory.GetRAM()[(a + i / 2) & 0x01FFFFFF] = (u8)strtoul(hex.substr(i, 2).c_str(), nullptr, 16);
      system.GetJitInterface().InvalidateICache(0x80000000 | a, (u32)hex.size() / 2 + 32, true);
      Reply("ok poke");
    }
    else if (c == "loops")
    {
      long n = 1;
      in >> n;
      s_run_left = n;
      s_mode = 1;
      Reply("ok loops " + std::to_string(n) + " at " + std::to_string(s_loops));
      return;
    }
    else if (c == "hookloop")
    {
      unsigned a = 0x80004CCC;
      in >> std::hex >> a;
      HLE::Patch(system, a, "FFNYLabLoop");
      system.GetJitInterface().ClearSafe();
      auto r = HLE::TryReplaceFunction(system.GetPPCSymbolDB(), a, PowerPC::CoreMode::JIT);
      Reply("ok hookloop " + std::to_string(HLE::GetHookByAddress(a)) + " try " + std::to_string(r.hook_index) + " core " + std::to_string((int)system.GetPowerPC().GetMode()));
    }
    else if (c == "snap" || c == "restore" || c == "restorer")
    {
      int k = 0;
      in >> k;
      Snap& sn = s_snap[k & 7];
      auto& ppc = system.GetPPCState();
      u8* ram = memory.GetRAM();
      const u32 size = memory.GetRamSizeReal();
      if (c == "snap")
      {
        sn.ram.assign(ram, ram + size);
        if (memory.GetFakeVMEM())
          sn.vmem.assign(memory.GetFakeVMEM(), memory.GetFakeVMEM() + memory.GetFakeVMemSize());
        memcpy(sn.gpr, ppc.gpr, sizeof sn.gpr);
        memcpy(sn.ps, ppc.ps, sizeof sn.ps);
        sn.cr = ppc.cr; sn.fpscr = ppc.fpscr.Hex; sn.ca = ppc.xer_ca; sn.so = ppc.xer_so_ov;
        sn.lr = ppc.spr[SPR_LR]; sn.ctr = ppc.spr[SPR_CTR];
        for (int i = 0; i < 8; i++) sn.gqr[i] = ppc.spr[SPR_GQR0 + i];
        Reply("ok snap");
      }
      else if (c == "restore")
      {
        memcpy(ram, sn.ram.data(), size);
        if (memory.GetFakeVMEM() && !sn.vmem.empty())
          memcpy(memory.GetFakeVMEM(), sn.vmem.data(), sn.vmem.size());
        memcpy(ppc.gpr, sn.gpr, sizeof sn.gpr);
        memcpy(ppc.ps, sn.ps, sizeof sn.ps);
        ppc.cr = sn.cr; ppc.fpscr.Hex = sn.fpscr; ppc.xer_ca = sn.ca; ppc.xer_so_ov = sn.so;
        ppc.spr[SPR_LR] = sn.lr; ppc.spr[SPR_CTR] = sn.ctr;
        for (int i = 0; i < 8; i++) ppc.spr[SPR_GQR0 + i] = sn.gqr[i];
        Reply("ok restore");
      }
      else
      {
        // restorer K file [sndguard]: restore only the address ranges listed in file (lo hi per line, hex), plus registers.
        // sndguard: words whose snapshot or current value points into a live sound-library object ("Audio Subsystem"
        // blocks in the rbprobe heap table, current and snapshot) keep their current value (sound handles).
        std::string path, opt, word;
        in >> path;
        std::string heapexcl;
        while (in >> word)
        {
          if (word == "sndguard") opt = word;
          else if (word.rfind("heapdyn=", 0) == 0) heapexcl = word.substr(8);
        }
        std::vector<std::pair<u32, u32>> snd;
        if (opt == "sndguard")
        {
          for (const u8* m : {(const u8*)ram, (const u8*)sn.ram.data()})
          {
            auto rd = [&](u32 a) { a &= 0x01FFFFFF; return (u32)((m[a] << 24) | (m[a + 1] << 16) | (m[a + 2] << 8) | m[a + 3]); };
            if (rd(0x80480000) != 0x48454150) continue;
            const u32 cap = rd(0x8048000C);
            for (u32 i = 0; i < cap && i < 6000; i++)
            {
              const u32 e = 0x80480010 + 16 * i;
              if (rd(e) && rd(e + 8) == 0x80298F10) snd.push_back({rd(e), rd(e) + rd(e + 4)});
            }
          }
          std::sort(snd.begin(), snd.end());
        }
        auto insnd = [&](u32 v) {
          auto it = std::upper_bound(snd.begin(), snd.end(), std::make_pair(v, 0xFFFFFFFFu));
          if (it == snd.begin()) return false;
          --it;
          return it->first <= v && v < it->second;
        };
        u32 guarded = 0;
        std::ifstream rf(path);
        unsigned lo, hi, total = 0;
        u8* vm = memory.GetFakeVMEM();
        const u32 vsize = memory.GetFakeVMemSize();
        while (rf >> std::hex >> lo >> hi)
        {
          const bool isvm = lo < 0x80000000u;
          lo &= 0x01FFFFFF; hi = ((hi - 1) & 0x01FFFFFF) + 1;
          if (isvm)
          {
            if (!vm || sn.vmem.empty()) continue;
            if (hi > vsize) hi = vsize;
            if (lo < hi) { memcpy(vm + lo, sn.vmem.data() + lo, hi - lo); total += hi - lo; }
            continue;
          }
          if (hi > size) hi = size;
          if (!heapexcl.empty())
          {
            if (lo >= 0x004A2500 && lo < 0x017FF480) continue;
            if (hi > 0x004A2500 && lo < 0x004A2500) hi = 0x004A2500;
          }
          if (lo < hi && snd.empty()) { memcpy(ram + lo, sn.ram.data() + lo, hi - lo); total += hi - lo; }
          else if (lo < hi)
          {
            for (u32 a = lo; a < hi; a += 4)
            {
              const u8* o = sn.ram.data() + a; u8* c = ram + a;
              const u32 ov = (o[0] << 24) | (o[1] << 16) | (o[2] << 8) | o[3];
              const u32 cv = (c[0] << 24) | (c[1] << 16) | (c[2] << 8) | c[3];
              if (ov != cv && (insnd(ov) || insnd(cv))) { guarded++; continue; }
              memcpy(c, o, 4);
            }
            total += hi - lo;
          }
        }
        u32 hblocks = 0, hbytes = 0;
        if (!heapexcl.empty())
        {
          // heap arena, decided now: blocks present with the same pointer / size / name in the snapshot and the
          // current rbprobe table, names not blacklisted, plus the small-object pool; minus listed excluded words
          std::vector<std::pair<u32, u32>> ex;
          { std::ifstream xf(heapexcl); unsigned a, b; while (xf >> std::hex >> a >> b) ex.push_back({a, b}); }
          std::sort(ex.begin(), ex.end());
          auto rdm = [](const u8* m, u32 a) { a &= 0x01FFFFFF; return (u32)((m[a] << 24) | (m[a + 1] << 16) | (m[a + 2] << 8) | m[a + 3]); };
          std::map<u32, std::pair<u32, u32>> tb;   // ptr -> (size, name) in snapshot
          const u8* sm = sn.ram.data();
          if (rdm(sm, 0x80480000) == 0x48454150)
            for (u32 i = 0; i < rdm(sm, 0x8048000C) && i < 6000; i++)
            { const u32 e = 0x80480010 + 16 * i; if (rdm(sm, e)) tb[rdm(sm, e)] = {rdm(sm, e + 4), rdm(sm, e + 8)}; }
          static const char* black[] = {"AUDIO", "Audio", "audio", "GENRSTRM", "SPCHSTRM", "SPCHBNK", "ABKRES", "FIFO", "FrameBuffer",
                                        "GCBuffer", "ThreadStack", "File Sys", "bigfile", "Fixed MPF", "ASYNCFILE", "_AsyncLoaderCallback",
                                        "Stream", "InputBuffer", "MCThread", "MyThread"};
          std::vector<std::pair<u32, u32>> rest;
          if (rdm(ram, 0x80480000) == 0x48454150)
            for (u32 i = 0; i < rdm(ram, 0x8048000C) && i < 6000; i++)
            {
              const u32 e = 0x80480010 + 16 * i, p = rdm(ram, e), sz = rdm(ram, e + 4), nm = rdm(ram, e + 8);
              if (!p || p < 0x804A2500) continue;
              auto it = tb.find(p);
              if (it == tb.end() || it->second.first != sz || it->second.second != nm) continue;
              std::string name;
              if ((nm & 0xFE000000) == 0x80000000) for (u32 k = 0; k < 64; k++) { char ch = ram[(nm + k) & 0x01FFFFFF]; if (!ch) break; name += ch; }
              bool bl = false;
              for (auto b : black) if (name.find(b) != std::string::npos) bl = true;
              if (!bl) rest.push_back({p, p + sz});
            }
          const u32 pool = rdm(ram, 0x802E9B00);
          if (pool) rest.push_back({pool, rdm(ram, pool + 0x14C) + rdm(ram, pool + 0x150)});
          for (auto [lo2, hi2] : rest)
          {
            hblocks++;
            for (u32 a = lo2 & ~3u; a < hi2; a += 4)
            {
              auto xi = std::upper_bound(ex.begin(), ex.end(), std::make_pair(a, 0xFFFFFFFFu));
              if (xi != ex.begin() && std::prev(xi)->first <= a && a < std::prev(xi)->second) continue;
              const u32 o = a & 0x01FFFFFF;
              if (!snd.empty())
              {
                const u32 ov = rdm(sm, a), cv = rdm(ram, a);
                if (ov != cv && (insnd(ov) || insnd(cv))) { guarded++; continue; }
              }
              memcpy(ram + o, sm + o, 4); hbytes += 4;
            }
          }
        }
        memcpy(ppc.gpr, sn.gpr, sizeof sn.gpr);
        memcpy(ppc.ps, sn.ps, sizeof sn.ps);
        ppc.cr = sn.cr; ppc.fpscr.Hex = sn.fpscr; ppc.xer_ca = sn.ca; ppc.xer_so_ov = sn.so;
        ppc.spr[SPR_LR] = sn.lr; ppc.spr[SPR_CTR] = sn.ctr;
        for (int i = 0; i < 8; i++) ppc.spr[SPR_GQR0 + i] = sn.gqr[i];
        Reply("ok restorer " + std::to_string(total) + " heapblocks " + std::to_string(hblocks) + " heapbytes " + std::to_string(hbytes) + " sndblocks " + std::to_string(snd.size()) + " guarded " + std::to_string(guarded));
      }
    }
    else if (c == "dumpv")
    {
      std::string path;
      in >> path;
      FILE* f = fopen(path.c_str(), "wb");
      if (f && memory.GetFakeVMEM())
        fwrite(memory.GetFakeVMEM(), 1, memory.GetFakeVMemSize(), f);
      if (f) fclose(f);
      Reply("ok dumpv " + std::to_string(memory.GetFakeVMEM() ? memory.GetFakeVMemSize() : 0));
    }
    else if (c == "dumpr")
    {
      std::string path;
      unsigned lo, hi;
      in >> path >> std::hex >> lo >> hi;
      FILE* f = fopen(path.c_str(), "wb");
      if (f)
      {
        fwrite(memory.GetRAM() + (lo & 0x01FFFFFF), 1, (hi - lo), f);
        fclose(f);
      }
      Reply("ok dumpr");
    }
    else if (c == "script")
    {
      // script FILE: line i = pads for loop pass (now + 1 + i): "p btn sx sy; p btn sx sy"
      std::string path;
      in >> path;
      std::ifstream sf(path);
      std::string ln;
      s_script.clear();
      while (std::getline(sf, ln))
      {
        std::vector<std::string> v;
        std::stringstream ss(ln);
        std::string part;
        while (std::getline(ss, part, ';'))
          if (part.find_first_not_of(" \t") != std::string::npos)
            v.push_back(part);
        s_script.push_back(v);
      }
      s_script_start = s_loops + 1;
      Reply("ok script " + std::to_string(s_script.size()));
    }
    else if (c == "rec")
    {
      // rec TAG lo hi [lo hi ...] | rec off
      std::string tag; in >> tag;
      if (tag == "off") s_rec_tag.clear();
      else
      {
        s_rec_tag = tag; s_rec_ranges.clear();
        unsigned lo, hi;
        while (in >> std::hex >> lo >> hi) s_rec_ranges.push_back({lo, hi});
        ::system(("mkdir -p \"" + s_dir + "/rec\"").c_str());
        HLE::Patch(system, 0x80004D14, "FFNYLabPre");
        HLE::Patch(system, 0x80004D28, "FFNYLabPost");
        system.GetJitInterface().ClearSafe();
      }
      Reply("ok rec " + tag);
    }
    else if (c == "csave" || c == "cload")
    {
      // csave K STATIC.txt [OUT.txt]: compact snapshot (static ranges from file + heap walk decided now); cload K: restore it
      int k = 0;
      in >> k;
      CSnap& cs = s_csnap[k & 7];
      auto& ppc = system.GetPPCState();
      u8* ram = memory.GetRAM();
      const u32 size = memory.GetRamSizeReal();
      const auto t0 = std::chrono::steady_clock::now();
      if (c == "csave")
      {
        std::string path, outp, w;
        in >> path;
        s_ctag = false;
        while (in >> w) { if (w == "tag") s_ctag = true; else outp = w; }
        if (path != s_cstatic_path)
        {
          s_cstatic.clear();
          std::ifstream rf(path);
          unsigned lo, hi;
          while (rf >> std::hex >> lo >> hi) if (lo >= 0x80000000u && hi <= 0x804A2500u) s_cstatic.push_back({lo & 0x01FFFFFF, ((hi - 1) & 0x01FFFFFF) + 1});
          s_cstatic_path = path;
        }
        u32 nb = 0;
        CBuildRanges(ram, size, cs.ranges, &nb);
        const auto t1 = std::chrono::steady_clock::now();
        size_t tot = 0;
        for (auto [lo, hi] : cs.ranges) tot += hi - lo;
        cs.data.resize(tot);
        size_t o = 0;
        for (auto [lo, hi] : cs.ranges) { memcpy(cs.data.data() + o, ram + lo, hi - lo); o += hi - lo; }
        memcpy(cs.gpr, ppc.gpr, sizeof cs.gpr);
        memcpy(cs.ps, ppc.ps, sizeof cs.ps);
        cs.cr = ppc.cr; cs.fpscr = ppc.fpscr.Hex; cs.ca = ppc.xer_ca; cs.so = ppc.xer_so_ov;
        cs.lr = ppc.spr[SPR_LR]; cs.ctr = ppc.spr[SPR_CTR];
        for (int i = 0; i < 8; i++) cs.gqr[i] = ppc.spr[SPR_GQR0 + i];
        const auto t2 = std::chrono::steady_clock::now();
        if (!outp.empty())
        {
          FILE* f = fopen(outp.c_str(), "w");
          if (f) { for (auto [lo, hi] : cs.ranges) fprintf(f, "%08x %08x\n", lo | 0x80000000u, hi | 0x80000000u); fclose(f); }
        }
        Reply("ok csave ranges " + std::to_string(cs.ranges.size()) + " bytes " + std::to_string(tot) + " blocks " + std::to_string(nb) +
              " build_us " + std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()) +
              " copy_us " + std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()));
      }
      else
      {
        size_t o = 0;
        for (auto [lo, hi] : cs.ranges) { memcpy(ram + lo, cs.data.data() + o, hi - lo); o += hi - lo; }
        memcpy(ppc.gpr, cs.gpr, sizeof cs.gpr);
        memcpy(ppc.ps, cs.ps, sizeof cs.ps);
        ppc.cr = cs.cr; ppc.fpscr.Hex = cs.fpscr; ppc.xer_ca = cs.ca; ppc.xer_so_ov = cs.so;
        ppc.spr[SPR_LR] = cs.lr; ppc.spr[SPR_CTR] = cs.ctr;
        for (int i = 0; i < 8; i++) ppc.spr[SPR_GQR0 + i] = cs.gqr[i];
        const auto t1 = std::chrono::steady_clock::now();
        Reply("ok cload bytes " + std::to_string(o) + " us " + std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()));
      }
    }
    else if (c == "cp")
    {
      auto& f = system.GetCommandProcessor().GetFifo();
      auto& pi = system.GetProcessorInterface();
      char b[400];
      snprintf(b, sizeof b, "cp base %08x end %08x hi %08x lo %08x dist %08x wr %08x rd %08x bp %08x | rden %u link %u bpen %u bpint %u bp %u hiint %u loint %u hi %u lo %u | pi base %08x end %08x wr %08x int %08x mask %08x",
               f.CPBase.load(), f.CPEnd.load(), f.CPHiWatermark, f.CPLoWatermark, f.CPReadWriteDistance.load(), f.CPWritePointer.load(), f.CPReadPointer.load(), f.CPBreakpoint.load(),
               f.bFF_GPReadEnable.load(), f.bFF_GPLinkEnable.load(), f.bFF_BPEnable.load(), f.bFF_BPInt.load(), f.bFF_Breakpoint.load(), f.bFF_HiWatermarkInt.load(), f.bFF_LoWatermarkInt.load(), f.bFF_HiWatermark.load(), f.bFF_LoWatermark.load(),
               pi.m_fifo_cpu_base, pi.m_fifo_cpu_end, pi.m_fifo_cpu_write_pointer, pi.m_interrupt_cause, pi.m_interrupt_mask);
      Reply(b);
    }
    else if (c == "regs")
    {
      auto& ppc = system.GetPPCState();
      std::string r = "regs pc " + std::to_string(ppc.pc);
      for (int i = 0; i < 32; i++) r += " " + std::to_string(ppc.gpr[i]);
      Reply(r);
    }
    else if (c == "track")
    {
      in >> s_track_left;
      s_tbuf_valid = false;
      s_logicw.clear();
      HLE::Patch(system, 0x80004D14, "FFNYLabPre");
      HLE::Patch(system, 0x80004D28, "FFNYLabPost");
      system.GetJitInterface().ClearSafe();
      Reply("ok track");
    }
    else if (c == "trackdump")
    {
      std::string path;
      in >> path;
      FILE* f = fopen(path.c_str(), "wb");
      if (f)
      {
        fwrite(s_logicw.data(), 1, s_logicw.size(), f);
        fwrite(s_otherw.data(), 1, s_otherw.size(), f);
        fclose(f);
      }
      Reply("ok trackdump");
    }
    else if (c == "input")
    {
      std::string m;
      in >> m;
      if (m == "rec") s_inmode = 1;
      else if (m == "play") s_inmode = 2;
      else if (m == "off") s_inmode = 0;
      else if (m == "clear") { s_inputs.clear(); s_inplayed = s_inrecorded = 0; }
      HLE::Patch(system, 0x80004D14, "FFNYLabPre");
      HLE::Patch(system, 0x80004D28, "FFNYLabPost");
      system.GetJitInterface().ClearSafe();
      Reply("ok input " + m + " recorded " + std::to_string(s_inrecorded) + " played " + std::to_string(s_inplayed) + " entries " + std::to_string(s_inputs.size()));
    }
    else if (c == "storetrace")
    {
      std::string m;
      in >> m;
      if (m == "on" || m == "all") { g_store_trace = true; s_trace_logic_only = (m == "on"); }
      else if (m == "off") g_store_trace = false;
      else if (m == "clear") s_writes.clear();
      else if (m == "dump")
      {
        std::string path;
        in >> path;
        FILE* f = fopen(path.c_str(), "w");
        if (f)
        {
          for (auto& [a, w] : s_writes)
            fprintf(f, "%08x %08x %08x %08x %08x %u %u %u %u %08x %08x %08x %08x %u\n", a, w.pc[0], w.pc[1], w.pc[2], w.thread, w.count, w.mainw, w.otherw, w.threadw, w.tpc, w.lr[0], w.lr[1], w.lr[2], w.mainout);
          fclose(f);
        }
      }
      HLE::Patch(system, 0x80004D14, "FFNYLabPre");
      HLE::Patch(system, 0x80004D28, "FFNYLabPost");
      system.GetJitInterface().ClearSafe();
      Reply("ok storetrace " + m + " words " + std::to_string(s_writes.size()));
    }
    else if (c == "hooktrace")
    {
      unsigned a;
      in >> std::hex >> a;
      HLE::Patch(system, a, "FFNYLabTrace");
      system.GetJitInterface().ClearSafe();
      Reply("ok hooktrace");
    }
    else if (c == "msave" || c == "mload")
    {
      int k = 0;
      in >> k;
      k &= 7;
      auto t0 = std::chrono::steady_clock::now();
      bool ok = true;
      if (c == "msave") s_msize[k] = State::LabSaveToBuffer(system, s_mstate[k]);
      else ok = State::LabLoadFromBuffer(system, s_mstate[k], s_msize[k]);
      auto us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
      Reply("ok " + c + " size " + std::to_string(s_msize[k]) + " us " + std::to_string(us) + (ok ? "" : " FAIL"));
    }
    else if (c == "spr")
    {
      int n;
      in >> n;
      Reply("spr " + std::to_string(system.GetPPCState().spr[n & 1023]));
    }
    else if (c == "loopcount")
    {
      Reply("loopcount " + std::to_string(s_loops));
    }
    else if (c == "loaddbg")
    {
      std::string path;
      in >> path;
      Reply("loaddbg " + State::LabLoadDebug(system, path));
    }
    else if (c == "xfb")
    {
      auto& vi = system.GetVideoInterface();
      char b[64];
      snprintf(b, sizeof b, "xfb %08x %08x", vi.GetXFBAddressTop(), vi.GetXFBAddressBottom());
      Reply(b);
    }
    else if (c == "fields")
    {
      Reply("fields " + std::to_string(s_fields));
    }
    else if (c == "quit")
    {
      Reply("ok quit");
      _exit(0);
    }
    else
    {
      Reply("? " + c);
    }
  }
}
}  // namespace FFNYLab
