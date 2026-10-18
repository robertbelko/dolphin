// FFNY rollback netcode, see FFNYRollback.h.
//
// How it works (game side, all verified in the headless lab, see the handoff):
//  - The main loop (0x80004CC0..0x80004D3C) runs one logic update per pad snapshot that the
//    game's input thread produced: tick() 0x800EC144 copies the next snapshot (10-entry ring in
//    the input-thread object, 0x260 bytes each) to object+0x17CC and returns 1; then
//    0x800F2094 (input) + 0x800830D4 (mode / fight update) run, then the per-update work at
//    0x80004CDC, then tick() again; when tick() returns 0 the frame is rendered.
//  - The snapshot holds a 0x10-byte header and 4 pad records of 0x94 bytes (one byte per held
//    button at 8-byte stride, sticks etc.; no edge flags: edges are derived by the game logic,
//    whose state is rewound). So a frame's input = the 4 records.
//  - Rollback state = static data / bss / sdata / main stack (table below) + heap 0 walked at
//    every save (block headers, contents of used blocks that are not tagged by the rbtag DOL
//    patch and are not sound-engine data), minus allocator mutexes and OS thread structs /
//    non-main stacks. Saved before each logic update at the PRE hook.
//  - Re-simulation: load the state of frame S at the PRE hook (registers included, same PC),
//    then run updates S..F-1 with the corrected inputs; the TICK hook makes tick() "return 1"
//    without consuming a real snapshot while re-simulating, so everything happens inside one
//    main-loop pass before the render.
#include "Core/FFNYRollback.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <optional>
#include <unordered_map>

#include <SFML/Network/IpAddress.hpp>
#include <SFML/Network/UdpSocket.hpp>

#include "Common/Logging/Log.h"
#include "Core/Core.h"
#include "Core/HLE/HLE.h"
#include "Core/HW/Memmap.h"
#include "Core/PowerPC/Gekko.h"
#include "Core/PowerPC/JitInterface.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/System.h"
#include "VideoCommon/OnScreenDisplay.h"

namespace FFNYRollback
{
#include "Core/FFNYRollbackRanges.inc"

namespace
{
constexpr u32 ADDR_PRE = 0x80004D14;   // mr r3,r26 (right before the input / update calls)
constexpr u32 ADDR_TICK = 0x80004CFC;  // bl 0x800EC144 (tick)
constexpr u32 ADDR_TICK_FN = 0x800EC144;
constexpr u32 INSN_PRE = 0x7F43D378;
constexpr u32 INSN_TICK = 0x480E7449;
constexpr u32 ADDR_COUNTER = 0x80440C24;  // logic update counter (rewound with the state)
constexpr u32 ADDR_FIGHT = 0x803F1000;    // fight loop state, 1 = fight running
constexpr u32 ADDR_HEAP_HOOK = 0x8040BD64;  // allocator post-alloc hook word (rbtag installs 0x8049A454)
constexpr u32 SNAP_OFS = 0x17CC;  // pad snapshot in the input-thread object (r28)
constexpr u32 REC_OFS = 0x10;
constexpr u32 REC_SIZE = 0x94;
constexpr u32 RING = 128;  // frames of input history kept
constexpr u32 NSLOTS = 12;  // saved states
constexpr u32 SEND_WINDOW = 8;  // redundant frames per input message

using Record = std::array<u8, REC_SIZE>;

struct FrameInput
{
  u32 frame = 0xFFFFFFFF;
  bool have = false;  // received (remote) / captured (local)
  Record rec{};
};

struct SavedState
{
  u32 frame = 0xFFFFFFFF;
  std::vector<std::pair<u32, u32>> ranges;
  std::vector<u8> data;
  u32 gpr[32];
  PowerPC::PairedSingle ps[32];
  PowerPC::ConditionRegister cr;
  u32 fpscr, lr, ctr, gqr[8];
  u8 ca, so;
};

Settings s_settings;
std::function<void(Core::System&, u32)> s_fight_end_cb;
Transport* s_transport = nullptr;
bool s_installed = false;
bool s_install_failed = false;
bool s_active = false;
u32 s_start_frame = 0;
u32 s_resim_target = 0;  // != 0 while re-simulating up to (excluding) this frame
std::array<std::array<FrameInput, RING>, 4> s_input;  // per port: received / captured inputs
std::array<std::array<FrameInput, RING>, 4> s_used;   // per port: what the simulation used
std::array<Record, 4> s_base{};                       // records at fight start (prediction seed)
std::array<u32, 4> s_confirmed{};                     // all inputs <= this frame received
std::array<SavedState, NSLOTS> s_states;
std::unordered_map<u32, u32> s_local_hash, s_remote_hash;
u32 s_hashed_upto = 0;
u32 s_time_base = 0, s_time_dt = 0;  // game timing object 0x8035D618: +8 VI count, +0xC measured dt

// statistics
u32 s_frames = 0, s_rollbacks = 0, s_resim_frames = 0, s_max_depth = 0, s_waits = 0;
u64 s_wait_us = 0, s_save_us = 0, s_load_us = 0;
u32 s_desyncs = 0, s_hash_checked = 0;
bool s_log = false;
u32 s_stress = 0;
u32 s_loop = 0;  // lab: pad 1 is played locally but its inputs arrive as "remote" inputs s_loop frames late
std::vector<std::pair<u32, Record>> s_loop_q;  // lab: force a rollback of up to N frames every frame (inputs unchanged)
std::unordered_map<u32, u32> s_first_hash;  // lab stress: hash at the first visit of each frame
u32 s_stress_bad = 0, s_stress_checked = 0;

u32 Rd32(const u8* ram, u32 a)
{
  a &= 0x01FFFFFF;
  return (u32(ram[a]) << 24) | (u32(ram[a + 1]) << 16) | (u32(ram[a + 2]) << 8) | ram[a + 3];
}

bool IsRemote(int port)
{
  return (s_settings.active_ports >> port & 1) && !(s_settings.local_ports >> port & 1);
}

void BuildRanges(const u8* ram, u32 size, std::vector<std::pair<u32, u32>>& out)
{
  auto rd = [&](u32 a) { return Rd32(ram, a); };
  std::vector<std::pair<u32, u32>> inc, ex;
  inc.reserve(1200);
  for (const auto& r : s_static_ranges)
    inc.push_back({r[0] & 0x01FFFFFF, r[1] & 0x01FFFFFF});
  const u32 H0 = 0x8060C050, base = rd(H0 + 8), end = rd(H0 + 0xC);
  inc.push_back({base & 0x01FFFFFF, (base + 0x90) & 0x01FFFFFF});
  u32 a = base + 0x90, n = 0;
  while (a >= base && a < end && n < 20000)
  {
    const u32 w = rd(a), sz = rd(a + 4), nx = rd(a + 8);
    // header (heap layout); of a free block also the first 8 content bytes (its free-list links). Not of a used
    // block: a system block's first bytes are system data (e.g. the audio stream list links in the Aems stream buffer)
    inc.push_back({a & 0x01FFFFFF, (a + ((w >> 16) == 0x4246 ? 0x18 : 0x10)) & 0x01FFFFFF});
    if ((w >> 16) == 0x424D && sz && !(w & 0x80))
    {
      const u32 mg = rd(a + 16);
      if (mg != 0x4D4F4952 && mg != 0x41424B43 && mg != 0x454D4954)  // MOIR / ABKC / EMIT
        inc.push_back({(a + 16) & 0x01FFFFFF, (a + 16 + sz) & 0x01FFFFFF});
    }
    if (nx <= a)
      break;
    a = nx;
    n++;
  }
  for (u32 k = 0; k < 64; k++)  // allocator mutexes
  {
    const u32 h = rd(0x8040BD70 + 4 * k);
    if ((h & 0xFE000000) == 0x80000000)
      ex.push_back({(h + 0x3C) & 0x01FFFFFF, (h + 0x54) & 0x01FFFFFF});
  }
  ex.push_back({0x0049A400, 0x0049A410});  // rbtag: previous hook, SysHeap index
  ex.push_back({0x0049B400, 0x0049BF00});  // rbtag: deferred-free queue, sound-ref / free logs and flags (system state)
  const u32 pool = rd(0x802E9B00);
  if (pool)
    ex.push_back({(pool + 0x2A4) & 0x01FFFFFF, (pool + 0x2BC) & 0x01FFFFFF});
  for (u32 t = rd(0x800000DC), k = 0; t && k < 64; t = rd(t + 0x2FC), k++)  // OS threads
  {
    ex.push_back({t & 0x01FFFFFF, (t + 0x318) & 0x01FFFFFF});
    if (t != 0x80432D10)
      ex.push_back({rd(t + 0x308) & 0x01FFFFFF, rd(t + 0x304) & 0x01FFFFFF});
  }
  auto norm = [](std::vector<std::pair<u32, u32>>& v) {
    std::sort(v.begin(), v.end());
    std::vector<std::pair<u32, u32>> r;
    for (auto p : v)
    {
      if (p.first >= p.second)
        continue;
      if (!r.empty() && p.first <= r.back().second)
        r.back().second = std::max(r.back().second, p.second);
      else
        r.push_back(p);
    }
    v.swap(r);
  };
  norm(inc);
  norm(ex);
  if (s_stress)
  {
    static int shown = 0;
    for (auto [elo, ehi] : ex)
    {
      // does an exclusion cut into a heap block header?
      u32 a2 = base + 0x90, k = 0;
      while (a2 >= base && a2 < end && k++ < 20000)
      {
        const u32 h = a2 & 0x01FFFFFF;
        if (h < ehi && h + 16 > elo && shown < 10)
        {
          shown++;
          fprintf(stderr, "FFNYRB EXCL cuts header %08x: exclusion %08x-%08x\n", a2, elo | 0x80000000u, ehi | 0x80000000u);
        }
        const u32 nx2 = rd(a2 + 8);
        if (nx2 <= a2)
          break;
        a2 = nx2;
      }
    }
  }
  out.clear();
  size_t j = 0;
  for (auto [lo, hi] : inc)
  {
    hi = std::min(hi, size);
    while (j < ex.size() && ex[j].second <= lo)
      j++;
    u32 cur = lo;
    for (size_t k = j; k < ex.size() && ex[k].first < hi; k++)
    {
      if (ex[k].first > cur)
        out.push_back({cur, ex[k].first});
      cur = std::max(cur, ex[k].second);
    }
    if (cur < hi)
      out.push_back({cur, hi});
  }
}


// lab: check the main heap's block list (links and contiguity); returns the first bad block or 0
u32 CheckHeap(const u8* ram)
{
  auto rd = [&](u32 a) { return Rd32(ram, a); };
  const u32 H0 = 0x8060C050, base = rd(H0 + 8), end = rd(H0 + 0xC);
  u32 a = base + 0x90, prev = 0, n = 0;
  while (a >= base && a < end && n < 20000)
  {
    const u32 w = rd(a), sz = rd(a + 4), nx = rd(a + 8), pv = rd(a + 12);
    if ((w >> 16) != 0x424D && (w >> 16) != 0x4246 && (w >> 16) != 0x4253)
      return a | 1;
    if (prev && pv != prev)
    {
      static int shown2 = 0;
      if (shown2++ < 4)
        fprintf(stderr, "FFNYRB HEAPCHECK detail2: %08x: %08x size %08x next %08x prev %08x (came from %08x: %08x size %08x)\n", a, w, sz, nx, pv,
                prev, rd(prev), rd(prev + 4));
      return a | 2;
    }
    if (nx <= a)
      break;
    if (((w >> 16) == 0x424D && nx < a + 16 + sz) || ((w >> 16) == 0x4246 && nx < a + sz))
    {
      static int shown = 0;
      if (shown++ < 3)
        fprintf(stderr, "FFNYRB HEAPCHECK detail: %08x: %08x size %08x next %08x prev %08x\n", a, w, sz, nx, pv);
      return a | 3;
    }
    prev = a;
    a = nx;
    n++;
  }
  return 0;
}

void SaveState(Core::System& system, u32 frame)
{
  const auto t0 = std::chrono::steady_clock::now();
  auto& memory = system.GetMemory();
  auto& ppc = system.GetPPCState();
  const u8* ram = memory.GetRAM();
  SavedState& s = s_states[frame % NSLOTS];
  if (s_stress && s.frame == frame)
  {
    // lab stress: the re-simulated state of this frame must equal the first one
    std::vector<std::pair<u32, u32>> nr;
    BuildRanges(ram, memory.GetRamSizeReal(), nr);
    static u32 reports = 0;
    if (nr != s.ranges)
    {
      if (reports++ < 30)
        fprintf(stderr, "FFNYRB STRESS frame %u: range list differs (%zu vs %zu ranges)\n", frame, nr.size(), s.ranges.size());
    }
    else
    {
      size_t o2 = 0;
      u32 ndiff = 0;
      for (auto [lo, hi] : s.ranges)
      {
        for (u32 a = lo; a < hi; a += 4)
        {
          if (a >= 0x004424C0 && a < (system.GetPPCState().gpr[1] & 0x01FFFFFF))
            continue;  // dead stack below the current stack pointer
          if (a == 0x002F4CB0)
            continue;  // GPU fence counter (render-driven, harmless)
          if (memcmp(ram + a, s.data.data() + o2 + (a - lo), std::min<u32>(4, hi - a)) != 0)
          {
            if (ndiff < 12 && reports < 400)
            {
              reports++;
              const u8* op = s.data.data() + o2 + (a - lo);
              const u32 ov = (u32(op[0]) << 24) | (u32(op[1]) << 16) | (u32(op[2]) << 8) | op[3];
              fprintf(stderr, "FFNYRB STRESS frame %u: %08x differs (was %08x now %08x)\n", frame, a | 0x80000000u,
                      ov, Rd32(ram, a));
            }
            ndiff++;
          }
        }
        o2 += hi - lo;
      }
      if (ndiff)
        fprintf(stderr, "FFNYRB STRESS frame %u: %u words differ\n", frame, ndiff);
    }
  }
  if (s_stress)
  {
    static u32 bad_reports = 0;
    const u32 bad = CheckHeap(ram);
    if (bad && bad_reports++ < 20)
      fprintf(stderr, "FFNYRB HEAPCHECK save frame %u: bad block %08x (kind %u), heap0 mutex owner %08x\n", frame,
              bad & ~3u, bad & 3, Rd32(ram, 0x8060C094));
  }
  s.frame = frame;
  BuildRanges(ram, memory.GetRamSizeReal(), s.ranges);
  size_t tot = 0;
  for (auto [lo, hi] : s.ranges)
    tot += hi - lo;
  s.data.resize(tot);
  size_t o = 0;
  for (auto [lo, hi] : s.ranges)
  {
    memcpy(s.data.data() + o, ram + lo, hi - lo);
    o += hi - lo;
  }
  memcpy(s.gpr, ppc.gpr, sizeof s.gpr);
  memcpy(s.ps, ppc.ps, sizeof s.ps);
  s.cr = ppc.cr;
  s.fpscr = ppc.fpscr.Hex;
  s.ca = ppc.xer_ca;
  s.so = ppc.xer_so_ov;
  s.lr = ppc.spr[SPR_LR];
  s.ctr = ppc.spr[SPR_CTR];
  for (int i = 0; i < 8; i++)
    s.gqr[i] = ppc.spr[SPR_GQR0 + i];
  s_save_us += std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now() - t0)
                   .count();
}

bool LoadState(Core::System& system, u32 frame)
{
  SavedState& s = s_states[frame % NSLOTS];
  if (s.frame != frame)
    return false;
  const auto t0 = std::chrono::steady_clock::now();
  auto& ppc = system.GetPPCState();
  u8* ram = system.GetMemory().GetRAM();
  size_t o = 0;
  for (auto [lo, hi] : s.ranges)
  {
    memcpy(ram + lo, s.data.data() + o, hi - lo);
    o += hi - lo;
  }
  memcpy(ppc.gpr, s.gpr, sizeof s.gpr);
  memcpy(ppc.ps, s.ps, sizeof s.ps);
  ppc.cr = s.cr;
  ppc.fpscr.Hex = s.fpscr;
  ppc.xer_ca = s.ca;
  ppc.xer_so_ov = s.so;
  ppc.spr[SPR_LR] = s.lr;
  ppc.spr[SPR_CTR] = s.ctr;
  for (int i = 0; i < 8; i++)
    ppc.spr[SPR_GQR0 + i] = s.gqr[i];
  if (s_stress)
  {
    static u32 bad_reports = 0;
    const u32 bad = CheckHeap(ram);
    if (bad && bad_reports++ < 20)
      fprintf(stderr, "FFNYRB HEAPCHECK load frame %u: bad block %08x (kind %u)\n", frame, bad & ~3u, bad & 3);
  }
  s_load_us += std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now() - t0)
                   .count();
  return true;
}

void Put32(std::vector<u8>& m, u32 v)
{
  m.push_back(u8(v >> 24));
  m.push_back(u8(v >> 16));
  m.push_back(u8(v >> 8));
  m.push_back(u8(v));
}
u32 Get32(const u8* p)
{
  return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | p[3];
}

// message 'I': port, first frame, count, count records
void SendInputs(int port, u32 last)
{
  if (!s_transport)
    return;
  const u32 first = std::max(s_start_frame, last >= SEND_WINDOW - 1 ? last - (SEND_WINDOW - 1) : 0);
  std::vector<u8> m;
  m.push_back('I');
  m.push_back(u8(port));
  Put32(m, first);
  m.push_back(u8(last - first + 1));
  for (u32 f = first; f <= last; f++)
  {
    const FrameInput& fi = s_input[port][f % RING];
    m.insert(m.end(), fi.rec.begin(), fi.rec.end());
  }
  s_transport->Send(m);
}

u32 s_mispredict_from = 0xFFFFFFFF;
u32 s_last_rb_from = 0, s_last_rb_to = 0;  // earliest frame whose used input turned out wrong

void OnRemoteInput(int port, u32 f, const u8* rec)
{
  if (!IsRemote(port) || f < s_start_frame)
    return;
  FrameInput& fi = s_input[port][f % RING];
  if (fi.have && fi.frame == f)
    return;
  fi.frame = f;
  fi.have = true;
  memcpy(fi.rec.data(), rec, REC_SIZE);
  while (true)
  {
    const u32 nf = s_confirmed[port] + 1;
    const FrameInput& n = s_input[port][nf % RING];
    if (n.have && n.frame == nf)
      s_confirmed[port] = nf;
    else
      break;
  }
  const FrameInput& u = s_used[port][f % RING];
  if (u.have && u.frame == f && u.rec != fi.rec)
    s_mispredict_from = std::min(s_mispredict_from, f);
}

void Pump()
{
  if (!s_transport)
    return;
  std::vector<u8> m;
  while (s_transport->Receive(m))
  {
    if (m.size() >= 7 && m[0] == 'I')
    {
      const int port = m[1] & 3;
      const u32 first = Get32(&m[2]);
      const u32 count = m[6];
      if (m.size() < 7 + size_t(count) * REC_SIZE)
        continue;
      for (u32 i = 0; i < count; i++)
        OnRemoteInput(port, first + i, &m[7 + i * REC_SIZE]);
    }
    else if (m.size() >= 9 && m[0] == 'H')
    {
      const u32 f = Get32(&m[1]), h = Get32(&m[5]);
      s_remote_hash[f] = h;
      auto it = s_local_hash.find(f);
      if (it != s_local_hash.end())
      {
        s_hash_checked++;
        if (it->second != h)
        {
          s_desyncs++;
          fprintf(stderr, "FFNYRB DESYNC at frame %u (local %08x remote %08x)\n", f, it->second, h);
          if (s_desyncs <= 3)
            OSD::AddMessage("FFNY rollback: DESYNC detected at frame " + std::to_string(f), 8000,
                            OSD::Color::RED);
        }
      }
    }
  }
}

u32 MinConfirmed()
{
  u32 m = 0xFFFFFFFF;
  for (int p = 0; p < 4; p++)
    if (IsRemote(p))
      m = std::min(m, s_confirmed[p]);
  return m == 0xFFFFFFFF ? 0x7FFFFFFF : m;
}

const Record& InputFor(int port, u32 f)
{
  const FrameInput& fi = s_input[port][f % RING];
  if (fi.have && fi.frame == f)
    return fi.rec;
  // prediction: the newest input we have for that port before f
  for (u32 k = 1; k < RING && f >= s_start_frame + k; k++)
  {
    const FrameInput& p = s_input[port][(f - k) % RING];
    if (p.have && p.frame == f - k)
      return p.rec;
  }
  return s_base[port];
}

void Inject(u8* ram, u32 tick_obj, u32 f)
{
  // The game's per-update timing object (written by 0x80021D80 right before each update) holds the VI
  // retrace count of the update (+8) and the measured time step (+0xC). Both come from real time, so a
  // re-simulated or remote frame would see other values: make them a function of the frame number.
  {
    const u32 t = s_time_base + 2 * (f - s_start_frame);
    u8* o = ram + 0x0035D618;
    o[8] = u8(t >> 24); o[9] = u8(t >> 16); o[10] = u8(t >> 8); o[11] = u8(t);
    o[12] = u8(s_time_dt >> 24); o[13] = u8(s_time_dt >> 16); o[14] = u8(s_time_dt >> 8); o[15] = u8(s_time_dt);
  }
  for (int port = 0; port < 4; port++)
  {
    if (!(s_settings.active_ports >> port & 1))
      continue;
    const Record& r = InputFor(port, f);
    memcpy(ram + ((tick_obj + SNAP_OFS + REC_OFS + port * REC_SIZE) & 0x01FFFFFF), r.data(),
           REC_SIZE);
    FrameInput& u = s_used[port][f % RING];
    u.frame = f;
    u.have = true;
    u.rec = r;
  }
}

u32 Fnv(const u8* p, size_t n, u32 h)
{
  for (size_t i = 0; i < n; i++)
    h = (h ^ p[i]) * 16777619u;
  return h;
}

void HashFrame(const u8* ram, u32 f)
{
  // fighters, fight RNG, both shared random generators, logic counter
  u32 h = 2166136261u;
  h = Fnv(ram + 0x3F1144, 4 * 0x12F8, h);
  h = Fnv(ram + 0x3F6AC8, 0x88, h);
  h = Fnv(ram + 0x35D588, 0x18, h);
  h = Fnv(ram + 0x3BE03C, 0x88, h);
  h = Fnv(ram + 0x440C24, 4, h);
  s_local_hash[f] = h;
  if (s_local_hash.size() > 4096)
    s_local_hash.erase(s_local_hash.begin());
  auto it = s_remote_hash.find(f);
  if (it != s_remote_hash.end())
  {
    s_hash_checked++;
    if (it->second != h)
    {
      s_desyncs++;
      fprintf(stderr, "FFNYRB DESYNC at frame %u (local %08x remote %08x)\n", f, h, it->second);
      if (s_desyncs <= 3)
        OSD::AddMessage("FFNY rollback: DESYNC detected at frame " + std::to_string(f), 8000,
                        OSD::Color::RED);
    }
  }
  if (s_transport)
  {
    std::vector<u8> m;
    m.push_back('H');
    Put32(m, f);
    Put32(m, h);
    s_transport->Send(m);
  }
}

u32 StressHash(const u8* ram)
{
  u32 h = 2166136261u;
  h = Fnv(ram + 0x3F1144, 4 * 0x12F8, h);
  h = Fnv(ram + 0x3F6AC8, 0x88, h);
  h = Fnv(ram + 0x35D588, 0x18, h);
  h = Fnv(ram + 0x3BE03C, 0x88, h);
  return h;
}

void StressCheck(const u8* ram, u32 f, bool first)
{
  if (!s_stress)
    return;
  const u32 h = StressHash(ram);
  if (first)
  {
    s_first_hash[f] = h;
    if (s_first_hash.size() > 600)
      s_first_hash.erase(s_first_hash.begin());
    return;
  }
  auto it = s_first_hash.find(f);
  if (it == s_first_hash.end())
    return;
  s_stress_checked++;
  if (it->second != h && s_stress_bad++ < 20)
    fprintf(stderr, "FFNYRB STRESS MISMATCH at frame %u after re-simulation\n", f);
}

// rbtag v4 words (sound reference bookkeeping; see rbtag.c)
constexpr u32 W_CONF = 0x8049A40C, W_ACT = 0x8049B500, W_FRAME = 0x8049B504, W_RB = 0x8049B508;
void Wr32(u8* ram, u32 a, u32 v)
{
  u8* p = ram + (a & 0x01FFFFFF);
  p[0] = u8(v >> 24); p[1] = u8(v >> 16); p[2] = u8(v >> 8); p[3] = u8(v);
}

void StartFight(u8* ram, u32 tick_obj, u32 f)
{
  s_active = true;
  Wr32(ram, W_CONF, f - 1);
  Wr32(ram, W_RB, 0);
  Wr32(ram, W_FRAME, f);
  Wr32(ram, W_ACT, 1);
  s_start_frame = f;
  s_resim_target = 0;
  s_mispredict_from = 0xFFFFFFFF;
  s_hashed_upto = f;
  s_time_base = Rd32(ram, 0x8035D620);
  s_time_dt = 0x3C888889;  // 1/60 s
  for (int p = 0; p < 4; p++)
  {
    for (auto& fi : s_input[p])
      fi = FrameInput{};
    for (auto& fi : s_used[p])
      fi = FrameInput{};
    memcpy(s_base[p].data(), ram + ((tick_obj + SNAP_OFS + REC_OFS + p * REC_SIZE) & 0x01FFFFFF),
           REC_SIZE);
    s_confirmed[p] = f - 1;
  }
  for (auto& s : s_states)
    s.frame = 0xFFFFFFFF;
  s_local_hash.clear();
  s_remote_hash.clear();
  // frames f .. f+delay-1: every machine uses its own fight-start records for its local pads
  for (int p = 0; p < 4; p++)
    if (s_settings.local_ports >> p & 1)
      for (u32 k = 0; k < s_settings.input_delay; k++)
      {
        FrameInput& fi = s_input[p][(f + k) % RING];
        fi.frame = f + k;
        fi.have = true;
        fi.rec = s_base[p];
      }
  if (s_loop)
  {
    // lab loopback: pad 1 stands for the remote player; its first delay frames come from the fight start
    s_loop_q.clear();
    for (u32 k = 0; k < s_settings.input_delay; k++)
    {
      FrameInput& fi = s_input[1][(f + k) % RING];
      fi.frame = f + k;
      fi.have = true;
      fi.rec = s_base[1];
    }
    s_confirmed[1] = f + s_settings.input_delay - 1;
  }
  s_rollbacks = s_resim_frames = s_max_depth = s_waits = s_frames = 0;
  s_wait_us = s_save_us = s_load_us = 0;
  OSD::AddMessage("FFNY rollback: fight started (input delay " + std::to_string(s_settings.input_delay) +
                      ")",
                  3000);
  fprintf(stderr, "FFNYRB fight start at frame %u (local %x active %x delay %u)\n", f,
          s_settings.local_ports, s_settings.active_ports, s_settings.input_delay);
}

void PrintStats(u32 f)
{
  fprintf(stderr,
          "FFNYRB frame %u: %u frames, %u rollbacks, %u re-simulated (max depth %u), %u waits "
          "(%.1f ms), save %.2f ms/frame, load %.2f ms/rollback, hashes checked %u, desyncs %u\n",
          f, s_frames, s_rollbacks, s_resim_frames, s_max_depth, s_waits, s_wait_us / 1000.0,
          s_frames ? s_save_us / 1000.0 / s_frames : 0.0,
          s_rollbacks ? s_load_us / 1000.0 / s_rollbacks : 0.0, s_hash_checked, s_desyncs);
}
}  // namespace

void Configure(const Settings& settings, Transport* transport)
{
  s_settings = settings;
  s_transport = transport;
  s_active = false;
  s_log = std::getenv("FFNY_RB_LOG") != nullptr;
  s_stress = std::getenv("FFNY_RB_STRESS") ? std::atoi(std::getenv("FFNY_RB_STRESS")) : 0;
  s_loop = std::getenv("FFNY_RB_LOOP") ? std::atoi(std::getenv("FFNY_RB_LOOP")) : 0;
}

void SetFightEndCallback(std::function<void(Core::System&, u32 frame)> callback)
{
  s_fight_end_cb = std::move(callback);
}

void Shutdown()
{
  s_fight_end_cb = nullptr;
  s_settings.enabled = false;
  s_transport = nullptr;
  s_active = false;
  s_installed = false;
  s_install_failed = false;
}

bool IsActive()
{
  return s_active;
}

std::string GetStatus()
{
  char b[200];
  snprintf(b, sizeof b, "rollback %s: %u rollbacks, max %u frames, desyncs %u",
           s_active ? "on" : "off", s_rollbacks, s_max_depth, s_desyncs);
  return b;
}

// Lab / LAN test transport: UDP with optional artificial latency.
// FFNY_RB=LOCALPORTS:ACTIVEPORTS (hex masks, e.g. 1:3), FFNY_RB_UDP=listenport:peerhost:peerport,
// FFNY_RB_LAG=ms (one way), FFNY_RB_JITTER=ms, FFNY_RB_DELAY=frames.
class UdpTransport final : public Transport
{
public:
  UdpTransport(unsigned short listen, const std::string& host, unsigned short port, int lag, int jitter)
      : m_peer(*sf::IpAddress::resolve(host)), m_port(port), m_lag(lag), m_jitter(jitter)
  {
    m_sock.setBlocking(false);
    if (m_sock.bind(listen) != sf::Socket::Status::Done)
      fprintf(stderr, "FFNYRB: cannot bind UDP port %u\n", listen);
  }
  void Send(const std::vector<u8>& msg) override
  {
    const auto now = std::chrono::steady_clock::now();
    const int d = m_lag + (m_jitter ? std::rand() % (m_jitter + 1) : 0);
    m_out.push_back({now + std::chrono::milliseconds(d), msg});
    Flush();
  }
  bool Receive(std::vector<u8>& msg) override
  {
    Flush();
    std::array<u8, 4096> buf;
    std::size_t got = 0;
    std::optional<sf::IpAddress> from;
    unsigned short fport = 0;
    if (m_sock.receive(buf.data(), buf.size(), got, from, fport) != sf::Socket::Status::Done)
      return false;
    msg.assign(buf.begin(), buf.begin() + got);
    return true;
  }

private:
  void Flush()
  {
    const auto now = std::chrono::steady_clock::now();
    // keep order: send the queue head while it is due (jitter never reorders)
    while (!m_out.empty() && m_out.front().first <= now)
    {
      (void)m_sock.send(m_out.front().second.data(), m_out.front().second.size(), m_peer, m_port);
      m_out.erase(m_out.begin());
    }
  }
  sf::UdpSocket m_sock;
  sf::IpAddress m_peer;
  unsigned short m_port;
  int m_lag, m_jitter;
  std::vector<std::pair<std::chrono::steady_clock::time_point, std::vector<u8>>> m_out;
};
std::unique_ptr<UdpTransport> s_udp;
bool s_env_checked = false;

void OnVIField(Core::System& system)
{
  if (!s_env_checked)
  {
    s_env_checked = true;
    const char* e = std::getenv("FFNY_RB");
    const char* u = std::getenv("FFNY_RB_UDP");
    if (e && (u || std::getenv("FFNY_RB_STRESS") || std::getenv("FFNY_RB_LOOP")))
    {
      Settings st;
      unsigned lp = 1, ap = 3;
      sscanf(e, "%x:%x", &lp, &ap);
      st.enabled = true;
      st.local_ports = lp;
      st.active_ports = ap;
      if (const char* d = std::getenv("FFNY_RB_DELAY"))
        st.input_delay = std::atoi(d);
      unsigned listen = 0, pport = 0;
      char host[128] = {};
      if (u)
        sscanf(u, "%u:%127[^:]:%u", &listen, host, &pport);
      const int lag = std::getenv("FFNY_RB_LAG") ? std::atoi(std::getenv("FFNY_RB_LAG")) : 0;
      const int jit = std::getenv("FFNY_RB_JITTER") ? std::atoi(std::getenv("FFNY_RB_JITTER")) : 0;
      if (u)
        s_udp = std::make_unique<UdpTransport>((unsigned short)listen, host, (unsigned short)pport, lag, jit);
      Configure(st, s_udp.get());
      fprintf(stderr, "FFNYRB: lab mode local %x active %x delay %u udp %s lag %d jitter %d stress %u\n", lp, ap,
              st.input_delay, u ? u : "-", lag, jit, s_stress);
    }
  }
  if (s_installed && s_log)
  {
    // lab watchdog: logic counter stuck while a fight was running
    static u32 last_ctr = 0, same = 0, dumped = 0;
    const u8* r = system.GetMemory().GetRAM();
    const u32 c = Rd32(r, ADDR_COUNTER);
    same = (c == last_ctr) ? same + 1 : 0;
    last_ctr = c;
    static u32 trashed = 0;
    const bool bad = Rd32(r, ADDR_FIGHT) > 1 && trashed++ == 0;
    if (bad || (same == 600 && dumped < 2 && Rd32(r, ADDR_FIGHT) == 1) || (same == 1800 && dumped < 2))
    {
      dumped++;
      auto& ppc = system.GetPPCState();
      fprintf(stderr, "FFNYRB WATCHDOG: counter %u stuck; pc %08x lr %08x thread %08x fight %u scr %08x active %d resim %u last rollback %u->%u\n",
              c, ppc.pc, ppc.spr[SPR_LR], Rd32(r, 0x800000E4), Rd32(r, ADDR_FIGHT), Rd32(r, 0x803EA56C), s_active,
              s_resim_target, s_last_rb_from, s_last_rb_to);
      if (const char* d = std::getenv("FFNY_RB_DUMP"))
      {
        if (FILE* f = fopen(d, "wb")) { fwrite(r, 1, system.GetMemory().GetRamSizeReal(), f); fclose(f); }
      }
    }
  }
  if (!s_settings.enabled || s_installed || s_install_failed)
    return;
  auto& memory = system.GetMemory();
  const u8* ram = memory.GetRAM();
  if (!ram)
    return;
  // wait until the game's code is in memory and the rbtag hook has been installed by the game
  static int dbg = 0;
  if (s_log && dbg++ % 300 == 0)
    fprintf(stderr, "FFNYRB: waiting for game code: %08x %08x %08x\n", Rd32(ram, ADDR_PRE), Rd32(ram, ADDR_TICK), Rd32(ram, ADDR_HEAP_HOOK));
  if (Rd32(ram, ADDR_PRE) != INSN_PRE || Rd32(ram, ADDR_TICK) != INSN_TICK)
    return;
  const u32 hook = Rd32(ram, ADDR_HEAP_HOOK);
  if (hook < 0x8049A410 || hook >= 0x8049B400 || Rd32(ram, 0x8049A408) != 0x52425435)  // "RBT5"
  {
    // the game installs the hook in main() right after the memory init; before that it is 0
    if (hook != 0)
    {
      fprintf(stderr, "FFNYRB: this start.dol has no rbtag v5 patch (hook %08x) - rollback disabled\n", hook);
      s_install_failed = true;
    }
    return;
  }
  HLE::Patch(system, ADDR_PRE, "FFNYRollbackPre");
  HLE::Patch(system, ADDR_TICK, "FFNYRollbackTick");
  system.GetJitInterface().ClearSafe();
  s_installed = true;
  fprintf(stderr, "FFNYRB: hooks installed\n");
}

void HookTick(const Core::CPUThreadGuard& guard)
{
  auto& ppc = guard.GetSystem().GetPPCState();
  ppc.spr[SPR_LR] = ADDR_TICK + 4;
  if (s_active && s_resim_target)
  {
    ppc.gpr[3] = 1;  // run the next (re-simulated) update without consuming a real snapshot
    ppc.npc = ADDR_TICK + 4;
  }
  else
  {
    ppc.npc = ADDR_TICK_FN;
  }
}

void HookPreUpdate(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  u8* ram = system.GetMemory().GetRAM();
  // the replaced instruction
  ppc.gpr[3] = ppc.gpr[26];
  ppc.npc = ADDR_PRE + 4;
  if (!s_settings.enabled)
    return;

  const u32 tick_obj = ppc.gpr[28];
  u32 f = Rd32(ram, ADDR_COUNTER);
  const bool fight = Rd32(ram, ADDR_FIGHT) == 1;

  if (!s_active)
  {
    if (!fight)
      return;
    StartFight(ram, tick_obj, f);
  }

  if (s_resim_target && f < s_resim_target)
  {
    // re-simulated frame: inputs are known better now
    StressCheck(ram, f, false);
    Inject(ram, tick_obj, f);
    SaveState(system, f);
    Wr32(ram, W_CONF, std::min(MinConfirmed(), f - 1));
    Wr32(ram, W_FRAME, f);
    s_resim_frames++;
    if (f > s_hashed_upto && MinConfirmed() >= f)
    {
      HashFrame(ram, f);
      s_hashed_upto = f;
    }
    return;
  }
  const bool caught_up = s_resim_target != 0;  // f == target: this frame's local input is known
  s_resim_target = 0;
  if (caught_up)
    StressCheck(ram, f, false);
  Pump();

  if (!caught_up)
  {
    // a new real frame: capture the local pads (they become the input of frame f + delay)
    s_frames++;
    for (int p = 0; p < 4; p++)
    {
      if (!(s_settings.local_ports >> p & 1))
        continue;
      const u32 tf = f + s_settings.input_delay;
      FrameInput& fi = s_input[p][tf % RING];
      fi.frame = tf;
      fi.have = true;
      memcpy(fi.rec.data(), ram + ((tick_obj + SNAP_OFS + REC_OFS + p * REC_SIZE) & 0x01FFFFFF),
             REC_SIZE);
      s_confirmed[p] = tf;
      SendInputs(p, tf);
    }
    if (s_loop)
    {
      // lab loopback: pad 1's real input of this tick is delivered as remote input s_loop frames later
      Record r;
      memcpy(r.data(), ram + ((tick_obj + SNAP_OFS + REC_OFS + 1 * REC_SIZE) & 0x01FFFFFF), REC_SIZE);
      s_loop_q.push_back({f + s_settings.input_delay, r});
      while (!s_loop_q.empty() && s_loop_q.front().first + s_loop <= f + s_settings.input_delay)
      {
        OnRemoteInput(1, s_loop_q.front().first, s_loop_q.front().second.data());
        s_loop_q.erase(s_loop_q.begin());
      }
    }
    if (s_log && s_frames % 600 == 0)
    {
      PrintStats(f);
      if (s_stress)
        fprintf(stderr, "FFNYRB stress: %u re-simulated frames compared, %u mismatches\n", s_stress_checked, s_stress_bad);
    }
    StressCheck(ram, f, true);
    if (s_stress && f > s_start_frame + s_stress && fight)
      s_mispredict_from = f - 1 - (std::rand() % s_stress);
  }

  // too far ahead of the other side (or a fight that ended on this machine with frames still
  // unconfirmed): wait for its inputs
  const auto wait_start = std::chrono::steady_clock::now();
  bool waited = false;
  while (true)
  {
    const u32 mc = MinConfirmed();
    const bool ahead = f > mc + s_settings.max_rollback;
    const bool ending = !fight && mc + 1 < f;
    if (s_mispredict_from != 0xFFFFFFFF || (!ahead && !ending))
      break;
    if (std::chrono::steady_clock::now() - wait_start > std::chrono::seconds(10))
    {
      fprintf(stderr, "FFNYRB: no inputs from the other side for 10 s - rollback stopped\n");
      OSD::AddMessage("FFNY rollback: no inputs from the other player for 10 s - rollback stopped",
                      8000, OSD::Color::RED);
      s_settings.enabled = false;
      s_active = false;
      Wr32(ram, W_ACT, 0);
      return;
    }
    waited = true;
    std::this_thread::sleep_for(std::chrono::microseconds(250));
    Pump();
  }
  if (waited)
  {
    s_waits++;
    s_wait_us += std::chrono::duration_cast<std::chrono::microseconds>(
                     std::chrono::steady_clock::now() - wait_start)
                     .count();
  }

  if (s_mispredict_from != 0xFFFFFFFF)
  {
    const u32 s = std::max(s_mispredict_from, s_start_frame);
    s_mispredict_from = 0xFFFFFFFF;
    if (s < f && LoadState(system, s))
    {
      s_rollbacks++;
      s_last_rb_from = f;
      s_last_rb_to = s;
      if (s_log && s_rollbacks % 50 == 0)
        fprintf(stderr, "FFNYRB rollback %u: %u -> %u fight %u scr %08x\n", s_rollbacks, f, s, Rd32(ram, ADDR_FIGHT), Rd32(ram, 0x803EA56C));
      s_max_depth = std::max(s_max_depth, f - s);
      s_resim_target = f;
      // we are frame s again (registers restored); do the replaced instruction again
      ppc.gpr[3] = ppc.gpr[26];
      ppc.npc = ADDR_PRE + 4;
      Inject(ram, ppc.gpr[28], s);
      SaveState(system, s);
      {
        const u32 old = Rd32(ram, W_RB);
        Wr32(ram, W_RB, old ? std::min(old, s + 1) : s + 1);
        Wr32(ram, W_CONF, std::min(MinConfirmed(), s - 1));
        Wr32(ram, W_FRAME, s);
      }
      s_resim_frames++;
      if (s > s_hashed_upto && MinConfirmed() >= s)
      {
        HashFrame(ram, s);
        s_hashed_upto = s;
      }
      return;
    }
    if (s < f)
      fprintf(stderr, "FFNYRB: state of frame %u is gone (now %u) - cannot roll back\n", s, f);
  }

  if (!fight)
  {
    // fight over and every input confirmed: back to normal netplay
    s_active = false;
    Wr32(ram, W_ACT, 0);  // rbtag: do the remaining deferred sound releases
    if (s_log)
      PrintStats(f);
    fprintf(stderr, "FFNYRB fight end at frame %u\n", f);
    OSD::AddMessage("FFNY rollback: fight over - " + std::to_string(s_rollbacks) + " rollbacks (max " +
                        std::to_string(s_max_depth) + " frames), " + std::to_string(s_desyncs) +
                        " desyncs; synchronising...",
                    6000);
    if (s_fight_end_cb)
      s_fight_end_cb(system, f);
    return;
  }

  Inject(ram, tick_obj, f);
  SaveState(system, f);
  Wr32(ram, W_CONF, std::min(MinConfirmed(), f - 1));
  Wr32(ram, W_FRAME, f);
  if (f > s_hashed_upto && MinConfirmed() >= f)
  {
    HashFrame(ram, f);
    s_hashed_upto = f;
  }
}
}  // namespace FFNYRollback
