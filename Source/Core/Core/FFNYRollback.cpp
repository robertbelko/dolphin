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
#include "Core/CoreTiming.h"
#include "Core/System.h"
#include "VideoCommon/OnScreenDisplay.h"

namespace FFNYRollback
{
void DumpDraws(u32 from, u32 to);
bool s_heapcheck = false;  // lab: FFNY_RB_HEAPCHECK
u32 s_probe_addr = 0;  // lab: FFNY_RB_PROBE=hexaddr -> log r3 and its heap block at that function (once per r3)
#include "Core/FFNYRollbackRanges.inc"

namespace
{
constexpr u32 ADDR_PRE = 0x80004D14;   // mr r3,r26 (right before the input / update calls)
constexpr u32 ADDR_TICK = 0x80004CFC;  // bl 0x800EC144 (tick)
constexpr u32 ADDR_TICK_FN = 0x800EC144;
constexpr u32 INSN_PRE = 0x7F43D378;
constexpr u32 INSN_TICK = 0x480E7449;
constexpr u32 ADDR_LOADER = 0x80121B4C;  // EAGL ELF .o loader: converts the file data in place
constexpr u32 ADDR_COUNTER = 0x80440C24;  // logic update counter (rewound with the state)
constexpr u32 ADDR_FIGHT = 0x803F1000;    // fight loop state, 1 = fight running
constexpr u32 ADDR_HEAP_HOOK = 0x8040BD64;  // allocator post-alloc hook word (rbtag installs 0x8049A454)
constexpr u32 SNAP_OFS = 0x17CC;  // pad snapshot in the input-thread object (r28)
constexpr u32 REC_OFS = 0x10;
constexpr u32 REC_SIZE = 0x94;
constexpr u32 RING = 128;  // frames of input history kept
constexpr u32 NSLOTS = 12;  // saved states
constexpr u32 SEND_WINDOW = 8;  // redundant frames per input message (old 'I' format)
// unreliable input messages stay below one ENet packet (MTU 1400 minus headers), so they are never fragmented
constexpr size_t MAX_UNRELIABLE_BYTES = 1100;

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
std::function<void(const std::string&)> s_report_cb;
u32 s_reports = 0;
std::function<void(Core::System&, u32)> s_fight_end_cb;
Transport* s_transport = nullptr;
// s_ack[r][p]: newest frame of our port p's inputs that the player on remote port r has confirmed (it holds every
// frame up to it); input messages always restart there, so any number of lost packets is repaired by one
u32 s_ack[4][4] = {};
u32 s_input_msgs_in = 0, s_input_msgs_stale = 0;  // stats
// time synchronisation: the newest current frame each remote player reported, and how far ahead of us it
// believed it was. Our own estimate f - remote frame is too large by the one-way latency, the remote's estimate
// too; the difference of the two is twice the real lead.
u32 s_frame_now = 0;
u32 s_remote_frame[4] = {};
s32 s_remote_adv[4] = {};
std::array<s32, 16> s_adv_hist{};
u32 s_adv_n = 0, s_sync_next = 0, s_sync_delays = 0;
bool s_sync_enabled = true;  // lab: FFNY_RB_NOSYNC
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
std::unordered_map<u32, std::array<u32, 8>> s_local_parts, s_remote_parts;
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
std::unordered_map<u32, std::vector<u8>> s_first_copy;  // lab stress: the hashed regions at the first visit
u32 s_stress_from = 0;  // lab: FFNY_RB_STRESS_FROM=frame -> forced rollbacks only from that frame on
u32 s_stress_bad = 0, s_stress_checked = 0;

// The game's ELF .o resource loader (0x80121B4C) converts a loaded file IN PLACE (byte swap of the
// header / section table / symbols, relocations) and has no "already converted" check. The file data
// lives in memory the rollback state does not rewind. Whether a rewind makes the game call the loader
// again depends on the caller (lab, 22 October: the ~30 files loaded ~50 frames into a fight are NOT
// converted again after a rewind; Robert's friend's crash shows other callers are). A second
// conversion swaps the header back -> crash at 0x80121DF0 (read from 0xC). Fix: on every conversion of
// fresh (little-endian) data keep a copy of the original bytes; when the loader is called on data that
// is already converted and a copy of the same address and size exists, put the original bytes back
// first, so the loader converts fresh data again, exactly like the first time. Copies are dropped once
// their frame is confirmed (no rewind can reach it any more) and at fight end.
struct LoaderUndo
{
  u32 frame;
  u32 addr;
  std::vector<u8> orig;
  u32 post_hash = 0;  // hash of the converted bytes (taken at the next logic update); 0 = not taken yet
  bool have_post = false;
};
std::vector<LoaderUndo> s_loader_undo;
bool s_loader_fix = true;
u32 s_loader_calls = 0, s_loader_restores = 0;
u32 s_lab_mom_every = 0;
u32 s_rng_from = 0, s_rng_to = 0;  // lab: FFNY_RB_RNGLOG=from:to -> log draws from generator 2 (0x803BE03C)
char s_phase = '-';  // 'U' = inside a logic update (after PRE), 'T' = after the TICK hook, 'R' = render part of the loop
// Render-side code also draws from the two shared random generators (lab 22 October: 0x800EDA24, called from the
// mode's render method 0x800838F4 via 0x80086690, draws randint(gen2, 255) every rendered frame while its effect is
// on, e.g. in Blazin' mode). Rendering happens once per DISPLAYED frame, never for re-simulated frames, so the number
// of draws per logic frame differs between the two machines -> the fight's random numbers diverge -> desync (the
// "Blazin' desync"). Fix: while a rollback fight runs, draws made in the render part of the main loop use the
// private generator states the rbaudrng part of the DOL already has for the audio module.
// The VI retrace counter (0x8043FC20, incremented by the SDK's VI thread 0x8024D6F0 via 0x8024CD94 on every
// retrace) is read by game logic through 0x8024E22C (21 call sites, e.g. 0x800E09C8 in the per-update countdown
// 0x800E0994 that draws from generator 1). Re-simulated frames all run inside one retrace and the two machines see
// different counts per logic frame -> logic diverges (lab 22 October, the second Blazin'-mode desync). During a
// rollback fight's logic update the getter returns a value derived from the frame number (the same as the timing
// object +8 gets in Inject); the counter itself is real-time system state and is not rewound.
constexpr u32 ADDR_VICOUNT_GET = 0x8024E22C;  // lwz r3,-0x7BC0(r13); blr
constexpr u32 ADDR_VICOUNT = 0x8043FC20;
u32 s_vi_fixed = 0;
u32 s_vi_base = 0;     // real VI count at fight start
bool s_vi_fix = true;
constexpr u32 ADDR_RENDER = 0x80004D88;  // bl 0x8001ED94: start of the render part (tick() returned 0)
constexpr u32 GEN1_FN = 0x8001FC88, GEN1_SHARED = 0x8035D588, GEN1_PRIVATE = 0x8049A210;
constexpr u32 GEN2_FN = 0x800B7DAC, GEN2_SHARED = 0x803BE03C, GEN2_PRIVATE = 0x8049A228;
bool s_rng_redirect = false;  // the DOL has the private states (rbaudrng stub at 0x8049A200)
u32 s_rng_redirected = 0;
// lab stress: every shared-generator draw per frame, first visit and newest re-simulation
struct Draw
{
  char gen, phase;
  u32 lr, b1, b2, b3;
  u32 r29, r31;
};
std::unordered_map<u32, std::vector<Draw>> s_draws_first, s_draws_resim;
u32 s_watch_lo = 0, s_watch_len = 0;  // lab: FFNY_RB_WATCHRANGE=hexaddr:hexlen, recorded at every PRE
std::unordered_map<u32, std::vector<u8>> s_watch_first, s_watch_resim;
u32 s_draw_frame = 0;
bool s_draw_first = true;
u32 s_draw_dumps = 0;  // lab: FFNY_RB_MOM=N -> every N frames of the fight both momentum bars = 98.5 (deterministic)

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
  ex.push_back({0x0049B400, 0x0049BF00});
  ex.push_back({ADDR_VICOUNT & 0x01FFFFFF, (ADDR_VICOUNT + 8) & 0x01FFFFFF});  // VI retrace counters (real time)  // rbtag: deferred-free queue, sound-ref / free logs and flags (system state)
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

s32 LocalAdvantage()
{
  s32 adv = 0;
  bool any = false;
  for (int r = 0; r < 4; r++)
    if (IsRemote(r) && s_remote_frame[r] >= s_start_frame)
    {
      const s32 a = s32(s_frame_now - s_remote_frame[r]);
      adv = any ? std::max(adv, a) : a;
      any = true;
    }
  return adv;
}

// message 'J': port, first frame, count, 4 acks (newest confirmed frame we hold of each port, 0xFFFFFFFF for
// ports that are not remote here), then count records, each a flag byte (0: same as the previous record in this
// message, 1: 148 record bytes follow). Starts at the oldest frame a remote player still lacks.
void SendInputs(int port, u32 last, bool reliable = false)
{
  if (!s_transport)
    return;
  u32 first = last;
  for (int r = 0; r < 4; r++)
    if (IsRemote(r))
      first = std::min(first, s_ack[r][port] + 1);
  first = std::max({first, s_start_frame, last >= RING - 1 ? last - (RING - 2) : 0u});
  std::vector<u8> m;
  m.reserve(MAX_UNRELIABLE_BYTES);
  m.push_back('J');
  m.push_back(u8(port));
  Put32(m, first);
  const size_t count_pos = m.size();
  m.push_back(0);
  for (int r = 0; r < 4; r++)
    Put32(m, IsRemote(r) ? s_confirmed[r] : 0xFFFFFFFF);
  Put32(m, s_frame_now);
  Put32(m, u32(LocalAdvantage()));
  const Record* prev = nullptr;
  u32 n = 0;
  for (u32 f = first; f <= last && n < 255; f++)
  {
    const FrameInput& fi = s_input[port][f % RING];
    if (!fi.have || fi.frame != f)
      break;
    if (prev && *prev == fi.rec)
    {
      m.push_back(0);
    }
    else
    {
      if (!reliable && m.size() + 1 + REC_SIZE > MAX_UNRELIABLE_BYTES)
        break;  // the rest goes in the next message (the oldest frames first: they unblock the other side)
      m.push_back(1);
      m.insert(m.end(), fi.rec.begin(), fi.rec.end());
    }
    prev = &fi.rec;
    n++;
  }
  m[count_pos] = u8(n);
  if (reliable)
    s_transport->Send(m);
  else
    s_transport->SendUnreliable(m);
}

// all local pads: resend what the other side lacks (while waiting for it, and reliably at fight end)
void ResendInputs(bool reliable)
{
  for (int p = 0; p < 4; p++)
    if (s_settings.local_ports >> p & 1)
      SendInputs(p, s_confirmed[p], reliable);
}

u32 s_mispredict_from = 0xFFFFFFFF;
u32 s_last_rb_from = 0, s_last_rb_to = 0;  // earliest frame whose used input turned out wrong

void OnRemoteInput(int port, u32 f, const u8* rec)
{
  // unreliable delivery: late, duplicated and reordered frames arrive; keep only frames we lack that fit the ring
  if (!IsRemote(port) || f < s_start_frame || f <= s_confirmed[port] || f >= s_confirmed[port] + RING)
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

const u8* s_ram = nullptr;  // emulated RAM (set by the hooks; for reports from Pump)
constexpr int NPARTS_FWD = 8;
void CompareParts(u32 f, const std::array<u32, NPARTS_FWD>& local, const std::array<u32, NPARTS_FWD>& remote,
                  const u8* ram);
void Pump()
{
  if (!s_transport)
    return;
  std::vector<u8> m;
  while (s_transport->Receive(m))
  {
    if (m.size() >= 31 && m[0] == 'J')
    {
      const int port = m[1] & 3;
      const u32 first = Get32(&m[2]);
      const u32 count = m[6];
      s_input_msgs_in++;
      if (s_active && first + count <= s_confirmed[port] + 1)
        s_input_msgs_stale++;  // nothing new in it (duplicate or late)
      for (int lp = 0; lp < 4; lp++)
      {
        const u32 a = Get32(&m[7 + 4 * lp]);
        if (a != 0xFFFFFFFF && (s_settings.local_ports >> lp & 1) && a > s_ack[port][lp] && a <= s_confirmed[lp])
          s_ack[port][lp] = a;
      }
      {
        const u32 rf = Get32(&m[23]);
        if (rf >= s_start_frame && rf >= s_remote_frame[port])
        {
          s_remote_frame[port] = rf;
          s_remote_adv[port] = s32(Get32(&m[27]));
        }
      }
      size_t pos = 31;
      const u8* prev = nullptr;
      for (u32 i = 0; i < count && pos < m.size(); i++)
      {
        const u8* rec;
        if (m[pos++] == 0)
        {
          if (!prev)
            break;
          rec = prev;
        }
        else
        {
          if (pos + REC_SIZE > m.size())
            break;
          rec = &m[pos];
          pos += REC_SIZE;
        }
        OnRemoteInput(port, first + i, rec);
        prev = rec;
      }
    }
    else if (m.size() >= 7 && m[0] == 'I')
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
      if (m.size() >= 9 + 4 * NPARTS_FWD)
      {
        std::array<u32, NPARTS_FWD> parts;
        for (int i = 0; i < NPARTS_FWD; i++)
          parts[i] = Get32(&m[9 + 4 * i]);
        s_remote_parts[f] = parts;
        if (f >= 4096)
          s_remote_parts.erase(f - 4096);
      }
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
          auto lp = s_local_parts.find(f);
          auto rp = s_remote_parts.find(f);
          if (s_desyncs <= 3 && lp != s_local_parts.end() && rp != s_remote_parts.end() && s_ram)
            CompareParts(f, lp->second, rp->second, s_ram);
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

void Wr32(u8* ram, u32 a, u32 v);
void Inject(u8* ram, u32 tick_obj, u32 f)
{
  if (s_lab_mom_every && f > s_start_frame && (f - s_start_frame) % s_lab_mom_every == 0)
    for (u32 i = 0; i < 2; i++)
      Wr32(ram, 0x803F1144 + i * 0x12F8 + 0x364, 0x42C50000);  // 98.5
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

// parts of the checked state, hashed separately so that a desync report can name what differs
constexpr int NPARTS = 8;
const char* const PART_NAMES[NPARTS] = {"fighter 1", "fighter 2", "fighter 3", "fighter 4",
                                        "fight RNG (0x803F6AC8)", "random generator 1 (0x8035D588)",
                                        "random generator 2 (0x803BE03C)", "frame counter"};
constexpr u32 PART_RANGE[NPARTS][2] = {{0x3F1144, 0x12F8}, {0x3F1144 + 0x12F8, 0x12F8},
                                       {0x3F1144 + 2 * 0x12F8, 0x12F8}, {0x3F1144 + 3 * 0x12F8, 0x12F8},
                                       {0x3F6AC8, 0x88}, {0x35D588, 0x18}, {0x3BE03C, 0x88}, {0x440C24, 4}};
static_assert(NPARTS == NPARTS_FWD);

void Report(const std::string& text)
{
  // at most 12 reports per fight (a crash can repeat on every instruction that follows it)
  if (s_reports++ >= 12)
    return;
  fprintf(stderr, "FFNYRB REPORT: %s\n", text.c_str());
  if (s_report_cb)
    s_report_cb(text);
}

std::string FightSummary(const u8* ram)
{
  // Blazin' byte +0x93E, momentum +0x364, health +0x... not decoded here: state word +0x30
  char b[400];
  auto fl = [&](u32 a) { const u32 w = Rd32(ram, a); float v; memcpy(&v, &w, 4); return v; };
  const u32 F0 = 0x803F1144, F1 = F0 + 0x12F8;
  snprintf(b, sizeof b,
           "P1 state %u blazin %02x momentum %.1f | P2 state %u blazin %02x momentum %.1f | rollbacks %u (max %u, "
           "last %u->%u) | loads %u fixed %u | render draws %u | logic VI reads %u",
           Rd32(ram, F0 + 0x30), ram[(F0 + 0x93E) & 0x01FFFFFF], fl(F0 + 0x364), Rd32(ram, F1 + 0x30),
           ram[(F1 + 0x93E) & 0x01FFFFFF], fl(F1 + 0x364), s_rollbacks, s_max_depth, s_last_rb_from, s_last_rb_to,
           s_loader_calls, s_loader_restores, s_rng_redirected, s_vi_fixed);
  return b;
}

void CompareParts(u32 f, const std::array<u32, NPARTS>& local, const std::array<u32, NPARTS>& remote, const u8* ram)
{
  std::string which;
  for (int i = 0; i < NPARTS; i++)
    if (local[i] != remote[i])
      which += (which.empty() ? "" : ", ") + std::string(PART_NAMES[i]);
  if (which.empty())
    which = "(no single part - hash method mismatch: are both players on the same build?)";
  Report("FFNY DESYNC at frame " + std::to_string(f) + ": " + which + " differ. Mine now: " + FightSummary(ram));
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
  std::array<u32, NPARTS> parts;
  for (int i = 0; i < NPARTS; i++)
    parts[i] = Fnv(ram + PART_RANGE[i][0], PART_RANGE[i][1], 2166136261u);
  s_local_parts[f] = parts;
  if (f >= 4096)
    s_local_parts.erase(f - 4096);
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
      if (s_desyncs <= 3)
        if (auto rp = s_remote_parts.find(f); rp != s_remote_parts.end())
          CompareParts(f, parts, rp->second, ram);
    }
  }
  if (s_transport)
  {
    std::vector<u8> m;
    m.push_back('H');
    Put32(m, f);
    Put32(m, h);
    for (u32 v : parts)
      Put32(m, v);
    s_transport->Send(m);
  }
}

u32 Get32Fwd(const u8* p)
{
  return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | p[3];
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

constexpr u32 SC_REG[4][2] = {{0x3F1144, 4 * 0x12F8}, {0x3F6AC8, 0x88}, {0x35D588, 0x18}, {0x3BE03C, 0x88}};
void StressCopy(const u8* ram, std::vector<u8>& out)
{
  out.clear();
  for (auto& r : SC_REG)
    out.insert(out.end(), ram + r[0], ram + r[0] + r[1]);
}
void StressCheck(const u8* ram, u32 f, bool first)
{
  if (!s_stress)
    return;
  const u32 h = StressHash(ram);
  if (first)
  {
    s_first_hash[f] = h;
    StressCopy(ram, s_first_copy[f]);
    // drop old frames (unordered maps: erase by key, not begin())
    if (f >= 600)
      s_first_hash.erase(f - 600);
    if (f >= 64)
      s_first_copy.erase(f - 64);
    return;
  }
  if (auto c = s_first_copy.find(f); c != s_first_copy.end() && s_first_hash.count(f) && s_first_hash[f] != h &&
                                      s_stress_bad < 20)
  {
    std::vector<u8> now;
    StressCopy(ram, now);
    size_t o = 0;
    int shown = 0;
    for (auto& r : SC_REG)
    {
      for (u32 k = 0; k + 4 <= r[1]; k += 4)
      {
        if (memcmp(&now[o + k], &c->second[o + k], 4) && shown++ < 24)
        {
          const u32 a = 0x80000000u | (r[0] + k);
          const u32 was = Get32Fwd(&c->second[o + k]), is = Get32Fwd(&now[o + k]);
          if (r[0] == 0x3F1144)
            fprintf(stderr, "FFNYRB STRESS DIFF frame %u: fighter %u +0x%03X (%08x) first %08x now %08x\n", f,
                    k / 0x12F8, k % 0x12F8, a, was, is);
          else
            fprintf(stderr, "FFNYRB STRESS DIFF frame %u: %08x first %08x now %08x\n", f, a, was, is);
        }
      }
      o += r[1];
    }
  }
  auto it = s_first_hash.find(f);
  if (it == s_first_hash.end())
    return;
  s_stress_checked++;
  if (it->second != h && s_stress_bad++ < 20)
  {
    fprintf(stderr, "FFNYRB STRESS MISMATCH at frame %u after re-simulation\n", f);
    DumpDraws(s_last_rb_to, f);
  }
}

// rbtag v4 words (sound reference bookkeeping; see rbtag.c)
constexpr u32 W_CONF = 0x8049A40C, W_ACT = 0x8049B500, W_FRAME = 0x8049B504, W_RB = 0x8049B508;
void Wr32(u8* ram, u32 a, u32 v)
{
  u8* p = ram + (a & 0x01FFFFFF);
  p[0] = u8(v >> 24); p[1] = u8(v >> 16); p[2] = u8(v >> 8); p[3] = u8(v);
}

bool ElfConverted(const u8* h)
{
  // e_ehsize (0x28) is 52: little-endian 34 00 in a fresh file, 00 34 after the loader's byte swap
  return h[0] == 0x7F && h[1] == 'E' && h[2] == 'L' && h[3] == 'F' && h[0x28] == 0 && h[0x29] == 0x34;
}
bool ElfFresh(const u8* h)
{
  return h[0] == 0x7F && h[1] == 'E' && h[2] == 'L' && h[3] == 'F' && h[0x28] == 0x34 && h[0x29] == 0;
}

// heap-0 block that holds address a: header address and first header word (0 = not in heap 0)
std::pair<u32, u32> HeapBlockOf(const u8* ram, u32 a)
{
  const u32 H0 = 0x8060C050, base = Rd32(ram, H0 + 8), end = Rd32(ram, H0 + 0xC);
  u32 b = base + 0x90, n = 0;
  while (b >= base && b < end && n++ < 20000)
  {
    const u32 nx = Rd32(ram, b + 8);
    if (a >= b && (nx <= b || a < nx))
      return {b, Rd32(ram, b)};
    if (nx <= b)
      break;
    b = nx;
  }
  return {0, 0};
}

void LoaderDropConfirmed(u32 conf, u32 f, const u8* ram)
{
  // keep a copy while a saved state from before its conversion can still be loaded (states of the last NSLOTS
  // frames exist; a rewind never goes before the newest confirmed frame + 1)
  const u32 oldest = f > NSLOTS ? f - NSLOTS : 0;
  const u32 limit = std::min(conf, oldest);
  s_loader_undo.erase(std::remove_if(s_loader_undo.begin(), s_loader_undo.end(),
                                     [limit](const LoaderUndo& u) { return u.frame < limit; }),
                      s_loader_undo.end());
}

void LoaderFillPost(const u8* ram)
{
  // called at the start of every logic update: conversions of the previous update / render are finished,
  // remember what the converted bytes look like
  for (auto& u : s_loader_undo)
    if (!u.have_post)
    {
      u.post_hash = Fnv(ram + (u.addr & 0x01FFFFFF), u.orig.size(), 2166136261u);
      u.have_post = true;
    }
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
  s_vi_base = Rd32(ram, ADDR_VICOUNT);
  s_vi_fix = std::getenv("FFNY_RB_NOVIFIX") == nullptr;
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
  for (auto& row : s_ack)
    for (u32& a : row)
      a = f - 1;
  for (int r = 0; r < 4; r++)
  {
    s_remote_frame[r] = 0;
    s_remote_adv[r] = 0;
  }
  s_frame_now = f;
  s_adv_n = 0;
  s_sync_next = f + 60;
  s_sync_delays = 0;
  s_input_msgs_in = s_input_msgs_stale = 0;
  if (s_log)
  {
    // lab: the heaps at fight start (index, descriptor, range, base block tagged = not rewound)
    for (u32 i = 0; i < 16; i++)
    {
      const u32 h = Rd32(ram, 0x8040BD70 + 4 * i);
      if ((h & 0xFE000000) != 0x80000000)
        continue;
      const u32 base = Rd32(ram, h + 8), end = Rd32(ram, h + 0xC);
      const bool okb = (base & 0xFE000000) == 0x80000000;
      const u32 hdr = okb ? Rd32(ram, base - 16) : 0;
      fprintf(stderr, "FFNYRB heap %u: desc %08x range %08x-%08x base block header %08x%s\n", i, h, base, end, hdr,
              (hdr >> 16) == 0x424D && (hdr & 0x80) ? " (tagged: NOT rewound)" : "");
    }
  }
  for (auto& s : s_states)
    s.frame = 0xFFFFFFFF;
  s_local_hash.clear();
  s_remote_hash.clear();
  s_local_parts.clear();
  s_remote_parts.clear();
  s_reports = 0;
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
  s_loader_undo.clear();
  s_rollbacks = s_resim_frames = s_max_depth = s_waits = s_frames = 0;
  s_wait_us = s_save_us = s_load_us = 0;
  OSD::AddMessage("FFNY rollback: fight started (input delay " + std::to_string(s_settings.input_delay) +
                      ")",
                  3000);
  fprintf(stderr, "FFNYRB fight start at frame %u (local %x active %x delay %u)\n", f,
          s_settings.local_ports, s_settings.active_ports, s_settings.input_delay);
}

u32 s_max_acq = 0, s_max_rel = 0, s_max_frl = 0;  // lab: highest fill of the rbtag logs (96 entries each)
void PrintStats(u32 f)
{
  fprintf(stderr, "FFNYRB input messages received %u, without anything new %u; time sync: %u one-frame delays, "
          "lead now %d (2x frames)\n", s_input_msgs_in, s_input_msgs_stale, s_sync_delays,
          s_adv_n ? s_adv_hist[(s_adv_n - 1) % s_adv_hist.size()] : 0);
  if (s_ram)
  {
    // lab: free space of the excluded heaps (2 = SndHeap, 3 = SysHeap); a full SysHeap makes rbtag fall back to
    // the rewound main heap
    for (u32 i = 2; i <= 3; i++)
    {
      const u32 h = Rd32(s_ram, 0x8040BD70 + 4 * i);
      if ((h & 0xFE000000) != 0x80000000)
        continue;
      u32 n = Rd32(s_ram, h + 0x20), k = 0, total = 0, largest = 0;
      while (n && n != h + 0x10 && (n & 0xFE000000) == 0x80000000 && k++ < 5000)
      {
        const u32 sz = Rd32(s_ram, n + 4);
        total += sz;
        largest = std::max(largest, sz);
        n = Rd32(s_ram, n + 0x10);
      }
      fprintf(stderr, "FFNYRB heap %u free %u bytes in %u blocks (largest %u)\n", i, total, k, largest);
    }
  }
  fprintf(stderr, "FFNYRB rbtag logs max: acquires %u releases %u frees %u (of 96)\n", s_max_acq, s_max_rel,
          s_max_frl);
  if (s_ram)
    fprintf(stderr, "FFNYRB VI real %u at frame %u (fight start: VI %u frame %u)\n", Rd32(s_ram, ADDR_VICOUNT), f,
            s_vi_base, s_start_frame);
  fprintf(stderr,
          "FFNYRB frame %u: %u frames, %u rollbacks, %u re-simulated (max depth %u), %u waits "
          "(%.1f ms), save %.2f ms/frame, load %.2f ms/rollback, hashes checked %u, desyncs %u, "
          "loader calls %u second conversions fixed %u, render random draws redirected %u, logic VI reads %u\n",
          f, s_frames, s_rollbacks, s_resim_frames, s_max_depth, s_waits, s_wait_us / 1000.0,
          s_frames ? s_save_us / 1000.0 / s_frames : 0.0,
          s_rollbacks ? s_load_us / 1000.0 / s_rollbacks : 0.0, s_hash_checked, s_desyncs, s_loader_calls, s_loader_restores, s_rng_redirected, s_vi_fixed);
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
  s_loader_fix = std::getenv("FFNY_RB_NOLOADFIX") == nullptr;
  s_heapcheck = std::getenv("FFNY_RB_HEAPCHECK") != nullptr;
  s_sync_enabled = std::getenv("FFNY_RB_NOSYNC") == nullptr;
  if (const char* w = std::getenv("FFNY_RB_WATCHRANGE"))
    sscanf(w, "%x:%x", &s_watch_lo, &s_watch_len);
  s_stress_from = std::getenv("FFNY_RB_STRESS_FROM") ? std::atoi(std::getenv("FFNY_RB_STRESS_FROM")) : 0;
  if (const char* r = std::getenv("FFNY_RB_RNGLOG"))
    sscanf(r, "%u:%u", &s_rng_from, &s_rng_to);
  s_lab_mom_every = std::getenv("FFNY_RB_MOM") ? std::atoi(std::getenv("FFNY_RB_MOM")) : 0;  // lab: reproduce the double conversion
}

void SetFightEndCallback(std::function<void(Core::System&, u32 frame)> callback)
{
  s_fight_end_cb = std::move(callback);
}

void Shutdown()
{
  s_fight_end_cb = nullptr;
  s_report_cb = nullptr;
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
  // lab: FFNY_RB_LOSS=percent drops unreliable messages, FFNY_RB_BURST=n makes every drop a run of n
  void SendUnreliable(const std::vector<u8>& msg) override
  {
    if (m_burst_left > 0)
    {
      m_burst_left--;
      m_dropped++;
      return;
    }
    if (m_loss && std::rand() % 100 < m_loss)
    {
      m_burst_left = m_burst - 1;
      m_dropped++;
      return;
    }
    Send(msg);
  }
  int m_loss = 0, m_burst = 1, m_burst_left = 0;
  u32 m_dropped = 0;
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
      {
        s_udp = std::make_unique<UdpTransport>((unsigned short)listen, host, (unsigned short)pport, lag, jit);
        if (const char* l = std::getenv("FFNY_RB_LOSS"))
          s_udp->m_loss = std::atoi(l);
        if (const char* b = std::getenv("FFNY_RB_BURST"))
          s_udp->m_burst = std::max(1, std::atoi(b));
      }
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
  HLE::Patch(system, ADDR_LOADER, "FFNYRollbackLoader");
  if (Rd32(ram, ADDR_VICOUNT_GET) == 0x806D8440 && Rd32(ram, ADDR_VICOUNT_GET + 4) == 0x4E800020)
    HLE::Patch(system, ADDR_VICOUNT_GET, "FFNYRollbackViCount");
  if (Rd32(ram, 0x80274180) == 0x9421FFC0)  // stwu sp,-0x40(sp): the SDK's unhandled-exception handler
    HLE::Patch(system, 0x80274180, "FFNYRollbackCrash");
  s_rng_redirect = Rd32(ram, 0x8049A200) == 0x3C60804A && Rd32(ram, 0x8049A204) == 0x3863A210 &&
                   std::getenv("FFNY_RB_NORNGFIX") == nullptr;
  if (std::getenv("FFNY_RB_THREADHEAP"))
  {
    HLE::Patch(system, 0x80249884, "FFNYRollbackHeapLog");
    HLE::Patch(system, 0x8049A898, "FFNYRollbackHeapLog");
  }
  if (std::getenv("FFNY_RB_SNDCHECK"))
    HLE::Patch(system, 0x801F33E4, "FFNYRollbackSndRel");
  if (const char* pa = std::getenv("FFNY_RB_PROBE"))
  {
    s_probe_addr = std::strtoul(pa, nullptr, 16);
    HLE::Patch(system, s_probe_addr, "FFNYRollbackProbe");
  }
  if (s_rng_redirect || s_rng_to)
  {
    HLE::Patch(system, ADDR_RENDER, "FFNYRollbackRender");
    HLE::Patch(system, GEN1_FN, "FFNYRollbackRng1");
    HLE::Patch(system, GEN2_FN, "FFNYRollbackRng");
  }
  fprintf(stderr, "FFNYRB: render-side random draws %s\n", s_rng_redirect ? "use private generators" : "NOT redirected");
  system.GetJitInterface().ClearSafe();
  s_installed = true;
  fprintf(stderr, "FFNYRB: hooks installed\n");
}

void HookLoader(const Core::CPUThreadGuard& guard)
{
  // HookType::Start: runs before the loader's first instruction, then the loader runs normally
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  u8* ram = system.GetMemory().GetRAM();
  const u32 desc = ppc.gpr[3];
  if ((desc & 0xFE000000) != 0x80000000)
    return;
  const u32 data = Rd32(ram, desc + 0x10), size = Rd32(ram, desc + 0x14), sht = Rd32(ram, desc + 0x18);
  const u32 thread = Rd32(ram, 0x800000E4);
  const u32 frame = Rd32(ram, W_FRAME);
  const u8* h = ram + (data & 0x01FFFFFF);
  const bool le = (data & 0xFE000000) == 0x80000000 && h[5] == 1;
  const u32 shoff = le ? (h[0x20] | h[0x21] << 8 | h[0x22] << 16 | u32(h[0x23]) << 24) : 0;
  const u32 shnum = le ? (h[0x30] | h[0x31] << 8) : 0;
  s_loader_calls++;
  if (s_log)
  {
    const auto blk = HeapBlockOf(ram, data);
    fprintf(stderr,
            "FFNYRB LOADER call: frame %u active %d resim %d thread %08x lr %08x desc %08x data %08x size %08x "
            "shtab %08x ident %02x%02x%02x%02x ei_data %u shoff %x shnum %u block %08x tag %08x\n",
            Rd32(ram, ADDR_COUNTER), s_active, s_resim_target != 0, thread, ppc.spr[SPR_LR], desc, data, size, sht,
            h[0], h[1], h[2], h[3], h[5], shoff, shnum, blk.first, blk.second);
  }
  if (!s_active || !s_loader_fix || (data & 0xFE000000) != 0x80000000 || size < 0x34 || size > 0x01000000 ||
      sht != 0)
    return;
  u8* d = ram + (data & 0x01FFFFFF);
  // newest copy of this address and size whose converted bytes are exactly what is there now
  auto it = s_loader_undo.end();
  const u32 now_hash = Fnv(ram + (data & 0x01FFFFFF), size, 2166136261u);
  for (auto i = s_loader_undo.begin(); i != s_loader_undo.end(); ++i)
    if (i->addr == data && i->orig.size() == size && (ElfFresh(ram + (data & 0x01FFFFFF)) || (i->have_post && i->post_hash == now_hash)))
      it = i;
  if (ElfFresh(d))
  {
    if (it != s_loader_undo.end())
      s_loader_undo.erase(it);
    s_loader_undo.push_back(LoaderUndo{frame, data, std::vector<u8>(d, d + size)});
  }
  else if (ElfConverted(d) && it == s_loader_undo.end())
  {
    fprintf(stderr, "FFNYRB LOADER second conversion of %08x (%u bytes) at frame %u: NO matching copy\n", data, size,
            Rd32(ram, ADDR_COUNTER));
  }
  else if (ElfConverted(d) && it != s_loader_undo.end())
  {
    memcpy(d, it->orig.data(), size);
    s_loader_restores++;
    fprintf(stderr, "FFNYRB LOADER second conversion of %08x (%u bytes, first in frame %u) at frame %u: original bytes restored\n",
            data, size, it->frame, Rd32(ram, ADDR_COUNTER));
  }
}

static void RecordDraw(Core::System& system, char gen)
{
  if (!s_stress || !s_active)
    return;
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  auto ok = [](u32 a) { return (a & 0xFE000000) == 0x80000000; };
  const u32 sp = ppc.gpr[1];
  const u32 c1 = ok(sp) ? Rd32(ram, sp) : 0, b1 = ok(c1) ? Rd32(ram, c1 + 4) : 0;
  const u32 c2 = ok(c1) ? Rd32(ram, c1) : 0, b2 = ok(c2) ? Rd32(ram, c2 + 4) : 0;
  const u32 c3 = ok(c2) ? Rd32(ram, c2) : 0, b3 = ok(c3) ? Rd32(ram, c3 + 4) : 0;
  auto& m = s_draw_first ? s_draws_first : s_draws_resim;
  m[s_draw_frame].push_back(Draw{gen, s_phase, ppc.spr[SPR_LR], b1, b2, b3, ppc.gpr[29], ppc.gpr[31]});
}

void DumpDraws(u32 from, u32 to)
{
  if (s_draw_dumps++ >= 3)
    return;
  for (u32 f = from; f <= to && s_watch_len; f++)
  {
    auto a = s_watch_first.find(f), b = s_watch_resim.find(f);
    if (a == s_watch_first.end() || b == s_watch_resim.end())
      continue;
    int shown = 0;
    for (u32 k = 0; k + 4 <= s_watch_len; k += 4)
      if (memcmp(&a->second[k], &b->second[k], 4) && shown++ < 12)
        fprintf(stderr, "WATCH frame %u (start of update): %08x first %08x resim %08x\n", f, s_watch_lo + k,
                Get32Fwd(&a->second[k]), Get32Fwd(&b->second[k]));
    if (!shown)
      fprintf(stderr, "WATCH frame %u: range identical at the start of the update\n", f);
  }
  for (u32 f = from; f <= to; f++)
  {
    const auto& a = s_draws_first[f];
    const auto& b = s_draws_resim[f];
    fprintf(stderr, "DRAWS frame %u: first visit %zu, re-simulation %zu%s\n", f, a.size(), b.size(),
            a.size() == b.size() ? "" : "  <-- different count");
    for (size_t i = 0; i < std::max(a.size(), b.size()); i++)
    {
      auto pr = [](const char* t, const Draw& d) {
        fprintf(stderr, "  %s gen%c %c lr %08x bt %08x %08x %08x r29 %08x r31 %08x\n", t, d.gen, d.phase, d.lr, d.b1,
                d.b2, d.b3, d.r29, d.r31);
      };
      const bool same = i < a.size() && i < b.size() && a[i].lr == b[i].lr && a[i].b1 == b[i].b1 &&
                        a[i].b2 == b[i].b2 && a[i].gen == b[i].gen;
      if (same)
        continue;
      if (i < a.size())
        pr("first", a[i]);
      if (i < b.size())
        pr("resim", b[i]);
    }
  }
}

void HookProbe(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  static std::vector<u32> seen;
  if (!s_active || Rd32(ram, ADDR_COUNTER) < s_start_frame + 20)
    return;
  if (std::find(seen.begin(), seen.end(), ppc.gpr[3]) != seen.end() || seen.size() > 40)
    return;
  seen.push_back(ppc.gpr[3]);
  const auto blk = HeapBlockOf(ram, ppc.gpr[3]);
  {
    // is the object inside the newest saved state's ranges?
    const u32 a = ppc.gpr[3] & 0x01FFFFFF;
    int covered = -1;
    for (const auto& st : s_states)
      if (st.frame != 0xFFFFFFFF)
      {
        covered = 0;
        for (auto [lo, hi] : st.ranges)
          if (a >= lo && a + 0x20 <= hi)
            covered = 1;
        if (!covered)
          for (auto [lo, hi] : st.ranges)
            if (a < hi && a + 0x40 > lo)
              fprintf(stderr, "PROBE   partial range %08x-%08x\n", lo | 0x80000000u, hi | 0x80000000u);
        break;
      }
    fprintf(stderr, "PROBE   %08x in saved state: %s\n", ppc.gpr[3],
            covered < 0 ? "no state yet" : covered ? "yes" : "NO");
  }
  fprintf(stderr, "PROBE %08x r3 %08x lr %08x block %08x tag %08x size %08x magic %08x frame %u\n", ppc.pc,
          ppc.gpr[3], ppc.spr[SPR_LR], blk.first, blk.second, blk.first ? Rd32(ram, blk.first + 4) : 0,
          blk.first ? Rd32(ram, blk.first + 16) : 0, Rd32(ram, ADDR_COUNTER));
}

void HookViCount(const Core::CPUThreadGuard& guard)
{
  // replaces lwz r3,-0x7BC0(r13); blr
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  if (s_active && s_vi_fix)
  {
    // every reader (logic and render) gets the frame-derived count during the whole fight, so nothing compares a
    // real count with a derived one; rate 1 per logic frame like the real 60 Hz retrace
    const u32 f = Rd32(ram, ADDR_COUNTER);
    ppc.gpr[3] = s_vi_base + (f - s_start_frame);
    s_vi_fixed++;
  }
  else
  {
    ppc.gpr[3] = Rd32(ram, ADDR_VICOUNT);
  }
  ppc.npc = ppc.spr[SPR_LR];
}

// lab: check the sound object's listener list before the inner release 0x801F33E4 walks it
// (node: +0 next, +8 callback, +0xC argument; list head at object +0xC)
void HookSndRelease(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  const u32 obj = ppc.gpr[3];
  static u32 bad_reports = 0;
  auto ok = [](u32 a) { return (a & 0xFE000000) == 0x80000000 && (a & 0x01FFFFFF) < 0x01800000; };
  auto code = [](u32 a) { return a >= 0x80003100 && a < 0x80400000; };
  if (!ok(obj))
    return;
  u32 n = Rd32(ram, obj + 0xC), k = 0;
  while (n && k++ < 64)
  {
    const u32 cb = ok(n) ? Rd32(ram, n + 8) : 0;
    if (!ok(n) || !code(cb))
    {
      if (bad_reports++ < 10)
      {
        const u32 sp = ppc.gpr[1];
        const u32 c1 = ok(sp) ? Rd32(ram, sp) : 0, b1 = ok(c1) ? Rd32(ram, c1 + 4) : 0;
        const u32 c2 = ok(c1) ? Rd32(ram, c1) : 0, b2 = ok(c2) ? Rd32(ram, c2 + 4) : 0;
        const u32 c3 = ok(c2) ? Rd32(ram, c2) : 0, b3 = ok(c3) ? Rd32(ram, c3 + 4) : 0;
        const auto ob = HeapBlockOf(ram, obj), nb = HeapBlockOf(ram, n);
        fprintf(stderr,
                "SNDREL BAD frame %u%s phase %c thread %08x obj %08x (refs %u, block %08x tag %08x) node %u = %08x "
                "(block %08x tag %08x) callback %08x | bt %08x %08x %08x %08x | rbtag rb %08x conf %08x frame %08x\n",
                Rd32(ram, ADDR_COUNTER), s_resim_target ? " resim" : "", s_phase, Rd32(ram, 0x800000E4), obj,
                Rd32(ram, obj + 4), ob.first, ob.second, k, n, nb.first, nb.second, cb, ppc.spr[SPR_LR], b1, b2,
                b3, Rd32(ram, W_RB), Rd32(ram, W_CONF), Rd32(ram, W_FRAME));
      }
      return;
    }
    n = Rd32(ram, n);
  }
}

// lab (FFNY_RB_THREADHEAP): allocations (core 0x80249884: r3 name, r4 size, r7 flags) and frees (rbtag's
// rb_free 0x8049A898: r3 pointer) made during a fight by any thread other than the game's main thread
void HookHeapLog(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  const u32 th = Rd32(ram, 0x800000E4);
  if (!s_active || th == 0x80432D10)
    return;
  static u32 n = 0;
  if (n++ > 3000)
    return;
  auto ok = [](u32 a) { return (a & 0xFE000000) == 0x80000000 && (a & 0x01FFFFFF) < 0x01800000; };
  const u32 f = Rd32(ram, ADDR_COUNTER);
  if (ppc.pc == 0x80249884)
  {
    char name[40] = {};
    if (ok(ppc.gpr[3]))
      for (int i = 0; i < 39; i++)
      {
        const u8 c = ram[(ppc.gpr[3] + i) & 0x01FFFFFF];
        if (!c)
          break;
        name[i] = (c >= 32 && c < 127) ? char(c) : '?';
      }
    fprintf(stderr, "THREADHEAP alloc frame %u%s phase %c thread %08x name '%s' size %x flags %x lr %08x\n", f,
            s_resim_target ? " resim" : "", s_phase, th, name, ppc.gpr[4], ppc.gpr[7], ppc.spr[SPR_LR]);
  }
  else
  {
    const u32 p = ppc.gpr[3];
    const u32 fl = ok(p) ? Rd32(ram, p - 16) : 0;
    fprintf(stderr, "THREADHEAP free frame %u%s phase %c thread %08x ptr %08x header %08x lr %08x\n", f,
            s_resim_target ? " resim" : "", s_phase, th, p, fl, ppc.spr[SPR_LR]);
  }
}

void OnDiscDma(Core::System& system, u32 address, u32 length)
{
  if (!s_active || !s_log)
    return;
  const u8* ram = system.GetMemory().GetRAM();
  static u32 shown = 0, total = 0, bad = 0;
  total++;
  // does the target overlap a free block of any heap (free list: head [h+0x20], node +4 size, +0x10 next)?
  const u32 lo = address | 0x80000000, hi = lo + length;
  for (u32 i = 0; i < 16; i++)
  {
    const u32 h = Rd32(ram, 0x8040BD70 + 4 * i);
    if ((h & 0xFE000000) != 0x80000000)
      continue;
    u32 n = Rd32(ram, h + 0x20), k = 0;
    while (n && n != h + 0x10 && (n & 0xFE000000) == 0x80000000 && k++ < 20000)
    {
      const u32 sz = Rd32(ram, n + 4);
      if (lo < n + sz && n < hi)
      {
        bad++;
        if (shown++ < 20)
          fprintf(stderr,
                  "DISCDMA INTO FREE MEMORY frame %u%s: %08x-%08x (%u bytes) overlaps free block %08x-%08x of heap %u "
                  "(free node %s) [%u of %u disc reads so far]\n",
                  Rd32(ram, ADDR_COUNTER), s_resim_target ? " resim" : "", lo, hi, length, n, n + sz, i,
                  (lo <= n + 0x14 && n + 0x10 < hi) ? "OVERWRITTEN" : "untouched", bad, total);
        return;
      }
      n = Rd32(ram, n + 0x10);
    }
  }
}

void HookRender(const Core::CPUThreadGuard& guard)
{
  s_phase = 'R';
}

void HookRng1(const Core::CPUThreadGuard& guard)
{
  auto& ppc = guard.GetSystem().GetPPCState();
  if (s_active && s_rng_redirect && s_phase == 'R' && ppc.gpr[3] == GEN1_SHARED)
  {
    ppc.gpr[3] = GEN1_PRIVATE;
    s_rng_redirected++;
  }
  if (ppc.gpr[3] == GEN1_SHARED)
    RecordDraw(guard.GetSystem(), '1');
}

void HookRng(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  if (s_active && s_rng_redirect && s_phase == 'R' && ppc.gpr[3] == GEN2_SHARED)
  {
    ppc.gpr[3] = GEN2_PRIVATE;
    s_rng_redirected++;
    if (!s_rng_to)
      return;
  }
  if (ppc.gpr[3] == GEN2_SHARED)
    RecordDraw(system, '2');
  if (ppc.gpr[3] != 0x803BE03C && !(s_phase == 'R' && ppc.gpr[3] == GEN2_PRIVATE))
    return;
  if (!s_rng_to)
    return;
  const u32 f = Rd32(ram, ADDR_COUNTER);
  if (f < s_rng_from || f > s_rng_to)
    return;
  auto ok = [](u32 a) { return (a & 0xFE000000) == 0x80000000; };
  const u32 sp = ppc.gpr[1];
  const u32 c1 = ok(sp) ? Rd32(ram, sp) : 0;
  const u32 b1 = ok(c1) ? Rd32(ram, c1 + 4) : 0;
  const u32 c2 = ok(c1) ? Rd32(ram, c1) : 0;
  const u32 b2 = ok(c2) ? Rd32(ram, c2 + 4) : 0;
  const u32 c3 = ok(c2) ? Rd32(ram, c2) : 0;
  const u32 b3 = ok(c3) ? Rd32(ram, c3 + 4) : 0;
  fprintf(stderr, "RNG2 f %u %c%s thread %08x lr %08x bt %08x %08x %08x state %08x\n", f, s_phase,
          s_resim_target ? " resim" : "", Rd32(ram, 0x800000E4), ppc.spr[SPR_LR], b1, b2, b3, Rd32(ram, 0x803BE0BC));
}

void HookTick(const Core::CPUThreadGuard& guard)
{
  s_phase = 'T';
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

// lab: walk every heap's size-sorted free list (descriptor table 0x8040BD70; head [h+0x20], node +0x10 next,
// +4 size, 'BF' magic) and report the first broken link per heap
void HeapCheck(const u8* ram, u32 f, const char* where)
{
  static u32 reported = 0;  // bit per heap
  auto ok = [](u32 a) { return (a & 0xFE000000) == 0x80000000 && (a & 0x01FFFFFF) < 0x01800000; };
  for (u32 i = 0; i < 32; i++)
  {
    const u32 h = Rd32(ram, 0x8040BD70 + 4 * i);
    if (!ok(h) || (reported >> i & 1))
      continue;
    const u32 base = Rd32(ram, h + 8), end = Rd32(ram, h + 0xC);
    u32 prev = h + 0x10, n = Rd32(ram, h + 0x20), k = 0;
    while (n && n != h + 0x10 && k++ < 20000)  // the list ends at the 'BS' sentinel inside the descriptor
    {
      if (!ok(n) || n < base || n >= end || (Rd32(ram, n) >> 16) != 0x4246)
      {
        reported |= 1u << i;
        fprintf(stderr,
                "HEAPCHECK %s frame %u%s: heap %u (desc %08x, %08x-%08x) free node %u = %08x (word %08x) after %08x "
                "(header %08x %08x %08x %08x %08x %08x)\n",
                where, f, s_resim_target ? " resim" : "", i, h, base, end, k, n, ok(n) ? Rd32(ram, n) : 0, prev,
                Rd32(ram, prev), Rd32(ram, prev + 4), Rd32(ram, prev + 8), Rd32(ram, prev + 12), Rd32(ram, prev + 16),
                Rd32(ram, prev + 20));
        break;
      }
      prev = n;
      n = Rd32(ram, n + 0x10);
    }
  }
}

// lab stress: from here on, draws (and the watched range) belong to the update of frame f
void DrawFrameBegin(const u8* ram, u32 f, bool first)
{
  if (s_heapcheck)
    HeapCheck(ram, f, first ? "before update" : "before re-simulated update");
  if (!s_stress)
    return;
  s_draw_frame = f;
  s_draw_first = first;
  (first ? s_draws_first : s_draws_resim)[f].clear();
  if (f >= 64)
  {
    s_draws_first.erase(f - 64);
    s_draws_resim.erase(f - 64);
  }
  if (s_watch_len)
  {
    const u8* w = ram + (s_watch_lo & 0x01FFFFFF);
    (first ? s_watch_first : s_watch_resim)[f].assign(w, w + s_watch_len);
    if (f >= 64)
    {
      s_watch_first.erase(f - 64);
      s_watch_resim.erase(f - 64);
    }
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
  s_phase = 'U';
  s_ram = ram;
  LoaderFillPost(ram);
  if (s_active)
  {
    s_max_acq = std::max(s_max_acq, Rd32(ram, 0x8049B510));
    s_max_rel = std::max(s_max_rel, Rd32(ram, 0x8049B514));
    s_max_frl = std::max(s_max_frl, Rd32(ram, 0x8049B518));
  }
  if (s_active && s_log)
  {
    // lab: the time step the update gets (f31 -> f1 at 0x80004D20)
    static u32 shown = 0;
    const double dt = ppc.ps[31].PS0AsDouble();
    if ((dt < 0.0166 || dt > 0.0168) && shown++ < 40)
      fprintf(stderr, "FFNYRB DT frame %u: f31 = %.6f%s\n", Rd32(ram, ADDR_COUNTER), dt, s_resim_target ? " (resim)" : "");
  }
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
    DrawFrameBegin(ram, f, false);
    Inject(ram, tick_obj, f);
    SaveState(system, f);
    LoaderDropConfirmed(std::min(MinConfirmed(), f - 1), f, ram);
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
    s_frame_now = f;
    // time synchronisation: if we run ahead of the other side (we would do all the rolling back and hit the
    // rollback limit, i.e. freeze), let this machine run one frame later, at most every 20 frames
    {
      bool known = false;
      for (int r = 0; r < 4; r++)
        if (IsRemote(r) && s_remote_frame[r] >= s_start_frame)
        {
          s_adv_hist[s_adv_n++ % s_adv_hist.size()] = LocalAdvantage() - s_remote_adv[r];
          known = true;
          break;
        }
      if (known && f >= s_sync_next && s_adv_n >= s_adv_hist.size() && s_sync_enabled)
      {
        s32 sum = 0;
        for (s32 d : s_adv_hist)
          sum += d;
        // sum / n = twice our lead in frames
        if (sum >= 2 * s32(s_adv_hist.size()) * 1)
        {
          system.GetCoreTiming().AddThrottleDelay(std::chrono::microseconds(16683));
          s_sync_delays++;
          s_sync_next = f + 20;
        }
      }
    }
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
    if (s_stress && f > s_start_frame + s_stress && f >= s_stress_from && fight)
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
    // a lost message must not stall us: resend what the other side lacks every 8 ms while waiting
    static auto last_resend = std::chrono::steady_clock::now();
    if (std::chrono::steady_clock::now() - last_resend > std::chrono::milliseconds(8))
    {
      last_resend = std::chrono::steady_clock::now();
      ResendInputs(false);
    }
  }
  if (waited)
  {
    // we were slowed down by the wait: keep that (no catch-up sprint that would put us ahead again)
    if (s_sync_enabled)
      system.GetCoreTiming().AddThrottleDelay(std::chrono::steady_clock::now() - wait_start);
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
      DrawFrameBegin(ram, s, false);
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
    // fight over and every input confirmed: back to normal netplay. The other side may still lack our last
    // frames (lost packets) and waits for them: send them once more, reliably.
    ResendInputs(true);
    s_active = false;
    s_loader_undo.clear();
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

  DrawFrameBegin(ram, f, true);  // first run of update f (also right after a re-simulation caught up)
  Inject(ram, tick_obj, f);
  SaveState(system, f);
  LoaderDropConfirmed(std::min(MinConfirmed(), f - 1), f, ram);
  Wr32(ram, W_CONF, std::min(MinConfirmed(), f - 1));
  Wr32(ram, W_FRAME, f);
  if (f > s_hashed_upto && MinConfirmed() >= f)
  {
    HashFrame(ram, f);
    s_hashed_upto = f;
  }
}

void SetReportCallback(std::function<void(const std::string&)> callback)
{
  s_report_cb = std::move(callback);
}

void OnInvalidAccess(Core::System& system, u32 address, u32 pc, bool write)
{
  if (s_log)
  {
    // lab: first invalid access -> state of every heap's free list and the thread / call chain, once
    static bool dumped = false;
    if (!dumped)
    {
      dumped = true;
      const u8* r = system.GetMemory().GetRAM();
      auto& ppc = system.GetPPCState();
      fprintf(stderr, "CRASHDUMP thread %08x (main %08x) r3 %08x r4 %08x r5 %08x r29 %08x r30 %08x r31 %08x\n",
              Rd32(r, 0x800000E4), 0x80432D10u, ppc.gpr[3], ppc.gpr[4], ppc.gpr[5], ppc.gpr[29], ppc.gpr[30],
              ppc.gpr[31]);
      u32 sp = ppc.gpr[1];
      for (int k = 0; k < 10 && (sp & 0xFE000000) == 0x80000000; k++)
      {
        const u32 up = Rd32(r, sp);
        if ((up & 0xFE000000) != 0x80000000)
          break;
        fprintf(stderr, "CRASHDUMP   caller %08x\n", Rd32(r, up + 4));
        sp = up;
      }
      HeapCheck(r, Rd32(r, ADDR_COUNTER), "at the crash");
    }
  }

  if (!s_settings.enabled)
    return;
  const u8* ram = system.GetMemory().GetRAM();
  auto& ppc = system.GetPPCState();
  char b[300];
  snprintf(b, sizeof b, "FFNY CRASH (Dolphin): invalid %s %08x at PC %08x LR %08x, frame %u, fight %s%s. ",
           write ? "write to" : "read from", address, pc, ppc.spr[SPR_LR], ram ? Rd32(ram, ADDR_COUNTER) : 0,
           s_active ? "running" : "not running", s_resim_target ? ", during a re-simulation" : "");
  Report(std::string(b) + (ram ? FightSummary(ram) : std::string()));
}

void HookUnhandledException(const Core::CPUThreadGuard& guard)
{
  // the GameCube SDK's crash handler (prints the context, then halts): r3 = exception number, r4 = OSContext,
  // r5 = DSISR, r6 = DAR
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  const u32 ctx = ppc.gpr[4];
  const bool ok = (ctx & 0xFE000000) == 0x80000000;
  char b[300];
  snprintf(b, sizeof b,
           "FFNY CRASH (game exception %u): PC %08x LR %08x DSISR %08x DAR %08x, frame %u, fight %s%s. ",
           ppc.gpr[3] & 0xFF, ok ? Rd32(ram, ctx + 0x198) : 0, ok ? Rd32(ram, ctx + 0x84) : 0, ppc.gpr[5],
           ppc.gpr[6], Rd32(ram, ADDR_COUNTER), s_active ? "running" : "not running",
           s_resim_target ? ", during a re-simulation" : "");
  Report(std::string(b) + FightSummary(ram));
}
}  // namespace FFNYRollback
