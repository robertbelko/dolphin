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
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <optional>
#include <unordered_map>

#include <SFML/Network/IpAddress.hpp>
#include <SFML/Network/UdpSocket.hpp>

#include "Common/FileUtil.h"
#include "Common/IOFile.h"
#include "Common/Logging/Log.h"
#include "InputCommon/GCPadStatus.h"
#include <atomic>
#include <ctime>
#include <random>
#include "Core/Core.h"
#include "Core/HLE/HLE.h"
#include "Core/HW/CPU.h"
#include "Core/HW/DVD/DVDThread.h"
#include "Core/HW/Memmap.h"
#include "Core/State.h"
#include "Common/Event.h"
#include "Common/Hash.h"
#include "Core/PowerPC/Gekko.h"
#include "Core/PowerPC/JitInterface.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/CoreTiming.h"
#include "Core/PowerPC/BreakPoints.h"
#include "Core/System.h"
#include "VideoCommon/OnScreenDisplay.h"

namespace FFNYRollback
{
// lab: decided at process start (before the JIT compiles anything); netplay would set it before the game boots
// FFNY_RB_MENUS=1 (27 October): rollback stays on after the first fight - KO scenes, results, rematch, menus,
// loading and the next fights, with no resync in between (the lab's FFNY_RB_ALWAYS=frame does the same from a frame)
bool s_menus = std::getenv("FFNY_RB_MENUS") != nullptr;
bool s_menus_on = false;
bool s_real_fight_now = false;  // set at every update start: the fight itself runs  // menus mode: rollback started (first fight) and stays on
// menus mode (27 October, Robert's first test: ~70 % speed in the KO scenes, results, pause and menus): outside the
// running fight every frame waits for all inputs (lockstep), and lockstep runs at most (input delay of both players)
// frames per round trip - with the rollback delay of 2 and 100 ms ping that is ~4 of the 6 frames needed. There the
// local inputs now get the netplay pad buffer as their delay (like Dolphin's own netplay menus before the first
// fight); the running fight keeps the rollback delay. Set by netplay (SetMenuDelay); lab: FFNY_RB_MENUDELAY=N.
std::atomic<u32> s_menu_delay{0};
constexpr u32 MAX_MENU_DELAY = 40;  // frames (the input ring holds 128)
u32 s_cur_delay = 0;                // delay used for the newest captured frame (for the log)
// menus mode: fights are marked at confirmed frames (HashFrame), for the per-fight summary, counters and checks
bool s_fight_raw_prev = false;
u32 s_fight_no = 0, s_fight_from = 0;
// watchdog (27 October: one machine stopped for good ~6 s into the next round, and closing Dolphin then froze):
// a thread that notices when the game has not reached its next update for a while and says where the CPU thread is
std::atomic<u64> s_pre_count{0};
std::atomic<const char*> s_wait_where{nullptr};
std::atomic<bool> s_resync_waiting{false};
std::atomic<s64> s_resync_since_ms{0};
bool g_virtual_tb = (std::getenv("FFNY_RB_ALWAYS") != nullptr || std::getenv("FFNY_RB_MENUS") != nullptr) &&
                    std::getenv("FFNY_RB_NOVTB") == nullptr;
u64 s_vtb_anchor = 0;
u32 s_vtb_frame0 = 0, s_vtb_steps = 0;
bool s_vtb_mode = false;
// match set-up seed reads (see HookSeedTbN)
u32 s_seed_reads = 0;  // seed reads since rollback started (reset in StartFight)
u32 s_apt_side = 0;    // Flash player clock reads by the loading-screen thread since rollback started (HookAptTbN)
constexpr u32 SEED_TB_SITES[] = {0x80095F68, 0x800982C8, 0x80096364, 0x80096610, 0x8003253C};
constexpr u32 APT_TB_SITES[] = {0x801A3F24, 0x801A3F80};  // menus Flash player clock (see HookAptTbN)
// one loading-screen tick = 6.5 frames: its loop (sleep + wait for the retrace) takes 6-7 frames of real time per
// pass (lab APTTB: 4.05M / 4.73M time base ticks); with 1 frame per pass the menus' transition movies, which also
// run on this clock, ended ~200 frames later after every loading screen (lab kor1 vs kor2)
constexpr u64 APT_SIDE_STEP = 4387500;

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
// fight state flags (fight state 0x803F0FF8 + 0x12C). Bit 0x8 = the fight is decided: set by every match-end path
// (match end 0x80203E38, environmental KO 0x80238720, 0x80206BF4, 0x80228FAC, 0x801FBD54, the end of the slow-motion
// phase 0x801FA0B8) and kept until the next fight's init (0x801F99E0 writes 1). Bits 0x20 / 0x40 (without 0x80) stop
// the fight loop's state machine (0x801FA0D8), so 0x803F1000 can stay 1 after an environmental KO.
constexpr u32 ADDR_FIGHT_FLAGS = 0x803F1124;
constexpr u32 FIGHT_DECIDED = 0x8;
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
  std::vector<std::pair<u32, u32>> vranges;  // fake VMEM (0x7E000000..) offsets: heaps that live there
  std::vector<u8> vdata;
  u32 gpr[32];
  PowerPC::PairedSingle ps[32];
  PowerPC::ConditionRegister cr;
  u32 fpscr, lr, ctr, gqr[8];
  u8 ca, so;
};

Settings s_settings;
// set by the netplay thread when the host ends the fight for everyone (the machines' fights did not end at the same
// frame, e.g. post-KO screens that went different ways); picked up by the CPU thread in HookPreUpdate
std::atomic<bool> s_force_end{false};
std::string s_force_reason;  // written before s_force_end is set
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
std::array<u32, 32> s_count_addr{};  // lab: FFNY_RB_COUNT
std::array<u32, 4> s_trace_addr{};  // lab: FFNY_RB_TRACE
// held-back sound calls (see HookSndPost)
constexpr u32 SND_POST = 0x801D3074, SND_SERVICE = 0x801D36B8;
// 0x80011108(ev) = the music module's MPF event send (from 0x80031B54, after its checks) -> 0x801D70EC
constexpr u32 SND_MUSIC = 0x80011108;
bool s_music_hold = true;  // FFNY_RB_NOMUSICHOLD=1 -> music events not held
struct SndCall
{
  u32 frame, fn;
  u32 r[8];  // r3..r10
};
std::deque<SndCall> s_snd_q;
std::deque<std::pair<SndCall, bool>> s_music_sent;  // music cues sent by first runs (+ matched in the current re-simulation)
u32 s_music_deduped = 0, s_music_resent = 0, s_music_refused = 0;
bool s_music_hold_old = std::getenv("FFNY_RB_MUSICHOLD") != nullptr;  // lab: the 153a233 behaviour (held until confirmed)
bool s_snd_defer = false;     // hooks installed
bool s_snd_trace = false, s_snd_hold_on = true;  // lab: FFNY_RB_SNDTRACE; FFNY_RB_NOSNDHOLD
const char* SndModeName()
{
  return !s_snd_hold_on ? "off" : s_music_hold_old ? "on (speech + music held until confirmed)" :
                                  "on (speech held until confirmed; music cues sent at once, not repeated by re-simulations)";
}
bool s_snd_sending = false;   // a held call is being made right now (the hooks let it through)
bool s_snd_end_after = false; // after the sends: end the fight (EndFight was due when they started)
u32 s_snd_end_frame = 0;
std::string s_snd_end_reason;  // EndFight's 'forced' text for the end after the sends
SavedState s_snd_regs;        // registers to continue with after the sends (only the register part is used)
u32 s_snd_regs_npc = 0;
u32 s_snd_held = 0, s_snd_sent = 0, s_snd_dropped = 0;  // stats of the current fight

u32 s_start_frame = 0;
u32 s_resim_target = 0;  // != 0 while re-simulating up to (excluding) this frame
bool s_stall_now = false;  // this PRE is a logic counter stall (same frame as the last first run)
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
u32 s_desyncs_fight = 0;  // desyncs since this fight started (the chat gets the first 3 of every fight)
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
u32 s_rng_effect = 0;  // of those: effect manager draws (any phase)
u32 s_rng2_calls[4][3] = {};  // lab: [phase U/T/R/other][r3 shared/private/other]
// ---- Random draws by the game's other threads (29 October) ----
// Robert's third match in menus mode (Hunt's Point Scrapyard, after 4th Heaven and Club Murder): generator 1 differed
// one frame after the match set-up (13743 -> 13744) and the heap layouts differed after it. The match set-up runs on
// the fight loader thread 0x806D8AE0 (0x80095F8C) and builds the crowd there (0x8008BDF4 -> 0x8008C010 ->
// 0x8008CD70 / 0x8009037C / 0x800905BC / 0x800953BC / 0x80095548: ~40 draw sites of the shared generator 1), while
// the main thread's logic draws from the same generator (e.g. the per-update countdown 0x800E0994). Which main-thread
// draw comes between which crowd draws depends on each machine's emulated timing, and after a rollback fight that is
// no longer the same on the two machines: the crowd and the main thread got different numbers. While rollback runs,
// draws from the two shared generators by any thread other than the main thread now come from side states kept here
// (the game's own algorithms, seeded from the shared states at the fight start, where they are equal on all
// machines); the order of the draws on that thread is the game's own and the same on every machine.
// FFNY_RB_NOSIDERNG=1 turns this off.
constexpr u32 MAIN_THREAD = 0x80432D10;
u32 s_side1[6] = {};   // generator 1 layout (0x8001FC88)
u32 s_side2[34] = {};  // generator 2 layout (0x800B7DAC): 32 floats (bits), +0x80 seed, +0x84 increment
const bool s_side_off = std::getenv("FFNY_RB_NOSIDERNG") != nullptr;
u32 s_side_draws[2] = {};          // since the last report
std::map<u32, u32> s_side_lr[2];  // caller (lr) -> draws since the last report
u32 s_match_updates = 0;           // match update 0x80096C04 calls since the last menus status line
// Song picks (29 October, Robert: after the menus-mode fights the main menu played a different song on each machine;
// dumps at 25737: A "music/hypno", B "music/where" in the song stream object at 0x80B16850). The music module's
// playlist picks 0x8002C294 (in 0x8002BC18, from the front end / loading screen 0x8008428C, 0x80084B90, 0x80089598)
// and 0x8002CB88 (in 0x8002C6BC, from 0x80041F18) draw "random index mod count" from the audio module's private
// generator (rbaudrng stub 0x8049A200 -> state 0x8049A210), which the voices, render and effect draws advance by a
// different amount on each machine. While rollback runs, these two calls get their own generator instead (seeded
// in StartFight, where the machines are identical): the same pick sequence on all machines. On the main thread in a
// running fight (re-simulations) a pick's value comes from (logic frame, n-th pick in the frame) instead.
// FFNY_RB_NOSONGPICK=1 turns it off.
constexpr u32 SONG_PICK_SITES[] = {0x8002C294, 0x8002CB88};
u32 s_song[6] = {};
u32 s_song_picks = 0, s_song_logged = 0;
u32 s_song_frame = 0xFFFFFFFF, s_song_in_frame = 0;
// 0x8001FC88 in C++ (checked instruction by instruction against the DOL's code on 20,000 random states)
static u32 Gen1Step(u32* s)
{
  u32 r0 = s[5], r9 = s[4];
  u32 r3 = r0 + r9;
  const u32 c = (r3 < r0 || r3 < r9) ? 1 : 0;
  r9 = s[3];
  const u32 r10 = s[2];
  r0 = r3 + r9;
  s[4] = r3;
  r3 = r0 + c;
  const u32 r8 = s[1];
  r0 = r3 + r10;
  s[3] = r3;
  r3 = r0 + (r3 < r9 ? 1 : 0);
  r0 = r3 + r8;
  s[2] = r3;
  r3 = r0 + (r3 < r10 ? 1 : 0);
  const u32 r11 = r3 + s[0];
  s[1] = r3;
  const u32 n = s[5] + 1;
  r3 = r11 + (r3 < r8 ? 1 : 0);
  s[0] = r3;
  s[5] = n;
  if (n != 0)
    return r3;
  for (int k = 4; k >= 1; k--)
    if (++s[k] != 0)
      return r3;
  s[0] = r3 + 1;
  return s[0];
}
// 0x800B7DAC in C++: two LCG steps, returns the table entry picked by the first, refills it from the second
static float Gen2Step(u32* s)
{
  constexpr u32 K = 0x5D588B65;
  const u32 r10 = s[32] * K + s[33];
  const u32 r0 = r10 * K + s[33];
  s[32] = r0;
  const u32 i = (((r10 << 2) | (r10 >> 30)) & 0x7C) / 4;
  float out, v;
  memcpy(&out, &s[i], 4);
  v = static_cast<float>(static_cast<double>(r0)) * 2.3283064365386963e-10f;  // frsp, fmuls by 2^-32 (0x802BA498)
  memcpy(&s[i], &v, 4);
  return out;
}
static std::string SideDrawText(bool reset)
{
  std::string t = fmt::format("generator 1 {}, generator 2 {}, song picks {}", s_side_draws[0], s_side_draws[1],
                              s_song_picks);
  for (int g = 0; g < 2; g++)
  {
    if (s_side_lr[g].empty())
      continue;
    std::vector<std::pair<u32, u32>> v(s_side_lr[g].begin(), s_side_lr[g].end());
    std::sort(v.begin(), v.end(), [](auto& a, auto& b) { return a.second > b.second; });
    t += fmt::format(" (gen {} callers:", g + 1);
    for (size_t k = 0; k < v.size() && k < 8; k++)
      t += fmt::format(" {:08x} x{}", v[k].first, v[k].second);
    t += ")";
  }
  if (reset)
  {
    s_side_draws[0] = s_side_draws[1] = 0;
    s_side_lr[0].clear();
    s_side_lr[1].clear();
    s_song_picks = 0;
  }
  return t;
}
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

// Heaps that live in fake VMEM (heap 4 at 0x7E000010: the Apt / Flash front end's heap). Its block headers and the
// contents of its used, untagged blocks are part of the rollback state, like heap 0's; without them a rewind restored
// the front end's pointers (in MEM1) but not the heap they point into (lab 26 October, rollback in the menus: crash in
// the heap code 0x80249818 freeing 0x7E128000 from the front end at 0x801A315C).
constexpr u32 VMEM_BASE = 0x7E000000, VMEM_SIZE = 0x02000000;
u32 VRd32(const u8* vmem, u32 a)
{
  const u32 o = a - VMEM_BASE;
  if (o > VMEM_SIZE - 4)
    return 0;
  return (u32(vmem[o]) << 24) | (u32(vmem[o + 1]) << 16) | (u32(vmem[o + 2]) << 8) | vmem[o + 3];
}
void BuildVRanges(const u8* ram, const u8* vmem, std::vector<std::pair<u32, u32>>& out)
{
  out.clear();
  if (!vmem)
    return;
  for (u32 k = 0; k < 16; k++)
  {
    const u32 h = Rd32(ram, 0x8040BD70 + 4 * k);
    if (h < VMEM_BASE || h >= VMEM_BASE + VMEM_SIZE)
      continue;
    const u32 base = VRd32(vmem, h + 8), end = VRd32(vmem, h + 0xC);
    if (base < VMEM_BASE || end > VMEM_BASE + VMEM_SIZE || end <= base)
      continue;
    out.push_back({std::min(h, base) - VMEM_BASE, base + 0x90 - VMEM_BASE});  // descriptor
    // its blocks are page-aligned (headers at 0x...FF0) and doubly linked (+8 next, +0xC previous): find the first
    // one by walking back from the free-list head (descriptor +0x20)
    auto is_block = [&](u32 b) {
      const u32 m = VRd32(vmem, b) >> 16;
      return b >= base && b < end && (m == 0x424D || m == 0x4246);
    };
    u32 a = VRd32(vmem, h + 0x20), n = 0;
    if (!is_block(a))
      a = base + 0x90;
    while (n++ < 40000)
    {
      const u32 pv = VRd32(vmem, a + 0xC);
      if (!is_block(pv) || pv >= a)
        break;
      a = pv;
    }
    n = 0;
    while (a >= base && a < end && n < 40000)
    {
      const u32 w = VRd32(vmem, a), sz = VRd32(vmem, a + 4), nx = VRd32(vmem, a + 8);
      out.push_back({a - VMEM_BASE, a + ((w >> 16) == 0x4246 ? 0x18 : 0x10) - VMEM_BASE});
      if ((w >> 16) == 0x424D && sz && !(w & 0x80) && a + 16 + sz <= end)
        out.push_back({a + 16 - VMEM_BASE, a + 16 + sz - VMEM_BASE});
      if (nx <= a)
        break;
      a = nx;
      n++;
    }
  }
  std::sort(out.begin(), out.end());
  std::vector<std::pair<u32, u32>> r;
  for (auto p : out)
  {
    if (p.first >= p.second)
      continue;
    if (!r.empty() && p.first <= r.back().second)
      r.back().second = std::max(r.back().second, p.second);
    else
      r.push_back(p);
  }
  out.swap(r);
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
  ex.push_back({ADDR_VICOUNT & 0x01FFFFFF, (ADDR_VICOUNT + 8) & 0x01FFFFFF});
  // pacefix DOL patch (29 October; "PACE" at 0x80468F28): its words 0x80468F00-0x80468F2F are main-loop pacing state
  // (pass flags, the main thread's wait queue, VI callback count, pass start / length from the real time base, wait
  // counters) - real-time state like the VI counter: not rewound, not compared
  if (rd(0x80468F28) == 0x50414345)
    ex.push_back({0x00468F00, 0x00468F30});  // VI retrace counters (real time)  // rbtag: deferred-free queue, sound-ref / free logs and flags (system state)
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

u32 Fnv(const u8* p, size_t n, u32 h);
void SaveState(Core::System& system, u32 frame)
{
  const auto t0 = std::chrono::steady_clock::now();
  auto& memory = system.GetMemory();
  auto& ppc = system.GetPPCState();
  const u8* ram = memory.GetRAM();
  SavedState& s = s_states[frame % NSLOTS];
  const bool resaved = s.frame == frame;  // lab stress: this slot already holds this frame (a re-simulation)
  if (s_stress && s.frame == frame && !s_stall_now)  // (a counter stall saves the same frame again: no compare)
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
  {
    const u8* vmem = memory.GetFakeVMEM();
    if (s_stress && vmem && resaved && !s.vranges.empty())
    {
      // lab stress: the re-simulated VMEM heap must equal the first run's
      size_t vo = 0;
      u32 vdiff = 0;
      for (auto [lo, hi] : s.vranges)
      {
        for (u32 x = lo; x + 4 <= hi; x += 4)
          if (memcmp(vmem + x, s.vdata.data() + vo + (x - lo), 4) != 0 && vdiff++ < 8)
            fprintf(stderr, "FFNYRB STRESS frame %u: VMEM %08x differs\n", frame, x + VMEM_BASE);
        vo += hi - lo;
      }
      if (vdiff)
        fprintf(stderr, "FFNYRB STRESS frame %u: %u VMEM words differ\n", frame, vdiff);
    }
    // outside fights only: in a fight the VMEM heap holds ~5 MB of loaded data the fight logic does not change
    // (fights ran without it), and copying it every frame would cost speed
    if (Rd32(ram, ADDR_FIGHT) == 1 && !(Rd32(ram, ADDR_FIGHT_FLAGS) & FIGHT_DECIDED))
      s.vranges.clear();
    else
      BuildVRanges(ram, vmem, s.vranges);
    size_t vt = 0;
    for (auto [lo, hi] : s.vranges)
      vt += hi - lo;
    s.vdata.resize(vt);
    size_t vo = 0;
    for (auto [lo, hi] : s.vranges)
    {
      memcpy(s.vdata.data() + vo, vmem + lo, hi - lo);
      vo += hi - lo;
    }
    // lab: FFNY_RB_HASHLOG=file -> hash of every first-run state (compare a no-rewind run with a stress run);
    // FFNY_RB_STATEDUMP=frame,file -> that frame's state (ranges + data) to a file
    if (!resaved)
    {
      static FILE* hl = std::getenv("FFNY_RB_HASHLOG") ? fopen(std::getenv("FFNY_RB_HASHLOG"), "w") : nullptr;
      if (hl)
      {
        u32 h = Fnv(s.data.data(), s.data.size(), 2166136261u);
        h = Fnv(reinterpret_cast<const u8*>(s.ranges.data()), s.ranges.size() * 8, h);
        fprintf(hl, "%u %08x %zu %08x\n", frame, h, s.ranges.size(), Fnv(s.vdata.data(), s.vdata.size(), 2166136261u));
        fflush(hl);
      }
      static u32 dump_at = 0, dump_stride = 1;
      static std::string dump_file;
      if (const char* d = std::getenv("FFNY_RB_STATEDUMP"); d && dump_file.empty())
      {
        dump_at = u32(std::atoi(d));
        dump_file = strchr(d, ',') ? strchr(d, ',') + 1 : "dump.bin";
        if (const char* st = std::getenv("FFNY_RB_STATEDUMP_STRIDE"))
          dump_stride = std::max(1, std::atoi(st));
      }
      if (dump_at && frame >= dump_at && frame < dump_at + 40 * dump_stride && (frame - dump_at) % dump_stride == 0)
      {
        if (FILE* df = fopen((dump_file + "." + std::to_string(frame)).c_str(), "wb"))
        {
          const u32 n = u32(s.ranges.size());
          fwrite(&n, 4, 1, df);
          fwrite(s.ranges.data(), 8, n, df);
          fwrite(s.data.data(), 1, s.data.size(), df);
          fclose(df);
        }
      }
    }
    if (frame == 2600 || frame == 3000)
      fprintf(stderr, "FFNYRB AFPOOL frame %u: array %08x count %u free head %08x gen %08x; array block header %08x %08x\n",
              frame, Rd32(ram, 0x8043FBB8), Rd32(ram, 0x804405A0), Rd32(ram, 0x804405A4), Rd32(ram, 0x804405AC),
              Rd32(ram, Rd32(ram, 0x8043FBB8) - 16), Rd32(ram, Rd32(ram, 0x8043FBB8) - 12));
    static u32 shown_at = 0;
    if (vmem && (frame >= shown_at + 300 || frame == 3280))
    {
      shown_at = frame;
      const u32 h = Rd32(ram, 0x8040BD70 + 16);
      fprintf(stderr, "FFNYRB VMEM frame %u: %zu ranges, %zu KB; heap 4 desc %08x words %08x %08x %08x %08x %08x %08x\n",
              frame, s.vranges.size(), vt / 1024, h, VRd32(vmem, h), VRd32(vmem, h + 4), VRd32(vmem, h + 8),
              VRd32(vmem, h + 12), VRd32(vmem, h + 16), VRd32(vmem, h + 20));
      if (frame == 3280 || frame == 2500)
      {
        for (u32 x = 0x8060C050; x < 0x8060C0E0; x += 16)
          fprintf(stderr, "FFNYRB HEAP0 %08x: %08x %08x %08x %08x\n", x, Rd32(ram, x), Rd32(ram, x + 4), Rd32(ram, x + 8), Rd32(ram, x + 12));
      }
      if (frame == 3280 || frame == 2500)
        for (u32 x : {0x7E000010u, 0x7E000020u, 0x7E000030u, 0x7E000040u, 0x7E000050u, 0x7E000060u, 0x7E000070u, 0x7E125FF0u})
          fprintf(stderr, "FFNYRB VMEM %08x: %08x %08x %08x %08x\n", x, VRd32(vmem, x), VRd32(vmem, x + 4),
                  VRd32(vmem, x + 8), VRd32(vmem, x + 12));
    }
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
  if (u8* vmem = system.GetMemory().GetFakeVMEM())
  {
    size_t vo = 0;
    for (auto [lo, hi] : s.vranges)
    {
      memcpy(vmem + lo, s.vdata.data() + vo, hi - lo);
      vo += hi - lo;
    }
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
std::array<std::vector<u8>, 64> s_hashed_copy;
std::array<u32, 64> s_hashed_copy_frame{};
static void DesyncDump(u32 f)
{
  static const char* dd = std::getenv("FFNY_RB_DESYNCDUMP");
  static u32 n = 0;
  if (!dd || n >= 6 || s_hashed_copy_frame[f % 64] != f)
    return;
  n++;
  if (FILE* df = fopen((std::string(dd) + "." + std::to_string(f)).c_str(), "wb"))
  {
    fwrite(s_hashed_copy[f % 64].data(), 1, s_hashed_copy[f % 64].size(), df);
    fclose(df);
  }
}
std::vector<std::array<u32, 3>> s_lab_pokes;  // frame, address, value
std::mutex s_lab_pokes_mutex;
static void OnRemoteStateCheck(const std::vector<u8>& m);
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
    else if (m.size() >= 13 && m[0] == 'K')
    {
      // lab: a bench poke decided by the other machine (LabSchedulePokeInner)
      auto g = [&](int o) { return (u32(m[o]) << 24) | (u32(m[o + 1]) << 16) | (u32(m[o + 2]) << 8) | m[o + 3]; };
      const u32 tf = g(1);
      {
        std::lock_guard lk(s_lab_pokes_mutex);
        s_lab_pokes.push_back({tf, g(5), g(9)});
      }
      if (s_ram && tf < Rd32(s_ram, ADDR_COUNTER))
        s_mispredict_from = std::min(s_mispredict_from, tf);
      fprintf(stderr, "FFNYRB lab poke %08x = %08x at frame %u (received)\n", g(5), g(9), tf);
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
    else if (m.size() >= 9 && m[0] == 'C')
    {
      OnRemoteStateCheck(m);  // the other machine's full-state check (see StateCheck)
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
      s_desyncs_fight++;
          fprintf(stderr, "FFNYRB DESYNC at frame %u (local %08x remote %08x)\n", f, it->second, h);
          DesyncDump(f);
          if (s_desyncs_fight <= 3)
            OSD::AddMessage("FFNY rollback: DESYNC detected at frame " + std::to_string(f), 8000,
                            OSD::Color::RED);
          auto lp = s_local_parts.find(f);
          auto rp = s_remote_parts.find(f);
          if (s_desyncs_fight <= 3 && lp != s_local_parts.end() && rp != s_remote_parts.end() && s_ram)
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
// lab: memory pokes of the bench script applied at an agreed logic frame (two-machine tests: the script reaches a poke
// at different moments on the two machines; applied here, in every run of that frame, they are part of the timeline)
void LabSchedulePokeInner(u32 addr, u32 value, u32 frame)
{
  std::lock_guard lk(s_lab_pokes_mutex);
  if (s_transport && s_ram)
  {
    // two machines: the one with port 0 decides the frame (90 frames ahead) and sends it; the other one's script
    // pokes are ignored (a late message rolls that machine back to the poke frame, see Pump)
    if (!(s_settings.local_ports & 1))
      return;
    const u32 tf = Rd32(s_ram, ADDR_COUNTER) + 90;
    s_lab_pokes.push_back({tf, addr, value});
    std::vector<u8> m(13);
    m[0] = 'K';
    for (int i = 0; i < 4; i++)
    {
      m[1 + i] = u8(tf >> (24 - 8 * i));
      m[5 + i] = u8(addr >> (24 - 8 * i));
      m[9 + i] = u8(value >> (24 - 8 * i));
    }
    s_transport->Send(m);
    fprintf(stderr, "FFNYRB lab poke %08x = %08x at frame %u (sent)\n", addr, value, tf);
    return;
  }
  s_lab_pokes.push_back({frame, addr, value});
}
void Inject(u8* ram, u32 tick_obj, u32 f)
{
  {
    std::lock_guard lk(s_lab_pokes_mutex);
    for (const auto& p : s_lab_pokes)
      if (p[0] == f)
        Wr32(ram, p[1], p[2]);
  }
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
  LogLine(text);
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
  // Automatic repair (29 October, Robert: long sessions must stay in sync): a difference in the fighters, the fight
  // RNG, the shared generators or the frame counter is real - end rollback here and take the host's state, as after
  // a forced fight end (both machines see the mismatch and stop; the host's whole emulator state replaces the
  // others'). Rollback starts again at the next fight. FFNY_RB_NOAUTORESYNC=1: only report.
  static const bool no_auto = std::getenv("FFNY_RB_NOAUTORESYNC") != nullptr;
  if (!no_auto && s_active && s_fight_end_cb && !s_force_end && !s_resync_waiting)
  {
    Report("FFNY: desync - ending rollback here and taking the host's state (automatic resync); rollback starts "
           "again at the next fight");
    RequestForceEnd("desync at frame " + std::to_string(f) + " (" + which + ")");
  }
}

static void MenusFightMarks(const u8* ram, u32 f);
void HashFrame(const u8* ram, u32 f)
{
  MenusFightMarks(ram, f);
  // lab: FFNY_RB_CONFDUMP=from,to,file -> MEM1 (24 MB raw) of the confirmed frames from..to (compare two machines)
  {
    static u32 cd_from = 0, cd_to = 0;
    static std::string cd_file;
    static bool cd_read = false;
    if (!cd_read)
    {
      cd_read = true;
      if (const char* c = std::getenv("FFNY_RB_CONFDUMP"))
      {
        char fn[512] = {};
        if (sscanf(c, "%u,%u,%511s", &cd_from, &cd_to, fn) == 3)
          cd_file = fn;
      }
    }
    if (!cd_file.empty() && f >= cd_from && f <= cd_to)
      if (FILE* df = fopen((cd_file + "." + std::to_string(f)).c_str(), "wb"))
      {
        fwrite(ram, 1, 0x01800000, df);
        fclose(df);
      }
  }
  // lab: FFNY_RB_DESYNCDUMP=prefix -> keep the hashed areas of the last 64 confirmed frames; on a desync they are
  // written to prefix.FRAME (compare the two machines' files)
  static const char* dd = std::getenv("FFNY_RB_DESYNCDUMP");
  if (dd)
  {
    std::vector<u8>& c = s_hashed_copy[f % 64];
    c.assign(ram + 0x3F1144, ram + 0x3F1144 + 4 * 0x12F8);
    c.insert(c.end(), ram + 0x3F6AC8, ram + 0x3F6AC8 + 0x88);
    c.insert(c.end(), ram + 0x35D588, ram + 0x35D588 + 0x18);
    c.insert(c.end(), ram + 0x3BE03C, ram + 0x3BE03C + 0x88);
    s_hashed_copy_frame[f % 64] = f;
  }
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
      s_desyncs_fight++;
      fprintf(stderr, "FFNYRB DESYNC at frame %u (local %08x remote %08x)\n", f, h, it->second);
      DesyncDump(f);
      if (s_desyncs_fight <= 3)
        OSD::AddMessage("FFNY rollback: DESYNC detected at frame " + std::to_string(f), 8000,
                        OSD::Color::RED);
      if (s_desyncs_fight <= 3)
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
// set to 1 by HookMatchUpdate, read and cleared at every update start: "the match update ran in the previous update".
// In emulated RAM (free word of rbtag's area, which rollback never rewinds), so a resync carries it: both machines then
// agree on it at the next update even if the host paused a little later inside that update than the client.
constexpr u32 W_MATCH = 0x8049B5F0;
// "rollback fight running" for rbtag (written by Dolphin: 1 from the fight start to the fight end). rbtag v5 reads
// 0x803F1000 == 1 for its SysHeap routing (alloc hook 0x8049A670 / 0x8049A694) and its deferred main-heap frees
// (free hook 0x8049A8B0 / 0x8049A8B8, flushed once the flag is not 1); 0x803F1000 stays 1 after a quit to the menus
// and after environmental KOs, so frees stayed deferred in the menus (24 of 63 queued when both Dolphins crashed in
// the heap code at 0x80249818 after a quit, 24 October). Dolphin points both reads at this word.
constexpr u32 W_FIGHTW = 0x8049B5F4;
bool s_rbtag_redirected = false;
// instant loads (see InstantDiscReads): the file-system thread's two waits for its DVD read
// OSReceiveMessage 0x80274E5C puts the receiving thread to sleep at 0x80274EB0 (bl OSSleepThread 0x802776E8, interrupts
// disabled); the file-system thread object is 0x804099A0, its message queue 0x8040BCB8
constexpr u32 OS_RECV_SLEEP = 0x80274EB0, FS_THREAD = 0x804099A0, FS_QUEUE = 0x8040BCB8;
constexpr u32 AUDIO_THREAD = 0x803E23C8;  // the music / speech streams' reads come from it
u32 s_fs_instant = 0;  // reads finished at once in this fight
u32 s_fs_not_main = 0;  // waits of requests from other threads (streams): left to normal timing
std::unordered_map<u32, u32> s_fs_req_thread;  // file-system request node -> thread that queued it
// Lockstep for file loads only for requests whose data reaches the rewound game state (29 October, Robert: short
// regular slowdowns in the fights of menus mode, 0.4-1.3 s of waiting per fight). The "busy" test looked at the file
// system's current request 0x804018D4 and its queue 0x804018E0 (nodes linked at +0x2C, sorted by +0x10) whoever made
// them, so every refill of the music / speech streams (the audio thread's reads, a few per second of play) put the
// fight into lockstep for that frame + LOAD_TAIL - each switch costs about the ping. The streams' data goes to the
// sound player (not rewound; the music module's view of it already differs per machine and is kept out of the logic),
// so those reads no longer count. Requests of the logic, the loader and unknown requesters still do.
// 29 October, evening: without lockstep for the stream reads (1bf61b8) Robert's test brought back fast, choppy music
// and the sound-thread crash at 0x801E7FC8 (read from 0x1C) - rollbacks across a stream refill do break the sound side
// (mechanism not found yet). Back on by default; the stream reads are counted separately, with their own tail
// (FFNY_RB_STREAMTAIL=N frames, default LOAD_TAIL) and FFNY_RB_NOSTREAMLOCK=1 = no lockstep for them (lab only).
const bool s_stream_nolock = std::getenv("FFNY_RB_NOSTREAMLOCK") != nullptr;
const u32 s_stream_tail = std::getenv("FFNY_RB_STREAMTAIL") ? u32(std::atoi(std::getenv("FFNY_RB_STREAMTAIL"))) : 4;
u32 s_fs_stream_last = 0;           // newest frame where the file system served only the music / speech streams
u32 s_fs_stream_only = 0;           // update starts where the file system served only the music / speech streams
u32 s_ls_files = 0, s_ls_stream = 0, s_ls_other = 0;  // lockstep frames of the fight by cause
static bool FsBusyForLogic(const u8* ram)
{
  auto counts = [&](u32 node) {
    const auto it = s_fs_req_thread.find(node);
    return it == s_fs_req_thread.end() || it->second != AUDIO_THREAD;
  };
  if (const u32 cur = Rd32(ram, 0x804018D4); cur && counts(cur))
    return true;
  u32 n = Rd32(ram, 0x804018E0), k = 0;
  while ((n & 0xFE000000) == 0x80000000 && k++ < 64)
  {
    if (counts(n))
      return true;
    n = Rd32(ram, n + 0x2C);
  }
  return false;
}
bool s_fs_main_queued = false;  // a request of the game logic waits behind another thread's
u32 s_fs_hurried = 0;  // reads of other threads finished at once because of that
const bool s_fs_hurry_off = std::getenv("FFNY_RB_NOFSHURRY") != nullptr;
// re-simulation cycles per re-simulated frame that may pass without emulated time (a logic frame takes ~0.7 M; a
// frame's worth of CPU is 8.1 M)
constexpr u64 FROZEN_BUDGET_PER_FRAME = 8000000;
constexpr u32 W_AFCTR = 0x8049CFF0;  // bigspace, rewound (rbtag's lists end at 0x8049C800)
constexpr u32 AF_ARRAY = 0x8043FBB8, AF_COUNT = 0x804405A0, AF_FREE = 0x804405A4;
constexpr u32 AF_SETID = 0x80244DD0, AF_FIND = 0x80244E00, AF_RELEASE = 0x80244E48;
bool s_af_on = false;  // deterministic async file ids (HookAfSetId)
// ---- The game's worker threads (26 October) ----
// Besides the main thread the game runs worker threads that do work for the logic asynchronously and write the results
// into game memory: the loader 0x806D8AE0 (fight loading; its own file requests, not instant), the worker 0x806D5A34
// (memory card and more, via queue 0x80A1CD1C), "MCThread" 0x80A29C80. Their own state is not rewound. A rollback across
// their work loses the results the logic already consumed (the re-simulation then waits for a result that never comes
// again: the stress run hung in the fight loading). Rules, like the memory card's: a frame in which such a thread is busy
// at the update start (not asleep on the queue it sleeps on when idle, learned when rollback starts) and the next
// ASYNC_TAIL frames run with all inputs known (lockstep), and no rollback goes back into them; a message from the
// logic that wakes such a thread (OSSendMessage 0x80274D94 to the queue it sleeps on) is a barrier like a card call.
// Workers = threads other than the main thread with priority >= 5 (the file system 2, audio 1, the per-frame pump 4
// are not: their work is either instant for the logic or handled elsewhere).
std::map<u32, u32> s_idle_wait;  // worker thread -> what it waits on when idle
u32 s_async_last = 0, s_async_busy_frames = 0, s_async_barriers = 0, s_async_aborts = 0;
u32 s_async_last_thread = 0;
bool s_async_on = false;
constexpr u32 ASYNC_TAIL = 2;
bool s_card_on = false;
constexpr std::array<u32, 19> CARD_FUNCS = {0x8025BF7C, 0x8025C184, 0x8025C2D4, 0x8025D9BC, 0x8025DADC, 0x8025E340, 0x8025E424,
                                             0x8025EC6C, 0x8025F134, 0x8025F238, 0x8025F354, 0x8025F700, 0x8025FB74, 0x8025FEF0,
                                             0x80260108, 0x80260260, 0x802604A0, 0x80260728, 0x80260854};
constexpr u32 ADDR_MATCH_UPDATE = 0x80096C04;  // match update (state machine at match +0x302A0; fight step 0x80098014)
bool s_match_hook = false;
bool s_left_match = false;  // for the fight-over line
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

u32 s_max_acq = 0, s_max_rel = 0, s_max_frl = 0;  // highest fill of the rbtag logs (96 entries each)
bool s_diag_heap_reported = false;
u32 s_diag_full_reported = 0;  // bit per log
u32 s_min_free2 = 0xFFFFFFFF, s_min_free3 = 0xFFFFFFFF;
// where heap 3's lowest free space was seen (frame, whether re-simulating, whether the frame's inputs were all known)
u32 s_min_free3_frame = 0, s_min_free3_largest = 0, s_min_free3_nodes = 0;
bool s_min_free3_resim = false, s_min_free3_predicted = false, s_low3_reported = false;
u32 s_heap_samples_skipped = 0;
// SysHeap blocks the main thread allocated during the fight, with rbtag's frame (W_FRAME) at that time. SysHeap is not
// rewound: a rewind to frame S leaves blocks allocated in frames >= S allocated, and the re-simulation allocates
// them again (e.g. the ELF modules loaded at the fight start / KO scenes: ~127 KB per rewind across that frame;
// laptop 25 October: heap 3 down to 0 KB after 4 rollbacks across file conversions). They are handed to rbtag's
// free log after the rewind, so the game frees them itself at its next sound call.
std::vector<std::pair<u32, u32>> s_sys_allocs;
std::vector<u32> s_sys_pending;  // thrown-away SysHeap blocks waiting for room in rbtag's free log
u32 s_sys_freed = 0, s_sys_free_overflow = 0, s_sys_alloc_hook = 0;

// ---- Auto-play (test tool) ----
std::atomic<bool> s_autoplay{false};
struct AutoPlayer
{
  std::mt19937 rng{static_cast<u32>(std::time(nullptr)) ^ 0x9E3779B9u};
  u32 left = 0;    // polls left in the current action
  u32 kind = 0;
  u16 buttons = 0;
  u8 sx = 0x80, sy = 0x80, cx = 0x80, cy = 0x80, trig = 0;
};
AutoPlayer s_bots[4];

// file loads vs rollbacks (see DiagLoads): frames with the file system busy, requests finished, rewinds that went back
// across a finished request / across a loader (ELF) conversion of fresh data
u32 s_fs_busy_frames = 0, s_fs_done = 0, s_fs_done_frame = 0, s_fs_prev_cur = 0;
u32 s_rb_across_load = 0, s_rb_across_conv = 0, s_conv_frame = 0;
u32 s_bank_guard = 0;  // animation lookups in an empty bank answered "not found"
u32 s_fs_last_busy = 0;      // newest frame at which the file system was seen busy
// measurements for shortening LOAD_TAIL (24 October): newest frame the file system itself was busy (not counting
// conversions), busy episodes, lockstep frames run only because of the tail, conversions of data outside the
// rollback state and how many frames after the file system was last busy they came
u32 s_fs_real_busy = 0, s_fs_episodes = 0, s_lockstep_tail = 0, s_conv_out = 0, s_conv_gap_max = 0;
bool s_fs_was_busy = false;
u32 s_lockstep_frames = 0;   // frames run in lockstep because of file loads
u64 s_lockstep_us = 0;       // time spent waiting for that
bool s_conv_warned = false;
// frames of lockstep after the file system went idle (the game converts loaded data after that). Was 20; Robert's
// measurement (24 October, window KO fight, both machines): 92 % of the lockstep frames came only from this tail
// (739 / 758 of 803 / 827) and all 159 conversions of data outside the rollback state came at most 1 frame after the
// file system was busy. A later conversion still gets lockstep from its own frame on and the chat warning if a
// rollback crosses it.
constexpr u32 LOAD_TAIL = 4;
constexpr u32 START_LOCKSTEP = 20;  // lockstep frames at the start of a fight (its files are still loading)

// return addresses on the emulated stack (back chain), for crash reports
std::string CallChain(const u8* ram, u32 sp, int max)
{
  std::string out;
  for (int k = 0; k < max && (sp & 0xFE000000) == 0x80000000; k++)
  {
    const u32 up = Rd32(ram, sp);
    if ((up & 0xFE000000) != 0x80000000 || up <= sp)
      break;
    char b[16];
    snprintf(b, sizeof b, "%s%08x", out.empty() ? "" : " < ", Rd32(ram, up + 4));
    out += b;
    sp = up;
  }
  return out.empty() ? "-" : out;
}

// ---- Menus mode: per-fight marks and full-state checks (27 October) ----
// The desync check of every frame covers the fighters, the RNGs and the frame counter only. Without the resync after a
// fight, anything else that differs between the machines (game memory of the menus, the music module, heaps) stays.
// At the start of every fight and every 10 s outside the fights, both machines hash their whole rollback state (the
// same memory a rewind restores, plus the menus' VMEM heap) in 64 KB pieces and compare them: the log names the
// pieces that differ (the chat says how many).
// input delay for the frame captured now: the rollback delay in the running fight, the netplay pad buffer outside it
// in menus mode (every frame there runs in lockstep; see s_menu_delay)
static u32 CaptureDelay()
{
  const u32 d = s_settings.input_delay;
  static const char* md = std::getenv("FFNY_RB_MENUDELAY");
  static const bool menu_rollback = std::getenv("FFNY_RB_MENUROLLBACK") != nullptr;
  if (!s_menus_on || s_real_fight_now || menu_rollback)
    return d;
  const u32 want = md ? u32(std::atoi(md)) : s_menu_delay.load();
  return std::clamp<u32>(want, d, std::max(d, MAX_MENU_DELAY));
}

u32 s_mark_frame = 0;  // frame of the newest fight start / end mark
struct ChunkCheck
{
  std::string what;
  std::vector<std::array<u32, 3>> chunks;  // address, length, hash
};
std::map<u32, ChunkCheck> s_chk_local;
std::map<u32, std::vector<std::array<u32, 3>>> s_chk_remote;
u32 s_chk_reports = 0;

static u32 HashBytes(const u8* p, size_t n)
{
  u64 h = 0x84222325CBF29CE4ull;
  size_t i = 0;
  for (; i + 8 <= n; i += 8)
  {
    u64 w;
    memcpy(&w, p + i, 8);
    h = (h ^ w) * 0x9E3779B97F4A7C15ull;
    h ^= h >> 29;
  }
  for (; i < n; i++)
    h = (h ^ p[i]) * 0x100000001B3ull;
  return u32(h ^ (h >> 32));
}

static u32 MenuDelayNow()
{
  const bool keep = s_real_fight_now;
  s_real_fight_now = false;
  const u32 d = CaptureDelay();
  s_real_fight_now = keep;
  return d;
}

static const char* ChunkLabel(u32 addr, u32 len)
{
  auto in = [&](u32 a) { return a >= addr && a < addr + len; };
  if (in(0x8043FAC8))
    return " (Apt draw ring: render side)";
  if (in(0x80653C00))
    return " (GX texture objects: render side)";
  if (in(0x806D8E4C))
    return " (front-end FPS meter: render side)";
  if (in(0x80440AD0) || in(0x804372A4) || in(0x80440B38))
    return " (music module)";
  if (addr >= 0x803F1144 && addr < 0x803F1144 + 4 * 0x12F8)
    return " (fighters)";
  // classified from Robert's A/B dumps of 29 October (see the handoff, "Remaining state-check differences"): none of
  // these is read by the fight logic; they differ because rendering, sound and the GPU's deferred releases follow
  // each machine's own displayed frames and audio
  if (in(0x80433FA8) || in(0x80434004))
    return " (song position from the sound player: sound side)";
  if (in(0x8049A210))
    return " (audio module's private generators: sound side)";
  if (in(0x803D5950))
    return " (Flash player's deferred GPU releases: render side)";
  if (in(0x80437600))
    return " (stack leftovers: junk)";
  if (in(0x802F9748) || in(0x802F9874))
    return " (frame timing meter: render side)";
  if (addr < 0x806E84F0 && addr + len > 0x8060C3D0)
    return " (small-object pool: same objects, other slots)";
  if (in(0x809C2110) || in(0x809D0000))
    return " (object table: same objects, other slots)";
  if (in(0x80A02130))
    return " (matrices: cosmetic)";
  if (addr >= VMEM_BASE && addr < 0x80000000u)
    return " (VMEM: front-end heap)";
  return "";
}

static void CompareStateCheck(u32 f)
{
  auto l = s_chk_local.find(f);
  auto r = s_chk_remote.find(f);
  if (l == s_chk_local.end() || r == s_chk_remote.end())
    return;
  std::map<std::pair<u32, u32>, u32> theirs;
  for (const auto& c : r->second)
    theirs[{c[0], c[1]}] = c[2];
  u32 differ = 0, only_mine = 0, compared = 0;
  u64 differ_bytes = 0;
  std::string list;
  for (const auto& c : l->second.chunks)
  {
    auto it = theirs.find({c[0], c[1]});
    if (it == theirs.end())
    {
      only_mine++;
      continue;
    }
    compared++;
    if (it->second != c[2])
    {
      differ++;
      differ_bytes += c[1];
      if (differ <= 40)
        list += fmt::format("{}{:08x}+{:x}{}", list.empty() ? "" : ", ", c[0], c[1], ChunkLabel(c[0], c[1]));
    }
    theirs.erase(it);
  }
  const u32 only_theirs = u32(theirs.size());
  std::string line;
  if (!differ && !only_mine && !only_theirs)
    line = fmt::format("FFNY state check at frame {} ({}): whole rollback state equal on both machines ({} pieces)", f,
                       l->second.what, compared);
  else
    line = fmt::format("FFNY state check at frame {} ({}): {} of {} pieces DIFFER ({} KB){}{}; memory layout: {} pieces "
                       "only here, {} only on the other machine",
                       f, l->second.what, differ, compared, differ_bytes / 1024, list.empty() ? "" : ": ", list,
                       only_mine, only_theirs);
  LogLine(line);
  fprintf(stderr, "FFNYRB %s\n", line.c_str());
  // the chat: the machine playing port 1 speaks for both (the other one only logs); at most 8 lines per session
  if (s_report_cb && (s_settings.local_ports & 1) && s_chk_reports++ < 8)
  {
    if (!differ && !only_mine && !only_theirs)
      s_report_cb(line);
    else
      s_report_cb(fmt::format("FFNY state check at frame {} ({}): {} of {} memory pieces differ between the machines, "
                              "layout differs in {} - details in Logs/FFNY_rollback.log",
                              f, l->second.what, differ, compared, only_mine + only_theirs));
  }
  s_chk_local.erase(l);
  s_chk_remote.erase(r);
}

static void OnRemoteStateCheck(const std::vector<u8>& m)
{
  const u32 f = Get32(&m[1]), n = Get32(&m[5]);
  if (m.size() < 9 + size_t(n) * 12)
    return;
  std::vector<std::array<u32, 3>> v(n);
  for (u32 i = 0; i < n; i++)
    v[i] = {Get32(&m[9 + 12 * i]), Get32(&m[13 + 12 * i]), Get32(&m[17 + 12 * i])};
  s_chk_remote[f] = std::move(v);
  while (s_chk_remote.size() > 8)
    s_chk_remote.erase(s_chk_remote.begin());
  CompareStateCheck(f);
}

static void StateCheck(const u8* ram, u32 f, const char* what)
{
  if (!s_transport || std::getenv("FFNY_RB_NOSTATECHECK"))
    return;
  auto& memory = Core::System::GetInstance().GetMemory();
  std::vector<std::pair<u32, u32>> ranges;
  BuildRanges(ram, memory.GetRamSizeReal(), ranges);
  // not compared: the main thread's stack (dead space below the stack pointer holds whatever the last calls left) and
  // the GPU fence counter (render driven)
  const u32 st_hi = Rd32(ram, 0x80432D10 + 0x304) & 0x01FFFFFF, st_lo = Rd32(ram, 0x80432D10 + 0x308) & 0x01FFFFFF;
  std::vector<std::pair<u32, u32>> excl = {{0x002F4CB0, 0x002F4CB4}};
  if (st_lo < st_hi && st_hi - st_lo < 0x100000)
    excl.push_back({st_lo, st_hi});
  ChunkCheck cc;
  cc.what = what;
  auto add = [&](const u8* base, u32 addr0, u32 lo, u32 hi) {
    while (lo < hi)
    {
      const u32 end = std::min(hi, (lo | 0xFFFF) + 1);
      cc.chunks.push_back({addr0 + lo, end - lo, HashBytes(base + lo, end - lo)});
      lo = end;
    }
  };
  for (auto [lo, hi] : ranges)
  {
    std::vector<std::pair<u32, u32>> parts = {{lo, hi}};
    for (auto [xl, xh] : excl)
    {
      std::vector<std::pair<u32, u32>> next;
      for (auto [a, b] : parts)
      {
        if (xh <= a || xl >= b)
        {
          next.push_back({a, b});
          continue;
        }
        if (a < xl)
          next.push_back({a, xl});
        if (xh < b)
          next.push_back({xh, b});
      }
      parts = std::move(next);
    }
    for (auto [a, b] : parts)
      add(ram, 0x80000000u, a, b);
  }
  if (const u8* vmem = memory.GetFakeVMEM())
  {
    std::vector<std::pair<u32, u32>> vr;
    BuildVRanges(ram, vmem, vr);
    for (auto [lo, hi] : vr)
      add(vmem, VMEM_BASE, lo, hi);
  }
  std::vector<u8> m;
  m.reserve(9 + cc.chunks.size() * 12);
  m.push_back('C');
  Put32(m, f);
  Put32(m, u32(cc.chunks.size()));
  for (const auto& c : cc.chunks)
    for (u32 v : c)
      Put32(m, v);
  s_transport->Send(m);
  s_chk_local[f] = std::move(cc);
  while (s_chk_local.size() > 8)
    s_chk_local.erase(s_chk_local.begin());
  CompareStateCheck(f);
}

// lab / diagnosis: FFNY_RB_EVDUMP=prefix -> MEM1 (and the VMEM heap) at every fight start / end mark of menus mode, to
// prefix.FRAME.mem1 / .vmem (give each Dolphin its own prefix and compare the files of the same frame)
static void EvDump(const u8* ram, u32 f)
{
  static const char* ev = std::getenv("FFNY_RB_EVDUMP");
  static u32 n = 0;
  if (!ev || n >= 16)
    return;
  n++;
  if (FILE* df = fopen(fmt::format("{}.{}.mem1", ev, f).c_str(), "wb"))
  {
    fwrite(ram, 1, 0x01800000, df);
    fclose(df);
  }
  if (const u8* vmem = Core::System::GetInstance().GetMemory().GetFakeVMEM())
    if (FILE* df = fopen(fmt::format("{}.{}.vmem", ev, f).c_str(), "wb"))
    {
      fwrite(vmem, 1, 0x00A00000, df);
      fclose(df);
    }
}

static void MenusFightMarks(const u8* ram, u32 f)
{
  if (!s_menus_on || !s_active)
    return;
  const bool raw = Rd32(ram, ADDR_FIGHT) == 1 && !(Rd32(ram, ADDR_FIGHT_FLAGS) & FIGHT_DECIDED);
  // match set-up (fight flags bit 0 set by the fight init; 13743 in Robert's 29 October log): log line + MEM1 / VMEM
  // dumps (FFNY_RB_EVDUMP) at the set-up frame and 1 / 30 frames later, when the loader thread is still setting up
  {
    static bool setup_prev = false;
    static u32 dump_at[2] = {0, 0};
    const bool setup = !raw && (Rd32(ram, ADDR_FIGHT_FLAGS) & 1);
    if (setup && !setup_prev)
    {
      LogLine(fmt::format("FFNY match set-up at frame {}: match rule {}, fight loop {}, other threads' random draws "
                          "since the last line: {}",
                          f, Rd32(ram, 0x803F0DA0), Rd32(ram, ADDR_FIGHT), SideDrawText(true)));
      EvDump(ram, f);
      dump_at[0] = f + 1;
      dump_at[1] = f + 30;
    }
    else if (f == dump_at[0] || f == dump_at[1])
      EvDump(ram, f);
    setup_prev = setup;
  }
  if (raw == s_fight_raw_prev)
  {
    // outside the fights (lockstep: every frame is hashed here, at the same frames on both machines): every 10 s
    if (!raw && f % 600 == 0 && f >= s_mark_frame + 10)
    {
      LogLine(fmt::format("FFNY menus status at frame {}: fight loop {}, fight flags {:03x}, match rule {}, match "
                          "updates {}, other threads' random draws: {}",
                          f, Rd32(ram, ADDR_FIGHT), Rd32(ram, ADDR_FIGHT_FLAGS) & 0x1FF, Rd32(ram, 0x803F0DA0),
                          s_match_updates, SideDrawText(true)));
      s_match_updates = 0;
      StateCheck(ram, f, "between fights");
    }
    return;
  }
  s_fight_raw_prev = raw;
  s_mark_frame = f;
  std::string line;
  if (!raw)
  {
    line = fmt::format("FFNY fight {} over at frame {} (menus mode: rollback stays on, no resync): {} rollbacks (max {} "
                       "frames, {} frames re-simulated), {} waits for the other side ({:.1f} s), {} desyncs; lockstep "
                       "frames {} (game file loads incl. the fight start {}, music / speech stream reads {}, memory card "
                       "/ worker threads {}; {:.1f} s waiting), update starts during stream reads {}; other threads' "
                       "random draws: {}",
                       s_fight_no, f, s_rollbacks, s_max_depth, s_resim_frames, s_waits, s_wait_us / 1e6,
                       s_desyncs_fight, s_lockstep_frames, s_ls_files, s_ls_stream, s_ls_other, s_lockstep_us / 1e6,
                       s_fs_stream_only, SideDrawText(true));
  }
  else
  {
    s_fight_no++;
    line = fmt::format("FFNY fight {} starts at frame {} (menus mode). Since the last fight: {} frames, {} waits for the "
                       "other side ({:.1f} s), input delay outside the fight {}, {} desyncs; other threads' random "
                       "draws: {}",
                       s_fight_no, f, f - s_fight_from, s_waits, s_wait_us / 1e6, MenuDelayNow(), s_desyncs_fight,
                       SideDrawText(true));
    // the new fight's files are usually still loading (as at the first fight, see StartFight)
    s_fs_last_busy = std::max(s_fs_last_busy, f + START_LOCKSTEP - LOAD_TAIL);
    s_reports = 0;
    s_desyncs_fight = 0;
  }
  s_fight_from = f;
  s_rollbacks = s_resim_frames = s_max_depth = s_waits = 0;
  s_wait_us = 0;
  s_lockstep_frames = 0;
  s_lockstep_us = 0;
  s_ls_files = s_ls_stream = s_ls_other = s_fs_stream_only = 0;
  LogLine(line);
  if (s_report_cb)
    s_report_cb(line);
  if (raw)
    StateCheck(ram, f, "fight start");
  EvDump(ram, f);
}

void StartFight(u8* ram, u32 tick_obj, u32 f)
{
  s_active = true;
  if (s_menus && !s_menus_on)
  {
    // rollback in the menus needs rbtag v6 (frees of the menus' main-heap blocks kept in the rollback state)
    if (Rd32(ram, 0x8049A408) == 0x52425436)
    {
      s_menus_on = true;
      Report("FFNY: rollback stays on after this fight (menus mode)");
    }
    else
      Report("FFNY: menus mode needs rbtag v6 in the DOL (rbtag_v6_install.py) - rollback in fights only");
  }
  // The frame clock (menus mode / lab always mode) starts here, from the time base both machines share at this moment
  // (before the first rollback fight they ran in Dolphin's lockstep netplay, cycle for cycle). Before 27 October the
  // anchor was the real time base at the first frame-clock read - in menus mode that is after the first fight (or at
  // the first pause), when the machines' emulated times already differ: every clock read in the menus then differed
  // by a constant on the two machines (Apt's clock 0x804404BC; random generators seeded from it at the next fight's
  // set-up, 0x80396BB8 and the array at 0x803F92D0 -> fighter 2 differed 4 frames into the next fight).
  s_vtb_anchor = Core::System::GetInstance().GetPowerPC().ReadFullTimeBaseValue();
  s_vtb_frame0 = f;
  s_seed_reads = 0;
  s_apt_side = 0;
  // side states for the other threads' random draws (see Gen1Step): from the shared ones, equal on all machines here
  for (int i = 0; i < 6; i++)
    s_side1[i] = Rd32(ram, GEN1_SHARED + 4 * i) ^ (0x9E3779B9u * u32(i + 1));
  for (int i = 0; i < 34; i++)
    s_side2[i] = Rd32(ram, GEN2_SHARED + 4 * i);
  s_side2[32] ^= 0x7F4A7C15u;
  for (int i = 0; i < 6; i++)
    s_song[i] = Rd32(ram, GEN1_PRIVATE + 4 * i) ^ (0x85EBCA6Bu * u32(i + 3));
  s_song_picks = 0;
  s_song_frame = 0xFFFFFFFF;
  s_side_draws[0] = s_side_draws[1] = 0;
  s_side_lr[0].clear();
  s_side_lr[1].clear();
  s_snd_q.clear();
  s_snd_held = s_snd_sent = s_snd_dropped = 0;
  Wr32(ram, W_CONF, f - 1);
  Wr32(ram, W_RB, 0);
  Wr32(ram, W_FRAME, f);
  Wr32(ram, W_ACT, 1);
  if (s_rbtag_redirected)
    Wr32(ram, W_FIGHTW, 1);
  s_fs_instant = 0;
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
      if (h && (h & 0xFE000000) != 0x80000000)
        fprintf(stderr, "FFNYRB heap %u: desc %08x (outside MEM1)\n", i, h);
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
  for (int p = 0; p < 4; p++)
    if (s_settings.local_ports >> p & 1)
      s_confirmed[p] = f + s_settings.input_delay - 1;  // captured up to here (see the capture in HookPreUpdate)
  s_cur_delay = s_settings.input_delay;
  s_fight_raw_prev = true;
  s_fight_no = 1;
  s_fight_from = f;
  s_desyncs_fight = 0;
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
  s_max_acq = s_max_rel = s_max_frl = 0;
  s_diag_heap_reported = false;
  s_diag_full_reported = 0;
  s_fs_busy_frames = s_fs_done = s_rb_across_load = s_rb_across_conv = 0;
  s_fs_done_frame = s_conv_frame = 0;
  s_fs_prev_cur = Rd32(ram, 0x804018D4);
  s_fs_last_busy = f + START_LOCKSTEP - LOAD_TAIL;  // files are usually still being loaded when a fight starts
  s_fs_real_busy = f;
  s_fs_episodes = s_lockstep_tail = s_conv_out = s_conv_gap_max = 0;
  s_fs_was_busy = true;
  s_lockstep_frames = 0;
  s_lockstep_us = 0;
  s_ls_files = s_ls_stream = s_ls_other = s_fs_stream_only = 0;
  s_conv_warned = false;
  s_min_free2 = s_min_free3 = 0xFFFFFFFF;
  s_min_free3_frame = s_min_free3_largest = s_min_free3_nodes = 0;
  s_min_free3_resim = s_min_free3_predicted = s_low3_reported = false;
  s_heap_samples_skipped = 0;
  s_sys_allocs.clear();
  s_sys_pending.clear();
  s_sys_freed = s_sys_free_overflow = 0;
  OSD::AddMessage("FFNY rollback: fight started (input delay " + std::to_string(s_settings.input_delay) +
                      (s_settings.load_lockstep ? ", lockstep while loading" : "") + ")",
                  3000);
  LogLine("FFNY fight started at frame " + std::to_string(f) + " (input delay " +
          std::to_string(s_settings.input_delay) + (s_settings.load_lockstep ? ", lockstep while loading" : "") +
          (s_autoplay ? ", auto-play on" : "") + ")");
  fprintf(stderr, "FFNYRB fight start at frame %u (local %x active %x delay %u)\n", f,
          s_settings.local_ports, s_settings.active_ports, s_settings.input_delay);
}

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
  {
    fprintf(stderr, "FFNYRB gen2 hook calls (shared/private/other): U %u/%u/%u T %u/%u/%u R %u/%u/%u ? %u/%u/%u\n",
            s_rng2_calls[0][0], s_rng2_calls[0][1], s_rng2_calls[0][2], s_rng2_calls[1][0], s_rng2_calls[1][1],
            s_rng2_calls[1][2], s_rng2_calls[2][0], s_rng2_calls[2][1], s_rng2_calls[2][2], s_rng2_calls[3][0],
            s_rng2_calls[3][1], s_rng2_calls[3][2]);
  }
  fprintf(stderr, "FFNYRB effect manager draws on the private generator: %u\n", s_rng_effect);
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

std::mutex s_logfile_lock;
void LogLine(const std::string& text)
{
  std::lock_guard lk(s_logfile_lock);
  File::IOFile f(File::GetUserPath(D_LOGS_IDX) + "FFNY_rollback.log", "ab");
  if (!f)
    return;
  char ts[32];
  const std::time_t now = std::time(nullptr);
  std::strftime(ts, sizeof ts, "%Y-%m-%d %H:%M:%S", std::localtime(&now));
  // both Dolphins of a one-PC test write to the same file: say which player this machine plays
  std::string who;
  for (int p = 0; p < 4; p++)
    if (s_settings.local_ports >> p & 1)
      who += (who.empty() ? "P" : "+P") + std::to_string(p + 1);
  const std::string line = std::string("[") + ts + "] " + (who.empty() ? "" : "[" + who + "] ") + text + "\n";
  f.WriteBytes(line.data(), line.size());
}

void SetAutoPlay(bool enabled)
{
  s_autoplay = enabled;
}

bool AutoPlayEnabled()
{
  return s_autoplay;
}

void AutoPlayPad(GCPadStatus* pad, int local_pad)
{
  AutoPlayer& b = s_bots[local_pad & 3];
  auto rnd = [&](u32 n) { return std::uniform_int_distribution<u32>(0, n - 1)(b.rng); };
  if (b.left == 0)
  {
    b.buttons = 0;
    b.sx = b.sy = b.cx = b.cy = 0x80;
    b.trig = 0;
    const u32 r = rnd(100);
    const u8 dir[3] = {0x10, 0x80, 0xF0};
    if (r < 30)  // move (mostly left / right)
    {
      b.kind = 0;
      b.sx = dir[rnd(3)];
      b.sy = rnd(4) == 0 ? dir[rnd(3)] : 0x80;
      b.left = 10 + rnd(50);
    }
    else if (r < 68)  // attacks: one of A / B / X / Y, sometimes with a direction (grapples, specials)
    {
      b.kind = 1;
      static constexpr u16 face[4] = {PAD_BUTTON_A, PAD_BUTTON_B, PAD_BUTTON_X, PAD_BUTTON_Y};
      b.buttons = face[rnd(4)];
      if (rnd(3) == 0)
        b.sx = dir[rnd(3)];
      b.left = 4 + rnd(10);
    }
    else if (r < 82)  // block (R / L) for a while
    {
      b.kind = 2;
      b.buttons = rnd(2) ? PAD_TRIGGER_R : PAD_TRIGGER_L;
      b.trig = 0xFF;
      b.left = 15 + rnd(60);
    }
    else if (r < 86)  // C-stick flick (Blazin' when the meter is full, taunts otherwise)
    {
      b.kind = 3;
      b.cx = dir[rnd(3)];
      b.cy = b.cx == 0x80 ? (rnd(2) ? 0x10 : 0xF0) : 0x80;
      b.left = 5;
    }
    else if (r < 90)  // Z
    {
      b.kind = 4;
      b.buttons = PAD_TRIGGER_Z;
      b.left = 4 + rnd(6);
    }
    else  // nothing
    {
      b.kind = 5;
      b.left = 5 + rnd(25);
    }
  }
  b.left--;
  // attacks: press / release alternately so a held action means repeated presses (combos)
  u16 buttons = b.buttons;
  if (b.kind == 1 && b.left % 5 < 2)
    buttons = 0;  // 3 polls pressed, 2 released
  *pad = GCPadStatus{};
  pad->button = buttons;  // never START (pause menu)
  pad->stickX = b.sx;
  pad->stickY = b.sy;
  pad->substickX = b.cx;
  pad->substickY = b.cy;
  pad->triggerLeft = (buttons & PAD_TRIGGER_L) ? b.trig : 0;
  pad->triggerRight = (buttons & PAD_TRIGGER_R) ? b.trig : 0;
  pad->isConnected = true;
}


std::thread s_watchdog;
std::atomic<bool> s_watchdog_quit{false};

static s64 NowMs()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// Watchdog (27 October): Robert's second test session ended with one machine that stopped sending inputs ~6 s into
// the next round and never answered the "end the fight" message, and Dolphin froze when he closed it. This thread
// writes into the chat and the log when the game has not reached its next update for 6 s while emulation runs (or the
// resync after a fight has not come for 20 s), with what the CPU thread is doing.
static void WatchdogLoop()
{
  auto& system = Core::System::GetInstance();
  u64 last = s_pre_count.load();
  s64 since = NowMs();
  bool reported = false, resync_reported = false;
  while (!s_watchdog_quit)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const s64 now = NowMs();
    std::string text;
    if (s_resync_waiting)
    {
      if (!resync_reported && now - s_resync_since_ms.load() > 20000)
      {
        resync_reported = true;
        text = "FFNY WATCHDOG: the host's state after the fight has not arrived for 20 s (this machine is paused "
               "waiting for it)";
      }
    }
    else
    {
      resync_reported = false;
    }
    const u64 c = s_pre_count.load();
    const bool watch = (s_active || s_menus_on) && Core::GetState(system) == Core::State::Running && !s_resync_waiting;
    if (c != last || !watch)
    {
      last = c;
      since = now;
      reported = false;
    }
    else if (!reported && now - since > 6000)
    {
      reported = true;
      const u8* ram = system.GetMemory().GetRAM();
      auto& ppc = system.GetPPCState();
      const char* where = s_wait_where.load();
      text = fmt::format(
          "FFNY WATCHDOG: the game has not reached its next update for 6 s (frame {}, rollback {}{}). CPU thread: {}{}; "
          "emulated pc {:08x} lr {:08x}, OS thread {:08x}, calls {}",
          Rd32(ram, ADDR_COUNTER), s_active ? "on" : "off", s_menus_on ? ", menus mode" : "",
          where ? where : "in game code (not in a wait of the rollback layer)",
          s_snd_sending ? ", sending held sound calls" : "", ppc.pc, ppc.spr[SPR_LR], Rd32(ram, 0x800000E4),
          CallChain(ram, ppc.gpr[1], 10));
    }
    if (!text.empty())
    {
      fprintf(stderr, "FFNYRB %s\n", text.c_str());
      LogLine(text);
      if (auto cb = s_report_cb)
        cb(text);
    }
  }
}

void Configure(const Settings& settings, Transport* transport)
{
  s_settings = settings;
  s_menus_on = false;
  s_real_fight_now = false;
  s_resync_waiting = false;
  s_chk_local.clear();
  s_chk_remote.clear();
  s_chk_reports = 0;
  if (settings.enabled && !s_watchdog.joinable())
  {
    s_watchdog_quit = false;
    s_watchdog = std::thread(WatchdogLoop);
  }
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
  s_force_end = false;
  s_fight_end_cb = std::move(callback);
}

void Shutdown()
{
  if (s_watchdog.joinable())
  {
    s_watchdog_quit = true;
    s_watchdog.join();
  }
  s_menus_on = false;
  s_resync_waiting = false;
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

// FFNY's file system (EA FILESYS, struct 0x804018C0): the request start 0x80244408 hands a read to the file-system
// thread 0x802490BC (priority 2, above the main thread; created at 0x8024894C; message queue 0x8040BCB8) through
// 0x802491B8 (OSSendMessage, so the thread runs at once); that thread starts DVD reads (0x80248EC8), waits for them
// (OSReceiveMessage at 0x80249140 / 0x80249164) and then calls the request-done function 0x80244880, which runs the
// request's callback. The DVD read normally completes after an emulated seek / transfer time while the main thread
// runs on, so the done step came at a moment that depended on emulated time - another game frame on each machine,
// and not again in a re-simulation: rollbacks across a finished load broke the game (23 October) and needed the
// lockstep while loading. Instant loads: when the file-system thread is about to wait for its read, the read's
// completion is moved to the current time (DVDThread::FinishPendingReadsNow), so its interrupt is taken as soon as the
// thread sleeps and the thread finishes the whole load before the main thread runs another instruction.
bool InstantDiscReads()
{
  static const bool off = std::getenv("FFNY_RB_NOINSTANTDISC") != nullptr;
  // only in rollback fights: with every read instant from the boot on, the game crashed early in the boot (OS run
  // queue broken after ~4300 reads, lab 25 October) - some boot code expects a read to take time
  return s_active && !off;
}

void RequestForceEnd(const std::string& reason)
{
  s_force_reason = reason;
  s_force_end = true;
}

void ClearForceEnd()
{
  s_force_end = false;
  s_resync_waiting = false;  // netplay calls this when the resync is done and everyone continues
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
      st.load_lockstep = std::getenv("FFNY_RB_NOLOCKSTEP") == nullptr;
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
      // lab: post-fight resync between two lab Dolphins like netplay does it (CPU stopped at the fight end, the
      // "host" saves its state with State::FFNYSaveCompressed, the other loads it, both continue).
      // FFNY_RB_RESYNC=save:FILE or load:FILE
      if (const char* rs = std::getenv("FFNY_RB_RESYNC"))
      {
        const std::string spec = rs;
        const bool save = spec.rfind("save:", 0) == 0;
        const std::string file = spec.substr(5);
        SetFightEndCallback([save, file](Core::System& sys, u32 frame) {
          sys.GetCPU().Break();
          fprintf(stderr, "FFNYRB LAB RESYNC: fight end at frame %u, %s %s\n", frame, save ? "saving" : "loading",
                  file.c_str());
          std::thread([&sys, save, file, frame] {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            if (save)
            {
              std::vector<u8> st;
              bool ok = false;
              Common::Event done;
              Core::RunOnCPUThread(sys, [&] {
                ok = State::FFNYSaveCompressed(sys, st);
                done.Set();
              });
              done.Wait();
              if (FILE* f = fopen((file + ".tmp").c_str(), "wb"))
              {
                fwrite(st.data(), 1, st.size(), f);
                fclose(f);
                rename((file + ".tmp").c_str(), file.c_str());
              }
              fprintf(stderr, "FFNYRB LAB RESYNC: saved %zu bytes ok %d crc %08x\n", st.size(), ok,
                      Common::ComputeCRC32(st.data(), st.size()));
            }
            else
            {
              FILE* f = nullptr;
              for (int i = 0; i < 6000 && !(f = fopen(file.c_str(), "rb")); i++)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
              std::vector<u8> st;
              if (f)
              {
                fseek(f, 0, SEEK_END);
                st.resize(static_cast<size_t>(ftell(f)));
                fseek(f, 0, SEEK_SET);
                if (fread(st.data(), 1, st.size(), f) != st.size())
                  st.clear();
                fclose(f);
              }
              bool ok = false;
              std::string where;
              Common::Event done;
              Core::RunOnCPUThread(sys, [&] {
                ok = !st.empty() && State::FFNYLoadCompressed(sys, st.data(), st.size(), &where);
                done.Set();
              });
              done.Wait();
              fprintf(stderr, "FFNYRB LAB RESYNC: loaded %zu bytes ok %d crc %08x (stopped after: %s)\n", st.size(),
                      ok, Common::ComputeCRC32(st.data(), st.size()), ok ? "-" : where.c_str());
            }
            sys.GetCPU().Continue();
            fprintf(stderr, "FFNYRB LAB RESYNC: continuing after frame %u\n", frame);
          }).detach();
        });
      }
      fprintf(stderr, "FFNYRB: lab mode local %x active %x delay %u udp %s lag %d jitter %d stress %u\n", lp, ap,
              st.input_delay, u ? u : "-", lag, jit, s_stress);
    }
  }
  if (s_settings.enabled)
  {
    // log changes of the fight-end markers (24 October): the fight loop state 0x803F1000, the fight flags 0x803F1124
    // (bits kept from frame to frame: 0x8 = decided, 0x20 / 0x40 = fight loop's state machine stopped), the
    // "lost" bits +0x932 of fighters 1 / 2 and the match rule 0x803F0DA0. The screen counter 0x803EA56C logged by
    // fbf9fca turned out to be a timer (+0x100 about every 2.5 s) - not logged any more.
    static u32 prev = 0xFFFFFFFF, prev_flags = 0xFFFFFFFF, prev_lost = 0xFFFFFFFF, prev_rule = 0xFFFFFFFF, logged = 0;
    const u8* r = system.GetMemory().GetRAM();
    if (r && logged < 400)
    {
      const u32 fl = Rd32(r, ADDR_FIGHT), flags = Rd32(r, ADDR_FIGHT_FLAGS) & 0x17F, rule = Rd32(r, 0x803F0DA0);
      const u32 lost = Rd32(r, 0x803F1144 + 0x930) << 16 | (Rd32(r, 0x803F1144 + 0x12F8 + 0x930) & 0xFFFF);
      if (fl != prev || flags != prev_flags || lost != prev_lost || rule != prev_rule)
      {
        char b[200];
        snprintf(b, sizeof b,
                 "state: fight loop %u (was %d), fight flags %03x (was %x), lost bits P1 %04x P2 %04x, match rule %u, "
                 "frame %u, rollback %s",
                 fl, static_cast<int>(prev), flags, prev_flags, lost >> 16, lost & 0xFFFF, rule, Rd32(r, ADDR_COUNTER),
                 s_active ? "on" : "off");
        LogLine(b);
        logged++;
        prev = fl;
        prev_flags = flags;
        prev_lost = lost;
        prev_rule = rule;
      }
    }
  }
  if (s_active)
  {
    // finished file requests between two updates (the update-start sampling alone misses short ones)
    const u8* r = system.GetMemory().GetRAM();
    const u32 cur = Rd32(r, 0x804018D4);
    const bool busy = cur || Rd32(r, 0x804018E0);
    if (busy && FsBusyForLogic(r))
    {
      s_fs_last_busy = std::max(s_fs_last_busy, Rd32(r, ADDR_COUNTER));
      s_fs_real_busy = std::max(s_fs_real_busy, Rd32(r, ADDR_COUNTER));
    }
    else if (busy)
    {
      s_fs_stream_last = std::max(s_fs_stream_last, Rd32(r, ADDR_COUNTER));
    }
    if (busy && !s_fs_was_busy)
      s_fs_episodes++;
    s_fs_was_busy = busy;
    if (s_fs_prev_cur && cur != s_fs_prev_cur)
    {
      s_fs_done++;
      s_fs_done_frame = std::max(s_fs_done_frame, Rd32(r, ADDR_COUNTER));
    }
    s_fs_prev_cur = cur;
  }
  if (s_udp)
    s_udp->Flush();  // lab: delayed messages also go out when nothing else is sent (e.g. after the fight end)
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
  const u32 rbt = Rd32(ram, 0x8049A408);
  if (hook < 0x8049A410 || hook >= 0x8049B400 || (rbt != 0x52425435 && rbt != 0x52425436))  // "RBT5" / "RBT6"
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
  if (Rd32(ram, 0x801D7EE0) == 0x93EB00B8)  // stw r31, 0xB8(r11)
    HLE::Patch(system, 0x801D7EE0, "FFNYRollbackSndCtx");
  if (Rd32(ram, 0x8001841C) == 0x807F0004)  // lwz r3, 4(r31) in the animation-by-name lookup 0x80018408
    HLE::Patch(system, 0x8001841C, "FFNYRollbackBankGuard");
  if (Rd32(ram, 0x80274180) == 0x9421FFC0)  // stwu sp,-0x40(sp): the SDK's unhandled-exception handler
    HLE::Patch(system, 0x80274180, "FFNYRollbackCrash");
  // rbtag v5's two reads of 0x803F1000 -> W_FIGHTW (see there)
  if (Rd32(ram, 0x8049A670) == 0x3C60803F && Rd32(ram, 0x8049A694) == 0x80631000 &&
      Rd32(ram, 0x8049A8B0) == 0x3C60803F && Rd32(ram, 0x8049A8B8) == 0x60631000)
  {
    u8* wram = memory.GetRAM();
    Wr32(wram, W_FIGHTW, 0);
    Wr32(wram, 0x8049A670, 0x3C60804A);  // lis r3,0x804A
    Wr32(wram, 0x8049A694, 0x8063B5F4);  // lwz r3,-0x4A0C(r3) = 0x8049B5F4
    Wr32(wram, 0x8049A8B0, 0x3C608049);  // lis r3,0x8049
    Wr32(wram, 0x8049A8B8, 0x6063B5F4);  // ori r3,r3,0xB5F4
    s_rbtag_redirected = true;
  }
  else if (rbt == 0x52425436)
  {
    // rbtag v6 reads W_FIGHTW itself (and keeps the main thread's frees of its tagged main-heap blocks in a list that
    // is part of the rollback state, done 12 logic frames later)
    Wr32(memory.GetRAM(), W_FIGHTW, 0);
    s_rbtag_redirected = true;
  }
  if (Rd32(ram, OS_RECV_SLEEP) == 0x48002839)  // bl 0x802776E8
  {
    HLE::Patch(system, OS_RECV_SLEEP, "FFNYRollbackFsWait");
    HLE::Patch(system, 0x80244408, "FFNYRollbackFsKick");
  }
  s_async_on = std::getenv("FFNY_RB_NOASYNCLOCK") == nullptr && Rd32(ram, 0x80274D94) == 0x7C0802A6;
  if (s_async_on)
    HLE::Patch(system, 0x80274D94, "FFNYRollbackAsyncSend");
  s_card_on = std::getenv("FFNY_RB_NOCARDLOCK") == nullptr;
  if (s_card_on)
  {
    for (size_t i = 0; i < CARD_FUNCS.size(); i++)
    {
      if (Rd32(ram, CARD_FUNCS[i]) != 0x7C0802A6)
      {
        s_card_on = false;
        fprintf(stderr, "FFNYRB: memory-card function %08x not as expected - card lockstep off\n", CARD_FUNCS[i]);
        break;
      }
    }
    if (s_card_on)
      for (size_t i = 0; i < CARD_FUNCS.size(); i++)
        HLE::Patch(system, CARD_FUNCS[i], "FFNYRollbackCard" + std::to_string(i));
  }
  // deterministic file ids (see HookAfSetId)
  s_af_on = std::getenv("FFNY_RB_NOAFID") == nullptr && Rd32(ram, AF_SETID) == 0x812D8DCC &&
            Rd32(ram, AF_FIND) == 0x7C6A1B78 && Rd32(ram, AF_RELEASE) == 0x9421FFF0;
  if (s_af_on)
  {
    HLE::Patch(system, AF_SETID, "FFNYRollbackAfSetId");
    HLE::Patch(system, AF_FIND, "FFNYRollbackAfFind");
    HLE::Patch(system, AF_RELEASE, "FFNYRollbackAfRelease");
    if (Rd32(ram, 0x802450D8) == 0x7C0803A6 && Rd32(ram, 0x8049CFE0) == 0)
    {
      u8* w = memory.GetRAM();
      memcpy(w + (0x8049CFE0 & 0x01FFFFFF), "LOADBUF", 8);
      HLE::Patch(system, 0x802450D8, "FFNYRollbackAfBufAlloc");
    }
  }
  if (const char* c = std::getenv("FFNY_RB_COUNT"))
  {
    std::stringstream ss(c);
    std::string t;
    int i = 0;
    while (std::getline(ss, t, ',') && i < 32)
    {
      s_count_addr[i] = u32(std::stoul(t, nullptr, 16));
      HLE::Patch(system, s_count_addr[i], "FFNYRollbackCount" + std::to_string(i));
      i++;
    }
  }
  if (const char* c = std::getenv("FFNY_RB_TRACE"))
  {
    std::stringstream ss(c);
    std::string t;
    int i = 0;
    while (std::getline(ss, t, ',') && i < 4)
    {
      s_trace_addr[i] = u32(std::stoul(t, nullptr, 16));
      HLE::Patch(system, s_trace_addr[i], "FFNYRollbackTrace" + std::to_string(i));
      i++;
    }
  }
  // sound calls held back until their frame is confirmed (see HookSndPost)
  s_snd_trace = std::getenv("FFNY_RB_SNDTRACE") != nullptr;
  s_vtb_mode = (std::getenv("FFNY_RB_ALWAYS") != nullptr || s_menus) && std::getenv("FFNY_RB_NOVTB") == nullptr;

  s_snd_hold_on = std::getenv("FFNY_RB_NOSNDHOLD") == nullptr;
  if (Rd32(ram, SND_POST) == 0x9421FF60 && Rd32(ram, SND_SERVICE) == 0x9421FFF0 && (s_snd_hold_on || s_snd_trace))
  {
    HLE::Patch(system, SND_POST, "FFNYRollbackSndPost");
    HLE::Patch(system, SND_SERVICE, "FFNYRollbackSndService");
    s_snd_defer = true;
  }
  s_music_hold = s_snd_hold_on && std::getenv("FFNY_RB_NOMUSICHOLD") == nullptr;
  if (Rd32(ram, SND_MUSIC) == 0x9421FFF8 && (s_music_hold || s_snd_trace))
    HLE::Patch(system, SND_MUSIC, "FFNYRollbackSndMusic");
  if (Rd32(ram, SND_MUSIC) == 0x9421FFF8 && Rd32(ram, 0x8001112C) == 0x8001000C && s_music_hold && !s_music_hold_old &&
      std::getenv("FFNY_RB_MUSICREALRESULT") == nullptr)
    HLE::Patch(system, 0x8001112C, "FFNYRollbackSndMusicResult");
  for (u32 i = 0; i < 5; i++)
    if ((Rd32(ram, SEED_TB_SITES[i]) & 0xFC1FFFFE) == 0x7C0C42E6 && std::getenv("FFNY_RB_NOSEEDTB") == nullptr)  // mftbl
      HLE::Patch(system, SEED_TB_SITES[i], "FFNYRollbackSeedTb" + std::to_string(i));
  for (u32 i = 0; i < 2; i++)
    if ((Rd32(ram, APT_TB_SITES[i]) & 0xFC1FFFFE) == 0x7C0C42E6 && s_vtb_mode && std::getenv("FFNY_RB_NOAPTTB") == nullptr)
      HLE::Patch(system, APT_TB_SITES[i], "FFNYRollbackAptTb" + std::to_string(i));
  for (u32 i = 0; i < 2; i++)
    if (Rd32(ram, SONG_PICK_SITES[i]) == (0x48000001u | ((0x8049A200u - SONG_PICK_SITES[i]) & 0x03FFFFFCu)) &&
        std::getenv("FFNY_RB_NOSONGPICK") == nullptr)  // bl 0x8049A200 (rbaudrng)
      HLE::Patch(system, SONG_PICK_SITES[i], "FFNYRollbackSongPick" + std::to_string(i));
  for (const u32 a : {0x80011164u})
    if ((Rd32(ram, a) >> 16) == 0x9421 && (s_music_hold || s_snd_trace))
      HLE::Patch(system, a, "FFNYRollbackSndExtra");
  if (s_snd_trace)
  {
    HLE::Patch(system, 0x801D3198, "FFNYSndTrAccept");  // r3 = 0x801D2F08's result, *(r31) = event key
    HLE::Patch(system, 0x801D5B2C, "FFNYSndTrStart");   // r3 = request slot started by the service
    HLE::Patch(system, 0x801D347C, "FFNYSndTrEvict");   // r3 = request slot discarded
    HLE::Patch(system, 0x80011124, "FFNYSndTrMusRes");  // r3 = MPF post-event result of the music cue send
  }
  if (!s_snd_defer)
    s_snd_hold_on = s_music_hold = false;
  Report(std::string("FFNY: sound hold ") + SndModeName());
  if (std::getenv("FFNY_RB_NOSYSFREE") == nullptr)
  {
    s_sys_alloc_hook = hook;
    HLE::Patch(system, hook, "FFNYRollbackSysAlloc");
  }
  fprintf(stderr, "FFNYRB: rbtag fight flag %s, instant disc reads in rollback fights %s\n",
          s_rbtag_redirected ? "redirected" : "NOT redirected",
          std::getenv("FFNY_RB_NOINSTANTDISC") ? "off" : "on");
  s_match_hook = Rd32(ram, ADDR_MATCH_UPDATE) == 0x9421FFD0;  // stwu sp,-0x30(sp)
  if (s_match_hook)
  {
    Wr32(memory.GetRAM(), W_MATCH, 0);
    HLE::Patch(system, ADDR_MATCH_UPDATE, "FFNYRollbackMatch");
  }
  s_rng_redirect = Rd32(ram, 0x8049A200) == 0x3C60804A && Rd32(ram, 0x8049A204) == 0x3863A210 &&
                   std::getenv("FFNY_RB_NORNGFIX") == nullptr;
  if (const char* w = std::getenv("FFNY_RB_WATCHW"))
  {
    TMemCheck mc;
    mc.start_address = u32(strtoul(w, nullptr, 16));
    mc.end_address = mc.start_address + 3;
    mc.is_ranged = true;
    mc.is_break_on_write = true;
    mc.log_on_hit = true;
    mc.break_on_hit = false;
    system.GetPowerPC().GetMemChecks().Add(std::move(mc));
    fprintf(stderr, "FFNYRB: write watch on %08x\n", mc.start_address);
  }
  if (std::getenv("FFNY_RB_SEEDLOG"))
    HLE::Patch(system, 0x800B7D38, "FFNYRollbackSeedLog");
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
  if (s_active && le && thread == 0x80432D10)
  {
    // fresh data converted by the game logic. Only data OUTSIDE the rollback state matters: data in a rewound heap
    // block is unconverted again after a rewind and the game converts it again (seen in the lab, harmless); data in
    // a block that is not rewound (rbtag-tagged, e.g. SysHeap) stays converted while the objects that got filled
    // from it are rewound - the game never converts it again (KO-scene crash, 23 October).
    const auto blk = HeapBlockOf(ram, data);
    const bool rewound = blk.first != 0 && (blk.second >> 16) == 0x424D && !(blk.second & 0x80);
    if (!rewound)
    {
      const u32 cf = Rd32(ram, ADDR_COUNTER);
      if (s_resim_target == 0)
      {
        s_conv_out++;
        s_conv_gap_max = std::max(s_conv_gap_max, cf > s_fs_real_busy ? cf - s_fs_real_busy : 0);
      }
      s_conv_frame = std::max(s_conv_frame, cf);
      // With instant loads the data comes from a read that finished inside the logic frame that asked for it: a
      // rewind before that frame reads it again, a rewind between the read and the conversion converts it again
      // (loader fix: original bytes restored) - no lockstep needed (lab 25 October: stress rollbacks on every frame,
      // no lockstep, whole fight to the KO: 0 mismatches, 0 empty-bank lookups)
      if (!InstantDiscReads())
        s_fs_last_busy = std::max(s_fs_last_busy, s_conv_frame);
    }
  }
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

bool OnMemCheck(Core::System& system, u32 addr, u64 value, u32 pc, bool write)
{
  if (!std::getenv("FFNY_RB_WATCHW"))
    return false;
  const u8* ram = system.GetMemory().GetRAM();
  if (!s_active)
    return true;
  static u32 n = 0;
  if (n++ > 200000)
    return true;
  auto& ppc = system.GetPPCState();
  char chain[200] = {};
  {
    u32 sp = ppc.gpr[1];
    size_t o = 0;
    for (int k = 0; k < 10 && (sp & 0xFE000000) == 0x80000000; k++)
    {
      const u32 nsp = Rd32(ram, sp);
      if ((nsp & 0xFE000000) != 0x80000000)
        break;
      o += snprintf(chain + o, sizeof chain - o, " %08x", Rd32(ram, nsp + 4));
      sp = nsp;
      if (o > 180)
        break;
    }
  }
  fprintf(stderr, "MEMW frame %u%s phase %c thread %08x pc %08x lr %08x %08x <- %08x chain%s\n", Rd32(ram, ADDR_COUNTER),
          s_resim_target ? " resim" : "", s_phase, Rd32(ram, 0x800000E4), pc, ppc.spr[SPR_LR], addr, u32(value), chain);
  return true;
}

// lab (FFNY_RB_SEEDLOG): generator (re)seeding 0x800B7D38 (r3 generator, r4 / r5 seed)
void HookSeedLog(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  static u32 n = 0;
  if (n++ > 2000)
    return;
  auto ok = [](u32 a) { return (a & 0xFE000000) == 0x80000000; };
  const u32 sp = ppc.gpr[1];
  const u32 c1 = ok(sp) ? Rd32(ram, sp) : 0, b1 = ok(c1) ? Rd32(ram, c1 + 4) : 0;
  const u32 c2 = ok(c1) ? Rd32(ram, c1) : 0, b2 = ok(c2) ? Rd32(ram, c2 + 4) : 0;
  fprintf(stderr, "SEED frame %u%s phase %c active %d thread %08x gen %08x seed %08x %08x lr %08x bt %08x %08x\n",
          Rd32(ram, ADDR_COUNTER), s_resim_target ? " resim" : "", s_phase, s_active, Rd32(ram, 0x800000E4),
          ppc.gpr[3], ppc.gpr[4], ppc.gpr[5], ppc.spr[SPR_LR], b1, b2);
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

// Sound library stop function 0x801D7DD0: when nothing of the handle's group is active it clears two fields of
// the current context object (pointer 0x802FE714) - which is null when the game's (rewound) sound handle no
// longer matches the sound library's (not rewound) voice tables after rollbacks. Lab: crash ~10 s after a
// rollback fight on one of two machines (invalid write to 0xB8 at 0x801D7EE0, called via 0x8001108C). Replaces
// "stw r31, 0xB8(r11)": with a null context both stores are skipped.
u32 s_snd_null_ctx = 0;
void HookSndNullContext(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u32 ctx = ppc.gpr[11];
  if ((ctx & 0xFE000000) != 0x80000000)
  {
    if (s_snd_null_ctx++ < 5)
      fprintf(stderr, "FFNYRB: sound stop with null context (%08x) skipped, frame %u\n", ctx,
              Rd32(system.GetMemory().GetRAM(), ADDR_COUNTER));
    ppc.npc = 0x801D7EE8;  // after both stores
    return;
  }
  Wr32(system.GetMemory().GetRAM(), ctx + 0xB8, ppc.gpr[31]);
  ppc.npc = 0x801D7EE4;
}

// Animation-by-name lookup 0x80018408(bank, name): reads the bank's name table at +4 (binary search 0x800F7294).
// Robert's netplay test (23 October, ping 100 / jitter 20 / loss 2 % / jumps): crash "invalid read from 4 at
// 0x800F72AC" at the KO, i.e. a bank whose table pointer is 0 (unloaded / never filled in this timeline; likely a file
// finished loading in a frame a rollback threw away). Replaces "lwz r3, 4(r31)": an empty bank answers "not found"
// (the function's own -1 path returns 0) instead of crashing, and the first case per fight is reported with the
// caller. The original game would crash here too.
void HookBankGuard(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  const u32 bank = ppc.gpr[31];
  const u32 table = (bank & 0xFE000000) == 0x80000000 ? Rd32(ram, bank + 4) : 0;
  if (table != 0)
  {
    ppc.gpr[3] = table;
    ppc.npc = 0x80018420;
    return;
  }
  if (s_bank_guard++ == 0)
  {
    char b[300];
    snprintf(b, sizeof b,
             "FFNY WARNING: animation lookup in an empty bank %08x (name at %08x) skipped at frame %u%s; called "
             "from %s; last rollback %u->%u, last finished file request in frame %u, last file conversion in frame "
             "%u",
             bank, ppc.gpr[4], Rd32(ram, ADDR_COUNTER), s_resim_target ? " (re-simulating)" : "",
             CallChain(ram, ppc.gpr[1], 6).c_str(), s_last_rb_from, s_last_rb_to, s_fs_done_frame, s_conv_frame);
    Report(b);
  }
  ppc.gpr[3] = 0;
  ppc.npc = 0x8001843C;  // epilogue with r3 = 0 ("not found")
}

void HookRender(const Core::CPUThreadGuard& guard)
{
  s_phase = 'R';
}

// 0x80244408 (HookType::Start): a file-system request is queued / started (r3 = request node) by the thread that
// wants the data
void HookFsKick(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  const u8* ram = system.GetMemory().GetRAM();
  u32 thread = Rd32(ram, 0x800000E4);
  if (thread == FS_THREAD)
  {
    // a follow-up request made by the file-system thread itself (in the callback of the request it is serving):
    // it belongs to whoever made that request
    const auto it = s_fs_req_thread.find(Rd32(ram, 0x804018D4));
    if (it != s_fs_req_thread.end())
      thread = it->second;
  }
  s_fs_req_thread[u32(system.GetPPCState().gpr[3])] = thread;
  // the game logic asks for a file while the file system still serves another thread's request (the music / speech
  // stream's reads take emulated disc time): the logic's request would wait behind it for a number of frames that
  // depends on the audio timing - another frame in a re-simulation, another frame on the other machine. The reads in
  // the way are finished at once (HookFsWait) until the file system gets to the logic's request.
  if (thread == 0x80432D10 && InstantDiscReads() && !s_fs_hurry_off)
  {
    s_fs_main_queued = true;
    const u8* r2 = system.GetMemory().GetRAM();
    const u32 cur = Rd32(r2, 0x804018D4);
    const auto it2 = s_fs_req_thread.find(cur);
    // only the audio thread's stream reads: hurrying the fight loader's (0x806D8AE0) reads hung the fight loading
    // (lab, counter stuck at 9061 even without rollbacks); the loader is a worker thread (lockstep while busy)
    if (cur && it2 != s_fs_req_thread.end() && it2->second == AUDIO_THREAD)
    {
      system.GetDVDThread().FinishPendingReadsNow();
      s_fs_hurried++;
    }
  }
}

void HookFsWait(const Core::CPUThreadGuard& guard)
{
  // HookType::Start at OS_RECV_SLEEP: a thread is about to sleep in OSReceiveMessage (r31 = queue). When it is the
  // file-system thread waiting for its DVD read, the read completes now: its interrupt is taken right after the switch
  // to the next thread (interrupts are off until then), and the file-system thread runs again before that thread
  // executes anything - like a real read, only without the wait in between.
  if (!InstantDiscReads())
    return;
  auto& system = guard.GetSystem();
  const u8* ram = system.GetMemory().GetRAM();
  if (system.GetPPCState().gpr[31] != FS_QUEUE || Rd32(ram, 0x800000E4) != FS_THREAD)
    return;
  // only reads of requests the game logic (main thread) made: the music / speech streams also read through the
  // file-system thread (requests from the audio thread); finishing those at once broke the audio thread at song
  // changes (lab 26 October, rollback on in the menus: crash at 0x801E03FC when the menu song made way for the
  // loading song)
  const u32 node = Rd32(ram, 0x804018D4);
  const auto it = s_fs_req_thread.find(node);
  const u32 requester = it == s_fs_req_thread.end() ? 0 : it->second;
  if (std::getenv("FFNY_RB_FSDEBUG"))
    fprintf(stderr, "FSWAIT queue count %u state %u node %08x requester %08x\n", Rd32(ram, FS_QUEUE + 0x1C),
            Rd32(ram, 0x80401940), node, requester);
  static const u32 extra_req = std::getenv("FFNY_RB_FSEXTRA") ? u32(std::strtoul(std::getenv("FFNY_RB_FSEXTRA"), nullptr, 16)) : 0;
  if (requester == 0x80432D10)
    s_fs_main_queued = false;  // the file system serves the logic's request now
  else if (s_fs_main_queued && requester == AUDIO_THREAD)
    s_fs_hurried++;  // a stream read in the way of a waiting request of the logic (see HookFsKick)
  else if (!(extra_req && requester == extra_req))
  {
    s_fs_not_main++;
    return;
  }
  system.GetDVDThread().FinishPendingReadsNow();
  s_fs_instant++;
}

// ---- Deterministic file ids for rewound game code (menu rollback, 26 October) ----
// FFNY's async file objects (EA ASYNCFILE: pool of 0x30-byte objects, array *(0x8043FBB8), count 0x804405A0, free
// list head / tail 0x804405A4 / 0x804405A8 (a FIFO: freed objects go to the tail), generation 0x804405AC) live in
// system memory that rollback does not rewind. Open = 0x80245334 (the file system allocates the buffer) /
// 0x802453E4 (caller's buffer, used by 0x8000F064); both give the object its id with 0x80244DD0 (generation + 0x100,
// or'd with the pool index) and return it. Every other call finds the object by id with 0x80244E00 (set callback
// 0x80245494, done? 0x80245510, result + release 0x80245578, cancel 0x8024569C); 0x80244E48 returns an object to
// the pool. With instant disc reads a load of the main thread is finished before the open returns.
// Without help a re-simulated open got another id (generation not rewound, and the FIFO hands out another object)
// that the game keeps in rewound memory (loader slots 0x80350520, music 0x804393AC, Apt 0x81018A80): the re-simulated
// state differed, the first run's object leaked, and a release made in a thrown-away frame was missing afterwards.
// Now an open made by the game logic during rollback gets a VIRTUAL id 0xA5ccccHH (c = a counter in rewound memory,
// W_AFCTR, so every machine and every re-simulation hands out the same ids in the same order; H = a hash byte of the
// path, for the logs); the object stores it, the lookup finds such ids by scanning the pool. A release by the game
// logic is only noted (the object stays as it is) and done for real once its frame can no longer be rolled back;
// a rewind takes back the releases of the thrown-away frames and returns the objects they opened to the pool.
struct AfEntry
{
  u32 obj, id, open_frame, freed_frame;  // freed_frame 0xFFFFFFFF = in use
};
std::vector<AfEntry> s_af;
u32 s_af_opened = 0, s_af_orphans = 0, s_af_unfreed = 0, s_af_freed = 0, s_af_busy_orphans = 0;

static bool AfVirtual(u32 id)
{
  return (id >> 24) == 0xA5;
}

// the object goes back to the pool (0x80244E48 + 0x80244D08 without the file system's own free of a cancelled,
// self-allocated buffer: only reached for objects of thrown-away frames, whose buffer the rewind took back anyway)
static void AfRealFree(u8* ram, u32 obj)
{
  const u32 arr = Rd32(ram, AF_ARRAY);
  const u32 idx = (obj - arr) / 0x30;
  if (Rd32(ram, obj + 0x1C) != 0)
  {
    // still busy (not a main-thread load?): cancel it; the completion returns it to the pool (0x80244EB4)
    Wr32(ram, obj + 0x10, 1);
    Wr32(ram, obj + 0xC, 0);
    Wr32(ram, obj + 0, idx);
    s_af_busy_orphans++;
    return;
  }
  Wr32(ram, obj + 0, idx);
  Wr32(ram, obj + 0x1C, 0);
  if (Rd32(ram, AF_FREE) == 0)
    Wr32(ram, AF_FREE, obj);
  else
    Wr32(ram, Rd32(ram, AF_FREE + 4) + 4, obj);
  Wr32(ram, AF_FREE + 4, obj);
  Wr32(ram, obj + 4, 0);
  s_af_freed++;
}

// pool mutex 0x80401914 held by another thread? (then the pool is left alone this frame)
static bool AfPoolLocked(const u8* ram)
{
  return Rd32(ram, 0x80401914 + 8) != 0 && Rd32(ram, 0x80401914 + 8) != 0x80432D10;
}

// PRE: releases of frames that can no longer be rolled back are done for real
static void AfConfirm(u8* ram, u32 upto)
{
  if (s_af.empty() || AfPoolLocked(ram))
    return;
  for (size_t i = 0; i < s_af.size();)
  {
    const AfEntry& e = s_af[i];
    if (e.freed_frame != 0xFFFFFFFF && e.freed_frame <= upto)
    {
      if (Rd32(ram, e.obj) == e.id)
        AfRealFree(ram, e.obj);
      s_af.erase(s_af.begin() + i);
    }
    else if (e.freed_frame == 0xFFFFFFFF && e.open_frame <= upto && s_af.size() > 64)
      s_af.erase(s_af.begin() + i);  // long-lived, confirmed: no rewind can take its open back any more
    else
      i++;
  }
}

// a rewind to frame s: frames >= s are thrown away
static void AfRewind(u8* ram, u32 s)
{
  for (size_t i = 0; i < s_af.size();)
  {
    AfEntry& e = s_af[i];
    if (e.open_frame >= s)
    {
      if (Rd32(ram, e.obj) == e.id)
        AfRealFree(ram, e.obj);
      s_af_orphans++;
      s_af.erase(s_af.begin() + i);
      continue;
    }
    if (e.freed_frame != 0xFFFFFFFF && e.freed_frame >= s)
    {
      e.freed_frame = 0xFFFFFFFF;
      s_af_unfreed++;
    }
    i++;
  }
}

// rollback ends: every noted release is done (the frames are all confirmed by then, or the host's state replaces ours)
static void AfEnd(u8* ram)
{
  for (const AfEntry& e : s_af)
    if (e.freed_frame != 0xFFFFFFFF && Rd32(ram, e.obj) == e.id)
      AfRealFree(ram, e.obj);
  s_af.clear();
}

static bool AfLogicCall(const u8* ram)
{
  return s_active && s_af_on && s_phase == 'U' && Rd32(ram, 0x800000E4) == 0x80432D10;
}

// 0x80244DD0 (Replace; r3 = object): give the new object its id
void HookAfSetId(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  u8* ram = system.GetMemory().GetRAM();
  if (AfLogicCall(ram))
  {
    const u32 obj = ppc.gpr[3];
    // the path: r29 in 0x80245334, r28 in 0x802453E4
    const u32 lr = ppc.spr[SPR_LR];
    const u32 path = lr == 0x80245368 ? ppc.gpr[29] : lr == 0x8024541C ? ppc.gpr[28] : 0;
    u32 h = 0;
    if ((path & 0xFE000000) == 0x80000000)
      for (u32 k = 0; k < 128; k++)
      {
        const u8 c = ram[(path + k) & 0x01FFFFFF];
        if (!c)
          break;
        h = h * 31 + c;
      }
    const u32 c = (Rd32(ram, W_AFCTR) + 1) & 0xFFFF;
    Wr32(ram, W_AFCTR, c);
    const u32 id = 0xA5000000 | (c << 8) | ((h ^ (h >> 8) ^ (h >> 16)) & 0xFF);
    Wr32(ram, obj, id);
    const u32 f = Rd32(ram, W_FRAME);
    s_af.push_back({obj, id, f, 0xFFFFFFFF});
    s_af_opened++;
    if (s_log)
    {
      char name[64] = {};
      for (u32 k = 0; k < 63 && path; k++)
        if (!(name[k] = char(ram[(path + k) & 0x01FFFFFF])))
          break;
      fprintf(stderr, "FFNYRB AFOPEN frame %u%s: %08x obj %08x %s\n", f, s_resim_target ? " (resim)" : "", id, obj, name);
    }
    ppc.npc = lr;
    return;
  }
  ppc.gpr[9] = Rd32(ram, 0x804405AC);  // lwz r9,-29236(r13)
  ppc.npc = AF_SETID + 4;
}

// 0x802450D8 (Replace: mtspr lr,r0 before the call of the file system's buffer allocator *(0x80401904); r3 = name
// "ASYNCFILEBUF", r31 = object). rbtag counts that name as system memory: tagged (left out of the rollback state) and,
// while rollback runs, put in the not rewound SysHeap. A buffer of a virtual-id load (the game logic's, finished
// inside its frame) is game data like any other: it gets an untagged name.
constexpr u32 AF_BUFALLOC = 0x802450D8, AF_BUFNAME = 0x8049CFE0;
void HookAfBufAlloc(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  if (AfVirtual(Rd32(ram, ppc.gpr[31])))
    ppc.gpr[3] = AF_BUFNAME;
  ppc.spr[SPR_LR] = ppc.gpr[0];
  ppc.npc = AF_BUFALLOC + 4;
}

// 0x80244E00 (Replace; r3 = id -> object or 0)
void HookAfFind(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u32 id = ppc.gpr[3];
  if (AfVirtual(id))
  {
    const u8* ram = system.GetMemory().GetRAM();
    const u32 arr = Rd32(ram, AF_ARRAY), n = std::min<u32>(Rd32(ram, AF_COUNT), 256);
    u32 found = 0;
    for (u32 i = 0; i < n && !found; i++)
      if (Rd32(ram, arr + i * 0x30) == id)
        found = arr + i * 0x30;
    ppc.gpr[3] = found;
    ppc.npc = ppc.spr[SPR_LR];
    return;
  }
  ppc.gpr[10] = id;  // mr r10,r3
  ppc.npc = AF_FIND + 4;
}

// 0x80244E48 (Replace; r3 = object): back to the pool
void HookAfRelease(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  u8* ram = system.GetMemory().GetRAM();
  const u32 obj = ppc.gpr[3];
  const u32 id = Rd32(ram, obj);
  if (AfVirtual(id))
  {
    auto it = std::find_if(s_af.begin(), s_af.end(), [&](const AfEntry& e) { return e.obj == obj && e.id == id; });
    if (AfLogicCall(ram))
    {
      // noted only; done once the frame is confirmed (AfConfirm) or taken back by a rewind (AfRewind)
      const u32 f = Rd32(ram, W_FRAME);
      if (it == s_af.end())
        s_af.push_back({obj, id, 0, f});
      else
        it->freed_frame = f;
      if (s_log)
        fprintf(stderr, "FFNYRB AFREL frame %u%s: %08x (noted)\n", f, s_resim_target ? " (resim)" : "", id);
      ppc.npc = ppc.spr[SPR_LR];
      return;
    }
    if (it != s_af.end())
      s_af.erase(it);
    // the pool code takes the index from the id's low byte
    Wr32(ram, obj, (obj - Rd32(ram, AF_ARRAY)) / 0x30);
  }
  // stwu r1,-16(r1)
  const u32 sp = ppc.gpr[1];
  Wr32(ram, sp - 16, sp);
  ppc.gpr[1] = sp - 16;
  ppc.npc = AF_RELEASE + 4;
}

// The match update 0x80096C04 runs once per logic update while the game is in fight mode (every match state: fight,
// pause, KO scenes, results). After "quit" in the pause menu the game goes to the menus without touching 0x803F1000 /
// 0x803F1124 (Robert, 24 October: rollback stayed on in the main menu, both Dolphins crashed in the heap code at
// 0x80249818 from 0x801A315C when going back from SELECT USER ID).
void HookMatchUpdate(const Core::CPUThreadGuard& guard)
{
  Wr32(guard.GetSystem().GetMemory().GetRAM(), W_MATCH, 1);
  s_match_updates++;
}

// The generator hooks are Replace hooks: a Start hook's change of r3 is not seen by the JIT code after it when
// the caller set r3 with constant instructions (lis / subi): the JIT keeps the constant across the call, so the
// "redirected" draw still used the shared generator (lab: 1,159 redirected render draws all wrote the shared
// generator-2 state). A Replace hook ends the JIT block, the code after it reads r3 from the PPC state. The hook
// therefore also does the function's first instruction itself and continues after it.
void HookRng1(const Core::CPUThreadGuard& guard)
{
  auto& ppc = guard.GetSystem().GetPPCState();
  if (s_active && !s_side_off && ppc.gpr[3] == GEN1_SHARED &&
      Rd32(guard.GetSystem().GetMemory().GetRAM(), 0x800000E4) != MAIN_THREAD)
  {
    // another thread (the loader's match / crowd set-up): side state, then return to the caller
    ppc.gpr[3] = Gen1Step(s_side1);
    s_side_draws[0]++;
    if (s_side_lr[0].size() < 64)
      s_side_lr[0][ppc.spr[SPR_LR]]++;
    ppc.npc = ppc.spr[SPR_LR];
    return;
  }
  if (s_active && s_rng_redirect && s_phase == 'R' && ppc.gpr[3] == GEN1_SHARED)
  {
    ppc.gpr[3] = GEN1_PRIVATE;
    s_rng_redirected++;
  }
  if (ppc.gpr[3] == GEN1_SHARED)
    RecordDraw(guard.GetSystem(), '1');
  ppc.gpr[7] = ppc.gpr[3];  // 0x8001FC88: mr r7, r3
  ppc.npc = GEN1_FN + 4;
}

static void HookRngBody(const Core::CPUThreadGuard& guard);
void HookRng(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  if (s_active && !s_side_off && ppc.gpr[3] == GEN2_SHARED &&
      Rd32(system.GetMemory().GetRAM(), 0x800000E4) != MAIN_THREAD)
  {
    // another thread: side state (see Gen1Step), result in f1 like lfsx, return to the caller
    const double v = Gen2Step(s_side2);
    ppc.ps[1].SetBoth(v, v);
    s_side_draws[1]++;
    if (s_side_lr[1].size() < 64)
      s_side_lr[1][ppc.spr[SPR_LR]]++;
    ppc.npc = ppc.spr[SPR_LR];
    return;
  }
  HookRngBody(guard);
  // 0x800B7DAC: stwu sp, -0x10(sp)
  const u32 sp = ppc.gpr[1];
  Wr32(system.GetMemory().GetRAM(), sp - 0x10, sp);
  ppc.gpr[1] = sp - 0x10;
  ppc.npc = GEN2_FN + 4;
}

static void HookRngBody(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  if (s_active)
    s_rng2_calls[s_phase == 'U' ? 0 : s_phase == 'T' ? 1 : s_phase == 'R' ? 2 : 3]
                [ppc.gpr[3] == GEN2_SHARED ? 0 : ppc.gpr[3] == GEN2_PRIVATE ? 1 : 2]++;
  // The effect manager (singleton 0x803C3230) draws from generator 2 through the wrapper 0x800B7E90 at five
  // places. Its state is driven by render code (start flag +0x4F4 written only in the render phase by
  // 0x800ECA00 from 0x800EDA24), i.e. by the frame each machine displays, which can be a predicted one: the two
  // machines started the Blazin' effect timer 9 frames apart and their fight generator diverged. The effect
  // draws are visual only: during rollback fights they all use the private generator, in every phase.
  bool effect = false;
  if (s_active && s_rng_redirect && ppc.gpr[3] == GEN2_SHARED && ppc.spr[SPR_LR] == 0x800B7EBC)
  {
    const u32 back = Rd32(ram, ppc.gpr[1]);
    const u32 site = (back & 0xFE000000) == 0x80000000 ? Rd32(ram, back + 4) : 0;
    effect = site == 0x800ECFEC || site == 0x800EDB50 || site == 0x800EE098 || site == 0x800EE0C8 ||
             site == 0x800EE0F4;
  }
  if (s_active && s_rng_redirect && (s_phase == 'R' || effect) && ppc.gpr[3] == GEN2_SHARED)
  {
    ppc.gpr[3] = GEN2_PRIVATE;
    s_rng_redirected++;
    if (effect)
      s_rng_effect++;
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

// ---- Diagnostics for the two open problems, always on during rollback fights (reported in the netplay chat) ----
// (1) the rare allocator crash = a heap's free list holds a garbage link; this finds the broken list early,
//     tells which heap, and whether it was found right after a rewind (restore problem) or after a normal update
//     (double free / use after free by the game or the sound library).
// (2) rbtag's logs (0x8049B510.. counts, 96 entries each; deferred main-heap frees 0x8049B400, 63): when a log is
//     full, rbtag does the release / free AT ONCE instead of delaying it - a later rewind then makes the
//     re-simulation release / free it a second time (sound-library use after free, SndHeap double free).
static std::string FreeListProblem(const u8* ram, bool skip_locked = false)
{
  auto ok = [](u32 a) { return (a & 0xFE000000) == 0x80000000 && (a & 0x01FFFFFF) < 0x01800000; };
  for (u32 i = 0; i < 32; i++)
  {
    const u32 h = Rd32(ram, 0x8040BD70 + 4 * i);
    if (!ok(h))
      continue;
    if (skip_locked && Rd32(ram, h + 0x44) != 0)
      continue;  // another thread is inside this heap's allocator (its mutex at +0x3C has an owner): may be mid-update
    const u32 base = Rd32(ram, h + 8), end = Rd32(ram, h + 0xC);
    u32 prev = h + 0x10, n = Rd32(ram, h + 0x20), k = 0;
    while (n != h + 0x10 && k++ < 20000)  // ends at the 'BS' sentinel inside the descriptor
    {
      if (!ok(n) || n < base || n >= end || (Rd32(ram, n) >> 16) != 0x4246 || Rd32(ram, n + 0x14) != prev)
      {
        char b[200];
        snprintf(b, sizeof b, "heap %u (%08x-%08x): free node %u = %08x (word %08x, back link %08x) after %08x", i,
                 base, end, k, n, ok(n) ? Rd32(ram, n) : 0, ok(n) ? Rd32(ram, n + 0x14) : 0, prev);
        return b;
      }
      prev = n;
      n = Rd32(ram, n + 0x10);
    }
  }
  return {};
}

static u32 HeapFree(const u8* ram, u32 i, u32* largest = nullptr, u32* nodes = nullptr)
{
  const u32 h = Rd32(ram, 0x8040BD70 + 4 * i);
  if ((h & 0xFE000000) != 0x80000000)
    return 0;
  u32 n = Rd32(ram, h + 0x20), k = 0, total = 0, big = 0;
  while (n != h + 0x10 && (n & 0xFE000000) == 0x80000000 && k++ < 20000)
  {
    const u32 sz = Rd32(ram, n + 4);
    total += sz;
    big = std::max(big, sz);
    n = Rd32(ram, n + 0x10);
  }
  if (largest)
    *largest = big;
  if (nodes)
    *nodes = k;
  return total;
}

// another thread is inside heap i's allocator (its mutex has an owner): its free list may be half updated
static bool HeapLocked(const u8* ram, u32 i)
{
  const u32 h = Rd32(ram, 0x8040BD70 + 4 * i);
  return (h & 0xFE000000) == 0x80000000 && Rd32(ram, h + 0x44) != 0;
}

static std::string RbtagSummary(const u8* ram)
{
  char b[200];
  snprintf(b, sizeof b,
           "rbtag logs now acq %u rel %u free %u (max %u/%u/%u of 96), deferred frees %u of 63; free KB heap2 %u "
           "heap3 %u",
           Rd32(ram, 0x8049B510), Rd32(ram, 0x8049B514), Rd32(ram, 0x8049B518), s_max_acq, s_max_rel, s_max_frl,
           Rd32(ram, 0x8049B400), HeapFree(ram, 2) / 1024, HeapFree(ram, 3) / 1024);
  return b;
}


static void DiagLoads(const u8* ram, u32 f)
{
  // the game's file system (struct 0x804018C0): +0x14 = request being processed, +0x20 = queue head
  const u32 cur = Rd32(ram, 0x804018D4), head = Rd32(ram, 0x804018E0);
  if (cur || head)
  {
    s_fs_busy_frames++;
    s_fs_last_busy = std::max(s_fs_last_busy, f);
    s_fs_real_busy = std::max(s_fs_real_busy, f);
  }
  if ((cur || head) && !s_fs_was_busy)
    s_fs_episodes++;
  s_fs_was_busy = cur || head;
  if (s_fs_prev_cur && cur != s_fs_prev_cur)
  {
    s_fs_done++;  // the request seen before has finished (sampled at update starts and VI fields)
    s_fs_done_frame = std::max(s_fs_done_frame, f);
  }
  s_fs_prev_cur = cur;
}

static void DiagCheck(const u8* ram, u32 f, const char* where)
{
  if (!s_active)
    return;
  if (!s_sys_allocs.empty())
  {
    // frames up to the confirmed one are never thrown away (and no rewind goes back more than 16 frames; the lab's
    // stress mode rewinds frames that count as confirmed)
    const u32 mc = MinConfirmed();
    std::erase_if(s_sys_allocs, [mc, f](const auto& a) { return a.second <= mc && a.second + 16 < f; });
  }
  DiagLoads(ram, f);
  if (!s_diag_heap_reported)
  {
    const std::string p = FreeListProblem(ram, true);
    if (!p.empty())
    {
      s_diag_heap_reported = true;
      char b[200];
      snprintf(b, sizeof b, "FFNY WARNING: broken heap free list found %s, frame %u (last rollback %u->%u%s): ",
               where, f, s_last_rb_from, s_last_rb_to, s_resim_target ? ", re-simulating" : "");
      Report(b + p + ". " + RbtagSummary(ram));
    }
  }
  static const char* const names[4] = {"acquire", "release", "free", "deferred main-heap free"};
  const u32 fill[4] = {Rd32(ram, 0x8049B510), Rd32(ram, 0x8049B514), Rd32(ram, 0x8049B518),
                       Rd32(ram, 0x8049B400)};
  const u32 cap[4] = {96, 96, 96, 63};
  for (u32 i = 0; i < 4; i++)
  {
    if (fill[i] >= cap[i] && !(s_diag_full_reported >> i & 1))
    {
      s_diag_full_reported |= 1u << i;
      Report("FFNY WARNING: rbtag " + std::string(names[i]) + " log full at frame " + std::to_string(f) +
             " - from now on such calls are done at once instead of being delayed, so a later rollback can "
             "repeat them (possible cause of a later sound or heap crash). " + RbtagSummary(ram));
    }
  }
  // free space of heaps 2 and 3 at every update start (24 October version: once a second, and also while another
  // thread was inside the heap - a half-updated free list can read as 0)
  if (HeapLocked(ram, 2) || HeapLocked(ram, 3))
  {
    s_heap_samples_skipped++;
    if (s_log)
    {
      // lab: what the old sampling would have read while another thread holds the heap
      static u32 low_locked = 0xFFFFFFFF;
      const u32 raw3 = HeapFree(ram, 3), raw2 = HeapFree(ram, 2);
      if (raw3 < low_locked)
      {
        low_locked = raw3;
        fprintf(stderr, "FFNYRB LAB heap sample while locked (2:%d 3:%d) at frame %u: heap2 %u KB heap3 %u KB\n",
                HeapLocked(ram, 2), HeapLocked(ram, 3), f, raw2 / 1024, raw3 / 1024);
      }
    }
    return;
  }
  s_min_free2 = std::min(s_min_free2, HeapFree(ram, 2));
  u32 largest = 0, nodes = 0;
  const u32 free3 = HeapFree(ram, 3, &largest, &nodes);
  if (s_log)
  {
    static u32 last3 = 0;
    const u32 d = free3 > last3 ? free3 - last3 : last3 - free3;
    if (d >= 4096)
      fprintf(stderr, "FFNYRB LAB heap3 %u KB -> %u KB at frame %u (%s, conf %u)\n", last3 / 1024, free3 / 1024, f,
              s_resim_target ? "re-sim" : "first", MinConfirmed());
    last3 = free3;
  }
  if (free3 < s_min_free3)
  {
    s_min_free3 = free3;
    s_min_free3_frame = f;
    s_min_free3_largest = largest;
    s_min_free3_nodes = nodes;
    s_min_free3_resim = s_resim_target != 0;
    s_min_free3_predicted = f > MinConfirmed();
  }
  if (free3 < 64 * 1024 && !s_low3_reported)
  {
    s_low3_reported = true;
    const u32 h = Rd32(ram, 0x8040BD70 + 12);
    char b[400];
    snprintf(b, sizeof b,
             "FFNY NOTE: memory heap 3 nearly full at frame %u: %u KB free (largest free block %u KB, %u free "
             "blocks; heap %08x-%08x); %s, inputs of this frame %s; rollbacks so far %u (last %u->%u)",
             f, free3 / 1024, largest / 1024, nodes, Rd32(ram, h + 8), Rd32(ram, h + 0xC),
             s_resim_target ? "during a re-simulation" : "first run of the frame",
             f > MinConfirmed() ? "partly predicted" : "all known", s_rollbacks, s_last_rb_from, s_last_rb_to);
    Report(std::string(b) + ". " + RbtagSummary(ram));
  }
}

// lab stress: from here on, draws (and the watched range) belong to the update of frame f
void DrawFrameBegin(const u8* ram, u32 f, bool first)
{
  // lab: FFNY_RB_DUMPAT=off1,off2,...:prefix -> MEM1 at the first run of fight-start + off (prefix_off.raw)
  if (first && s_active)
  {
    static std::vector<u32> offs;
    static std::string prefix;
    static bool parsed = false;
    if (!parsed)
    {
      parsed = true;
      if (const char* e = std::getenv("FFNY_RB_DUMPAT"))
      {
        std::string v = e;
        const auto c = v.find(':');
        prefix = v.substr(c + 1);
        std::stringstream ss(v.substr(0, c));
        std::string t;
        while (std::getline(ss, t, ','))
          offs.push_back(u32(std::stoul(t)));
      }
    }
    for (const u32 o : offs)
    {
      if (f == s_start_frame + o)
      {
        const std::string path = prefix + "_" + std::to_string(o) + ".raw";
        if (FILE* fp = fopen(path.c_str(), "wb"))
        {
          fwrite(ram, 1, 0x01800000, fp);
          fclose(fp);
          fprintf(stderr, "FFNYRB LAB dumped MEM1 at frame %u (start + %u) to %s\n", f, o, path.c_str());
        }
      }
    }
  }
  DiagCheck(ram, f, first ? "before the update" : "before a re-simulated update");
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

// Leave rollback mode: back to normal (lockstep) netplay. The fight-end callback pauses this machine until the host's
// state arrives. forced = the reason when this machine did not reach the end of the fight itself (the host ended
// it for everyone, or no inputs came for 10 s); its state is then replaced by the host's anyway.
// lab: FFNY_RB_COUNT=addr,addr,... (up to 32) -> calls of those functions by the main thread during the fight,
// first runs and re-simulations separately, with the most frequent callers (printed at the fight end)
std::map<u32, std::array<u64, 2>> s_call_counts;
std::map<std::pair<u32, u32>, u64> s_call_sites;
static void CountCall(const Core::CPUThreadGuard& guard, int i)
{
  if (!s_active)
    return;
  auto& system = guard.GetSystem();
  const u8* ram = system.GetMemory().GetRAM();
  static const bool trace_all = std::getenv("FFNY_RB_COUNT_TRACE") != nullptr;
  static const bool any_thread = std::getenv("FFNY_RB_COUNT_ANYTHREAD") != nullptr;
  if (Rd32(ram, 0x800000E4) != 0x80432D10)
  {
    if (trace_all && any_thread)
    {
      const auto& ppc = system.GetPPCState();
      fprintf(stderr, "CALL %08x ctr %u thread %08x r3 %08x r4 %08x r5 %08x lr %08x\n", s_count_addr[i],
              Rd32(ram, ADDR_COUNTER), Rd32(ram, 0x800000E4), ppc.gpr[3], ppc.gpr[4], ppc.gpr[5], ppc.spr[SPR_LR]);
    }
    return;
  }
  s_call_counts[s_count_addr[i]][s_resim_target != 0 ? 1 : 0]++;
  s_call_sites[{s_count_addr[i], system.GetPPCState().spr[SPR_LR]}]++;
  if (trace_all)
  {
    const auto& ppc = system.GetPPCState();
    fprintf(stderr, "CALL %08x ctr %u %c %c r3 %08x r4 %08x r5 %08x lr %08x\n", s_count_addr[i], Rd32(ram, ADDR_COUNTER),
            s_resim_target != 0 ? 'r' : 'f', s_phase, ppc.gpr[3], ppc.gpr[4], ppc.gpr[5], ppc.spr[SPR_LR]);
  }
}
template <int I>
void HookCountN(const Core::CPUThreadGuard& guard)
{
  CountCall(guard, I);
}
template void HookCountN<0>(const Core::CPUThreadGuard&);
template void HookCountN<1>(const Core::CPUThreadGuard&);
template void HookCountN<2>(const Core::CPUThreadGuard&);
template void HookCountN<3>(const Core::CPUThreadGuard&);
template void HookCountN<4>(const Core::CPUThreadGuard&);
template void HookCountN<5>(const Core::CPUThreadGuard&);
template void HookCountN<6>(const Core::CPUThreadGuard&);
template void HookCountN<7>(const Core::CPUThreadGuard&);
template void HookCountN<8>(const Core::CPUThreadGuard&);
template void HookCountN<9>(const Core::CPUThreadGuard&);
template void HookCountN<10>(const Core::CPUThreadGuard&);
template void HookCountN<11>(const Core::CPUThreadGuard&);
template void HookCountN<12>(const Core::CPUThreadGuard&);
template void HookCountN<13>(const Core::CPUThreadGuard&);
template void HookCountN<14>(const Core::CPUThreadGuard&);
template void HookCountN<15>(const Core::CPUThreadGuard&);
template void HookCountN<16>(const Core::CPUThreadGuard&);
template void HookCountN<17>(const Core::CPUThreadGuard&);
template void HookCountN<18>(const Core::CPUThreadGuard&);
template void HookCountN<19>(const Core::CPUThreadGuard&);
template void HookCountN<20>(const Core::CPUThreadGuard&);
template void HookCountN<21>(const Core::CPUThreadGuard&);
template void HookCountN<22>(const Core::CPUThreadGuard&);
template void HookCountN<23>(const Core::CPUThreadGuard&);
template void HookCountN<24>(const Core::CPUThreadGuard&);
template void HookCountN<25>(const Core::CPUThreadGuard&);
template void HookCountN<26>(const Core::CPUThreadGuard&);
template void HookCountN<27>(const Core::CPUThreadGuard&);
template void HookCountN<28>(const Core::CPUThreadGuard&);
template void HookCountN<29>(const Core::CPUThreadGuard&);
template void HookCountN<30>(const Core::CPUThreadGuard&);
template void HookCountN<31>(const Core::CPUThreadGuard&);

// lab: FFNY_RB_TRACE=addr,... (up to 4) -> every main-thread call of those functions during the fight on stderr:
// frame (rbtag's W_FRAME), r = re-simulation / f = first run, r3..r10, LR
template <int I>
void HookTraceN(const Core::CPUThreadGuard& guard)
{
  if (!s_active)
    return;
  auto& system = guard.GetSystem();
  const u8* ram = system.GetMemory().GetRAM();
  if (Rd32(ram, 0x800000E4) != 0x80432D10)
    return;
  const auto& ppc = system.GetPPCState();
  fprintf(stderr, "TRACE %08x %u %c %08x %08x %08x %08x %08x %08x %08x %08x lr %08x\n", s_trace_addr[I],
          Rd32(ram, W_FRAME), s_resim_target != 0 ? 'r' : 'f', ppc.gpr[3], ppc.gpr[4], ppc.gpr[5], ppc.gpr[6],
          ppc.gpr[7], ppc.gpr[8], ppc.gpr[9], ppc.gpr[10], ppc.spr[SPR_LR]);
}
template void HookTraceN<0>(const Core::CPUThreadGuard&);
template void HookTraceN<1>(const Core::CPUThreadGuard&);
template void HookTraceN<2>(const Core::CPUThreadGuard&);
template void HookTraceN<3>(const Core::CPUThreadGuard&);

// rbtag's post-allocation hook (called by the heap allocator for every allocation, r3 = info: +0x18 pointer)
void HookSysAlloc(const Core::CPUThreadGuard& guard)
{
  if (!s_active)
    return;
  auto& system = guard.GetSystem();
  const u8* ram = system.GetMemory().GetRAM();
  // the main thread's allocations belong to the rewound logic, and so do the file-system thread's: with instant
  // disc reads it allocates the file buffers inside the logic frame that asked for the file
  const u32 thread = Rd32(ram, 0x800000E4);
  if ((thread != 0x80432D10 && thread != FS_THREAD) || Rd32(ram, W_ACT) != 1)
  {
    if (s_log)
    {
      const u32 inf = system.GetPPCState().gpr[3], pp = Rd32(ram, inf + 0x18);
      static u32 m = 0;
      if ((pp & 0xFE000000) == 0x80000000 && ((Rd32(ram, pp - 16) & 0x3F) == Rd32(ram, 0x8049A404)) && m++ < 100)
        fprintf(stderr, "FFNYRB LAB sysalloc by other thread %08x: %08x size %u frame %u\n", thread, pp,
                Rd32(ram, inf + 0x1C), Rd32(ram, ADDR_COUNTER));
    }
    return;
  }
  const u32 info = system.GetPPCState().gpr[3];
  if ((info & 0xFE000000) != 0x80000000)
    return;
  const u32 p = Rd32(ram, info + 0x18);
  const u32 sysidx = Rd32(ram, 0x8049A404);
  if ((p & 0xFE000000) != 0x80000000 || sysidx == 0 || (Rd32(ram, p - 16) >> 16) != 0x424D ||
      ((Rd32(ram, p - 16) & 0xFFFF) & 0x3F) != sysidx)
    return;
  s_sys_allocs.emplace_back(p, Rd32(ram, W_FRAME));
  if (s_log)
  {
    static u32 n = 0;
    if (n++ < 400)
    {
      char nm[40] = {};
      const u32 na = Rd32(ram, info);
      for (u32 k = 0; k < 39 && (na & 0xFE000000) == 0x80000000; k++)
        if (!(nm[k] = char(ram[(na + k) & 0x01FFFFFF])))
          break;
      fprintf(stderr, "FFNYRB LAB sysalloc %08x size %u thread %08x frame %u (W_FRAME %u, resim %d) %s lr %08x\n", p,
              Rd32(ram, info + 0x1C), thread, Rd32(ram, ADDR_COUNTER), Rd32(ram, W_FRAME), s_resim_target != 0, nm,
              system.GetPPCState().spr[SPR_LR]);
    }
  }
}

// after a rewind to frame s: SysHeap blocks the main thread allocated in frames >= s go to rbtag's free log with
// frame 0 (freed at its next settle; rbtag drops its own log entries of frames >= s, so a free the thrown-away
// frames did of the same block is not done twice)
static void FreeThrownSysAllocs(u8* ram, u32 s)
{
  size_t k = 0;
  for (const auto& [p, fr] : s_sys_allocs)
  {
    if (fr < s)
      s_sys_allocs[k++] = {p, fr};
    else
      s_sys_pending.push_back(p);
  }
  s_sys_allocs.resize(k);
}

// hand pending blocks to rbtag's free log, leaving half of it to rbtag's own logging (a full log makes rbtag free
// at once, which a later rewind would repeat); called at every update start
static void DrainSysPending(u8* ram)
{
  if (s_sys_pending.empty())
    return;
  const u32 sysidx = Rd32(ram, 0x8049A404);
  size_t k = 0;
  for (const u32 p : s_sys_pending)
  {
    const u32 hw = Rd32(ram, p - 16);
    if ((hw >> 16) != 0x424D || (hw & 0x3F) != sysidx || (hw & 0xC000) == 0x4000)
      continue;  // already freed
    const u32 n = Rd32(ram, 0x8049B518);  // FRL_N
    if (n >= 48)
    {
      s_sys_pending[k++] = p;
      continue;
    }
    Wr32(ram, 0x8049BC00 + 8 * n, p);
    Wr32(ram, 0x8049BC00 + 8 * n + 4, 0);
    Wr32(ram, 0x8049B518, n + 1);
    s_sys_freed++;
  }
  s_sys_pending.resize(k);
  s_sys_free_overflow = static_cast<u32>(k);
}

static std::string Low3Where()
{
  if (s_min_free3 == 0xFFFFFFFF)
    return "";
  char b[160];
  snprintf(b, sizeof b, " (at frame %u, %s, %s; largest free block %u KB)", s_min_free3_frame,
           s_min_free3_resim ? "re-simulated" : "first run", s_min_free3_predicted ? "predicted" : "confirmed",
           s_min_free3_largest / 1024);
  return b;
}

static void EndFight(Core::System& system, u8* ram, u32 f, const std::string& forced)
{
  // The other side may still lack our last frames (lost packets) and waits for them: send them once more, reliably.
  ResendInputs(true);
  s_active = false;
  s_menus_on = false;  // a forced end (disconnect, the host ended it): the next fight starts menus mode again
  system.GetCoreTiming().SetFrozen(false);
  s_loader_undo.clear();
  s_snd_dropped += u32(s_snd_q.size());  // only left when the fight was ended early (they never got confirmed)
  s_snd_q.clear();
  AfEnd(ram);
  s_music_sent.clear();
  s_idle_wait.clear();
  s_fs_main_queued = false;
  fprintf(stderr, "FFNYRB file ids: %u virtual opens, %u objects of thrown-away frames returned (%u busy), %u releases taken back, %u done after confirmation; %u reads of other threads hurried for the logic\n",
          s_af_opened, s_af_orphans, s_af_busy_orphans, s_af_unfreed, s_af_freed, s_fs_hurried);
  Wr32(ram, W_ACT, 0);  // rbtag: do the remaining deferred sound releases
  if (s_rbtag_redirected)
    Wr32(ram, W_FIGHTW, 0);  // rbtag: no more SysHeap routing / deferred frees; the queued frees are done at the next free
  if (s_log)
    PrintStats(f);
  fprintf(stderr, "FFNYRB fight end at frame %u%s\n", f, forced.empty() ? "" : " (forced)");
  if (s_report_cb)
  {
    // always (not counted against the report cap): how the connection treated this fight
    char b[300];
    snprintf(b, sizeof b,
             "FFNY fight over at frame %u: %u rollbacks (max %u frames, %u frames re-simulated), %u waits "
             "for the other side (%.1f s in total), %u desyncs",
             f, s_rollbacks, s_max_depth, s_resim_frames, s_waits, s_wait_us / 1e6, s_desyncs);
    char c[800];
    snprintf(c, sizeof c,
             "; rbtag logs max %u/%u/%u of 96, lowest free KB heap2 %u heap3 %u%s; file system busy in %u frames, "
             "%u requests finished, rollbacks across a finished request %u, across a file conversion %u; empty "
             "animation bank lookups caught %u; lockstep for file loads %u frames (%.1f s waiting)",
             s_max_acq, s_max_rel, s_max_frl, s_min_free2 == 0xFFFFFFFF ? 0 : s_min_free2 / 1024,
             s_min_free3 == 0xFFFFFFFF ? 0 : s_min_free3 / 1024, Low3Where().c_str(), s_fs_busy_frames, s_fs_done, s_rb_across_load,
             s_rb_across_conv, s_bank_guard, s_lockstep_frames, s_lockstep_us / 1e6);
    std::string line = std::string(b) + c;
    {
      char e[320];
      snprintf(e, sizeof e,
               "; load episodes %u, lockstep frames only for the %u-frame tail %u, conversions of data outside the "
               "rollback state %u (at most %u frames after the file system was busy)",
               s_fs_episodes, LOAD_TAIL, s_lockstep_tail, s_conv_out, s_conv_gap_max);
      line += e;
      snprintf(e, sizeof e, "; sound events held %u, sent %u, dropped %u (hold %s); music cues not repeated %u",
               s_snd_held, s_snd_sent, s_snd_dropped, SndModeName(), s_music_deduped);
      line += e;
      snprintf(e, sizeof e, "; reads finished at once %u; SysHeap blocks of thrown-away frames freed %u%s", s_fs_instant,
               s_sys_freed, s_sys_free_overflow ? (" (" + std::to_string(s_sys_free_overflow) + " left: free log full)").c_str() : "");
      line += e;

    }
    if (!forced.empty())
      line += ". THIS MACHINE DID NOT REACH THE END ITSELF: " + forced + " - the host's state replaces mine";
    char d[160];
    snprintf(d, sizeof d, "; end marker: fight loop %u, fight flags %08x, %s", Rd32(ram, ADDR_FIGHT),
             Rd32(ram, ADDR_FIGHT_FLAGS), s_left_match ? "match left (quit)" : "in the match");
    line += d;
    s_report_cb(line);
    LogLine(line);
  }
  else if (s_log)
  {
    for (const auto& [a, c] : s_call_counts)
    {
      std::string sites;
      for (const auto& [k, n] : s_call_sites)
        if (k.first == a)
          sites += fmt::format(" {:08x}x{}", k.second, n);
      fprintf(stderr, "FFNYRB LAB calls %08x: first runs %llu, re-simulations %llu; callers%s\n", a,
              (unsigned long long)c[0], (unsigned long long)c[1], sites.c_str());
    }
    s_call_counts.clear();
    s_call_sites.clear();
    fprintf(stderr, "FFNYRB sound events: held %u, sent %u, dropped (thrown-away frames) %u\n", s_snd_held, s_snd_sent,
            s_snd_dropped);
    fprintf(stderr, "FFNYRB heap: lowest free KB heap2 %u heap3 %u%s; samples skipped (heap held by another thread) %u; SysHeap blocks of thrown-away frames freed %u, left %u\n",
            s_min_free2 / 1024, s_min_free3 / 1024, Low3Where().c_str(), s_heap_samples_skipped, s_sys_freed, s_sys_free_overflow);
    fprintf(stderr, "FFNYRB fight-over: rollbacks %u max %u resim %u waits %u lockstep %u frames, fs busy %u frames, "
                    "requests finished %u, across finished %u, across conversion %u, outside conversions %u, "
                    "reads finished at once %u, empty-bank lookups %u, desyncs %u\n",
            s_rollbacks, s_max_depth, s_resim_frames, s_waits, s_lockstep_frames, s_fs_busy_frames, s_fs_done,
            s_rb_across_load, s_rb_across_conv, s_conv_out, s_fs_instant, s_bank_guard, s_desyncs);
  }
  s_resync_since_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
  s_resync_waiting = s_fight_end_cb != nullptr;
  OSD::AddMessage("FFNY rollback: fight over - " + std::to_string(s_rollbacks) + " rollbacks (max " +
                      std::to_string(s_max_depth) + " frames), " + std::to_string(s_desyncs) +
                      " desyncs; synchronising...",
                  6000, forced.empty() ? OSD::Color::YELLOW : OSD::Color::RED);
  if (s_fight_end_cb)
    s_fight_end_cb(system, f);
}


// Sound events (25 October, Robert's 8+ fight session: volume creeping up over the fights, missed music switches
// fight -> Blazin' -> chorus -> back, then fast crackling audio and a crash in the sound thread at 0x801E7FC8).
// The game talks to EA's sound library (not rewound) through the event poster at *0x8043FB40 = 0x801D3074
// (event id, argument count, arguments) and the channel service 0x801D36B8 (channel). A rollback made the game post
// the same events again in every re-simulation (lab: every event of a re-simulated frame 2-3 times), and events of
// thrown-away predictions that the real timeline never had (lab: music event 0x2028 predicted, 0x202B real). Now
// main-thread calls of both during the fight's updates are held back per frame and sent to the library only once
// the frame can no longer be rolled back (exactly once, in order); a rewind to frame s drops the held calls of
// frames >= s. Sending = a call of the library function made from the PRE hook with all registers saved and
// restored (the function returns to ADDR_PRE, where the PRE hook sends the next one or restores the registers).
static void SaveRegs(const PowerPC::PowerPCState& ppc, SavedState& s)
{
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
}

static void LoadRegs(PowerPC::PowerPCState& ppc, const SavedState& s)
{
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
}

// the newest frame whose held sound calls can go out (it can no longer be rolled back)
static u32 SndReadyUpTo(u32 f)
{
  if (s_stress)
    return f > s_stress + 1 ? f - 1 - s_stress : 0;
  return std::min(MinConfirmed(), f - 1);
}

static void SndDropFrom(u32 s)
{
  for (auto& e : s_music_sent)
    if (e.first.frame >= s)
      e.second = false;  // the re-simulation from s may send these again: match them anew
  while (!s_snd_q.empty() && s_snd_q.back().frame >= s)
  {
    s_snd_q.pop_back();
    s_snd_dropped++;
  }
}

// set up the call of the oldest held sound call that is due (registers already saved); false if none is due
static bool SndSendNext(PowerPC::PowerPCState& ppc, u32 upto)
{
  if (s_snd_q.empty() || s_snd_q.front().frame > upto)
    return false;
  const SndCall c = s_snd_q.front();
  s_snd_q.pop_front();
  for (int i = 0; i < 8; i++)
    ppc.gpr[3 + i] = c.r[i];
  ppc.spr[SPR_LR] = ADDR_PRE;
  ppc.npc = c.fn;
  s_snd_sending = true;
  s_snd_sent++;
  return true;
}

// the start of the sends (end of the PRE hook, registers = how to go on); true if a call was set up
static bool SndStartSends(PowerPC::PowerPCState& ppc, u32 upto)
{
  if (s_snd_q.empty() || s_snd_q.front().frame > upto)
    return false;
  SaveRegs(ppc, s_snd_regs);
  s_snd_regs_npc = ppc.npc;
  return SndSendNext(ppc, upto);
}

// a fight that ends: make the held sound calls first, then end it (from the sends' return in the PRE hook)
static bool SndFlushThenEnd(PowerPC::PowerPCState& ppc, u32 f, const std::string& reason)
{
  if (!SndStartSends(ppc, 0xFFFFFFFF))
    return false;
  s_snd_end_after = true;
  s_snd_end_frame = f;
  s_snd_end_reason = reason;
  return true;
}

// main-thread call of the event poster / channel service: hold it back (fight updates) or let it through
static void SndTr(const char* what, u8* ram, const PowerPC::PowerPCState& ppc, u32 extra)
{
  fprintf(stderr, "SND %s ctr %u w %u %c act %d th %08x r3 %08x r4 %08x r5 %08x r6 %08x r7 %08x x %08x lr %08x\n",
          what, Rd32(ram, ADDR_COUNTER), Rd32(ram, W_FRAME), s_resim_target != 0 ? 'r' : 'f', s_active,
          Rd32(ram, 0x800000E4), ppc.gpr[3], ppc.gpr[4], ppc.gpr[5], ppc.gpr[6], ppc.gpr[7], extra,
          ppc.spr[SPR_LR]);
}
static u32 SlotKey(u8* ram, u32 idx)
{
  if (idx > 15)
    return 0xFFFFFFFF;
  const u32 blk = Rd32(ram, 0x803D99F4 + 0x4C + idx * 16);
  return (blk & 0xFE000000) == 0x80000000 ? Rd32(ram, blk) : 0;
}
void HookSndTrAccept(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  u8* ram = system.GetMemory().GetRAM();
  const auto& ppc = system.GetPPCState();
  SndTr(ppc.gpr[3] ? "ACCEPT" : "REJECT", ram, ppc, Rd32(ram, ppc.gpr[31]));
}
void HookSndTrMusRes(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  u8* ram = system.GetMemory().GetRAM();
  const auto& ppc = system.GetPPCState();
  SndTr("MUSRES", ram, ppc, Rd32(ram, 0x80440AD0));
}
void HookSndTrStart(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  u8* ram = system.GetMemory().GetRAM();
  const auto& ppc = system.GetPPCState();
  SndTr("START", ram, ppc, SlotKey(ram, ppc.gpr[3]));
}
void HookSndTrEvict(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  u8* ram = system.GetMemory().GetRAM();
  const auto& ppc = system.GetPPCState();
  SndTr("DISCARD", ram, ppc, SlotKey(ram, ppc.gpr[3]));
}
static void SndExecOriginal(u8* ram, PowerPC::PowerPCState& ppc, u32 fn)
{
  // the replaced instruction: stwu sp, d(sp)
  const u32 ins = Rd32(ram, fn);
  const u32 sp = ppc.gpr[1], nsp = sp + u32(s32(s16(ins & 0xFFFF)));
  Wr32(ram, nsp, sp);
  ppc.gpr[1] = nsp;
  ppc.npc = fn + 4;
}

static void SndHook(const Core::CPUThreadGuard& guard, u32 fn)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  u8* ram = system.GetMemory().GetRAM();
  const bool hold = ((fn == SND_POST || fn == SND_SERVICE) ? s_snd_hold_on : s_music_hold) && s_active && !s_snd_sending && (s_phase == 'U' || s_phase == 'T') &&
                    Rd32(ram, 0x800000E4) == 0x80432D10 && !(fn == SND_POST && ppc.gpr[4] > 6);
  if (s_snd_trace)
    SndTr(fn == SND_POST    ? (hold ? "post-HOLD" : s_snd_sending ? "post-SEND" : "post")
          : fn != SND_SERVICE ? (hold ? "mus-HOLD" : s_snd_sending ? "mus-SEND" : "mus")
                            : (hold ? "serv-HOLD" : s_snd_sending ? "serv-SEND" : "serv"),
          ram, ppc, 0);
  if (!hold)
  {
    // the replaced instruction: stwu sp, d(sp)
    const u32 ins = Rd32(ram, fn);
    const u32 sp = ppc.gpr[1], nsp = sp + u32(s32(s16(ins & 0xFFFF)));
    Wr32(ram, nsp, sp);
    ppc.gpr[1] = nsp;
    ppc.npc = fn + 4;
    return;
  }
  SndCall c;
  c.frame = Rd32(ram, W_FRAME);
  c.fn = fn;
  for (int i = 0; i < 8; i++)
    c.r[i] = ppc.gpr[3 + i];
  if (fn != SND_POST && fn != SND_SERVICE && !s_music_hold_old)
  {
    // Music cues: sent at once in a first run, not held. Held until confirmed (153a233), a cue reached the music
    // player some frames late, and the music module - which asks the player for the song position every frame
    // (0x800111C0 at 0x80031568) - took "no position yet" for "song over" and went to the next song (lab: the menu
    // song changed every 376 frames in stress mode, never without rollback; with the hold off it stayed). In a
    // re-simulation a cue the first run of that frame already sent (same arguments) is not sent again; a different
    // one (the inputs were mispredicted) is sent then.
    // 29 October (friend session, 10th fight: the music stream's chunk list ran empty in the audio thread, crash at
    // 0x801E7FC8, right after P1's Blazin' activation; Robert heard fast, choppy music before such crashes): a cue the
    // first run sent at frame F that the corrected timeline makes at another frame (a Blazin' activated a frame or two
    // later, a KO a frame earlier) used to be sent again - the music player jumped to the same part twice within a few
    // frames, each jump a seek of the stream. Now an unmatched cue of the thrown-away frames with the same event
    // within the rollback window counts as this one (FFNY_RB_MUSICEXACT=1: exact frame only, the old rule). Every cue
    // is logged (first 400 per session) so a crash report can be matched with the cues before it.
    static const bool exact = std::getenv("FFNY_RB_MUSICEXACT") != nullptr;
    static u32 cue_logged = 0;
    const u32 ev = c.r[0] & 0xFFFF;  // 0x80011108 uses only (s16)r3
    auto log_cue = [&](const char* what) {
      if (cue_logged++ < 400)
        LogLine(fmt::format("FFNY music cue {:04x} ({:08x}) at frame {}: {}", ev, fn, c.frame, what));
    };
    if (!s_resim_target)
    {
      s_music_sent.push_back({c, false});
      log_cue("sent (first run)");
      SndExecOriginal(ram, ppc, fn);
      return;
    }
    const u32 window = s_settings.max_rollback + 2;
    for (int pass = 0; pass < (exact ? 1 : 2); pass++)
      for (auto& e : s_music_sent)
        if (!e.second && e.first.fn == c.fn && (e.first.r[0] & 0xFFFF) == ev &&
            (pass == 0 ? e.first.frame == c.frame :
                         (e.first.frame + window >= c.frame && c.frame + window >= e.first.frame)))
        {
          e.second = true;  // matched in this re-simulation
          s_music_deduped++;
          if (pass == 1)
            log_cue(fmt::format("not sent again (the first run sent it at frame {})", e.first.frame).c_str());
          ppc.gpr[3] = 1;
          ppc.npc = ppc.spr[SPR_LR];
          return;
        }
    s_music_sent.push_back({c, true});
    s_music_resent++;
    log_cue("sent by a re-simulation (the first run did not send it)");
    SndExecOriginal(ram, ppc, fn);
    return;
  }
  s_snd_q.push_back(c);
  s_snd_held++;
  ppc.gpr[3] = 1;
  ppc.npc = ppc.spr[SPR_LR];
}

void HookSndPost(const Core::CPUThreadGuard& guard)
{
  SndHook(guard, SND_POST);
}

void HookSndService(const Core::CPUThreadGuard& guard)
{
  SndHook(guard, SND_SERVICE);
}


// The time base the game reads (mftb). While rollback runs in the menus, the main thread gets anchor + 1/60 s per logic
// frame (+1 tick per read inside a frame, reset at every update start): the menus (Apt Flash player 0x801A3F24 /
// 0x801A4010, the timer at 0x8015BB4C, ...) take their timing from mftb, so a re-simulated frame read a later real
// time than its first run (lab 26 October: 0x804404BC / 0x8044040C differed from the first stress frame on). The
// anchor is the real time base when rollback starts: in lockstep netplay it is the same on every machine.
// Match set-up seeds (28 October, Robert: desync in random generator 2 right after a new match was set up from the
// main menu, frame 9512 -> 9513; a rematch does not reseed). The game seeds its generators from inline mftbl reads:
// 0x80095F68 (match object +0x2F8 = TBL * 12, next seed +0x2F4 = old +0x2F8 + 0x07714F95; generator 2 0x803BE03C is
// seeded from those at 0x80095C5C), 0x800982C8 (match object +0x302FC generator), 0x80096364 / 0x80096610 (0x800DBA94)
// and the audio module's 0x8003253C. They are not all read inside the logic update, where the frame clock applies,
// so each machine used its own real time. While rollback runs these reads get the frame clock (anchor + 1/60 s per
// logic frame) whatever phase they happen in.
template <int I>
void HookSeedTbN(const Core::CPUThreadGuard& guard)
{
  // The reads happen on the fight loader thread 0x806D8AE0 (lab), at moments that depend on its progress, so neither
  // the real time nor the logic frame is the same on all machines; the order of the reads is (same game path). Each
  // read gets anchor + n * a large odd step (n = the read's number since rollback started); a Replace hook at a fixed
  // address (ppc.pc is not that address inside a JIT block).
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  const u32 site = SEED_TB_SITES[I], ins = Rd32(ram, site);
  u64 v = system.GetPowerPC().ReadFullTimeBaseValue();
  if (s_active && s_vtb_anchor)
    v = s_vtb_anchor + u64(++s_seed_reads) * 0x9E3779B1ull;
  ppc.gpr[(ins >> 21) & 31] = u32(v);  // mftbl
  ppc.npc = site + 4;
}
// The menus' Flash player clock (0x801A3F18 reset: mftbl at 0x801A3F24; 0x801A3F60 tick: mftbl at 0x801A3F80; the tick's
// millisecond count 0x804404D4 and its step 0x804404D8 drive the Apt movies' playheads 0x8044040C / 0x80433FC0..).
// 28 October (Robert's A/B dumps, third match desynced): after the first fight the clock's base 0x804404C4 and both
// millisecond counts were equal, before the second fight the counts differed by 84 ms (5 frames) while the time base
// words were equal again - one reset between the fights had read a different time on each machine (the frame clock
// applied only to the main thread's logic update; the loading screen's thread resets and ticks this clock too), and the
// first tick after the next reset turns the stale count into a different step. Both reads get the frame clock (anchor
// + 1/60 s per logic frame) whenever rollback runs, in any phase.
template <int I>
void HookAptTbN(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  const u32 site = APT_TB_SITES[I], ins = Rd32(ram, site);
  u64 v = system.GetPowerPC().ReadFullTimeBaseValue();
  const u32 f = Rd32(ram, ADDR_COUNTER);
  // The loading screen's thread (0x806D8AE0, loop 0x800845F8: play the loading movie until it says it is done, sleep)
  // runs while the main thread waits, so the logic frame does not move: each of its ticks moves this clock on by
  // APT_SIDE_STEP (with a frozen clock its movie never ends - lab: hung at the title screen's load), and the main thread's
  // reads stay after them.
  if (s_active && s_vtb_mode && s_vtb_anchor)
  {
    if (Rd32(ram, 0x800000E4) != 0x80432D10 && I == 1)
      ++s_apt_side;
    v = s_vtb_anchor + u64(s32(f - s_vtb_frame0)) * 675000 + u64(s_apt_side) * APT_SIDE_STEP + 1;
  }
  ppc.gpr[(ins >> 21) & 31] = u32(v);  // mftbl
  ppc.npc = site + 4;
}
// see SONG_PICK_SITES: Replace hook on the "bl 0x8049A200" of a playlist pick
template <int I>
void HookSongPickN(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  const u32 site = SONG_PICK_SITES[I];
  if (!s_active)
  {
    // the original call: bl 0x8049A200
    ppc.spr[SPR_LR] = site + 4;
    ppc.npc = 0x8049A200;
    return;
  }
  const u32 thread = Rd32(ram, 0x800000E4), f = Rd32(ram, ADDR_COUNTER);
  u32 v;
  if (thread == MAIN_THREAD && Rd32(ram, ADDR_FIGHT) == 1)
  {
    if (f != s_song_frame)
    {
      s_song_frame = f;
      s_song_in_frame = 0;
    }
    u32 st[6];
    for (int i = 0; i < 6; i++)
      st[i] = s_song[i] ^ (f * 0x9E3779B1u) ^ (s_song_in_frame * 0x7FEB352Du) ^ u32(i);
    s_song_in_frame++;
    v = Gen1Step(st);
    v = Gen1Step(st);
  }
  else
  {
    v = Gen1Step(s_song);
  }
  s_song_picks++;
  if (s_song_logged < 200)
  {
    s_song_logged++;
    LogLine(fmt::format("FFNY song pick {} at {:08x}: frame {}, thread {:08x}, value {:08x}, count {}",
                        s_song_logged, site, f, thread, v, ppc.gpr[I == 0 ? 21 : 22]));
  }
  ppc.gpr[3] = v;
  ppc.npc = site + 4;
}
template void HookSongPickN<0>(const Core::CPUThreadGuard&);
template void HookSongPickN<1>(const Core::CPUThreadGuard&);
template void HookAptTbN<0>(const Core::CPUThreadGuard&);
template void HookAptTbN<1>(const Core::CPUThreadGuard&);
template void HookSeedTbN<0>(const Core::CPUThreadGuard&);
template void HookSeedTbN<1>(const Core::CPUThreadGuard&);
template void HookSeedTbN<2>(const Core::CPUThreadGuard&);
template void HookSeedTbN<3>(const Core::CPUThreadGuard&);
template void HookSeedTbN<4>(const Core::CPUThreadGuard&);

u64 JitTimeBase()
{
  auto& system = Core::System::GetInstance();
  auto& ppc = system.GetPPCState();
  u64 real;
  memcpy(&real, &ppc.spr[SPR_TL], 8);
  // not in the running fight: rollback fights ran with the real clock from the start (no desyncs in Robert's tests),
  // and with the frame clock the lab's two machines desynced during Blazin's (lab switch FFNY_RB_VTBFIGHT=1: keep it)
  static const bool vtb_fight = std::getenv("FFNY_RB_VTBFIGHT") != nullptr;
  if (!s_active || !s_vtb_mode || (s_real_fight_now && !vtb_fight))
    return real;
  const u8* ram = system.GetMemory().GetRAM();
  // only inside the logic update: the main loop's tick() (0x800EC144) paces the loop with the clock, and a clock that
  // only moves with the updates froze it (lab: logic counter stuck at 2992 on the title screen)
  if (!ram || Rd32(ram, 0x800000E4) != 0x80432D10 || s_phase != 'U')
    return real;
  // only the game's own code: the SDK (0x80240000 and up: OS alarms / sleeps, DVD, AI ...) times hardware with the
  // real clock (OSSetAlarm from 0x80274C70 read OSGetTime 0x80277A44 in the menus); OSGetTime / OSGetTick called by
  // game code count as the game's
  const u32 pc = ppc.pc, lr = ppc.spr[SPR_LR];
  const bool game_code = pc < 0x80240000 || (pc >= 0x80277A40 && pc < 0x80277A70 && lr < 0x80240000);
  if (!game_code)
    return real;
  const u32 f = Rd32(ram, ADDR_COUNTER);
  if (!s_vtb_anchor)
  {
    s_vtb_anchor = real;
    s_vtb_frame0 = f;
  }
  // reads with interrupts off (game callbacks run from interrupt handlers - DVD / audio / alarms - at moments that
  // depend on emulated hardware timing, so a re-simulation has them elsewhere or not at all) do not move the count:
  // otherwise every later read of the update got another value (lab: Apt's clock 0x804404BC off by 12 in re-simulations)
  if (ppc.msr.EE)
    ++s_vtb_steps;
  if (s_log && (s_vtb_steps == 50 || s_vtb_steps == 5000))
  {
    static u32 shown = 0;
    if (shown++ < 60)
      fprintf(stderr, "FFNYRB VTB frame %u: %u reads in this update, pc %08x lr %08x\n", f, s_vtb_steps, ppc.pc,
              ppc.spr[SPR_LR]);
  }
  return s_vtb_anchor + u64(s32(f - s_vtb_frame0)) * 675000 + s_vtb_steps;
}

// 0x8001112C (Replace; inside 0x80011108 after the post: r3 = 1 if the music player took the cue): while rollback runs
// the logic always gets 1 ("taken"), as with the hold (153a233 returned 1 for every held cue). The real answer depends
// on the audio thread's state, which is different on the other machine, and the music module acts on it - with the
// real answer passed through the two lab machines desynced in the fight during Blazin's (12688: fighters 1 and 2).
void HookSndMusicResult(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  const u8* ram = system.GetMemory().GetRAM();
  if (s_active && Rd32(ram, 0x800000E4) == 0x80432D10)
  {
    if (ppc.gpr[3] != 1)
      s_music_refused++;
    ppc.gpr[3] = 1;
  }
  ppc.gpr[0] = Rd32(ram, ppc.gpr[1] + 12);  // lwz r0,12(r1)
  ppc.npc = 0x8001112C + 4;
}

void HookSndMusic(const Core::CPUThreadGuard& guard)
{
  SndHook(guard, SND_MUSIC);
}

// more music-library wrappers held like the music cues (no return value the game keeps): 0x80011164 = music volume
// (0x801D8C14(-1, vol); a re-simulated call crashed in the library's stream callbacks on the title screen, lab
// 26 October, rollback in the menus)
void HookSndExtra(const Core::CPUThreadGuard& guard)
{
  // volume: made at once in a first run (its natural time: held, the call landed while a stream had no player object
  // and the game's callback 0x800D0ADC crashed), skipped in re-simulations (setting the same volume again; a
  // re-simulated call crashed in the same callback)
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  u8* ram = system.GetMemory().GetRAM();
  const u32 fn = 0x80011164;
  if (s_active && s_resim_target != 0 && Rd32(ram, 0x800000E4) == 0x80432D10)
  {
    ppc.gpr[3] = 1;
    ppc.npc = ppc.spr[SPR_LR];
    return;
  }
  const u32 ins = Rd32(ram, fn);
  const u32 sp = ppc.gpr[1], nsp = sp + u32(s32(s16(ins & 0xFFFF)));
  Wr32(ram, nsp, sp);
  ppc.gpr[1] = nsp;
  ppc.npc = fn + 4;
}

static u32 HeapFree(const u8* ram, u32 i, u32* largest, u32* nodes);
static u32 HeapFreeFwd(const u8* ram, u32 i, u32* largest, u32* nodes) { return HeapFree(ram, i, largest, nodes); }

// ---- Memory card: never rolled back (26 October) ----
// FFNY's memory-card module (0x8014C000..0x80150FFF, on the main thread inside the logic update) calls the SDK's CARD
// functions directly and polls them in loops that sleep (e.g. the probe loop 0x8014F4E0); the "MCThread" (0x80A29C80)
// does more card work in the background. The CARD library and the card are not part of the rollback state, so a
// re-simulated card call meets a card that already did it (lab, same inputs: SELECT USER ID's card prompt never
// finished under stress). Two rules make sure no card work is ever re-simulated:
// 1. barrier: a card call of the logic in a frame whose inputs are not all known yet waits for them; if one turns
//    out mispredicted, the frame is abandoned right there (rollback from inside the call, before the card is touched);
// 2. lockstep: while the card is in use (any CARD call or memory-card device command) and CARD_TAIL frames after,
//    a frame only runs once all its inputs are known - so no rollback can go back into card activity.
u32 g_card_activity = 0;
constexpr u32 CARD_TAIL = 10;
u32 s_card_last = 0, s_card_seen = 0, s_card_waits = 0, s_card_aborts = 0, s_card_resim_calls = 0, s_card_calls = 0;
u64 s_card_wait_us = 0;
u32 s_prev_first_f = 0, s_stall_guard = 0, s_stalls = 0;  // logic counter stalls (see HookPreUpdate)
u32 s_barrier_late = 0;  // barrier calls that found a mispredicted input (handled at the next update start)
u32 s_last_lock_frame = 0;  // newest frame that ran in lockstep (never rolled back)
u32 s_menu_lock_frames = 0;
std::array<u32, 19> s_card_fn_calls{};

static bool CardLocked(u32 f)
{
  return (s_card_on && s_card_last && f <= s_card_last + CARD_TAIL) ||
         (s_async_on && s_async_last && f <= s_async_last + ASYNC_TAIL);
}

// worker threads (see s_idle_wait): learn their idle waits (rollback start), find busy ones (update start)
static void LearnIdleWaits(const u8* ram)
{
  s_idle_wait.clear();
  for (u32 t = Rd32(ram, 0x800000DC), k = 0; t && k < 64; t = Rd32(ram, t + 0x2FC), k++)
    if (t != 0x80432D10 && s32(Rd32(ram, t + 0x2D0)) >= 5 && (Rd32(ram, t + 0x2C8) >> 16) == 4)
      s_idle_wait[t] = Rd32(ram, t + 0x2DC);
}
static u32 BusyWorker(const u8* ram)
{
  // busy = asleep on something else than its idle queue (waiting for a file read, the card ...): in the middle of a
  // job. Ready (1: woken, not yet run - the workers have a lower priority than the main thread) is not: the memory-card
  // worker 0x806D5A34 is woken every few frames and answers at once when the main thread sleeps; a job that has to wait
  // for hardware shows up as busy at the next update start, and the lockstep rule then protects the frames before too.
  for (const auto& [t, w] : s_idle_wait)
    if ((Rd32(ram, t + 0x2C8) >> 16) == 4 && Rd32(ram, t + 0x2DC) != w)
      return t;
  return 0;
}


// wait until all inputs up to f are known; if one was mispredicted, roll back from here (true: rolled back)
static bool BarrierWait(Core::System& system, u8* ram, u32 f, const char* what)
{
  if (s_resim_target || (MinConfirmed() >= f && s_mispredict_from > f))
    return false;
  const auto t0 = std::chrono::steady_clock::now();
  auto last_resend = t0;
  s_wait_where = "barrier before a call to a worker thread / the memory card: waiting for the other side's inputs";
  while (MinConfirmed() < f && !s_force_end && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(10) &&
         system.GetCPU().GetState() != CPU::State::PowerDown)
  {
    if (s_loop)
    {
      for (const auto& q : s_loop_q)
        OnRemoteInput(1, q.first, q.second.data());
      s_loop_q.clear();
    }
    std::this_thread::sleep_for(std::chrono::microseconds(250));
    Pump();
    if (std::chrono::steady_clock::now() - last_resend > std::chrono::milliseconds(8))
    {
      last_resend = std::chrono::steady_clock::now();
      ResendInputs(false);
    }
  }
  s_wait_where = nullptr;
  s_card_waits++;
  s_card_wait_us += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
  // A misprediction is NOT acted on here: rolling back from inside the call hung the game (lab, loopback lag: the
  // message went through EA's port wrapper 0x8024D1B4, which holds a lock that lives outside the rollback state, so
  // after the rewind the main thread slept on it for ever). It waits for the next update start; outside the running
  // fight that does not happen anyway (every frame there runs with all inputs known, see HookPreUpdate).
  if (s_mispredict_from <= f)
  {
    s_barrier_late++;
    if (s_log)
      fprintf(stderr, "FFNYRB %s in frame %u with a mispredicted input (rollback at the next update start)\n", what, f);
  }
  return false;
}

// OSSendMessage 0x80274D94 (Replace hook; r3 = queue): the logic wakes a worker thread
void HookAsyncSend(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  u8* ram = system.GetMemory().GetRAM();
  if (s_active && s_async_on && s_phase == 'U' && Rd32(ram, 0x800000E4) == 0x80432D10)
  {
    const u32 q = ppc.gpr[3];
    for (const auto& [t, w] : s_idle_wait)
      if (w == q + 8)
      {
        // A barrier outside the running fight: a job started by a frame with a mispredicted input would be started
        // again by the re-simulation (lab stress run: the fight loader's job at 9060 crashed the front end at 9061 when a
        // rollback went back across it). Most messages are calls the logic waits for in the same frame (the card
        // worker: ~1 per 7 frames in the menus), so with inputs not yet known this costs a short wait in the menus. In
        // the running fight (~1 message per 10 frames, calls only) no barrier, so rollback keeps its feel there.
        // Lab: FFNY_RB_NOASYNCBARRIER.
        static const bool no_barrier = std::getenv("FFNY_RB_NOASYNCBARRIER") != nullptr;
        const bool in_fight = s_real_fight_now;
        const u32 f = Rd32(ram, W_FRAME);
        s_async_barriers++;
        if (!no_barrier && !in_fight)
        {
          s_async_last = std::max(s_async_last, f);
          s_async_last_thread = t;
          if (BarrierWait(system, ram, f, "message to a worker thread"))
          {
            s_async_aborts++;
            return;
          }
        }
        break;
      }
  }
  ppc.gpr[0] = ppc.spr[SPR_LR];  // mflr r0 (first instruction of OSSendMessage)
  ppc.npc = 0x80274D94 + 4;
}

template <int I>
void HookCardBarrierN(const Core::CPUThreadGuard& guard)
{
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  u8* ram = system.GetMemory().GetRAM();
  if (s_active && s_card_on)
  {
    const u32 f = Rd32(ram, W_FRAME);
    s_card_calls++;
    s_card_fn_calls[I]++;
    // 0x8025DADC (CARDProbeEx: is a card in the slot) is polled every frame by the logic (0x8014D660); it only reads
    // the slot state, so it neither waits nor starts card activity
    if (CARD_FUNCS[I] == 0x8025DADC)
    {
      ppc.gpr[0] = ppc.spr[SPR_LR];
      ppc.npc = CARD_FUNCS[I] + 4;
      return;
    }
    if (s_log && s_card_fn_calls[I] <= 3)
      fprintf(stderr, "FFNYRB CARD %08x frame %u thread %08x lr %08x r3 %08x\n", CARD_FUNCS[I], f, Rd32(ram, 0x800000E4),
              ppc.spr[SPR_LR], ppc.gpr[3]);
    s_card_last = std::max(s_card_last, f);
    if (Rd32(ram, 0x800000E4) == 0x80432D10 && s_phase == 'U')
    {
      if (s_resim_target)
        s_card_resim_calls++;  // should not happen (lab stress mode only)
      else if (MinConfirmed() < f || s_mispredict_from <= f)
      {
        const auto t0 = std::chrono::steady_clock::now();
        auto last_resend = t0;
        s_wait_where = "memory-card call: waiting for the other side's inputs";
        while (MinConfirmed() < f && !s_force_end && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(10) &&
               guard.GetSystem().GetCPU().GetState() != CPU::State::PowerDown)
        {
          if (s_loop)
          {
            // lab loopback: the "other side" would send them anyway
            for (const auto& q : s_loop_q)
              OnRemoteInput(1, q.first, q.second.data());
            s_loop_q.clear();
          }
          std::this_thread::sleep_for(std::chrono::microseconds(250));
          Pump();
          if (std::chrono::steady_clock::now() - last_resend > std::chrono::milliseconds(8))
          {
            last_resend = std::chrono::steady_clock::now();
            ResendInputs(false);
          }
        }
        s_wait_where = nullptr;
        s_card_waits++;
        s_card_wait_us += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
        if (s_mispredict_from <= f)
          s_barrier_late++;  // see BarrierWait: no rollback from inside the call
      }
    }
  }
  ppc.gpr[0] = ppc.spr[SPR_LR];  // mflr r0
  ppc.npc = CARD_FUNCS[I] + 4;
}
#define FFNY_CARD_INST(n) template void HookCardBarrierN<n>(const Core::CPUThreadGuard&);
FFNY_CARD_INST(0) FFNY_CARD_INST(1) FFNY_CARD_INST(2) FFNY_CARD_INST(3) FFNY_CARD_INST(4) FFNY_CARD_INST(5) FFNY_CARD_INST(6)
FFNY_CARD_INST(7) FFNY_CARD_INST(8) FFNY_CARD_INST(9) FFNY_CARD_INST(10) FFNY_CARD_INST(11) FFNY_CARD_INST(12)
FFNY_CARD_INST(13) FFNY_CARD_INST(14) FFNY_CARD_INST(15) FFNY_CARD_INST(16) FFNY_CARD_INST(17) FFNY_CARD_INST(18)
#undef FFNY_CARD_INST

// lab: FFNY_RB_INPUTREPLAY=file (lines "port frame bytes...", written by FFNY_RB_INPUTLOG): replace a pad record
static void ReplayInput(int p, u32 frame, Record& rec)
{
  static std::map<std::pair<int, u32>, Record> replay;
  static bool loaded = false;
  if (!loaded)
  {
    loaded = true;
    if (const char* rf = std::getenv("FFNY_RB_INPUTREPLAY"))
      if (FILE* fr = fopen(rf, "r"))
      {
        char line[1024];
        while (fgets(line, sizeof line, fr))
        {
          int pp = 0, n = 0;
          unsigned fr2 = 0;
          if (sscanf(line, "%d %u%n", &pp, &fr2, &n) < 2)
            continue;
          Record r{};
          const char* q = line + n;
          for (u32 k = 0; k < REC_SIZE; k++)
          {
            while (*q == ' ')
              q++;
            unsigned b = 0;
            if (sscanf(q, "%2x", &b) != 1)
              break;
            r[k] = u8(b);
            q += 2;
          }
          replay[{pp, fr2}] = r;
        }
        fclose(fr);
        fprintf(stderr, "FFNYRB lab: %zu replayed inputs\n", replay.size());
      }
  }
  if (auto it = replay.find({p, frame}); it != replay.end())
    rec = it->second;
}

void SetMenuDelay(u32 frames)
{
  s_menu_delay = frames;
}

void HookPreUpdate(const Core::CPUThreadGuard& guard)
{
  s_pre_count++;
  auto& system = guard.GetSystem();
  auto& ppc = system.GetPPCState();
  u8* ram = system.GetMemory().GetRAM();
  s_vtb_steps = 0;
  if (s_log && s_active && !s_snd_sending)
  {
    // lab: main heap free space / largest block and rbtag's free queues, every 25 logic frames
    const u8* hr = guard.GetSystem().GetMemory().GetRAM();
    const u32 fr = Rd32(hr, ADDR_COUNTER);
    static u32 last = 0;
    if (fr >= last + 25 || fr + 100 < last)
    {
      last = fr;
      u32 lg = 0, nd = 0;
      const u32 tot = HeapFreeFwd(hr, 0, &lg, &nd);
      fprintf(stderr, "FFNYRB HEAP0 frame %u%s: free %u KB in %u blocks, largest %u KB; deferred %u, delayed %u\n", fr,
              s_resim_target ? " (resim)" : "", tot / 1024, nd, lg / 1024, Rd32(hr, 0x8049B400), Rd32(hr, 0x8049C000));
    }
  }
  if (s_snd_sending)
  {
    // a held sound call returned here: the next one, or go on where the PRE hook left off
    s_snd_sending = false;
    if (SndSendNext(ppc, s_snd_end_after ? 0xFFFFFFFF : SndReadyUpTo(Rd32(ram, ADDR_COUNTER))))
      return;
    LoadRegs(ppc, s_snd_regs);
    ppc.npc = s_snd_regs_npc;
    if (s_snd_end_after)
    {
      s_snd_end_after = false;
      EndFight(system, ram, s_snd_end_frame, s_snd_end_reason);
    }
    return;
  }
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
  // the game is still in fight mode (the match update ran in the previous update; see HookMatchUpdate)
  const bool in_match = !s_match_hook || Rd32(ram, W_MATCH) != 0;
  Wr32(ram, W_MATCH, 0);
  // the fight loop runs and the fight is not decided yet. After an environmental KO (thrown out of a window) the
  // loop state stays 1 through the KO scene, results and rematch screen (Robert, 23 / 24 October); those must run in
  // lockstep after the resync, not in rollback (the rematch choice went different ways on the two machines).
  s_left_match = !in_match;
  bool fight = in_match && Rd32(ram, ADDR_FIGHT) == 1 && !(Rd32(ram, ADDR_FIGHT_FLAGS) & FIGHT_DECIDED);
  s_real_fight_now = fight;  // the fight itself is running (not the menus / KO scenes / loading of menus mode)
  // lab: FFNY_RB_ALWAYS=F -> rollback runs everywhere (menus, loading, fights) from logic frame F on
  static const u32 s_always = std::getenv("FFNY_RB_ALWAYS") ? u32(std::atoi(std::getenv("FFNY_RB_ALWAYS"))) : 0;
  if ((s_always && f >= s_always) || s_menus_on)
  {
    fight = true;
    s_left_match = false;
  }

  if (!s_active)
  {
    if (s_force_end.exchange(false))
    {
      // the host ends the fight for everyone, but this machine is not in rollback mode (its fight already ended
      // without the resync, or never started): pause here and take the host's state like everyone else
      Report("FFNY: the host ended the fight for everyone (" + s_force_reason +
             "); this machine was not in a rollback fight - taking the host's state");
      if (s_fight_end_cb)
      {
        s_resync_since_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();
        s_resync_waiting = true;
        s_fight_end_cb(system, f);
      }
      return;
    }
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
    DrainSysPending(ram);
    s_resim_frames++;
    if (f > s_hashed_upto && MinConfirmed() >= f)
    {
      HashFrame(ram, f);
      s_hashed_upto = f;
    }
    return;
  }
  // lab: re-simulations that had to let emulated time pass (every thread waited for hardware): where the main thread
  // waited (its saved context: srr0, lr, call chain)
  if (s_log && s_resim_target)
  {
    static u64 last_idle = 0;
    const u64 idle = system.GetCoreTiming().GetFrozenIdleCycles();
    static u32 shown = 0;
    if (idle != last_idle && shown < 60)
    {
      shown++;
      // the wait already ended (we are at the next update start): report the frame
      fprintf(stderr, "FFNYRB FROZEN IDLE before update %u (re-simulation): %llu cycles let through\n", f,
              (unsigned long long)(idle - last_idle));
    }
    last_idle = idle;
  }
  // lab: FFNY_RB_THREADS=f1,f2,... -> the OS thread list at those frames (address, state, priority, the queue it waits
  // on, saved pc / lr)
  {
    static std::vector<u32> tf;
    static bool tf_read = false;
    if (!tf_read)
    {
      tf_read = true;
      if (const char* t = std::getenv("FFNY_RB_THREADS"))
      {
        std::stringstream ss(t);
        std::string x;
        while (std::getline(ss, x, ','))
          tf.push_back(u32(std::atoi(x.c_str())));
      }
    }
    if (std::find(tf.begin(), tf.end(), f) != tf.end() && !s_resim_target)
      for (u32 t = Rd32(ram, 0x800000DC), k = 0; t && k < 64; t = Rd32(ram, t + 0x2FC), k++)
        fprintf(stderr, "FFNYRB THREAD frame %u: %08x state %u prio %d waits %08x pc %08x lr %08x stack %08x-%08x\n", f, t,
                Rd32(ram, t + 0x2C8) >> 16, s32(Rd32(ram, t + 0x2D0)), Rd32(ram, t + 0x2DC), Rd32(ram, t + 0x198),
                Rd32(ram, t + 0x84), Rd32(ram, t + 0x308), Rd32(ram, t + 0x304));
  }
  const bool caught_up = s_resim_target != 0;  // f == target: this frame's local input is known
  s_resim_target = 0;
  if (caught_up)
    system.GetCoreTiming().SetFrozen(false);  // the re-simulation is over: emulated time runs again
  if (caught_up)
    StressCheck(ram, f, false);
  Pump();

  if (!caught_up)
  {
    // Logic counter stalls (26 October): while the fight loads, the update runs again and again without moving the
    // logic counter (lab: 9270). Frame numbers then no longer name one state: a rollback to an earlier frame would undo
    // every stalled update (the loading screen's progress) and a re-simulation runs it once - the stress run looped at
    // 9270 forever. A stall therefore waits for all inputs up to f (below), and no rollback goes back to f or before.
    s_stall_now = f == s_prev_first_f;
    if (s_stall_now)
    {
      s_stall_guard = f;
      s_stalls++;
    }
    s_prev_first_f = f;
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
    // Every frame gets its local input exactly once. Before 27 October a stalled update (the same logic frame again
    // while a fight loads - in menus mode that is under this layer) sampled the pad again for frame f + delay and sent
    // it again; the other machine keeps the first copy it received, this one used the newest -> different inputs for
    // one frame whenever the pad changed during the loading (Robert's test: desync 10-26 frames into the next fight,
    // both times). Now a frame already captured is never captured again; a larger delay (fight -> menus) fills the
    // frames in between with the current pad, a smaller one (menus -> fight) waits until f + delay is new again.
    const u32 delay = CaptureDelay();
    s_cur_delay = delay;
    for (int p = 0; p < 4; p++)
    {
      if (!(s_settings.local_ports >> p & 1))
        continue;
      const u32 tf = f + delay;
      if (tf <= s_confirmed[p])
      {
        SendInputs(p, s_confirmed[p]);  // nothing new: still send (unacknowledged frames, our frame / lead for time sync)
        continue;
      }
      Record rec;
      memcpy(rec.data(), ram + ((tick_obj + SNAP_OFS + REC_OFS + p * REC_SIZE) & 0x01FFFFFF), REC_SIZE);
      for (u32 t = s_confirmed[p] + 1; t <= tf; t++)
      {
        FrameInput& fi = s_input[p][t % RING];
        fi.frame = t;
        fi.have = true;
        fi.rec = rec;
        // lab: FFNY_RB_INPUTREPLAY=file (written by FFNY_RB_INPUTLOG) -> the local inputs of those frames come from it
        ReplayInput(p, t, fi.rec);
        static FILE* il = std::getenv("FFNY_RB_INPUTLOG") ? fopen(std::getenv("FFNY_RB_INPUTLOG"), "w") : nullptr;
        if (il)
        {
          fprintf(il, "%d %u", p, t);
          for (u32 k = 0; k < REC_SIZE; k++)
            fprintf(il, "%s%02x", k % 4 ? "" : " ", fi.rec[k]);
          fprintf(il, "\n");
          fflush(il);
        }
      }
      s_confirmed[p] = tf;
      SendInputs(p, tf);
    }
    if (s_loop)
    {
      // lab loopback: pad 1's real input of this tick is delivered as remote input s_loop frames later
      Record r;
      memcpy(r.data(), ram + ((tick_obj + SNAP_OFS + REC_OFS + 1 * REC_SIZE) & 0x01FFFFFF), REC_SIZE);
      ReplayInput(1, f + s_settings.input_delay, r);
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
      fprintf(stderr, "FFNYRB file ids: %u virtual opens, %u returned, %u releases taken back, %u done; %u reads hurried\n",
              s_af_opened, s_af_orphans, s_af_unfreed, s_af_freed, s_fs_hurried);
      {
        std::string per;
        for (size_t i = 0; i < CARD_FUNCS.size(); i++)
          if (s_card_fn_calls[i])
            per += fmt::format(" {:08x}:{}", CARD_FUNCS[i], s_card_fn_calls[i]);
        fprintf(stderr, "FFNYRB memory card calls per function:%s; device commands %u\n", per.c_str(), g_card_activity);
      }
      fprintf(stderr, "FFNYRB worker threads: %zu known, busy at %u update starts (last frame %u, thread %08x), %u messages to them, %u frames abandoned\n",
              s_idle_wait.size(), s_async_busy_frames, s_async_last, s_async_last_thread, s_async_barriers, s_async_aborts);
      fprintf(stderr, "FFNYRB lockstep outside the fight: %u frames; barrier calls with a late misprediction %u\n", s_menu_lock_frames, s_barrier_late);
      fprintf(stderr, "FFNYRB logic counter stalls: %u (last at frame %u)\n", s_stalls, s_stall_guard);
      fprintf(stderr, "FFNYRB music cues: %u not sent again in re-simulations, %u sent by re-simulations, %u refused by the player (the logic got 1)\n", s_music_deduped, s_music_resent, s_music_refused);
      fprintf(stderr, "FFNYRB memory card: %u calls, last activity frame %u, %u barrier waits (%.1f ms), %u frames abandoned, %u calls in re-simulations\n",
              s_card_calls, s_card_last, s_card_waits, s_card_wait_us / 1000.0, s_card_aborts, s_card_resim_calls);
      fprintf(stderr, "FFNYRB frozen time: %llu cycles of re-simulation not counted, %llu idle cycles let through, %llu over the budget\n",
              (unsigned long long)system.GetCoreTiming().GetFrozenCycles(),
              (unsigned long long)system.GetCoreTiming().GetFrozenIdleCycles(),
              (unsigned long long)system.GetCoreTiming().GetFrozenLeakCycles());
    }
    StressCheck(ram, f, true);
    static u32 pause_lo = 0, pause_hi = 0;
    static bool pause_read = false;
    if (!pause_read)
    {
      pause_read = true;
      if (const char* pz = std::getenv("FFNY_RB_STRESS_PAUSE"))  // lab: lo:hi -> no forced rollbacks into [lo, hi)
        sscanf(pz, "%u:%u", &pause_lo, &pause_hi);
    }
    const bool stress_paused = pause_hi && f >= pause_lo && f < pause_hi + s_stress + 1;
    if (s_stress && f > s_start_frame + s_stress && f >= s_stress_from && fight && !stress_paused)
    {
      const u32 t = f - 1 - (std::rand() % s_stress);
      if (!CardLocked(t) && t > s_stall_guard && t > s_last_lock_frame)  // lockstep frames are never mispredicted
        s_mispredict_from = t;
    }
  }

  // too far ahead of the other side (or a fight that ended on this machine with frames still
  // unconfirmed): wait for its inputs
  const auto wait_start = std::chrono::steady_clock::now();
  bool waited = false;
  // file loads: the loading code and its bookkeeping are not rewound, the objects that receive the data are. A
  // rollback back across a finished load / conversion leaves those objects empty for good (crash at the KO scene,
  // 23 October). So while files load (and LOAD_TAIL frames after), frame f runs only once every input up to f is
  // known: the frames of a load are never rolled back. Costs a wait per frame of about ping - input delay.
  if (Rd32(ram, 0x804018D4) != 0 || Rd32(ram, 0x804018E0) != 0)
  {
    if (FsBusyForLogic(ram))
    {
      s_fs_last_busy = std::max(s_fs_last_busy, f);
      s_fs_real_busy = std::max(s_fs_real_busy, f);
    }
    else
    {
      s_fs_stream_only++;
      s_fs_stream_last = std::max(s_fs_stream_last, f);
    }
  }
  if (s_async_on)
  {
    if (s_idle_wait.empty())
      LearnIdleWaits(ram);
    if (const u32 bt = BusyWorker(ram))
    {
      s_async_last = std::max(s_async_last, f);
      s_async_last_thread = bt;
      s_async_busy_frames++;
      if (s_log && s_async_busy_frames % 25 == 1)
      {
        // lab: where the busy worker is (its saved context and call chain)
        u32 sp = Rd32(ram, bt + 4);
        std::string ch;
        for (int k = 0; k < 10 && (sp & 0xFE000000) == 0x80000000; k++)
        {
          const u32 nsp = Rd32(ram, sp);
          if ((nsp & 0xFE000000) != 0x80000000)
            break;
          ch += fmt::format(" {:08x}", Rd32(ram, nsp + 4));
          sp = nsp;
        }
        fprintf(stderr, "FFNYRB worker %08x busy at frame %u: state %u waits %08x pc %08x lr %08x chain%s\n", bt, f,
                Rd32(ram, bt + 0x2C8) >> 16, Rd32(ram, bt + 0x2DC), Rd32(ram, bt + 0x198), Rd32(ram, bt + 0x84), ch.c_str());
      }
    }
  }
  if (g_card_activity != s_card_seen)
  {
    s_card_seen = g_card_activity;
    if (s_card_on)
      s_card_last = std::max(s_card_last, f > 0 ? f - 1 : 0);
  }
  // Menus mode / lab always mode: outside the running fight every frame runs with all inputs known (like normal
  // netplay there) - so the menus, KO scenes, results, rematch and loading are never rolled back and none of their
  // asynchronous work (card, loader, worker threads, front-end loads) can be crossed by a rollback. Rollback (no
  // waiting) only in the fight itself. Lab: FFNY_RB_MENUROLLBACK=1 lets the menus roll back too (the card / worker /
  // stall rules below then do the protecting).
  static const bool menu_rollback = std::getenv("FFNY_RB_MENUROLLBACK") != nullptr;
  const bool menu_lock = fight && !s_real_fight_now && !menu_rollback && (s_menus_on || (s_always && f >= s_always));
  if (menu_lock)
    s_menu_lock_frames++;
  const bool card_lock = fight && (CardLocked(f) || (s_stall_guard && f <= s_stall_guard + 1) || menu_lock);
  const bool stream_lock = s_settings.load_lockstep && fight && !s_stream_nolock && s_fs_stream_last &&
                           f <= s_fs_stream_last + s_stream_tail;
  const bool lockstep = (s_settings.load_lockstep && fight && f <= s_fs_last_busy + LOAD_TAIL) || stream_lock || card_lock;
  if (lockstep)
    s_last_lock_frame = std::max(s_last_lock_frame, f);
  if (lockstep)
  {
    s_lockstep_frames++;
    if (s_settings.load_lockstep && fight && f <= s_fs_last_busy + LOAD_TAIL)
      s_ls_files++;
    else if (stream_lock)
      s_ls_stream++;
    else
      s_ls_other++;
    if (f > s_fs_last_busy && f > s_start_frame + START_LOCKSTEP)
      s_lockstep_tail++;  // only because of the tail: the file system is idle and no conversion is this recent
  }
  s_wait_where = "waiting for the other side's inputs (start of an update)";
  while (true)
  {
    const u32 mc = MinConfirmed();
    const bool ahead = f > mc + s_settings.max_rollback;
    const bool ending = !fight && mc + 1 < f;
    const bool need = lockstep && mc < f;  // in lockstep: also roll back only with all inputs known
    if (!need && (s_mispredict_from != 0xFFFFFFFF || (!ahead && !ending)))
      break;
    if (s_force_end)
      break;  // the host ends the fight for everyone (below)
    if (system.GetCPU().GetState() == CPU::State::PowerDown)
      break;  // emulation is being stopped (closing Dolphin must not wait here)
    if (std::chrono::steady_clock::now() - wait_start > std::chrono::seconds(10))
    {
      // the other side stopped (its fight ended at another frame, it paused, or the connection is gone). Before 24
      // October rollback was switched off here for the rest of the session and both machines went on unsynchronised;
      // now this machine ends the fight and waits for the host's state like after a normal fight.
      fprintf(stderr, "FFNYRB: no inputs from the other side for 10 s - fight ended here\n");
      // the held sound calls of the frames this machine ran go out first (on the host they are the real timeline)
      if (SndFlushThenEnd(ppc, f, "no inputs from the other side for 10 s"))
        return;
      EndFight(system, ram, f, "no inputs from the other side for 10 s");
      return;
    }
    waited = true;
    if (s_loop && need)
    {
      // lab loopback in lockstep: the "other side" would send these anyway
      for (const auto& q : s_loop_q)
        OnRemoteInput(1, q.first, q.second.data());
      s_loop_q.clear();
    }
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
  s_wait_where = nullptr;
  if (waited)
  {
    // we were slowed down by the wait: keep that (no catch-up sprint that would put us ahead again)
    if (s_sync_enabled)
      system.GetCoreTiming().AddThrottleDelay(std::chrono::steady_clock::now() - wait_start);
    s_waits++;
    const u64 us = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::steady_clock::now() - wait_start)
                       .count();
    s_wait_us += us;
    if (lockstep)
      s_lockstep_us += us;
  }

  // Menus mode (28 October): rbtag's "fight" word decides whether frees of its tagged main-heap blocks by other
  // threads are queued until the fight ends (point 3), whether system allocations go to SysHeap and whether the main
  // thread's delayed frees wait (point 7). In menus mode the fight never ended, so the queue never emptied: in the lab
  // two sound banks (1154 + 814 KB) and a 1 MB file buffer stayed queued, the main heap had 1.4 MB left, and quitting to
  // the main menu crashed (mainMenu.big unpacks to 2 MB: the allocation got 0 and the decompressor wrote to address 0).
  // Outside the running fight every frame runs with all inputs known, so nothing before it can be rolled back any
  // more: there the word is 0 (frees done at once, the queue emptied, as between fights without menus mode).
  if (s_menus_on && s_rbtag_redirected)
    Wr32(ram, W_FIGHTW, (menu_lock && MinConfirmed() >= f) ? 0 : 1);

  if (s_force_end.exchange(false))
  {
    // the host ends the fight for everyone: the machines did not reach the end of the fight at the same frame (e.g.
    // the post-KO screens went different ways). This machine's state does not matter: the host's replaces it.
    if (SndFlushThenEnd(ppc, f, s_force_reason))
      return;
    EndFight(system, ram, f, s_force_reason);
    return;
  }

  if (s_mispredict_from != 0xFFFFFFFF && s_stress && (CardLocked(std::max(s_mispredict_from, s_start_frame)) ||
                                                      std::max(s_mispredict_from, s_start_frame) <= s_stall_guard ||
                                                      std::max(s_mispredict_from, s_start_frame) <= s_last_lock_frame))
    s_mispredict_from = 0xFFFFFFFF;  // lab stress: frames that ran in lockstep are never mispredicted
  if (s_mispredict_from != 0xFFFFFFFF)
  {
    const u32 s = std::max(s_mispredict_from, s_start_frame);
    s_mispredict_from = 0xFFFFFFFF;
    if (s < f && LoadState(system, s))
    {
      if (!s_diag_heap_reported)
      {
        // a list broken right here was broken by the restore itself (or was broken in the saved state)
        const std::string p = FreeListProblem(ram, true);
        if (!p.empty())
        {
          s_diag_heap_reported = true;
          Report("FFNY WARNING: broken heap free list right after the rewind " + std::to_string(f) + "->" +
                 std::to_string(s) + " (restored state): " + p + ". " + RbtagSummary(ram));
        }
      }
      s_rollbacks++;
      s_last_rb_from = f;
      s_last_rb_to = s;
      if (s_fs_done_frame >= s)
        s_rb_across_load++;  // a file request finished in a frame that is now thrown away
      if (s_conv_frame >= s)
      {
        s_rb_across_conv++;  // the game converted a loaded file in a frame that is now thrown away
        if (!s_conv_warned && !InstantDiscReads())  // with instant loads the re-simulation converts it again
        {
          s_conv_warned = true;
          Report("FFNY WARNING: rollback " + std::to_string(f) + "->" + std::to_string(s) +
                 " goes back across a file conversion in frame " + std::to_string(s_conv_frame) +
                 " (its data may now be missing in this fight)");
        }
      }
      if (s_log && s_rollbacks % 50 == 0)
        fprintf(stderr, "FFNYRB rollback %u: %u -> %u fight %u scr %08x\n", s_rollbacks, f, s, Rd32(ram, ADDR_FIGHT), Rd32(ram, 0x803EA56C));
      s_max_depth = std::max(s_max_depth, f - s);
      s_resim_target = f;
      // Time-free re-simulation (26 October): the re-simulated frames take no emulated time. Before, every rollback
      // added the re-simulation's cycles to emulated time, so per logic frame the hardware and the game's other
      // threads (VI pacing, audio / streams, the memory-card thread, loader threads) got a different amount of time
      // than without rollback and than on the other machine (lab, same inputs: 116 VIs more by frame 4299 in stress
      // mode; the SELECT USER ID memory-card step then went another way and the menus never reached the fighter
      // select). Frozen time keeps the hardware's view = the first runs only.
      static const bool no_freeze = std::getenv("FFNY_RB_NOFREEZE") != nullptr;
      if (!no_freeze)
        system.GetCoreTiming().SetFrozen(true, u64(f - s) * FROZEN_BUDGET_PER_FRAME);
      // we are frame s again (registers restored); do the replaced instruction again
      ppc.gpr[3] = ppc.gpr[26];
      ppc.npc = ADDR_PRE + 4;
      DrawFrameBegin(ram, s, false);
      Inject(ram, ppc.gpr[28], s);
      SaveState(system, s);
      {
        const u32 old = Rd32(ram, W_RB);
        Wr32(ram, W_RB, old ? std::min(old, s + 1) : s + 1);
        FreeThrownSysAllocs(ram, s);
        DrainSysPending(ram);
        SndDropFrom(s);
        AfRewind(ram, s);
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
    // fight over and every input confirmed: back to normal netplay (the held sound calls go out first)
    if (SndFlushThenEnd(ppc, f, ""))
      return;
    EndFight(system, ram, f, "");
    return;
  }

  DrawFrameBegin(ram, f, true);  // first run of update f (also right after a re-simulation caught up)
  Inject(ram, tick_obj, f);
  SaveState(system, f);
  s_stall_now = false;
  LoaderDropConfirmed(std::min(MinConfirmed(), f - 1), f, ram);
  Wr32(ram, W_CONF, std::min(MinConfirmed(), f - 1));
  Wr32(ram, W_FRAME, f);
  DrainSysPending(ram);
  if (f > s_hashed_upto && MinConfirmed() >= f)
  {
    HashFrame(ram, f);
    s_hashed_upto = f;
  }
  AfConfirm(ram, SndReadyUpTo(f));
  while (!s_music_sent.empty() && s_music_sent.front().first.frame + 1 <= SndReadyUpTo(f))
    s_music_sent.pop_front();
  SndStartSends(ppc, SndReadyUpTo(f));
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
  snprintf(b, sizeof b, "FFNY CRASH (Dolphin): invalid %s %08x at PC %08x LR %08x, frame %u, fight %s%s, calls %s. ",
           write ? "write to" : "read from", address, pc, ppc.spr[SPR_LR], ram ? Rd32(ram, ADDR_COUNTER) : 0,
           s_active ? "running" : "not running", s_resim_target ? ", during a re-simulation" : "",
           ram ? CallChain(ram, ppc.gpr[1], 6).c_str() : "-");
  std::string heap;
  if (ram)
  {
    const std::string p = FreeListProblem(ram);
    heap = " | " + (p.empty() ? std::string("heap free lists ok") : "BROKEN " + p) + " | " + RbtagSummary(ram);
  }
  Report(std::string(b) + (ram ? FightSummary(ram) : std::string()) + heap);
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
  const std::string p = FreeListProblem(ram);
  Report(std::string(b) + FightSummary(ram) + " | calls " + (ok ? CallChain(ram, Rd32(ram, ctx + 4), 6) : "-") +
         " | " + (p.empty() ? std::string("heap free lists ok") : "BROKEN " + p) + " | " + RbtagSummary(ram));
}
}  // namespace FFNYRollback

namespace FFNYRollback
{
void LabSchedulePoke(u32 addr, u32 value, u32 frame)
{
  LabSchedulePokeInner(addr, value, frame);
}
}  // namespace FFNYRollback
