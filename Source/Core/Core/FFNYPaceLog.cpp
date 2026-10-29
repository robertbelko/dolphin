// FFNY frame pacing logger, see FFNYPaceLog.h.

#include "Core/FFNYPaceLog.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>

#include "Common/CommonPaths.h"
#include "Common/FileUtil.h"
#include "Common/IOFile.h"
#include "Common/Version.h"
#include "Core/Config/GraphicsSettings.h"
#include "Core/Config/MainSettings.h"
#include "Core/Core.h"
#include "Core/CoreTiming.h"
#include "Core/HLE/HLE.h"
#include "Core/HW/Memmap.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/System.h"
#include "VideoCommon/CommandProcessor.h"

#ifdef _WIN32
#include <Windows.h>
#endif

namespace FFNYPaceLog
{
namespace
{
using Clock = std::chrono::steady_clock;

// game addresses (USA GameCube Start.dol, main loop 0x80004CC0-0x80004DA0)
constexpr u32 ADDR_PASS = 0x80004CD0;      // li r3,0 (pass start, before bl 0x8024E170)
constexpr u32 ADDR_TICKLOOP = 0x80004CD8;  // b 0x80004CF8
constexpr u32 ADDR_UPD = 0x80004D24;       // bl 0x800830D4
constexpr u32 ADDR_UPD_END = 0x80004D28;   // cmpwi r3,0
constexpr u32 ADDR_PRERENDER = 0x80004D80;  // cmpwi r24,0
constexpr u32 ADDR_REND = 0x80004D98;      // bl 0x800838F4
constexpr u32 ADDR_REND_END = 0x80004D9C;  // bl 0x8001EDD4
constexpr u32 INSN_UPD = 0x4807E3B1;
constexpr u32 INSN_REND = 0x4807EB5D;
constexpr u32 ADDR_COUNTER = 0x80440C24;  // logic update counter
constexpr u32 ADDR_FIGHT = 0x803F1000;    // 1 while a fight runs (rbtag DOLs)

bool s_env_checked = false;
bool s_wanted = false;
std::string s_path_env;
bool s_installed = false;
File::IOFile s_iofile;
FILE* s_file = nullptr;
u64 s_field = 0;
Clock::time_point s_t0, s_prev_host;
u64 s_prev_ticks = 0;
Clock::time_point s_last_new_frame;
bool s_have_new_frame = false;

// main loop (CPU thread)
u32 s_passes = 0, s_updates = 0, s_max_updates_in_pass = 0, s_updates_this_pass = 0;
u64 s_upd_ticks = 0, s_rend_ticks = 0, s_wait_ticks = 0;
u64 s_t_pass = 0, s_t_upd = 0, s_t_rend = 0;
s64 s_rend_end_tick = -1;
u32 s_tick_obj = 0;

// presenter / GPU side (any thread)
std::atomic<u32> s_new_frames{0}, s_dups{0}, s_imm{0}, s_last_xfb{0};
std::atomic<u32> s_copies{0};
std::atomic<s64> s_copy_tick{-1};
std::atomic<s64> s_copy_host_us{-1};

u32 Rd32(const u8* ram, u32 a)
{
  a &= 0x01FFFFFF;
  return (u32(ram[a]) << 24) | (u32(ram[a + 1]) << 16) | (u32(ram[a + 2]) << 8) | ram[a + 3];
}

s64 HostUs(Clock::time_point t)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(t - s_t0).count();
}

void Open()
{
  std::string path = s_path_env;
  if (path.empty() || path == "1")
  {
    const std::time_t now = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    File::CreateFullPath(File::GetUserPath(D_LOGS_IDX));
    path = File::GetUserPath(D_LOGS_IDX) + "ffny_pacelog_" + stamp + ".csv";
  }
  s_iofile.Open(path, "w");
  s_file = s_iofile.GetHandle();
  if (!s_file)
  {
    std::fprintf(stderr, "FFNY pacelog: cannot write %s\n", path.c_str());
    s_wanted = false;
    return;
  }
  std::fprintf(s_file,
               "# FFNY pacelog v1, Dolphin %s; overclock %s %.2f; dual core %d; sync gpu %d; SyncGpuOverclock %.2f; "
               "immediate XFB %d; skip duplicate XFBs %d; vsync %d; shader mode %d; backend %s; speed limit %.2f\n",
               Common::GetScmDescStr().c_str(), Config::Get(Config::MAIN_OVERCLOCK_ENABLE) ? "on" : "off",
               Config::Get(Config::MAIN_OVERCLOCK), Config::Get(Config::MAIN_CPU_THREAD),
               Config::Get(Config::MAIN_SYNC_GPU), Config::Get(Config::MAIN_SYNC_GPU_OVERCLOCK),
               Config::Get(Config::GFX_HACK_IMMEDIATE_XFB), Config::Get(Config::GFX_HACK_SKIP_DUPLICATE_XFBS),
               Config::Get(Config::GFX_VSYNC), static_cast<int>(Config::Get(Config::GFX_SHADER_COMPILATION_MODE)),
               Config::Get(Config::MAIN_GFX_BACKEND).c_str(), Config::Get(Config::MAIN_EMULATION_SPEED));
  std::fprintf(s_file, "field,host_ms,dt_ms,work_ms,sleep_ms,late_ms,new,dup,imm,since_new_ms,xfb,copies,copy_at,"
                       "passes,updates,max_upd,upd_kc,rend_kc,wait_kc,rend_end_at,field_kc,gpu_kc,gpu_late,fifo_kb,"
                       "verts,xf_words,shaders,shader_ms,gpu_ms,present_ms,counter,fight,pending,marker\n");
  s_t0 = Clock::now();
  s_prev_host = s_t0;
  s_field = 0;
  s_have_new_frame = false;
  g_on = true;
}

bool Marker()
{
#ifdef _WIN32
  return (GetAsyncKeyState(VK_SCROLL) & 1) != 0;
#else
  return false;
#endif
}
}  // namespace

void OnViField(Core::System& system)
{
  if (!s_env_checked)
  {
    s_env_checked = true;
    if (const char* e = std::getenv("FFNY_PACELOG"))
    {
      s_wanted = true;
      s_path_env = e;
    }
    else if (File::Exists(File::GetUserPath(D_USER_IDX) + "ffny_pacelog.on"))
    {
      s_wanted = true;
    }
  }
  if (!s_wanted || s_installed)
    return;
  const u8* ram = system.GetMemory().GetRAM();
  if (!ram || Rd32(ram, ADDR_UPD) != INSN_UPD || Rd32(ram, ADDR_REND) != INSN_REND)
    return;  // game code not loaded yet
  HLE::Patch(system, ADDR_PASS, "FFNYPacePass");
  HLE::Patch(system, ADDR_TICKLOOP, "FFNYPaceTickLoop");
  HLE::Patch(system, ADDR_UPD, "FFNYPaceUpd");
  HLE::Patch(system, ADDR_UPD_END, "FFNYPaceUpdEnd");
  HLE::Patch(system, ADDR_PRERENDER, "FFNYPacePreRender");
  HLE::Patch(system, ADDR_REND, "FFNYPaceRend");
  HLE::Patch(system, ADDR_REND_END, "FFNYPaceRendEnd");
  s_installed = true;
  if (!s_file)
    Open();
}

void Shutdown()
{
  g_on = false;
  if (s_file)
  {
    s_iofile.Close();
    s_file = nullptr;
  }
  s_installed = false;
  s_env_checked = false;
}

static u64 Ticks(const Core::CPUThreadGuard& guard)
{
  return guard.GetSystem().GetCoreTiming().GetTicks();
}

void HookPassStart(const Core::CPUThreadGuard& guard)
{
  s_t_pass = Ticks(guard);
  s_passes++;
  s_updates_this_pass = 0;
}

void HookTickLoop(const Core::CPUThreadGuard& guard)
{
  if (s_t_pass)
    s_wait_ticks += Ticks(guard) - s_t_pass;
}

void HookUpdateStart(const Core::CPUThreadGuard& guard)
{
  s_t_upd = Ticks(guard);
}

void HookUpdateEnd(const Core::CPUThreadGuard& guard)
{
  if (s_t_upd)
    s_upd_ticks += Ticks(guard) - s_t_upd;
  s_t_upd = 0;
  s_updates++;
  s_updates_this_pass++;
}

void HookPreRender(const Core::CPUThreadGuard& guard)
{
  s_tick_obj = guard.GetSystem().GetPPCState().gpr[28];
  if (s_updates_this_pass > s_max_updates_in_pass)
    s_max_updates_in_pass = s_updates_this_pass;
}

void HookRenderStart(const Core::CPUThreadGuard& guard)
{
  s_t_rend = Ticks(guard);
}

void HookRenderEnd(const Core::CPUThreadGuard& guard)
{
  const u64 t = Ticks(guard);
  if (s_t_rend)
    s_rend_ticks += t - s_t_rend;
  s_t_rend = 0;
  s_rend_end_tick = static_cast<s64>(t);
}

void OnXfbCopy(Core::System& system, u32 dest_addr)
{
  if (!g_on)
    return;
  s_copies++;
  s_copy_host_us = HostUs(Clock::now());
  // the emulated time is only meaningful when the GPU runs on the CPU thread
  s_copy_tick = system.IsDualCoreMode() ? -1 : static_cast<s64>(system.GetCoreTiming().GetTicks());
}

void OnViSwap(u32 xfb_addr, bool duplicate, bool immediate)
{
  if (!g_on)
    return;
  s_last_xfb = xfb_addr;
  if (immediate)
    s_imm++;
  else if (duplicate)
    s_dups++;
  else
    s_new_frames++;
}

void OnEndField(Core::System& system, u64 ticks)
{
  if (!g_on || !s_file)
    return;
  const Clock::time_point now = Clock::now();
  const double dt_ms = std::chrono::duration<double, std::milli>(now - s_prev_host).count();
  const double sleep_ms = g_sleep_us.exchange(0) / 1000.0;
  const double late_ms = g_late_us.exchange(0) / 1000.0;
  const u32 new_frames = s_new_frames.exchange(0), dups = s_dups.exchange(0), imm = s_imm.exchange(0);
  if (new_frames || imm)
  {
    s_last_new_frame = now;
    s_have_new_frame = true;
  }
  const double since_new =
      s_have_new_frame ? std::chrono::duration<double, std::milli>(now - s_last_new_frame).count() : -1.0;
  const u64 field_ticks = s_prev_ticks && ticks > s_prev_ticks ? ticks - s_prev_ticks : 0;
  auto at = [&](s64 t) -> int {
    if (t < 0 || !field_ticks || static_cast<u64>(t) < s_prev_ticks)
      return -1;
    return static_cast<int>((static_cast<u64>(t) - s_prev_ticks) * 100 / field_ticks);
  };
  const u32 copies = s_copies.exchange(0);
  const s64 copy_tick = s_copy_tick.exchange(-1);
  const u8* ram = system.GetMemory().GetRAM();
  const u32 counter = ram ? Rd32(ram, ADDR_COUNTER) : 0;
  const u32 fight = ram ? Rd32(ram, ADDR_FIGHT) : 0;
  const u32 pending = (ram && (s_tick_obj >> 24) == 0x80) ? Rd32(ram, s_tick_obj + 0x17C8) : 0;
  const u32 fifo = system.GetCommandProcessor().GetFifo().CPReadWriteDistance.load(std::memory_order_relaxed);
  const bool marker = Marker();

  std::fprintf(s_file,
               "%llu,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%.2f,%08x,%u,%d,%u,%u,%u,%llu,%llu,%llu,%d,%llu,%llu,%u,%u,"
               "%u,%u,%u,%.3f,%.3f,%.3f,%u,%u,%u,%d\n",
               static_cast<unsigned long long>(s_field), HostUs(now) / 1000.0, dt_ms, dt_ms - sleep_ms, sleep_ms,
               late_ms, new_frames, dups, imm, since_new, s_last_xfb.load(), copies, copies ? at(copy_tick) : -1,
               s_passes, s_updates, s_max_updates_in_pass, static_cast<unsigned long long>(s_upd_ticks / 1000),
               static_cast<unsigned long long>(s_rend_ticks / 1000),
               static_cast<unsigned long long>(s_wait_ticks / 1000), at(s_rend_end_tick),
               static_cast<unsigned long long>(field_ticks / 1000),
               static_cast<unsigned long long>(g_gpu_cycles.exchange(0) / 1000), g_gpu_out_of_time.exchange(0),
               fifo / 1024, g_vertices.exchange(0), g_xf_words.exchange(0), g_shader_compiles.exchange(0),
               g_shader_us.exchange(0) / 1000.0, g_gpu_host_us.exchange(0) / 1000.0,
               g_present_us.exchange(0) / 1000.0, counter, fight, pending, marker ? 1 : 0);
  if ((s_field & 63) == 0 || marker)
    std::fflush(s_file);

  s_field++;
  s_prev_host = now;
  s_prev_ticks = ticks;
  s_passes = s_updates = s_max_updates_in_pass = 0;
  s_upd_ticks = s_rend_ticks = s_wait_ticks = 0;
  s_rend_end_tick = -1;
}
}  // namespace FFNYPaceLog
