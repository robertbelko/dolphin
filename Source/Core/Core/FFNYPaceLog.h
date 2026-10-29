// FFNY frame pacing logger (Def Jam: Fight for NY, GNWE69).
//
// Writes one CSV line per emulated VI field to <Dolphin user folder>/Logs/ffny_pacelog_<date>_<time>.csv:
// host timing (work / throttle sleep / present), what the VI showed (new picture or duplicate), the game's main loop
// (passes, logic updates, emulated cycles in update / render, time of the XFB copy inside the field) and the emulated
// GPU (cycles used, whether it ran out of its time budget, FIFO backlog, vertices, XF loads), shader compiles.
// Tools/ffny-pacelog/pacelog.py turns the file into a report that names the cause of every late frame.
//
// On when the environment variable FFNY_PACELOG is set (1 = default file, anything else = file path), or when the file
// <Dolphin user folder>/ffny_pacelog.on exists. Press Scroll Lock while playing to put a marker into the log.
#pragma once

#include <atomic>

#include "Common/CommonTypes.h"

namespace Core
{
class CPUThreadGuard;
class System;
}  // namespace Core

namespace FFNYPaceLog
{
// true while a log file is open (checked by the cheap counters below before doing anything)
inline std::atomic<bool> g_on{false};

// counters fed from the video / timing code (any thread); read and reset once per VI field
inline std::atomic<u64> g_gpu_cycles{0};     // emulated GPU cycles used by the FIFO commands (Dolphin's GPU model)
inline std::atomic<u32> g_gpu_out_of_time{0};  // single core: GPU slices that ended with work left (GPU-bound)
inline std::atomic<u64> g_gpu_host_us{0};    // single core: host time spent running the GPU on the CPU thread
inline std::atomic<u32> g_vertices{0};
inline std::atomic<u32> g_xf_words{0};
inline std::atomic<u32> g_shader_compiles{0};
inline std::atomic<u64> g_shader_us{0};
inline std::atomic<u64> g_present_us{0};
inline std::atomic<u64> g_sleep_us{0};       // throttle: host time slept waiting for real time
inline std::atomic<s64> g_late_us{0};        // throttle: the most the emulation was behind real time (> 0 = behind)

void OnXfbCopy(Core::System& system, u32 dest_addr);           // EFB -> XFB copy executed (GPU side)
void OnViSwap(u32 xfb_addr, bool duplicate, bool immediate);  // presenter: VI scanned out a picture
void OnEndField(Core::System& system, u64 ticks);             // CPU thread, end of each VI field
void OnViField(Core::System& system);                         // CPU thread, field boundary (installs the hooks)
void Shutdown();

// HLE hooks in the game's main loop (HookType::Start)
void HookPassStart(const Core::CPUThreadGuard& guard);    // 0x80004CD0 main-loop pass starts
void HookTickLoop(const Core::CPUThreadGuard& guard);     // 0x80004CD8 pass: 0x8024E170 done, tick loop starts
void HookUpdateStart(const Core::CPUThreadGuard& guard);  // 0x80004D24 bl 0x800830D4 (logic update)
void HookUpdateEnd(const Core::CPUThreadGuard& guard);    // 0x80004D28
void HookPreRender(const Core::CPUThreadGuard& guard);    // 0x80004D80 tick loop done
void HookRenderStart(const Core::CPUThreadGuard& guard);  // 0x80004D98 bl 0x800838F4 (render)
void HookRenderEnd(const Core::CPUThreadGuard& guard);    // 0x80004D9C
// event probes (Logs/ffny_paceev_<date>_<time>.csv)
void HookTickResult(const Core::CPUThreadGuard& guard);  // 0x80004D00 tick() result
void HookPadPush(const Core::CPUThreadGuard& guard);     // 0x800EC4F8 pad thread pushed a snapshot
void HookSleep(const Core::CPUThreadGuard& guard);       // 0x802776E8 OSSleepThread (main thread only)
void HookWake(const Core::CPUThreadGuard& guard);        // 0x802777B4 OSSleepThread returns (main thread only)
}  // namespace FFNYPaceLog
