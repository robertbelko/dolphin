// FFNY rollback netcode (Def Jam: Fight for NY, GNWE69, with the rbstate + rbtag DOL patches).
// Rollback runs only while a fight is running (0x803F1000 == 1); everything else is left to the
// normal (lockstep) netplay. See FFNY_RE_handoff.md, "Rollback netcode".
#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "Common/CommonTypes.h"

namespace Core
{
class CPUThreadGuard;
class System;
}  // namespace Core

struct GCPadStatus;

namespace FFNYRollback
{
// Transport for rollback messages between the players (netplay connection, or UDP in the lab).
class Transport
{
public:
  virtual ~Transport() = default;
  virtual void Send(const std::vector<u8>& msg) = 0;
  // may be lost, duplicated or reordered (inputs: every message repeats what the other side lacks)
  virtual void SendUnreliable(const std::vector<u8>& msg) { Send(msg); }
  // Returns false when no message is waiting.
  virtual bool Receive(std::vector<u8>& msg) = 0;
};

struct Settings
{
  bool enabled = false;
  u32 local_ports = 0;   // bit mask of in-game pads (0-3) played on this machine
  u32 active_ports = 0;  // bit mask of in-game pads played by anyone
  u32 input_delay = 1;   // local input delay in frames
  u32 max_rollback = 7;  // frames
  // while the game loads files (and a little after), run a frame only when every input up to it is confirmed,
  // so no rollback goes back across a finished load / file conversion (that state is not rewound)
  bool load_lockstep = true;
};

// Called by netplay (or the lab environment variables) before / when the game starts.
void Configure(const Settings& settings, Transport* transport);
// Menus mode: input delay (frames) outside the running fight, where every frame runs in lockstep - netplay passes its
// pad buffer (Dolphin's own netplay menus before the first fight use the same). The fight keeps Settings::input_delay.
void SetMenuDelay(u32 frames);
// Called on the CPU thread when a rollback fight is over (all inputs confirmed). Netplay pauses the
// CPU there and resynchronises the whole emulator state from the host before menus continue.
void SetFightEndCallback(std::function<void(Core::System&, u32 frame)> callback);
void Shutdown();
// Debug reports (desync details, crashes) for the players: netplay shows them in the chat window and sends
// them to the other players. Also printed to stderr.
void SetReportCallback(std::function<void(const std::string&)> callback);
// Every report, fight-start / fight-over line and resync line is also appended to
// <Dolphin user folder>/Logs/FFNY_rollback.log (with date and time), for unattended test sessions.
void LogLine(const std::string& text);
// Test tool: during rollback fights this machine's fighter presses plausible fighting inputs by itself
// (moving, attacks, blocks, C-stick flicks), so rollbacks happen without anyone playing.
void SetAutoPlay(bool enabled);
bool AutoPlayEnabled();
void AutoPlayPad(GCPadStatus* pad, int local_pad);
// Called by the MMU right before Dolphin's "Invalid read / write" alert (non-MMU mode).
void OnInvalidAccess(Core::System& system, u32 address, u32 pc, bool write);
bool IsActive();  // a fight is currently running in rollback mode
// netplay thread: the host ends the current fight for everyone (a machine did not reach the fight's end by itself);
// the CPU thread ends it at its next update (or right away when it waits for inputs) and calls the fight-end callback
void RequestForceEnd(const std::string& reason);
void ClearForceEnd();
// disc reads complete at once while FFNY rollback netplay is on (see FFNYRollback.cpp)
bool InstantDiscReads();

// Called once per VI field on the CPU thread: installs the hooks once the game code is in memory.
void OnVIField(Core::System& system);

// HLE hooks (registered in HLE.cpp as fixed Replace hooks).
void HookPreUpdate(const Core::CPUThreadGuard& guard);  // 0x80004D14 (mr r3,r26)
void HookTick(const Core::CPUThreadGuard& guard);       // 0x80004CFC (bl 0x800EC144)
void HookLoader(const Core::CPUThreadGuard& guard);  // 0x80121B4C (EAGL ELF .o loader entry)
void HookRng(const Core::CPUThreadGuard& guard);     // generator 2 entry 0x800B7DAC
void HookRng1(const Core::CPUThreadGuard& guard);    // generator 1 entry 0x8001FC88
void HookRender(const Core::CPUThreadGuard& guard);
void HookMatchUpdate(const Core::CPUThreadGuard& guard);
void HookFsKick(const Core::CPUThreadGuard& guard);  // 0x80244408: file-system request queued (r3 = node)
void HookFsWait(const Core::CPUThreadGuard& guard);
void HookAfSetId(const Core::CPUThreadGuard& guard);    // 0x80244DD0: async file object gets its id
void HookAfFind(const Core::CPUThreadGuard& guard);     // 0x80244E00: id -> async file object
void HookAfRelease(const Core::CPUThreadGuard& guard);  // 0x80244E48: async file object back to the pool
void HookAfBufAlloc(const Core::CPUThreadGuard& guard);  // 0x802450D8: buffer of an async file load
template <int I>
void HookCardBarrierN(const Core::CPUThreadGuard& guard);  // memory-card API entry points (see FFNYRollback.cpp)
extern u32 g_card_activity;
void LabSchedulePoke(u32 addr, u32 value, u32 frame);  // lab bench: poke at an agreed logic frame
void HookAsyncSend(const Core::CPUThreadGuard& guard);
void HookSndMusicResult(const Core::CPUThreadGuard& guard);  // 0x8001112C: the music cue send's result  // OSSendMessage: the logic wakes a worker thread  // memory card device commands (EXI chip selects) so far
void HookSndPost(const Core::CPUThreadGuard& guard);
template <int I>
void HookSeedTbN(const Core::CPUThreadGuard& guard);  // match set-up mftbl seed reads (deterministic values)
template <int I>
void HookAptTbN(const Core::CPUThreadGuard& guard);  // menus Flash player clock reads (frame clock)
template <int I>
void HookSongPickN(const Core::CPUThreadGuard& guard);  // playlist picks (own generator while rollback runs)
// rollback in the menus: the game's main thread reads a clock that follows the logic frames (Jit64 / interpreter
// mftb call JitTimeBase while g_virtual_tb is set; set before the JIT compiles, or the JIT cache is cleared)
extern bool g_virtual_tb;
u64 JitTimeBase();
void HookSndMusic(const Core::CPUThreadGuard& guard);
void HookStreamGuard(const Core::CPUThreadGuard& guard);  // 0x801E7F90, sound stream note bigger than the buffered data
void HookStreamReset(const Core::CPUThreadGuard& guard);    // 0x801E8BCC, sound stream reset
void HookNoteDispatch0(const Core::CPUThreadGuard& guard);  // 0x801E7344, sound note handler call
void HookNoteDispatch1(const Core::CPUThreadGuard& guard);  // 0x801E735C
void HookNotesDone(const Core::CPUThreadGuard& guard);      // 0x801E7370
void LabStreamReport();
void HookSndExtra(const Core::CPUThreadGuard& guard);  // more music wrappers, held the same way  // 0x80011108 music event send
void HookSndTrAccept(const Core::CPUThreadGuard& guard);  // lab sound trace
void HookSndTrMusRes(const Core::CPUThreadGuard& guard);
void HookSndTrStart(const Core::CPUThreadGuard& guard);
void HookSndTrEvict(const Core::CPUThreadGuard& guard);
void HookSndService(const Core::CPUThreadGuard& guard);
template <int I>
void HookTraceN(const Core::CPUThreadGuard& guard);  // lab
template <int I>
void HookCountN(const Core::CPUThreadGuard& guard);  // lab
void HookSysAlloc(const Core::CPUThreadGuard& guard);  // rbtag's post-allocation hook: SysHeap blocks of the main thread  // 0x80274EB0, OSReceiveMessage puts a thread to sleep
void HookFsRead(const Core::CPUThreadGuard& guard);  // 0x802491B8, read handed to the file-system thread  // 0x80096C04, the match update (fight mode only)
void HookProbe(const Core::CPUThreadGuard& guard);  // lab
void HookSndRelease(const Core::CPUThreadGuard& guard);  // lab, 0x801F33E4
void HookHeapLog(const Core::CPUThreadGuard& guard);     // lab
void HookSndNullContext(const Core::CPUThreadGuard& guard);  // 0x801D7EE0
void HookBankGuard(const Core::CPUThreadGuard& guard);       // 0x8001841C
void HookSeedLog(const Core::CPUThreadGuard& guard);     // lab
// lab: a memory check hit (FFNY_RB_WATCHW=addr); returns true when handled
bool OnMemCheck(Core::System& system, u32 addr, u64 value, u32 pc, bool write);
// lab: a disc read is about to be written to emulated RAM
void OnDiscDma(Core::System& system, u32 address, u32 length);
void HookViCount(const Core::CPUThreadGuard& guard);  // 0x8024E22C, VI retrace count getter
void HookUnhandledException(const Core::CPUThreadGuard& guard);  // 0x80274180, the SDK's crash screen  // 0x80004D88, render part of the main loop

// Status line for the OSD / logs.
std::string GetStatus();
}  // namespace FFNYRollback
