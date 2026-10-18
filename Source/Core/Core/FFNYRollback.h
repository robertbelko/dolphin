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

namespace FFNYRollback
{
// Transport for rollback messages between the players (netplay connection, or UDP in the lab).
class Transport
{
public:
  virtual ~Transport() = default;
  virtual void Send(const std::vector<u8>& msg) = 0;
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
};

// Called by netplay (or the lab environment variables) before / when the game starts.
void Configure(const Settings& settings, Transport* transport);
// Called on the CPU thread when a rollback fight is over (all inputs confirmed). Netplay pauses the
// CPU there and resynchronises the whole emulator state from the host before menus continue.
void SetFightEndCallback(std::function<void(Core::System&, u32 frame)> callback);
void Shutdown();
bool IsActive();  // a fight is currently running in rollback mode

// Called once per VI field on the CPU thread: installs the hooks once the game code is in memory.
void OnVIField(Core::System& system);

// HLE hooks (registered in HLE.cpp as fixed Replace hooks).
void HookPreUpdate(const Core::CPUThreadGuard& guard);  // 0x80004D14 (mr r3,r26)
void HookTick(const Core::CPUThreadGuard& guard);       // 0x80004CFC (bl 0x800EC144)

// Status line for the OSD / logs.
std::string GetStatus();
}  // namespace FFNYRollback
