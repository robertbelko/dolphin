// Copyright 2010 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/GCPad.h"
#include "Core/System.h"
#include "Core/HW/Memmap.h"
#include "Core/FFNYLab.h"

#include "Common/Common.h"
#include "Core/HW/GCPadEmu.h"
#include "InputCommon/ControllerEmu/ControlGroup/ControlGroup.h"
#include "InputCommon/GCPadStatus.h"
#include "InputCommon/InputConfig.h"

static u32 FFNYBench_Read32(u32 a)
{
  const u8* r = Core::System::GetInstance().GetMemory().GetRAM();
  if (!r) return 0;
  const u32 o = a & 0x01FFFFFF;
  return (u32(r[o]) << 24) | (u32(r[o + 1]) << 16) | (u32(r[o + 2]) << 8) | r[o + 3];
}
static void FFNYBench_Write32(u32 a, u32 v)
{
  u8* r = Core::System::GetInstance().GetMemory().GetRAM();
  if (!r) return;
  const u32 o = a & 0x01FFFFFF;
  r[o] = u8(v >> 24); r[o + 1] = u8(v >> 16); r[o + 2] = u8(v >> 8); r[o + 3] = u8(v);
}
#include "/home/claude/lab/bench/FFNYBench.inc"
namespace Pad
{
static InputConfig s_config("GCPadNew", _trans("Pad"), "GCPad", "Pad");
InputConfig* GetConfig()
{
  return &s_config;
}

void Shutdown()
{
  s_config.UnregisterHotplugCallback();

  s_config.ClearControllers();
}

void Initialize()
{
  if (s_config.ControllersNeedToBeCreated())
  {
    for (unsigned int i = 0; i < 4; ++i)
      s_config.CreateController<GCPad>(i);
  }

  s_config.RegisterHotplugCallback();

  // Load the saved controller config
  s_config.LoadConfig();
}

void LoadConfig()
{
  s_config.LoadConfig();
}

void GenerateDynamicInputTextures()
{
  s_config.GenerateControllerTextures();
}

bool IsInitialized()
{
  return !s_config.ControllersNeedToBeCreated();
}

GCPadStatus GetStatus(int pad_num)
{
  {
    GCPadStatus fb;
    if (FFNYBench::GetPad(pad_num, &fb))
      return fb;
  }

  GCPadStatus lab;
  if (FFNYLab::GetPad(pad_num, &lab))
    return lab;
  return static_cast<GCPad*>(s_config.GetController(pad_num))->GetInput();
}

ControllerEmu::ControlGroup* GetGroup(int pad_num, PadGroup group)
{
  return static_cast<GCPad*>(s_config.GetController(pad_num))->GetGroup(group);
}

void Rumble(const int pad_num, const ControlState strength)
{
  static_cast<GCPad*>(s_config.GetController(pad_num))->SetOutput(strength);
}

void ResetRumble(const int pad_num)
{
  static_cast<GCPad*>(s_config.GetController(pad_num))->SetOutput(0.0);
}

bool GetMicButton(const int pad_num)
{
  return static_cast<GCPad*>(s_config.GetController(pad_num))->GetMicButton();
}
}  // namespace Pad
