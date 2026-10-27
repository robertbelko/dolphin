// Copyright 2008 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HLE/HLE.h"
#include "Core/FFNYRollback.h"

#include <algorithm>
#include <array>
#include <map>

#include "Common/CommonTypes.h"

#include "Core/Config/MainSettings.h"
#include "Core/Core.h"
#include "Core/GeckoCode.h"
#include "Core/HLE/HLE_Misc.h"
#include "Core/HLE/HLE_OS.h"
#include "Core/HW/Memmap.h"
#include "Core/Host.h"
#include "Core/PowerPC/PPCSymbolDB.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/System.h"

namespace HLE
{
// Map addresses to the HLE hook index
static std::map<u32, u32> s_hooked_addresses;

// clang-format off
constexpr std::array<Hook, 108> os_patches{{
    // Placeholder, os_patches[0] is the "non-existent function" index
    {"FAKE_TO_SKIP_0",               HLE_Misc::UnimplementedFunction,       HookType::Replace, HookFlag::Generic},

    // Name doesn't matter, installed in CBoot::BootUp()
    {"HBReload",                     HLE_Misc::HBReload,                    HookType::Replace, HookFlag::Fixed},

    // Debug/OS Support
    {"OSPanic",                      HLE_OS::HLE_OSPanic,                   HookType::Replace, HookFlag::Debug},

    // This needs to be put before vprintf (because vprintf is called indirectly by this)
    {"JUTWarningConsole_f",          HLE_OS::HLE_GeneralDebugPrint,         HookType::Start,   HookFlag::Debug},

    {"OSReport",                     HLE_OS::HLE_GeneralDebugPrint,         HookType::Start,   HookFlag::Debug},
    {"DEBUGPrint",                   HLE_OS::HLE_GeneralDebugPrint,         HookType::Start,   HookFlag::Debug},
    {"WUD_DEBUGPrint",               HLE_OS::HLE_GeneralDebugPrint,         HookType::Start,   HookFlag::Debug},
    {"__DSP_debug_printf",           HLE_OS::HLE_GeneralDebugPrint,         HookType::Start,   HookFlag::Debug},
    {"vprintf",                      HLE_OS::HLE_GeneralDebugVPrint,        HookType::Start,   HookFlag::Debug},
    {"printf",                       HLE_OS::HLE_GeneralDebugPrint,         HookType::Start,   HookFlag::Debug},
    {"vdprintf",                     HLE_OS::HLE_LogVDPrint,                HookType::Start,   HookFlag::Debug},
    {"dprintf",                      HLE_OS::HLE_LogDPrint,                 HookType::Start,   HookFlag::Debug},
    {"vfprintf",                     HLE_OS::HLE_LogVFPrint,                HookType::Start,   HookFlag::Debug},
    {"fprintf",                      HLE_OS::HLE_LogFPrint,                 HookType::Start,   HookFlag::Debug},
    {"nlPrintf",                     HLE_OS::HLE_GeneralDebugPrint,         HookType::Start,   HookFlag::Debug},
    {"DWC_Printf",                   HLE_OS::HLE_GeneralDebugPrint,         HookType::Start,   HookFlag::Debug},
    {"RANK_Printf",                  HLE_OS::HLE_GeneralDebugPrint,         HookType::Start,   HookFlag::Debug},
    {"puts",                         HLE_OS::HLE_GeneralDebugPrint,         HookType::Start,   HookFlag::Debug}, // gcc-optimized printf?
    {"___blank",                     HLE_OS::HLE_GeneralDebugPrint,         HookType::Start,   HookFlag::Debug}, // used for early init things (normally)
    {"__write_console",              HLE_OS::HLE_write_console,             HookType::Start,   HookFlag::Debug}, // used by sysmenu (+more?)

    {"GeckoCodehandler",             HLE_Misc::GeckoCodeHandlerICacheFlush, HookType::Start,   HookFlag::Fixed},
    {"GeckoHandlerReturnTrampoline", HLE_Misc::GeckoReturnTrampoline,       HookType::Replace, HookFlag::Fixed},
    {"AppLoaderReport",              HLE_OS::HLE_GeneralDebugPrint,         HookType::Start,   HookFlag::Fixed}, // apploader needs OSReport-like function
    {"FFNYRollbackPre",              FFNYRollback::HookPreUpdate,           HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackTick",             FFNYRollback::HookTick,                HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackLoader",           FFNYRollback::HookLoader,              HookType::Start,   HookFlag::Fixed},
    {"FFNYRollbackRng",              FFNYRollback::HookRng,                 HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackRng1",             FFNYRollback::HookRng1,                HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackRender",           FFNYRollback::HookRender,              HookType::Start,   HookFlag::Fixed},
    {"FFNYRollbackMatch",            FFNYRollback::HookMatchUpdate,         HookType::Start,   HookFlag::Fixed},
    {"FFNYRollbackFsWait",           FFNYRollback::HookFsWait,              HookType::Start,   HookFlag::Fixed},
    {"FFNYRollbackFsKick",           FFNYRollback::HookFsKick,              HookType::Start,   HookFlag::Fixed},
    {"FFNYRollbackAfSetId",          FFNYRollback::HookAfSetId,             HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackAfFind",           FFNYRollback::HookAfFind,              HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackAfRelease",        FFNYRollback::HookAfRelease,           HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackAfBufAlloc",       FFNYRollback::HookAfBufAlloc,          HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackAsyncSend", FFNYRollback::HookAsyncSend, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackSndMusicResult", FFNYRollback::HookSndMusicResult, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard0", FFNYRollback::HookCardBarrierN<0>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard1", FFNYRollback::HookCardBarrierN<1>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard2", FFNYRollback::HookCardBarrierN<2>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard3", FFNYRollback::HookCardBarrierN<3>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard4", FFNYRollback::HookCardBarrierN<4>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard5", FFNYRollback::HookCardBarrierN<5>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard6", FFNYRollback::HookCardBarrierN<6>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard7", FFNYRollback::HookCardBarrierN<7>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard8", FFNYRollback::HookCardBarrierN<8>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard9", FFNYRollback::HookCardBarrierN<9>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard10", FFNYRollback::HookCardBarrierN<10>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard11", FFNYRollback::HookCardBarrierN<11>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard12", FFNYRollback::HookCardBarrierN<12>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard13", FFNYRollback::HookCardBarrierN<13>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard14", FFNYRollback::HookCardBarrierN<14>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard15", FFNYRollback::HookCardBarrierN<15>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard16", FFNYRollback::HookCardBarrierN<16>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard17", FFNYRollback::HookCardBarrierN<17>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackCard18", FFNYRollback::HookCardBarrierN<18>, HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackSysAlloc",         FFNYRollback::HookSysAlloc,            HookType::Start,   HookFlag::Fixed},
    {"FFNYRollbackCount0", FFNYRollback::HookCountN<0>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount1", FFNYRollback::HookCountN<1>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount2", FFNYRollback::HookCountN<2>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount3", FFNYRollback::HookCountN<3>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount4", FFNYRollback::HookCountN<4>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount5", FFNYRollback::HookCountN<5>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount6", FFNYRollback::HookCountN<6>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount7", FFNYRollback::HookCountN<7>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount8", FFNYRollback::HookCountN<8>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount9", FFNYRollback::HookCountN<9>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount10", FFNYRollback::HookCountN<10>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount11", FFNYRollback::HookCountN<11>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount12", FFNYRollback::HookCountN<12>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount13", FFNYRollback::HookCountN<13>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount14", FFNYRollback::HookCountN<14>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount15", FFNYRollback::HookCountN<15>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount16", FFNYRollback::HookCountN<16>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount17", FFNYRollback::HookCountN<17>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount18", FFNYRollback::HookCountN<18>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount19", FFNYRollback::HookCountN<19>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount20", FFNYRollback::HookCountN<20>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount21", FFNYRollback::HookCountN<21>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount22", FFNYRollback::HookCountN<22>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount23", FFNYRollback::HookCountN<23>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount24", FFNYRollback::HookCountN<24>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount25", FFNYRollback::HookCountN<25>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount26", FFNYRollback::HookCountN<26>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount27", FFNYRollback::HookCountN<27>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount28", FFNYRollback::HookCountN<28>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount29", FFNYRollback::HookCountN<29>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount30", FFNYRollback::HookCountN<30>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackCount31", FFNYRollback::HookCountN<31>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackTrace0", FFNYRollback::HookTraceN<0>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackTrace1", FFNYRollback::HookTraceN<1>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackTrace2", FFNYRollback::HookTraceN<2>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackTrace3", FFNYRollback::HookTraceN<3>, HookType::Start, HookFlag::Fixed},
    {"FFNYRollbackViCount",          FFNYRollback::HookViCount,             HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackSndCtx",           FFNYRollback::HookSndNullContext,      HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackSndPost",          FFNYRollback::HookSndPost,             HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackSndService",       FFNYRollback::HookSndService,          HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackSndMusic",         FFNYRollback::HookSndMusic,            HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackSndExtra",         FFNYRollback::HookSndExtra,            HookType::Replace, HookFlag::Fixed},
    {"FFNYSndTrAccept",              FFNYRollback::HookSndTrAccept,         HookType::Start,   HookFlag::Fixed},
    {"FFNYSndTrMusRes",              FFNYRollback::HookSndTrMusRes,         HookType::Start,   HookFlag::Fixed},
    {"FFNYSndTrStart",               FFNYRollback::HookSndTrStart,          HookType::Start,   HookFlag::Fixed},
    {"FFNYSndTrEvict",               FFNYRollback::HookSndTrEvict,          HookType::Start,   HookFlag::Fixed},
    {"FFNYRollbackBankGuard",        FFNYRollback::HookBankGuard,           HookType::Replace, HookFlag::Fixed},
    {"FFNYRollbackSndRel",           FFNYRollback::HookSndRelease,          HookType::Start,   HookFlag::Fixed},
    {"FFNYRollbackProbe",            FFNYRollback::HookProbe,               HookType::Start,   HookFlag::Fixed},
    {"FFNYRollbackCrash",            FFNYRollback::HookUnhandledException,  HookType::Start,   HookFlag::Fixed},
}};
// clang-format on

void Patch(Core::System& system, u32 addr, std::string_view func_name)
{
  auto& ppc_state = system.GetPPCState();
  auto& memory = system.GetMemory();
  auto& jit_interface = system.GetJitInterface();
  for (u32 i = 1; i < os_patches.size(); ++i)
  {
    if (os_patches[i].name == func_name)
    {
      s_hooked_addresses[addr] = i;
      ppc_state.iCache.Invalidate(memory, jit_interface, addr);
      Host_JitCacheInvalidation();
      return;
    }
  }
}

void PatchFixedFunctions(Core::System& system)
{
  // MIOS puts patch data in low MEM1 (0x1800-0x3000) for its own use.
  // Overwriting data in this range can cause the IPL to crash when launching games
  // that get patched by MIOS. See https://bugs.dolphin-emu.org/issues/11952 for more info.
  // Not applying the Gecko HLE patches means that Gecko codes will not work under MIOS,
  // but this is better than the alternative of having specific games crash.
  if (system.IsMIOS())
    return;

  // HLE jump to loader (homebrew).  Disabled when Gecko is active as it interferes with the code
  // handler
  if (!Config::AreCheatsEnabled())
  {
    Patch(system, 0x80001800, "HBReload");
    auto& memory = system.GetMemory();
    memory.CopyToEmu(0x00001804, "STUBHAXX", 8);
  }

  // Not part of the binary itself, but either we or Gecko OS might insert
  // this, and it doesn't clear the icache properly.
  Patch(system, Gecko::ENTRY_POINT, "GeckoCodehandler");
  // This has to always be installed even if cheats are not enabled because of the possibility of
  // loading a savestate where PC is inside the code handler while cheats are disabled.
  Patch(system, Gecko::HLE_TRAMPOLINE_ADDRESS, "GeckoHandlerReturnTrampoline");
}

void PatchFunctions(Core::System& system)
{
  auto& power_pc = system.GetPowerPC();
  auto& ppc_state = power_pc.GetPPCState();
  auto& memory = system.GetMemory();
  auto& jit_interface = system.GetJitInterface();
  auto& ppc_symbol_db = power_pc.GetSymbolDB();

  // Remove all hooks that aren't fixed address hooks
  for (auto i = s_hooked_addresses.begin(); i != s_hooked_addresses.end();)
  {
    if (os_patches[i->second].flags != HookFlag::Fixed)
    {
      ppc_state.iCache.Invalidate(memory, jit_interface, i->first);
      i = s_hooked_addresses.erase(i);
    }
    else
    {
      ++i;
    }
  }

  for (u32 i = 1; i < os_patches.size(); ++i)
  {
    // Fixed hooks don't map to symbols
    if (os_patches[i].flags == HookFlag::Fixed)
      continue;

    for (const auto& symbol : ppc_symbol_db.GetSymbolsFromName(os_patches[i].name))
    {
      for (u32 addr = symbol->address; addr < symbol->address + symbol->size; addr += 4)
      {
        s_hooked_addresses[addr] = i;
        ppc_state.iCache.Invalidate(memory, jit_interface, addr);
      }
      INFO_LOG_FMT(OSHLE, "Patching {} {:08x}", os_patches[i].name, symbol->address);
    }
  }

  Host_JitCacheInvalidation();
}

void Clear()
{
  s_hooked_addresses.clear();
}

void Reload(Core::System& system)
{
  Clear();
  PatchFixedFunctions(system);
  PatchFunctions(system);
}

void Execute(const Core::CPUThreadGuard& guard, u32 current_pc, u32 hook_index)
{
  hook_index &= 0xFFFFF;
  if (hook_index > 0 && hook_index < os_patches.size())
  {
    os_patches[hook_index].function(guard);
  }
  else
  {
    PanicAlertFmt("HLE system tried to call an undefined HLE function {}.", hook_index);
  }
}

void ExecuteFromJIT(u32 current_pc, u32 hook_index, Core::System& system)
{
  ASSERT(Core::IsCPUThread());
  Core::CPUThreadGuard guard(system);
  Execute(guard, current_pc, hook_index);
}

u32 GetHookByAddress(u32 address)
{
  auto iter = s_hooked_addresses.find(address);
  return (iter != s_hooked_addresses.end()) ? iter->second : 0;
}

u32 GetHookByFunctionAddress(PPCSymbolDB& ppc_symbol_db, u32 address)
{
  const u32 index = GetHookByAddress(address);
  // Fixed hooks use a fixed address and don't patch the whole function
  if (index == 0 || os_patches[index].flags == HookFlag::Fixed)
    return index;

  const Common::Symbol* const symbol = ppc_symbol_db.GetSymbolFromAddr(address);
  return (symbol && symbol->address == address) ? index : 0;
}

const char* GetHookNameByIndex(u32 index)
{
  return os_patches[index].name;
}

HookType GetHookTypeByIndex(u32 index)
{
  return os_patches[index].type;
}

HookFlag GetHookFlagsByIndex(u32 index)
{
  return os_patches[index].flags;
}

TryReplaceFunctionResult TryReplaceFunction(PPCSymbolDB& ppc_symbol_db, u32 address,
                                            PowerPC::CoreMode mode)
{
  const u32 hook_index = GetHookByFunctionAddress(ppc_symbol_db, address);
  if (hook_index == 0)
    return {};

  const HookType type = GetHookTypeByIndex(hook_index);
  if (type != HookType::Start && type != HookType::Replace)
    return {};

  const HookFlag flags = GetHookFlagsByIndex(hook_index);
  if (!IsEnabled(flags, mode))
    return {};

  return {type, hook_index};
}

bool IsEnabled(HookFlag flag, PowerPC::CoreMode mode)
{
  return flag != HLE::HookFlag::Debug || Config::IsDebuggingEnabled() ||
         mode == PowerPC::CoreMode::Interpreter;
}

u32 UnPatch(Core::System& system, std::string_view patch_name)
{
  const auto patch = std::ranges::find(os_patches, patch_name, &Hook::name);
  if (patch == std::end(os_patches))
    return 0;

  auto& power_pc = system.GetPowerPC();
  auto& ppc_state = power_pc.GetPPCState();
  auto& memory = system.GetMemory();
  auto& jit_interface = system.GetJitInterface();

  if (patch->flags == HookFlag::Fixed)
  {
    const u32 patch_idx = static_cast<u32>(std::distance(os_patches.begin(), patch));
    u32 addr = 0;
    // Reverse search by OSPatch key instead of address
    for (auto i = s_hooked_addresses.begin(); i != s_hooked_addresses.end();)
    {
      if (i->second == patch_idx)
      {
        addr = i->first;
        ppc_state.iCache.Invalidate(memory, jit_interface, i->first);
        i = s_hooked_addresses.erase(i);
      }
      else
      {
        ++i;
      }
    }
    Host_JitCacheInvalidation();
    return addr;
  }

  const auto symbols = power_pc.GetSymbolDB().GetSymbolsFromName(patch_name);
  if (!symbols.empty())
  {
    const Common::Symbol* const symbol = symbols.front();
    for (u32 addr = symbol->address; addr < symbol->address + symbol->size; addr += 4)
    {
      s_hooked_addresses.erase(addr);
      ppc_state.iCache.Invalidate(memory, jit_interface, addr);
    }
    Host_JitCacheInvalidation();
    return symbol->address;
  }

  return 0;
}

u32 UnpatchRange(Core::System& system, u32 start_addr, u32 end_addr)
{
  auto& ppc_state = system.GetPPCState();
  auto& memory = system.GetMemory();
  auto& jit_interface = system.GetJitInterface();

  u32 count = 0;

  auto i = s_hooked_addresses.lower_bound(start_addr);
  while (i != s_hooked_addresses.end() && i->first < end_addr)
  {
    INFO_LOG_FMT(OSHLE, "Unpatch HLE hooks [{:08x};{:08x}): {} at {:08x}", start_addr, end_addr,
                 os_patches[i->second].name, i->first);
    ppc_state.iCache.Invalidate(memory, jit_interface, i->first);
    i = s_hooked_addresses.erase(i);
    count += 1;
  }
  Host_JitCacheInvalidation();

  return count;
}
}  // namespace HLE
