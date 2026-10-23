// FFNY research harness: scripted control of emulation through a FIFO (env FFNY_LAB = directory).
#pragma once
#include "Common/CommonTypes.h"
#include "InputCommon/GCPadStatus.h"
namespace Core { class System; class CPUThreadGuard; }
namespace FFNYLab
{
bool Active();
void OnNewField(Core::System& system);
bool GetPad(int port, GCPadStatus* out);
extern bool g_store_trace;
void StoreTrace(Core::System& system, u32 addr, u32 size);
void OnTrace(const Core::CPUThreadGuard& guard);
void OnLoopTop(const Core::CPUThreadGuard& guard);
void OnPreUpdate(const Core::CPUThreadGuard& guard);
void OnPostUpdate(const Core::CPUThreadGuard& guard);
}
