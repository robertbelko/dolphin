// FFNYBench: version-portable scripted pads + timing for FFNY benchmarks.
// Included at the end of Core/HW/GCPad.cpp. The includer defines
//   static u32 FFNYBench_Read32(u32 addr);   (physical-or-virtual 0x80xxxxxx address)
// and calls FFNYBench::GetPad(pad_num, &status) at the top of Pad::GetStatus.
// Env FFNY_BENCH = script file. Ops (one per line, # comments):
//   pad P BTN SX SY      set override for port P (hex BTN), persists
//   rel P                port P neutral
//   wait N               wait N polls of port 0
//   until ADDR VAL [MAXP] wait until u32 at ADDR == VAL (hex); MAXP polls timeout -> "TIMEOUT" + quit
//   press P BTN N W      hold BTN N polls, release, wait W polls
//   mark NAME            print MARK line (polls, ms since start, u32 at 0x80440C24)
//   quit                 print and exit
#include <chrono>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

extern "C" __attribute__((weak)) void FFNY_GPHistDump();
extern "C" __attribute__((weak)) unsigned long long FFNY_Bursts();
extern "C" __attribute__((weak)) void FFNY_GxStats(char*, int);
extern "C" __attribute__((weak)) void FFNY_JitProfDump(const char*);
extern "C" __attribute__((weak)) void FFNY_GPHistReset();
extern "C" __attribute__((weak)) unsigned FFNY_MaxDist();
extern "C" __attribute__((weak)) unsigned FFNY_GuardWaits();
namespace FFNYBench
{
struct BStep
{
  std::string op, name;
  unsigned long a = 0, b = 0, c = 0, d = 0, e = 0, f = 0, g = 0;
};
static bool s_init = false, s_on = false;
static std::vector<BStep> s_steps;
static size_t s_pc = 0;
static unsigned long s_polls = 0, s_wait = 0, s_wstart = 0;
static int s_phase = 0;
static GCPadStatus s_ov[4];
static bool s_ovon[4];
static std::chrono::steady_clock::time_point s_t0;

static GCPadStatus Make(unsigned btn, unsigned sx, unsigned sy)
{
  GCPadStatus st{};
  st.button = (u16)btn;
  st.stickX = (u8)sx; st.stickY = (u8)sy; st.substickX = 128; st.substickY = 128;
  st.triggerLeft = 0; st.triggerRight = 0;
  st.analogA = (btn & 0x100) ? 0xFF : 0;
  st.analogB = (btn & 0x200) ? 0xFF : 0;
  st.isConnected = true;
  return st;
}

static void Init()
{
  s_init = true;
  const char* f = getenv("FFNY_BENCH");
  if (!f) return;
  std::ifstream in(f);
  std::string ln;
  while (std::getline(in, ln))
  {
    auto h = ln.find('#');
    if (h != std::string::npos) ln = ln.substr(0, h);
    std::istringstream ss(ln);
    BStep s;
    if (!(ss >> s.op)) continue;
    if (s.op == "mark" || s.op == "shot") ss >> s.name;
    else if (s.op == "pad" || s.op == "press") { ss >> s.a >> std::hex >> s.b >> std::dec >> s.c >> s.d; }
    else if (s.op == "until" || s.op == "untilge") { ss >> std::hex >> s.a >> s.b >> std::dec; if (!(ss >> s.c)) s.c = 0; }
    else if (s.op == "padc") { ss >> s.a >> std::hex >> s.b >> std::dec >> s.c >> s.d >> s.e >> s.f; }
    else if (s.op == "poke") { ss >> std::hex >> s.a >> s.b >> std::dec; }
    else if (s.op == "peek") { ss >> std::hex >> s.a >> std::dec >> s.b; }
    else if (s.op == "pokef") { float v = 0; ss >> std::hex >> s.a >> std::dec >> v; u32 w; memcpy(&w, &v, 4); s.b = w; }
    else if (s.op == "mash" || s.op == "mashge") { ss >> s.a >> std::hex >> s.b >> std::dec >> s.c >> s.d >> std::hex >> s.e >> s.f >> std::dec; if (!(ss >> s.g)) s.g = 0; }
    else ss >> s.a;
    s_steps.push_back(s);
  }
  for (int i = 0; i < 4; i++) { s_ov[i] = Make(0, 128, 128); s_ovon[i] = true; }
  s_on = true;
  s_t0 = std::chrono::steady_clock::now();
  fprintf(stderr, "FFNYBench: %zu steps\n", s_steps.size());
}

static void Mark(const std::string& n)
{
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - s_t0).count();
  fprintf(stderr, "MARK %s bursts %llu polls %lu ms %.1f scr %x ctr %u fight %u\n", n.c_str(), FFNY_Bursts ? FFNY_Bursts() : 0ULL, s_polls, ms, FFNYBench_Read32(0x803ea56c),
          FFNYBench_Read32(0x80440C24), FFNYBench_Read32(0x803F1000));
  if (FFNY_MaxDist) fprintf(stderr, "MAXDIST %s %x ovf %u guard %u\n", n.c_str(), FFNY_MaxDist(), FFNYBench_Read32(0x804408A0), FFNY_GuardWaits ? FFNY_GuardWaits() : 0);
  if (FFNY_GxStats) { char b[600]; FFNY_GxStats(b, sizeof b); fprintf(stderr, "GXST %s %s\n", n.c_str(), b); }
  if (FFNY_JitProfDump && getenv("FFNY_JITPROF")) FFNY_JitProfDump(n.c_str());
  fflush(stderr);
}

static void Watch()
{
  // fighter mode bytes: +0x93E (bit 1/2 = Blazin' mode / move), momentum +0x364
  static u32 last[2] = {0xFFFFFFFF, 0xFFFFFFFF};
  for (int i = 0; i < 2; i++)
  {
    const u32 a = 0x803F1144 + i * 0x12F8;
    const u32 w = FFNYBench_Read32(a + 0x93C);
    if (w != last[i])
    {
      fprintf(stderr, "WATCH fighter %d +93C %08x -> %08x (ctr %u)\n", i, last[i], w, FFNYBench_Read32(0x80440C24));
      last[i] = w;
    }
  }
}

static void Step()
{
  s_polls++;
  if (FFNYBench_Read32(0x803F1000) == 1) Watch();
  while (s_pc < s_steps.size())
  {
    BStep& s = s_steps[s_pc];
    {
      static size_t logged = (size_t)-1;
      static const bool steplog = getenv("FFNY_BENCH_STEPLOG") != nullptr;
      if (steplog && logged != s_pc && s_phase == 0)
      {
        logged = s_pc;
        fprintf(stderr, "STEP %zu %s ctr %u\n", s_pc, s.op.c_str(), FFNYBench_Read32(0x80440C24));
      }
    }
    if (s.op == "pad") { s_ov[s.a & 3] = Make((unsigned)s.b, (unsigned)s.c, (unsigned)s.d); s_pc++; continue; }
    if (s.op == "padc") { s_ov[s.a & 3] = Make((unsigned)s.b, (unsigned)s.c, (unsigned)s.d); s_ov[s.a & 3].substickX = (u8)s.e; s_ov[s.a & 3].substickY = (u8)s.f; s_pc++; continue; }
    if (s.op == "poke" || s.op == "pokef") { FFNYBench_Write32((u32)s.a, (u32)s.b); fprintf(stderr, "POKE %08lx = %08lx (ctr %u)\n", s.a, s.b, FFNYBench_Read32(0x80440C24)); s_pc++; continue; }
    if (s.op == "peek") { fprintf(stderr, "PEEK %08lx:", s.a); for (long i = 0; i < s.b; i++) fprintf(stderr, " %08x", FFNYBench_Read32((u32)s.a + 4 * (u32)i)); fprintf(stderr, " (ctr %u)\n", FFNYBench_Read32(0x80440C24)); s_pc++; continue; }
    if (s.op == "rel") { s_ov[s.a & 3] = Make(0, 128, 128); s_pc++; continue; }
    if (s.op == "mark") { Mark(s.name); if (s.name == "b0" && FFNY_GPHistReset) FFNY_GPHistReset(); s_pc++; continue; }
    if (s.op == "shot") {
#ifdef FFNYBENCH_SHOT
      FFNYBENCH_SHOT(s.name);
#endif
      s_pc++; continue; }
    if (s.op == "quit") { Mark("quit"); if (FFNY_GPHistDump) FFNY_GPHistDump(); fflush(stderr); std::_Exit(0); }
    if (s.op == "wait")
    {
      if (s_phase == 0) { s_wait = s.a; s_phase = 1; }
      if (s_wait == 0) { s_phase = 0; s_pc++; continue; }
      s_wait--;
      return;
    }
    if (s.op == "until" || s.op == "untilge")
    {
      if (s_phase == 0) { s_wstart = s_polls; s_phase = 1; }
      const u32 v = FFNYBench_Read32((u32)s.a);
      if (s.op == "until" ? v == (u32)s.b : v >= (u32)s.b) { s_phase = 0; s_pc++; continue; }
      if (s.c && s_polls - s_wstart > s.c) { Mark("TIMEOUT_line" + std::to_string(s_pc)); std::_Exit(3); }
      return;
    }
    if (s.op == "mash" || s.op == "mashge")
    {
      // a=port b=btn c=hold d=wait e=addr f=val g=maxpolls: repeat press until u32[e]==f (checked before each press)
      if (s_phase == 0) { s_wstart = s_polls; s_phase = 3; }
      if (s_phase == 3)
      {
        if (s.op == "mashge" ? FFNYBench_Read32((u32)s.e) >= (u32)s.f : FFNYBench_Read32((u32)s.e) == (u32)s.f) { s_phase = 0; s_pc++; continue; }
        if (s.g && s_polls - s_wstart > s.g) { Mark("TIMEOUT_line" + std::to_string(s_pc)); std::_Exit(3); }
        s_ov[s.a & 3] = Make((unsigned)s.b, 128, 128); s_wait = s.c; s_phase = 4;
      }
      if (s_phase == 4) { if (s_wait) { s_wait--; return; } s_ov[s.a & 3] = Make(0, 128, 128); s_wait = s.d; s_phase = 5; }
      if (s_phase == 5) { if (s_wait) { s_wait--; return; } s_phase = 3; continue; }
    }
    if (s.op == "press")
    {
      // a=port b=btn c=hold d=wait
      if (s_phase == 0) { s_ov[s.a & 3] = Make((unsigned)s.b, 128, 128); s_wait = s.c; s_phase = 1; }
      if (s_phase == 1) { if (s_wait) { s_wait--; return; } s_ov[s.a & 3] = Make(0, 128, 128); s_wait = s.d; s_phase = 2; }
      if (s_phase == 2) { if (s_wait) { s_wait--; return; } s_phase = 0; s_pc++; continue; }
    }
    s_pc++;
  }
}

static bool GetPad(int pad_num, GCPadStatus* out)
{
  if (!s_init) Init();
  if (!s_on) return false;
  if (pad_num == 0) Step();
  if (!s_ovon[pad_num & 3]) return false;
  *out = s_ov[pad_num & 3];
  return true;
}
}  // namespace FFNYBench
