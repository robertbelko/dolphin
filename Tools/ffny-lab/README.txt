FFNY rollback lab, 26 October 2026 (see FFNY_RE_handoff.md, "26 October").
Lab Dolphin = fork branch lab-26oct (lab-25oct + sound trace + music cue hold + flush before a forced fight end),
built headless: cmake -G Ninja -DENABLE_QT=OFF -DENABLE_NOGUI=ON -DENABLE_HEADLESS=ON -DENABLE_X11=OFF -DENABLE_EGL=OFF
-DENABLE_EVDEV=OFF -DENABLE_HWDB=OFF -DUSE_UPNP=OFF -DENABLE_SDL=OFF -DENABLE_LLVM=OFF -DENABLE_ALSA=OFF
-DENABLE_PULSEAUDIO=OFF -DENABLE_CUBEB=OFF -DENABLE_TESTS=OFF -DENABLE_VULKAN=OFF -DUSE_MGBA=OFF
-DUSE_RETRO_ACHIEVEMENTS=OFF -DUSE_DISCORD_PRESENCE=OFF -DENABLE_AUTOUPDATE=OFF -DENABLE_ANALYTICS=OFF
-DENCODE_FRAMEDUMPS=OFF -DENABLE_CLI_TOOL=OFF; ninja dolphin-nogui (~45 min on 2 cores). GCPad.cpp includes
/home/claude/lab/bench/FFNYBench.inc (lab22/FFNYBench.inc) - put it there before building.
trace.sh TAG SCRIPT [env...]   one headless Dolphin, fixed clock, both pads scripted; log -> res/TAG.log
blazin_pause.txt               lab22 blazin.txt + pause / unpause after marks g1 and g3
Runs used: reference FFNY_RB=3:3 FFNY_RB_LOOP=0 FFNY_RB_NOSNDHOLD=1; rollbacks FFNY_RB=1:3 FFNY_RB_LOOP=5
FFNY_RB_NOLOCKSTEP=1 (loopback needs NOLOCKSTEP since the start / load lockstep waits for pad 1 forever);
stress FFNY_RB=3:3 FFNY_RB_STRESS=5 FFNY_RB_NOLOCKSTEP=1. Trace: FFNY_RB_SNDTRACE=1 FFNY_RB_TRACE=80031b54,801d70ec.

27 October 2026 (menus mode, see FFNY_RE_handoff.md "night of 26 October" .. "Next steps (27 October)"):
run2s.sh TAG SCRIPT [env...]   two lab Dolphins over UDP (machine A = pad 1, B = pad 2, 30 ms simulated lag),
                               both run SCRIPT; logs res/TAG_A.log / _B.log, per-frame state hashes
                               (FFNY_RB_HASHLOG), dumps on a desync (FFNY_RB_DESYNCDUMP), CONFDUMP=from,to
                               dumps MEM1 of the confirmed frames from..to (res/cd/). Compare: cmp res/TAG_A.hash res/TAG_B.hash.
                               Used: FFNY_RB_MENUS=1 FFNY_BENCH_POKESYNC=1 (pokes at agreed frames, sent by A).
menuA.txt / menuB.txt          title -> CHOOSE FIGHTER -> venue -> fight loading, one script per machine.
rbtag/rbtag_v6_install.py      python rbtag_v6_install.py IN.dol OUT.dol (replaces v5 or installs v6).
Other lab switches (FFNYRollback.cpp): FFNY_RB_NOAFID, _NOFSHURRY, _NOFREEZE, _NOCARDLOCK, _NOASYNCLOCK,
_MUSICHOLD, _MENUROLLBACK, _VTBFIGHT, _INPUTLOG / _INPUTREPLAY, _STATEDUMP, _THREADS, _STRESS_PAUSE.
