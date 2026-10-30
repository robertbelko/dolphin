FFNY rollback lab, 29/30 October 2026 - the audio-thread crash at 0x801E7FC8 (see FFNY_RE_handoff.md,
"Lab reproduction of the friend-session audio crash").

0001-FFNY-rollback-skip-stale-sound-stream-notes-audio-th.patch
    THE FIX. One commit on top of ffny-menus2 1d49a2f (branch ffny-streamguard, commit cda5f6e in the lab session).
    In your dolphin clone:  git checkout -b ffny-streamguard origin/ffny-menus2
                            git am "path\to\0001-FFNY-rollback-skip-stale-sound-stream-notes-audio-th.patch"
                            git push -u origin ffny-streamguard      (GitHub Actions then builds Windows)
    Off switch: FFNY_RB_NOSTREAMGUARD=1. Chat line when it saves you: "FFNY: stale sound note skipped at frame ...".

lab-29oct_vs_1d49a2f.patch   lab Dolphin = ffny-menus2 1d49a2f + lab harness (FFNYLab, bench, State helpers) + lab checks
                             (FFNY_LAB_STREAMCHECK, FFNY_LAB_CUEDUP*, rewind-vs-stream check, consume ring dump).
FFNYBench.inc                bench include with the new ops: approach P DIST MAXP, FFNY_BENCH_SOFTTIMEOUT=1.
multi10.txt / multi5.txt     scripted sessions: menus -> 10 (5) x [Blazin' block, KO by Crack's haymaker, results, rematch].
run1.sh                      one headless lab Dolphin (paths of the lab session).
crash_frame21410_excerpt.txt the stream's last 64 consumes before the crash (reset at cue 0x0002, then the stale note).
v_on_FFNY_rollback.log       rollback log of the guarded 10-fight verification run.
Runs (all FFNY_RB_MENUS=1, loopback FFNY_RB=1:3 FFNY_RB_LOOP=5 unless noted):
  single fights (no rollback / old rule / stress 5 / no stream lockstep / 74k-frame fight): no underrun
  deliberate double cues FFNY_LAB_CUEDUP=1 and =2 _N=4: no underrun
  10 fights, new rule and FFNY_RB_MUSICEXACT=1: crash, fight 4 frame 21410 (identical)
  5 fights without rollback (FFNY_RB=3:3 FFNY_RB_LOOP=0): no crash
  guard on (K=5, K=3, verification): 10 fights, no crash;  guard off, same build: crash (frame 16665)
