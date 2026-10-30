#!/bin/bash
# run2m.sh TAG [extra env...]: two lab Dolphins over UDP, rollback on from ALWAYS (default 2400, before the title),
# A = pad 0 (menuA.txt), B = pad 1 (menuB.txt). Logs res/TAG_A.log, res/TAG_B.log
TAG=$1; shift
P=$((7200 + RANDOM % 600 * 2))
for side in A B; do
  U=/home/claude/game/duser_${TAG}_$side
  rm -rf "$U"; mkdir -p "$U/GC/USA" "$U/Config"; cp -r /home/claude/game/cardA_orig "$U/GC/USA/Card A"
  printf '[Core]\nCPUThread = False\nSIDevice0 = 6\nSIDevice1 = 6\nEnableCustomRTC = True\nCustomRTCValue = 946684800\nWiimoteContinuousScanning = False\nWiimoteControllersFromDolphin = False\n[DSP]\nBackend = No Audio Output\n[Interface]\nUsePanicHandlers = False\n' > "$U/Config/Dolphin.ini"
done
UA=/home/claude/game/duser_${TAG}_A; UB=/home/claude/game/duser_${TAG}_B
BIN=/home/claude/lab-dolphin/build/Binaries/dolphin-emu-nogui; ISO=${ISO:-/home/claude/game/modded-rollback_test.iso}
ulimit -f 4000000
env "$@" FFNY_RB_ALWAYS=${ALWAYS:-2400} FFNY_RB=1:3 FFNY_RB_UDP=$P:127.0.0.1:$((P+1)) FFNY_BENCH=/home/claude/lab/bench/menuA.txt timeout -s KILL ${TMO:-3600} $BIN -u $UA -p headless -v Null -e $ISO > /home/claude/lab/res/${TAG}_A.log 2>&1 &
env "$@" FFNY_RB_ALWAYS=${ALWAYS:-2400} FFNY_RB=2:3 FFNY_RB_UDP=$((P+1)):127.0.0.1:$P FFNY_BENCH=/home/claude/lab/bench/menuB.txt timeout -s KILL ${TMO:-3600} $BIN -u $UB -p headless -v Null -e $ISO > /home/claude/lab/res/${TAG}_B.log 2>&1 &
wait
echo done
