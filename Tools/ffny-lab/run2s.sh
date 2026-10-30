#!/bin/bash
# run2s.sh TAG [extra env...]: two lab Dolphins over UDP (A = pad 0, B = pad 1), both running the same bench script
# (each machine's own pad goes to the other); rollback on from ALWAYS (default 2500); state hash logs res/TAG_A.hash /
# res/TAG_B.hash; logs res/TAG_A.log / res/TAG_B.log
TAG=$1; shift
P=$((7200 + RANDOM % 600 * 2))
for side in A B; do
  U=/home/claude/game/duser_${TAG}_$side
  rm -rf "$U"; mkdir -p "$U/GC/USA" "$U/Config"; cp -r /home/claude/game/cardA_orig "$U/GC/USA/Card A"
  printf '[Core]\nCPUThread = False\nSIDevice0 = 6\nSIDevice1 = 6\nEnableCustomRTC = True\nCustomRTCValue = 946684800\nWiimoteContinuousScanning = False\nWiimoteControllersFromDolphin = False\n[DSP]\nBackend = No Audio Output\n[Interface]\nUsePanicHandlers = False\n' > "$U/Config/Dolphin.ini"
done
UA=/home/claude/game/duser_${TAG}_A; UB=/home/claude/game/duser_${TAG}_B
BIN=/home/claude/lab-dolphin/build/Binaries/dolphin-emu-nogui; ISO=${ISO:-/home/claude/game/rbt6.iso}
SCRIPT=${SCRIPT:-/home/claude/lab/bench/blazin_pause_ctr.txt}
ulimit -f 40000000; ulimit -c unlimited; echo 0x33 > /proc/self/coredump_filter
(env "$@" FFNY_RB_ALWAYS=${ALWAYS:-2500} FFNY_RB=1:3 FFNY_RB_UDP=$P:127.0.0.1:$((P+1)) FFNY_BENCH=$SCRIPT FFNY_BENCH_CTR=1 FFNY_RB_HASHLOG=/home/claude/lab/res/${TAG}_A.hash FFNY_RB_DESYNCDUMP=/home/claude/lab/res/dd/${TAG}_A FFNY_RB_CONFDUMP=${CONFDUMP:-0,0},/home/claude/lab/res/cd/${TAG}_A timeout -s KILL ${TMO:-3600} $BIN -u $UA -p headless -v Null -e $ISO > /home/claude/lab/res/${TAG}_A.log 2>&1; echo "EXIT $?" >> /home/claude/lab/res/${TAG}_A.log) &
(env "$@" FFNY_RB_ALWAYS=${ALWAYS:-2500} FFNY_RB=2:3 FFNY_RB_UDP=$((P+1)):127.0.0.1:$P FFNY_BENCH=$SCRIPT FFNY_BENCH_CTR=1 FFNY_RB_HASHLOG=/home/claude/lab/res/${TAG}_B.hash FFNY_RB_DESYNCDUMP=/home/claude/lab/res/dd/${TAG}_B FFNY_RB_CONFDUMP=${CONFDUMP:-0,0},/home/claude/lab/res/cd/${TAG}_B timeout -s KILL ${TMO:-3600} $BIN -u $UB -p headless -v Null -e $ISO > /home/claude/lab/res/${TAG}_B.log 2>&1; echo "EXIT $?" >> /home/claude/lab/res/${TAG}_B.log) &
wait
echo done
