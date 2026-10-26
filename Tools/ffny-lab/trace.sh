#!/bin/bash
# trace.sh TAG SCRIPT [env...]: one headless lab Dolphin with a fixed clock; log -> res/TAG.log
TAG=$1; SCRIPT=$2; shift 2
U=/home/claude/game/duser_$TAG
rm -rf "$U"; mkdir -p "$U/GC/USA" "$U/Config"; cp -r /home/claude/game/cardA_orig "$U/GC/USA/Card A"
printf '[Core]\nCPUThread = False\nSIDevice0 = 6\nSIDevice1 = 6\nEnableCustomRTC = True\nCustomRTCValue = 946684800\nWiimoteContinuousScanning = False\nWiimoteControllersFromDolphin = False\n[DSP]\nBackend = No Audio Output\n[Interface]\nUsePanicHandlers = False\n' > "$U/Config/Dolphin.ini"
ulimit -f 4000000
env "$@" FFNY_BENCH=$SCRIPT timeout -s KILL ${TMO:-3600} /home/claude/lab-dolphin/build/Binaries/dolphin-emu-nogui -u $U -p headless -v Null -e ${ISO:-/home/claude/game/modded-rollback_test.iso} > /home/claude/lab/res/$TAG.log 2>&1
echo "exit $?"
