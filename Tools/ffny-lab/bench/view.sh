#!/bin/bash
# view.sh STATE OUTNAME [FRAMES]: load a savestate in a software-renderer Dolphin, run FRAMES polls, dump the XFB as PNG
S=$1; N=$2; W=${3:-40}
U=/home/claude/game/duser_view_$N; rm -rf $U; mkdir -p $U/Config $U/GC/USA $U/Logs; cp -r /home/claude/game/cardA_orig "$U/GC/USA/Card A"
printf '[Core]\nCPUThread = False\nSIDevice0 = 6\nSIDevice1 = 6\nEnableCustomRTC = True\nCustomRTCValue = 946684800\n[DSP]\nBackend = No Audio Output\n[Interface]\nUsePanicHandlers = False\n' > $U/Config/Dolphin.ini
printf "wait $W\nshot $N\nquit\n" > /tmp/claude-0/view_$N.txt
FFNY_SWHEADLESS=1 FFNY_XFBDIR=/home/claude/lab/res/xfb FFNY_BENCH=/tmp/claude-0/view_$N.txt timeout 600 /home/claude/lab-dolphin/build/Binaries/dolphin-emu-nogui -u $U -p headless -v "Software Renderer" -s $S -e /home/claude/game/modded.iso > /home/claude/lab/res/xfb/view_$N.log 2>&1
python3 - "$N" <<'PY'
import sys
from PIL import Image
n=sys.argv[1]; d=open(f'/home/claude/lab/res/xfb/{n}.yuyv','rb').read(); d=d[d.index(b'\n')+1:]
W,H=640,448; img=Image.new('RGB',(W,H)); px=img.load()
for y in range(H):
    for x in range(0,W,2):
        o=(y*W+x)*2; Y0,U,Y1,V=d[o:o+4]
        for k,Yv in ((0,Y0),(1,Y1)):
            c=Yv-16; dd=U-128; e=V-128
            px[x+k,y]=tuple(max(0,min(255,int(v))) for v in (1.164*c+1.596*e,1.164*c-0.392*dd-0.813*e,1.164*c+2.017*dd))
img.save(f'/home/claude/lab/res/xfb/{n}.png')
PY
