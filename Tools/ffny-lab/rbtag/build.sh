#!/bin/sh
# builds rbtag.bin (code for 0x8049A410, v5) - the bytes are already inside ffny_dolpatch.py as PATCHES["rbtag"].
# mkrbtag.py IN.dol OUT.dol applies it by hand (lab helper; expects rbstate applied and no rbtag yet).
clang -target powerpc-unknown-eabi -mcpu=750 -O2 -ffreestanding -fno-pic -fno-builtin -fno-jump-tables -c rbtag.c -o rbtag.o
clang -target powerpc-unknown-eabi -mcpu=750 -c tramp.s -o tramp.o
ld.lld -T link.ld rbtag.o tramp.o -o rbtag.elf
llvm-objcopy -O binary -j .text rbtag.elf rbtag.bin
