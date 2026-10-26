free_body = 0x8024A0C4;
fsq_body = 0x8024440C;
acq_body = 0x801F3280;
rel_body = 0x801F33A8;
SECTIONS { . = 0x8049A410; .text : { *(.text.0boot) *(.text*) *(.rodata*) } }
