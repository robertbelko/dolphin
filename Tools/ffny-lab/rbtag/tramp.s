    .section .text.tramp,"ax"
    .globl orig_free
orig_free:
    stwu 1,-0x40(1)
    b free_body
    .globl orig_fsq
orig_fsq:
    stwu 1,-0x228(1)
    b fsq_body
    .globl orig_acq
orig_acq:
    stwu 1,-0x18(1)
    b acq_body
    .globl orig_rel
orig_rel:
    stwu 1,-0x10(1)
    b rel_body
