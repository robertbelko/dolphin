/* rbtag v6 (rollback; v6 = point 7, 26 October 2026): keeps the game's system memory apart from the rewindable game memory.
   1. Tag: heap blocks allocated with a system name get bit 0x0080 in the block header flags (u16 at header + 2;
      bits 0-5 = heap index, 0x0100 / 0x0400 / 0xC000 used by the allocator, 0x0040 / 0x0080 unused), so the rollback
      Dolphin skips them in its per-frame snapshot. Post-allocation debug hook (word 0x8040BD64, called with an info
      struct: +0 name, +0x18 pointer, +0x1C size), chaining to the previous hook (rbprobe in research builds).
   2. SysHeap: while a fight is running (0x803F1000 == 1), system-named allocations that would go to the main heap
      (heap index 0) go to a separate 1 MB heap "SysHeap" (created at boot), because a rewind of the main heap would
      erase blocks the (not rewound) system code allocated after the rewind point. Falls back to the main heap when full.
   4. File-system queue guard: a rewind can make the game submit a file request node again that is still queued
      or being processed by the file system (which is not rewound); the second insertion would link the node to
      itself and hang the file system. Such a resubmission only kicks the queue (FS function 0x80244408 with 0).
   5. Rollback-aware sound references: the game's audio code takes references to sound-library objects (acquire
      0x801F327C writes the new object to *out) and drops them (release 0x801F33A4). Both run inside the rewound
      logic, but the reference counts live in the (not rewound) sound library. While a rollback fight runs
      (word 0x8049B500 = 1, set by Dolphin), every acquire of the main thread is logged and every release deferred,
      with the frame of the newest saved rollback state (0x8049B504, set by Dolphin at every save: everything the
      main thread does after that save is undone by a rewind to that frame or earlier, logic and render alike). Dolphin writes the newest confirmed frame (0x8049A40C) and, after a rewind to frame S,
      S + 1 (0x8049B508): releases of confirmed frames are done, logged acquires of frames >= S are released
      (the restored game state does not hold them), deferred releases of frames >= S are dropped (the
      re-simulation issues them again). After the fight everything left is settled.
   6. Rollback-aware frees in the excluded heaps: a free of a SndHeap / SysHeap block (their memory is not rewound)
      by the main thread is logged with the frame, like a release (point 5): done once the
      frame is confirmed, dropped after a rewind to an earlier frame (the re-simulation frees it again). Without this
      a re-simulated frame frees the same sound block twice and the sound library corrupts its heap.
   3. Deferred frees: while a fight is running, frees of tagged main-heap blocks are queued (0x8049B400, not part of the
      rollback state) and done after the fight, so a rewind never resurrects a block the system code already freed.
   7. (v6) Delayed frees of tagged main-heap blocks by the main thread while rollback runs (W_ACT = 1): logged with the
      logic frame in a list that IS part of the rollback state (0x8049C000: count + up to 255 (block, frame) pairs) and
      done MLDELAY logic frames later, at the main thread's first allocation of that frame - the same logic point on
      every machine, so the main heap stays identical, and a rewind restores the list together with the heap. With
      rollback on in the menus the point-3 queue never emptied (the "fight" never ended) and the main heap ran out
      (lab: 2.3 MB file buffer at 0x801A3148 got 0, the decompressor wrote to address 0). Full list -> point 3. */
typedef unsigned int u32; typedef unsigned short u16; typedef unsigned char u8;
#define HOOK (*(volatile u32*)0x8040BD64)
#define PREV (*(volatile u32*)0x8049A400)
#define SYSIDX (*(volatile u32*)0x8049A404)
#define FIGHT (*(volatile u32*)0x8049B5F4)   /* v6: Dolphin's "rollback running" word W_FIGHTW (v5 read the fight loop state 0x803F1000 and Dolphin redirected it) */
#define DQ ((volatile u32*)0x8049B400)   /* [0] count, [1..63] pointers */
#define DQMAX 63
#define SYSSZ 0x100000
typedef u32 (*alloc_t)(u32, u32, u32, u32, u32, u32);
#define HEAP_ALLOC ((alloc_t)0x80249884)
#define DEF_FLAGS (*(volatile u32*)0x8043FBD0)   /* r13 - 0x7C10 */
#define OSDisableInterrupts ((u32(*)(void))0x802744B0)
#define OSRestoreInterrupts ((void(*)(u32))0x802744D8)
void rbtag_hook(u32* info);
void orig_free(u32 p);
void orig_fsq(u32 node);
u32 orig_acq(u32 a, u32 b, u32* out);
u32 orig_rel(u32 obj);
#define W_CONF (*(volatile u32*)0x8049A40C)
#define W_ACT (*(volatile u32*)0x8049B500)
#define W_FRAME (*(volatile u32*)0x8049B504)   /* frame of the newest saved rollback state (Dolphin) */
#define W_RB (*(volatile u32*)0x8049B508)
#define ACQ_N (*(volatile u32*)0x8049B510)
#define REL_N (*(volatile u32*)0x8049B514)
#define FRL_N (*(volatile u32*)0x8049B518)
#define IN_SETTLE (*(volatile u32*)0x8049B51C)
#define FRL ((volatile u32*)0x8049BC00)
#define HEAPTAB ((volatile u32*)0x8040BD70)
#define ACQ ((volatile u32*)0x8049B600)   /* pairs (object, frame) */
#define REL ((volatile u32*)0x8049B900)
#define LOGMAX 96
#define ML ((volatile u32*)0x8049C000)   /* [0] count, then (block, logic frame) pairs; part of the rollback state */
#define MLMAX 255
#define MLDELAY 12                       /* > the rollback window (7 frames) */
#define COUNTER (*(volatile u32*)0x80440C24)
#define CUR_THREAD (*(volatile u32*)0x800000E4)
#define MAIN_THREAD 0x80432D10
#define NEED_SETTLE (CUR_THREAD == MAIN_THREAD && !IN_SETTLE && (W_RB || REL_N || ACQ_N || FRL_N))
static void snd_settle(void);
static int snd_logging(void);
static int excluded_heap(u32 idx);
static const char kw[] =
    "AUDIO\0Audio\0audio\0GENRSTRM\0SPCHSTRM\0SPCHBNK\0ABKRES\0FIFO\0FrameBuffer\0GCBuffer\0ThreadStack\0"
    "File Sys\0bigfile\0Fixed MPF\0ASYNCFILE\0_AsyncLoaderCallback\0Stream\0InputBuffer\0MCThread\0MyThread\0SndHeap\0"
    "SysHeap\0";
static const char sysname[] = "SysHeap";

__attribute__((section(".text.0boot"))) u32 rbtag_boot(void)
{
    PREV = HOOK;
    HOOK = (u32)rbtag_hook;
    DQ[0] = 0;
    SYSIDX = 0;
    W_ACT = 0; W_FRAME = 0; W_RB = 0; ACQ_N = 0; REL_N = 0; FRL_N = 0; IN_SETTLE = 0; W_CONF = 0;
    *(volatile u32*)0x8049A408 = 0x52425436;   /* "RBT6": rollback Dolphin checks this */
    const u32 r = ((u32(*)(void))0x8049A100)();     /* rbstate's boot code (creates SndHeap: tagged now) */
    const u32 idx = ((u32(*)(void))0x8024A014)();
    const u32 blk = ((u32(*)(const char*, u32, u32))0x8001F174)(sysname, SYSSZ, 0x80000000);
    if (blk && idx >= 2 && idx < 64)
    {
        ((void(*)(u32, const char*, u32, u32, u32, u32, u32, u32, u32, u32))0x80249D60)(idx, sysname, blk, SYSSZ,
                                                                                          0x20, 0x20, 0, 0, 0, 1);
        SYSIDX = idx;
    }
    return r;
}

static int is_system(u32 name)
{
    const u8* nm = (const u8*)name;
    if ((name >> 25) != (0x80000000u >> 25))
        return 0;
    const char* k = kw;
    while (*k)
    {
        for (int i = 0; i < 64 && nm[i]; i++)
        {
            int j = 0;
            while (k[j] && nm[i + j] == (u8)k[j]) j++;
            if (!k[j]) return 1;
        }
        while (*k) k++;
        k++;
    }
    return 0;
}

void rbtag_hook(u32* info)
{
    u8* p = (u8*)info[6];
    if (p && ((u32)p >> 30) == 2 && *(u16*)(p - 16) == 0x424D && is_system(info[0]))
        p[-13] |= 0x80;
    void (*prev)(u32*) = (void (*)(u32*))PREV;
    if (prev) prev(info);
}

static void flush(void)
{
    while (DQ[0])
    {
        const u32 m = OSDisableInterrupts();
        u32 p = 0;
        if (DQ[0]) { p = DQ[DQ[0]]; DQ[0] = DQ[0] - 1; }
        OSRestoreInterrupts(m);
        if (p) orig_free(p);
    }
}

/* point 7: do the logged frees that are MLDELAY logic frames old (all of them when rollback is off) */
static void ml_run(int all)
{
    const u32 n = ML[0];
    u32 k = 0;
    for (u32 i = 0; i < n; i++)
    {
        const u32 p = ML[1 + 2 * i], f = ML[2 + 2 * i];
        if (all || f + MLDELAY <= COUNTER) orig_free(p);
        else { ML[1 + 2 * k] = p; ML[2 + 2 * k] = f; k++; }
    }
    ML[0] = k;
}

/* replaces the three calls of the heap allocation core 0x80249884 */
u32 rb_alloc(u32 name, u32 size, u32 align, u32 ofs, u32 flags, u32 mode)
{
    if (CUR_THREAD == MAIN_THREAD && ML[0] && ML[0] <= MLMAX)
        ml_run(FIGHT != 1 || !W_ACT);
    if (FIGHT != 1)
    {
        if (DQ[0]) flush();
    }
    else if (SYSIDX)
    {
        const u32 f = flags ? flags : DEF_FLAGS;
        if ((f & 0x3F) == 0 && is_system(name))
        {
            const u32 r = HEAP_ALLOC(name, size, align, ofs, f | SYSIDX, mode);
            if (r) return r;
        }
    }
    return HEAP_ALLOC(name, size, align, ofs, flags, mode);
}

/* the heap free function 0x8024A0C0 jumps here (its first instruction is replaced by a branch) */
void rb_free(u32 p)
{
    /* no settling here: free is called from inside the sound library's list work (settling releases objects) */
    if (p && snd_logging() && *(volatile u16*)(p - 16) == 0x424D)
    {
        /* free of a SndHeap / SysHeap block inside the rewound logic: done once its frame is confirmed */
        const u32 idx = *(volatile u16*)(p - 14) & 0x3F;
        if (idx && excluded_heap(idx) && FRL_N < LOGMAX)
        {
            const u32 n = FRL_N;
            FRL[2 * n] = p;
            FRL[2 * n + 1] = W_FRAME;
            FRL_N = n + 1;
            return;
        }
    }
    if (p && FIGHT == 1)
    {
        const u16 magic = *(volatile u16*)(p - 16), fl = *(volatile u16*)(p - 14);
        if (magic == 0x424D && (fl & 0x80) && (fl & 0x3F) == 0)
        {
            if (W_ACT && CUR_THREAD == MAIN_THREAD && ML[0] < MLMAX)
            {
                const u32 n = ML[0];
                for (u32 i = 0; i < n; i++) if (ML[1 + 2 * i] == p) return;
                ML[1 + 2 * n] = p;
                ML[2 + 2 * n] = COUNTER;
                ML[0] = n + 1;
                return;
            }
            const u32 m = OSDisableInterrupts();
            u32 n = DQ[0], dup = 0;
            for (u32 i = 1; i <= n; i++) if (DQ[i] == p) dup = 1;
            if (!dup && n < DQMAX) { DQ[n + 1] = p; DQ[0] = n + 1; }
            OSRestoreInterrupts(m);
            if (dup || n < DQMAX) return;
        }
    }
    else if (FIGHT != 1 && DQ[0])
        flush();
    orig_free(p);
}

/* the file-system enqueue / kick function 0x80244408 jumps here (first instruction replaced by a branch) */
void rb_fsq(u32 node)
{
    if (node)
    {
        const u32 m = OSDisableInterrupts();
        u32 found = (*(volatile u32*)0x804018D4 == node);   /* request being processed */
        u32 q = *(volatile u32*)0x804018E0;                  /* pending queue head, linked by +0x2C */
        for (int i = 0; q && i < 512 && !found; i++)
        {
            if (q == node) found = 1;
            q = *(volatile u32*)(q + 0x2C);
        }
        OSRestoreInterrupts(m);
        if (found) node = 0;
    }
    orig_fsq(node);
}

static void snd_settle(void)
{
    IN_SETTLE = 1;
    if (W_RB)
    {
        const u32 S = W_RB - 1;
        W_RB = 0;
        u32 fn = FRL_N, fk = 0;
        for (u32 i = 0; i < fn; i++)
        {
            const u32 o = FRL[2 * i], f = FRL[2 * i + 1];
            if (f < S) { FRL[2 * fk] = o; FRL[2 * fk + 1] = f; fk++; }   /* frames >= S: re-simulation frees again */
        }
        FRL_N = fk;
        u32 n = ACQ_N, k = 0;
        for (u32 i = 0; i < n; i++)
        {
            const u32 o = ACQ[2 * i], f = ACQ[2 * i + 1];
            if (f >= S) orig_rel(o);                 /* taken in a thrown-away frame */
            else { ACQ[2 * k] = o; ACQ[2 * k + 1] = f; k++; }
        }
        ACQ_N = k;
        n = REL_N; k = 0;
        for (u32 i = 0; i < n; i++)
        {
            const u32 o = REL[2 * i], f = REL[2 * i + 1];
            if (f < S) { REL[2 * k] = o; REL[2 * k + 1] = f; k++; }   /* frames >= S: re-simulation releases again */
        }
        REL_N = k;
    }
    const u32 conf = W_ACT ? W_CONF : 0xFFFFFFFF;
    u32 n = REL_N, k = 0;
    for (u32 i = 0; i < n; i++)
    {
        const u32 o = REL[2 * i], f = REL[2 * i + 1];
        if (f <= conf) orig_rel(o);
        else { REL[2 * k] = o; REL[2 * k + 1] = f; k++; }
    }
    REL_N = k;
    n = ACQ_N; k = 0;
    for (u32 i = 0; i < n; i++)
    {
        const u32 o = ACQ[2 * i], f = ACQ[2 * i + 1];
        if (f > conf) { ACQ[2 * k] = o; ACQ[2 * k + 1] = f; k++; }
    }
    ACQ_N = k;
    n = FRL_N; k = 0;
    for (u32 i = 0; i < n; i++)
    {
        const u32 o = FRL[2 * i], f = FRL[2 * i + 1];
        if (f <= conf) orig_free(o);
        else { FRL[2 * k] = o; FRL[2 * k + 1] = f; k++; }
    }
    FRL_N = k;
    IN_SETTLE = 0;
}


/* a heap other than the main heap whose memory is not part of the rollback state (SndHeap, SysHeap) */
static int excluded_heap(u32 idx)
{
    if (idx == SYSIDX) return 1;
    if (idx >= 16) return 0;
    const u32 h = HEAPTAB[idx];
    if ((h >> 25) != (0x80000000u >> 25)) return 0;
    const u32 base = *(volatile u32*)(h + 8);
    if ((base >> 25) != (0x80000000u >> 25)) return 0;
    return *(volatile u16*)(base - 16) == 0x424D && (*(volatile u16*)(base - 14) & 0x80);
}

static int snd_logging(void)
{
    return W_ACT && !IN_SETTLE && CUR_THREAD == MAIN_THREAD;
}

/* the sound-event (acquire) function 0x801F327C jumps here */
u32 rb_acq(u32 a, u32 b, u32* out)
{
    if (NEED_SETTLE) snd_settle();
    const u32 r = orig_acq(a, b, out);
    if (snd_logging() && out && *out && ACQ_N < LOGMAX)
    {
        const u32 n = ACQ_N;
        ACQ[2 * n] = *out;
        ACQ[2 * n + 1] = W_FRAME;
        ACQ_N = n + 1;
    }
    return r;
}

/* the release function 0x801F33A4 jumps here */
u32 rb_rel(u32 obj)
{
    if (NEED_SETTLE) snd_settle();
    if (obj && snd_logging() && REL_N < LOGMAX)
    {
        const u32 n = REL_N;
        REL[2 * n] = obj;
        REL[2 * n + 1] = W_FRAME;
        REL_N = n + 1;
        return 0;
    }
    return orig_rel(obj);
}
