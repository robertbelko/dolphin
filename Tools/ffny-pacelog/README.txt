FFNY pacing logger (Dolphin fork branch ffny-pacelog)

Turn it on (either way):
  - create an empty file named ffny_pacelog.on in your Dolphin user folder
    (Documents\Dolphin Emulator), or
  - set the environment variable FFNY_PACELOG=1 before starting Dolphin
    (or FFNY_PACELOG=C:\some\file.csv to choose the file).

Play normally. Every game session writes Documents\Dolphin Emulator\Logs\ffny_pacelog_<date>_<time>.csv
(one line per VI field, ~60 lines per second). Press Scroll Lock whenever something looks or feels uneven:
the moment is marked in the log.

Report:  python pacelog.py            (newest log)
         python pacelog.py <file.csv>
writes <file>.html next to the log with the cause of every repeated picture and every host hitch.

Columns (per VI field):
  host_ms / dt_ms     real time since the log started / real length of this field (= Dolphin's VBlank graph)
  work_ms / sleep_ms  Dolphin's work vs. time it slept waiting for real time; late_ms > 0 = it was behind
  new / dup / imm     the VI showed a new picture / the same picture again / immediate-XFB presents
  since_new_ms        real time since the last new picture
  xfb, copies, copy_at  XFB address shown, EFB->XFB copies in this field and when the last one happened (% of the field;
                      single core only)
  passes, updates, max_upd  main-loop passes (renders), logic updates, most updates in one pass
  upd_kc, rend_kc, wait_kc  emulated kilo-cycles in update / render (0x800838F4, includes any waiting inside it) /
                      0x8024E170 at the pass start
  rend_end_at         when the last render finished (% of the field)
  field_kc            emulated kilo-cycles in this field (~8100)
  gpu_kc, gpu_late    emulated GPU cycles used (Dolphin's GPU timing model) / GPU slices that ran out of time
  fifo_kb, verts, xf_words  FIFO backlog at the field end, vertices drawn, XF register words loaded (bone matrices)
  shaders, shader_ms  pipelines compiled on the spot and the host time it took
  gpu_ms, present_ms  host time running the GPU (single core) / presenting
  counter, fight, pending  logic counter 0x80440C24, fight flag 0x803F1000, pad snapshots waiting
  marker              1 = Scroll Lock pressed
