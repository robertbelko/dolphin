#!/usr/bin/env python3
"""FFNY pacing log report.

Usage:  python pacelog.py ffny_pacelog_YYYYMMDD_HHMMSS.csv   (or no argument = newest log in the Dolphin Logs folder)

Reads the CSV written by the FFNY pacelog Dolphin build (one line per VI field) and writes <log>.html next to it:
a summary of every late picture and its cause, and charts over time (markers from Scroll Lock shown as red lines).

Two different things can make the game look uneven, and the log separates them:
  * a REPEATED PICTURE: the emulated GameCube showed the same picture twice (the game or the emulated GPU did not
    finish the next picture in 1/60 s of emulated time). This happens the same way on every PC.
  * a HOST HITCH: the picture was new, but this PC delivered it late or early (Dolphin's work, a shader compile,
    the video driver). This depends on the PC.
Stdlib only, no extra packages.
"""
import csv
import glob
import html
import os
import sys

FIELD_MS = 1000.0 / 59.94


def load(path):
    header = ''
    rows = []
    with open(path, newline='') as f:
        first = f.readline()
        if first.startswith('#'):
            header = first[1:].strip()
        else:
            f.seek(0)
        for r in csv.DictReader(f):
            try:
                rows.append({k: (float(v) if k != 'xfb' else v) for k, v in r.items()})
            except (ValueError, TypeError):
                pass  # last line cut off while Dolphin was still writing
    return header, rows


def classify(rows):
    """Adds 'cause' to every field that showed a repeated picture, and 'host' to every host hitch."""
    n = len(rows)
    for i, r in enumerate(rows):
        r['cause'] = ''
        r['host'] = ''
        prev = rows[i - 1] if i else r
        fk = r['field_kc'] or prev['field_kc'] or 8100
        r['cpu_pct'] = 100.0 * (r['upd_kc'] + r['rend_kc']) / fk
        r['gpu_pct'] = 100.0 * r['gpu_kc'] / fk
        repeated = r['dup'] > 0 and r['new'] == 0 and r['imm'] == 0
        if repeated:
            win = [prev, r]
            if sum(w['passes'] for w in win) == 0 and sum(w['copies'] for w in win) == 0:
                r['cause'] = 'game not drawing (loading / paused screen)'
            elif sum(w['gpu_late'] for w in win) > 0:
                r['cause'] = 'emulated GPU: Dolphin\'s GPU time budget ran out (vertices / matrix loads)'
            elif max(w['rend_kc'] + w['upd_kc'] / max(w['updates'], 1) for w in win) > 0.95 * fk:
                r['cause'] = 'emulated CPU: one update + render took about 1/60 s or more'
            elif r['copies'] == 0 and prev['copies'] == 0:
                r['cause'] = 'game drew nothing new (its own pacing)'
            else:
                r['cause'] = 'picture finished just after the VI (timing phase)'
        # host side: how far this field's real duration is from 1/60 s
        if i and abs(r['dt_ms'] - FIELD_MS) > 2.0:
            if r['shaders'] > 0 or prev['shaders'] > 0:
                r['host'] = 'shader compile'
            elif r['present_ms'] > 4.0:
                r['host'] = 'present / video driver'
            elif r['late_ms'] > 2.0 or r['work_ms'] > FIELD_MS:
                r['host'] = 'PC too slow for this field (Dolphin work > 1/60 s)'
            elif r['dt_ms'] < FIELD_MS:
                r['host'] = 'catch-up (field after a slow one)'
            else:
                r['host'] = 'other (OS timer / scheduling)'
    return rows


def segments(rows):
    """Splits the log by the fight flag, so fights and menus are counted separately."""
    out = []
    start = 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or (rows[i]['fight'] > 0) != (rows[start]['fight'] > 0):
            out.append((start, i - 1, rows[start]['fight'] > 0))
            start = i
    return out


def tally(rows, key):
    t = {}
    for r in rows:
        if r[key]:
            t[r[key]] = t.get(r[key], 0) + 1
    return sorted(t.items(), key=lambda kv: -kv[1])


def polyline(xs, ys, x0, x1, y0, y1, w, h, color, step=False):
    if not xs:
        return ''
    pts = []
    for x, y in zip(xs, ys):
        px = (x - x0) / (x1 - x0 or 1) * w
        py = h - (min(max(y, y0), y1) - y0) / (y1 - y0 or 1) * h
        pts.append('%.1f,%.1f' % (px, py))
    return '<polyline fill="none" stroke="%s" stroke-width="1" points="%s"/>' % (color, ' '.join(pts))


def chart(title, rows, series, y1, unit, markers, w=1100, h=120):
    t = [r['host_ms'] / 1000.0 for r in rows]
    x0, x1 = (t[0], t[-1]) if t else (0, 1)
    body = []
    for frac in (0.25, 0.5, 0.75):
        y = h - frac * h
        body.append('<line x1="0" x2="%d" y1="%.1f" y2="%.1f" class="grid"/>' % (w, y, y))
        body.append('<text x="2" y="%.1f" class="lab">%g %s</text>' % (y - 2, round(frac * y1, 1), unit))
    for m in markers:
        px = (m - x0) / (x1 - x0 or 1) * w
        body.append('<line x1="%.1f" x2="%.1f" y1="0" y2="%d" class="mark"/>' % (px, px, h))
    legend = []
    for name, ys, color in series:
        body.append(polyline(t, ys, x0, x1, 0, y1, w, h, color))
        legend.append('<span style="color:%s">&#9632; %s</span>' % (color, html.escape(name)))
    return ('<h3>%s</h3><div class="leg">%s</div><svg viewBox="0 0 %d %d" width="100%%" preserveAspectRatio="none" '
            'class="ch">%s</svg><div class="axis"><span>%.0f s</span><span>%.0f s</span></div>'
            % (html.escape(title), ' '.join(legend), w, h, ''.join(body), x0, x1))


def report(path):
    header, rows = load(path)
    if not rows:
        print('no data in', path)
        return
    classify(rows)
    markers = [r['host_ms'] / 1000.0 for r in rows if r['marker'] > 0]
    total_s = rows[-1]['host_ms'] / 1000.0

    lines = []
    lines.append('Log: %s (%d fields, %.0f s)' % (os.path.basename(path), len(rows), total_s))
    lines.append('Settings: ' + header)
    for a, b, fight in segments(rows):
        seg = rows[a:b + 1]
        if len(seg) < 30:
            continue
        rep = [r for r in seg if r['cause']]
        hosts = [r for r in seg if r['host']]
        t0, t1 = seg[0]['host_ms'] / 1000.0, seg[-1]['host_ms'] / 1000.0
        lines.append('')
        lines.append('%s %.1f-%.1f s (%d fields): %d repeated pictures (%.1f %%), %d host hitches'
                     % ('FIGHT' if fight else 'menus/other', t0, t1, len(seg), len(rep), 100.0 * len(rep) / len(seg),
                        len(hosts)))
        for k, v in tally(seg, 'cause'):
            lines.append('    repeated: %5d  %s' % (v, k))
        for k, v in tally(seg, 'host'):
            lines.append('    host:     %5d  %s' % (v, k))
        cpu = sorted(r['cpu_pct'] for r in seg if r['passes'])
        gpu = sorted(r['gpu_pct'] for r in seg)
        if cpu:
            lines.append('    emulated CPU busy (update+render) median %.0f %%, 95th %.0f %% of a field'
                         % (cpu[len(cpu) // 2], cpu[int(len(cpu) * 0.95)]))
        if gpu:
            lines.append('    emulated GPU busy median %.0f %%, 95th %.0f %% of a field; fields with GPU out of time: %d'
                         % (gpu[len(gpu) // 2], gpu[int(len(gpu) * 0.95)], sum(1 for r in seg if r['gpu_late'] > 0)))
    if markers:
        lines.append('')
        lines.append('Markers (Scroll Lock) and what happened in the 2 s before each:')
        for m in markers:
            win = [r for r in rows if m - 2.0 <= r['host_ms'] / 1000.0 <= m]
            causes = tally(win, 'cause')
            hosts = tally(win, 'host')
            lines.append('    %.1f s: %d repeated (%s), %d host hitches (%s)'
                         % (m, sum(v for _, v in causes), '; '.join('%s x%d' % kv for kv in causes) or '-',
                            sum(v for _, v in hosts), '; '.join('%s x%d' % kv for kv in hosts) or '-'))
    text = '\n'.join(lines)
    print(text)

    # charts: the interval between new pictures (what Dolphin's orange frame-time graph shows), the field length
    # (blue VBlank graph), emulated CPU / GPU load, shader compiles
    frame_iv = []
    last = None
    for r in rows:
        if r['new'] > 0 or r['imm'] > 0:
            frame_iv.append(r['host_ms'] - last if last is not None else FIELD_MS)
            last = r['host_ms']
        else:
            frame_iv.append(frame_iv[-1] if frame_iv else FIELD_MS)
    rep_flag = [40.0 if r['cause'] else 0.0 for r in rows]
    charts = [
        chart('Time between new pictures (ms) and repeated pictures', rows,
              [('time between new pictures', frame_iv, '#e08a00'), ('repeated picture', rep_flag, '#c0392b')], 50, 'ms',
              markers),
        chart('Real length of each VI field (ms) = Dolphin\'s VBlank graph', rows,
              [('field length', [r['dt_ms'] for r in rows], '#2e86de'),
               ('Dolphin work', [r['work_ms'] for r in rows], '#7f8c8d')], 50, 'ms', markers),
        chart('Emulated CPU and GPU busy (% of a field)', rows,
              [('CPU update+render', [r['cpu_pct'] for r in rows], '#27ae60'),
               ('GPU', [r['gpu_pct'] for r in rows], '#8e44ad'),
               ('GPU out of time', [100.0 if r['gpu_late'] > 0 else 0.0 for r in rows], '#c0392b')], 150, '%', markers),
        chart('Logic updates per field and shader compiles', rows,
              [('updates', [r['updates'] for r in rows], '#16a085'),
               ('shader compiles', [r['shaders'] for r in rows], '#d35400')], 4, '', markers),
    ]
    page = ('<!doctype html><html><head><meta charset="utf-8"><title>FFNY pacing report</title><style>'
            'body{font-family:system-ui,sans-serif;margin:16px;background:#fff;color:#222}'
            'pre{background:#f4f4f4;padding:10px;overflow-x:auto}.ch{background:#fafafa;border:1px solid #ddd;height:140px}'
            '.grid{stroke:#e2e2e2}.mark{stroke:#e74c3c;stroke-width:1.5}.lab{font-size:9px;fill:#888}'
            '.leg span{margin-right:14px;font-size:13px}.axis{display:flex;justify-content:space-between;font-size:11px;'
            'color:#888}@media (prefers-color-scheme:dark){body{background:#1b1b1b;color:#ddd}pre{background:#262626}'
            '.ch{background:#222;border-color:#444}.grid{stroke:#333}}</style></head><body><h2>FFNY pacing report</h2>'
            '<pre>%s</pre>%s</body></html>' % (html.escape(text), ''.join(charts)))
    out = os.path.splitext(path)[0] + '.html'
    with open(out, 'w', encoding='utf-8') as f:
        f.write(page)
    print('\nreport:', out)


def newest_log():
    cands = []
    for base in (os.path.join(os.path.expanduser('~'), 'Documents', 'Dolphin Emulator', 'Logs'),
                 os.path.join(os.path.dirname(os.path.abspath(__file__)), 'User', 'Logs'), '.'):
        cands += glob.glob(os.path.join(base, 'ffny_pacelog_*.csv'))
    return max(cands, key=os.path.getmtime) if cands else None


if __name__ == '__main__':
    p = sys.argv[1] if len(sys.argv) > 1 else newest_log()
    if not p:
        sys.exit('no ffny_pacelog_*.csv found; pass the file as an argument')
    report(p)
