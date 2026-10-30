import struct, sys, subprocess
src, dst = sys.argv[1], sys.argv[2]
d = bytearray(open(src, 'rb').read())
offs = struct.unpack('>18I', d[0:72]); addrs = struct.unpack('>18I', d[72:144]); sizes = struct.unpack('>18I', d[144:216])
def fo(a):
    for o, ad, sz in zip(offs, addrs, sizes):
        if sz and ad <= a < ad + sz: return o + a - ad
    raise ValueError(hex(a))
syms = {}
for l in subprocess.run(['llvm-nm', '/home/claude/lab/rbtag/rbtag.elf'], capture_output=True, text=True).stdout.split('\n'):
    p = l.split()
    if len(p) == 3: syms[p[2]] = int(p[0], 16)
code = open('/home/claude/lab/rbtag/rbtag.bin', 'rb').read()
assert 0x8049A410 + len(code) <= 0x8049B400, "code runs into the data area"
o = fo(0x8049A400)
assert d[o:o + 0x10 + len(code)] == bytes(0x10 + len(code)), "area not empty"
d[o + 0x10:o + 0x10 + len(code)] = code
def put(a, old, new):
    p = fo(a); cur = struct.unpack('>I', d[p:p + 4])[0]
    assert cur == old, (hex(a), hex(cur), hex(old))
    d[p:p + 4] = struct.pack('>I', new)
    return (p, old, new)
bl = lambda frm, to: 0x48000001 | ((to - frm) & 0x3FFFFFC)
b = lambda frm, to: 0x48000000 | ((to - frm) & 0x3FFFFFC)
edits = [put(0x80003AD0, bl(0x80003AD0, 0x8049A100), bl(0x80003AD0, syms['rbtag_boot']))]
for site in (0x80249C68, 0x80249C98, 0x80249CC8):
    edits.append(put(site, bl(site, 0x80249884), bl(site, syms['rb_alloc'])))
edits.append(put(0x8024A0C0, 0x9421FFC0, b(0x8024A0C0, syms['rb_free'])))
edits.append(put(0x80244408, 0x9421FDD8, b(0x80244408, syms['rb_fsq'])))
edits.append(put(0x801F327C, 0x9421FFE8, b(0x801F327C, syms['rb_acq'])))
edits.append(put(0x801F33A4, 0x9421FFF0, b(0x801F33A4, syms['rb_rel'])))
open(dst, 'wb').write(d)
print("ok", len(code), "bytes;", ", ".join(f"{k}={v:08x}" for k, v in syms.items() if k.startswith('rb')))
for p, old, new in edits: print(f"  (0x{p:x}, {old:08x} -> {new:08x})")
print("  code at file 0x%x" % (o + 0x10))
