#!/usr/bin/env python3
"""rbtag v6 for a Start.dol (FFNY rollback, 27 October 2026).

    python rbtag_v6_install.py IN.dol OUT.dol

IN must already have `rbstate` (DOL patch, needed by rbtag) and may have rbtag v5 or no rbtag: the code area
0x8049A400-0x8049B400 is replaced by rbtag v6 and the eight hook sites are pointed at it (each site must hold either
its original instruction or a branch into the rbtag area). Also checks that the two bigspace words the rollback Dolphin
uses for menu rollback (0x8049CFE0 "LOADBUF" name, 0x8049CFF0 file-id counter) are free.
Source: rbtag.c / tramp.s / link.ld / build.sh in this folder (built with clang for PowerPC 750)."""
import base64, struct, sys

CODE = base64.b64decode(
    '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'
    'gJ3/9JCGAPQ4hQABkJ0AAEv//4R8CAKmlCH/0DxggEmQAQA0OIAAAZPBAChgfrUQkwEAEJMhABSTQQAYk2EAHJOBACCToQAkkJ4A'
    'DIB+//goAwAAQYIBHDuAAACAfv/4k57/+IC+AAg7o///OGAAACgFAABBggBEfKkDpjyggEk4ngboYKW8BEgAAAhCQAAshMQACIDk'
    'AAR8B+hAQID/8FRoGDh9PkIUOGMAAZDJBvB86CkuS///2JB+AAiDfgAAKBsAAEGCAFA8YIBJO14A6DuAAABgebYESAAAJFeFGDh8'
    '3ioUO5wAAZBmAPB8hckuO3v//ygbAABAgQAchHoACICaAAR8BOhAQYD/1EgAAt1L///gk54AAIB+AAQoAwAAQYIASDyggEk4ngPo'
    'fGkDpjhgAABgpbkESAAACEJAADCExAAIgOQABHwH6EBAgP/wVGgYOH0+QhQ4YwABkMkD8HzoKS5L///YOGAAAJB+AASAfv/wKAMA'
    'AEGCAAyDvu78SAAACDug//+DfgAEO4AAADtAAAAoGwAAQYIATDxggEk7PgPoYHi5BEgAACRXRRg4fN4qFDtaAAGQZgPwfIXBLjt7'
    '//8oGwAAQIEAHIR5AAiAmQAEfAToQEGB/9RIAAINS///4JNeAASAngAAKAQAAEGCAEh8iQOmPICASTh+AOg7gAAAYIS2BEgAAAhC'
    'QAAshKMACIDDAAR8BuhAQIH/8FeHGDh9HjoUO5wAAZCoAPB8xyEuS///2JOeAAA7gAAAg34ACDtAAAAoGwAAQYIATDxggEk7Pgbo'
    'YHi8BEgAACRXRRg4fN4qFDtaAAGQZgbwfIXBLjt7//8oGwAAQIEAHIR5AAiAmQAEfAToQEGB/9RIAAFBS///4JNeAAiTngAMg8EA'
    'KIOhACSDgQAgg2EAHINBABiDIQAUgwEAEIABADQ4IQAwfAgDpk6AACB8CAKmlCH/4JABACQ8gIAAk6EAFJPBABiAhADkbISAQygE'
    'LRA8gIBJYJ61FECCAEyAngAIKAQAAECCAECAnv/0KAQAAECCACiAngAAKAQAAECCAByAnv/8KAQAAECCABCAngAEKAQAAEGCABB8'
    'fRt4S//82X+j63goAwAAQYIAbICe/+woBAAAQYIAYICeAAgoBAAAQIIAVDyAgABghADkgIQAADyggENgpS0QfAQoQECCADiAngAA'
    'KAQAX0GBACyAngAAVIUYOHy+KhSQZQPsgH7/8JBlA/A4ZAABkH4AADhgAABIAAAISAAANYPBABiDoQAUgAEAJDghACB8CAOmToAA'
    'IJQh/8BL2u5UlCH92EvakZSUIf/oS9WAAJQh//BL1YEgU3lzSGVhcABBVURJTwBBdWRpbwBhdWRpbwBHRU5SU1RSTQBTUENIU1RS'
    'TQBTUENIQk5LAEFCS1JFUwBGSUZPAEZyYW1lQnVmZmVyAEdDQnVmZmVyAFRocmVhZFN0YWNrAEZpbGUgU3lzAGJpZ2ZpbGUARml4'
    'ZWQgTVBGAEFTWU5DRklMRQBfQXN5bmNMb2FkZXJDYWxsYmFjawBTdHJlYW0ASW5wdXRCdWZmZXIATUNUaHJlYWQATXlUaHJlYWQA'
    'U25kSGVhcABTeXNIZWFwAAA=')
SYMS = {'acq_body': 2149528192, 'free_body': 2149884100, 'fsq_body': 2149860364, 'orig_acq': 2152313468, 'orig_free': 2152313452, 'orig_fsq': 2152313460, 'orig_rel': 2152313476, 'rb_acq': 2152312196, 'rb_alloc': 2152310340, 'rb_free': 2152311160, 'rb_fsq': 2152311940, 'rb_rel': 2152313192, 'rbtag_boot': 2152309776, 'rbtag_hook': 2152310092, 'rel_body': 2149528488, 'snd_settle': 2152312496}

def main():
    if len(sys.argv) != 3:
        print(__doc__); sys.exit(1)
    d = bytearray(open(sys.argv[1], 'rb').read())
    offs = struct.unpack('>18I', d[0:72]); addrs = struct.unpack('>18I', d[72:144]); sizes = struct.unpack('>18I', d[144:216])
    def fo(a):
        for o, ad, sz in zip(offs, addrs, sizes):
            if sz and ad <= a < ad + sz:
                return o + a - ad
        raise SystemExit("address %08x is not in the DOL (bigspace patch missing?)" % a)
    rd = lambda a: struct.unpack('>I', d[fo(a):fo(a) + 4])[0]
    if rd(0x8049A100) == 0:
        raise SystemExit("rbstate is not in this DOL (0x8049A100 is empty) - apply rbstate first")
    for a in (0x8049CFE0, 0x8049CFF0):
        if rd(a) != 0:
            print("WARNING: %08x is used by another patch (%08x) - the rollback Dolphin writes there in menu rollback" % (a, rd(a)))
    assert 0x8049A410 + len(CODE) <= 0x8049B400
    o = fo(0x8049A400)
    d[o:o + 0x1000] = bytes(0x1000)            # 0x8049A400-0x8049B400: old rbtag code + its words
    d[o + 0x10:o + 0x10 + len(CODE)] = CODE
    bl = lambda frm, to: 0x48000001 | ((to - frm) & 0x3FFFFFC)
    b = lambda frm, to: 0x48000000 | ((to - frm) & 0x3FFFFFC)
    def target(frm, w):
        off = w & 0x3FFFFFC
        if off & 0x2000000: off -= 0x4000000
        return (frm + off) & 0xFFFFFFFF
    sites = [(0x80003AD0, bl(0x80003AD0, 0x8049A100), 'rbtag_boot', bl)]
    for s in (0x80249C68, 0x80249C98, 0x80249CC8):
        sites.append((s, bl(s, 0x80249884), 'rb_alloc', bl))
    sites += [(0x8024A0C0, 0x9421FFC0, 'rb_free', b), (0x80244408, 0x9421FDD8, 'rb_fsq', b),
              (0x801F327C, 0x9421FFE8, 'rb_acq', b), (0x801F33A4, 0x9421FFF0, 'rb_rel', b)]
    for a, orig, sym, mk in sites:
        cur = rd(a)
        old_rbtag = (cur >> 26) == 18 and 0x8049A400 <= target(a, cur) < 0x8049B400
        if cur != orig and not old_rbtag:
            raise SystemExit("hook site %08x holds %08x: neither the original %08x nor a branch into rbtag" % (a, cur, orig))
        new = mk(a, SYMS[sym])
        d[fo(a):fo(a) + 4] = struct.pack('>I', new)
        print("  %08x: %08x -> %08x (%s)" % (a, cur, new, sym))
    open(sys.argv[2], 'wb').write(d)
    print("rbtag v6 installed (%d bytes of code) -> %s" % (len(CODE), sys.argv[2]))

main()
