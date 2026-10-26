import lzma, zlib, sys, glob
parts=sorted(glob.glob(sys.argv[1]+'.0*')); out=open(sys.argv[2],'wb')
dec=lzma.LZMADecompressor(format=lzma.FORMAT_RAW, filters=[{'id':lzma.FILTER_LZMA2,'dict_size':1<<30}])
crc=0; n=0; first=True; total=846791688-32-98
for p in parts:
    f=open(p,'rb')
    if first: f.read(32); first=False
    while total>0:
        b=f.read(min(1<<22,total))
        if not b: break
        total-=len(b)
        o=dec.decompress(b); crc=zlib.crc32(o,crc); n+=len(o); out.write(o)
print(n, hex(crc))
