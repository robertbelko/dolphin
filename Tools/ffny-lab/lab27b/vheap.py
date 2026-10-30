import sys
def be(b,o): return int.from_bytes(b[o:o+4],'big')
def walk(mem, vmem):
    VB=0x7E000000
    rd=lambda a: be(mem,(a&0x1FFFFFF)) if (a>>24)==0x80 or (a>>24)==0x81 else be(vmem,a-VB)
    out=[]
    for k in range(16):
        h=be(mem,(0x8040BD70+4*k)&0x1FFFFFF)
        if not (VB<=h<VB+0x2000000): continue
        base=rd(h+8); end=rd(h+12)
        a=rd(h+0x20)
        ok=lambda b: base<=b<end and (rd(b)>>16) in (0x424D,0x4246)
        if not ok(a): a=base+0x90
        n=0
        while n<40000:
            pv=rd(a+12)
            if not ok(pv) or pv>=a: break
            a=pv; n+=1
        blocks=[]; n=0
        while base<=a<end and n<40000:
            w=rd(a); sz=rd(a+4); nx=rd(a+8)
            blocks.append((a,w>>16,sz,w&0xFFFF))
            if nx<=a: break
            a=nx; n+=1
        out.append((k,h,base,end,blocks))
    return out
if __name__=='__main__':
    mem=open(sys.argv[1],'rb').read(); vmem=open(sys.argv[2],'rb').read()
    for k,h,base,end,bl in walk(mem,vmem):
        free=[b for b in bl if b[1]==0x4246]; used=[b for b in bl if b[1]==0x424D]
        print('heap %d desc %08x %08x-%08x blocks %d used %d KB free %d KB largest free %d KB'%(k,h,base,end,len(bl),sum(b[2] for b in used)//1024,sum(b[2] for b in free)//1024,max([b[2] for b in free] or [0])//1024))
        for b in sorted(used,key=lambda b:-b[2])[:12]:
            print('   used %08x %7d KB flags %04x'%(b[0],b[2]//1024,b[3]))
