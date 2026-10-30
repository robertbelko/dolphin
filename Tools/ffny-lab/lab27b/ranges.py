import re, numpy as np, sys
SRC='/home/claude/lab-dolphin/Source/Core/Core/FFNYRollbackRanges.inc'
txt=open(SRC).read()
STATIC=[(int(a,16)&0x1FFFFFF,int(b,16)&0x1FFFFFF) for a,b in re.findall(r'\{\s*0x([0-9A-Fa-f]+)u?\s*,\s*0x([0-9A-Fa-f]+)u?\s*\}',txt)]
def load(p): return open(p,'rb').read()
def rd(m,a): a&=0x1FFFFFF; return int.from_bytes(m[a:a+4],'big')
def norm(v):
    v=sorted(p for p in v if p[0]<p[1]); r=[]
    for p in v:
        if r and p[0]<=r[-1][1]: r[-1]=(r[-1][0],max(r[-1][1],p[1]))
        else: r.append(p)
    return r
def blocks(m):
    H0=0x8060C050; base=rd(m,H0+8); end=rd(m,H0+0xC); a=base+0x90; out=[]; n=0
    while base<=a<end and n<20000:
        w=rd(m,a); sz=rd(m,a+4); nx=rd(m,a+8)
        out.append((a,w,sz,nx))
        if nx<=a: break
        a=nx; n+=1
    return out
def ranges(m):
    inc=list(STATIC); ex=[]
    H0=0x8060C050; base=rd(m,H0+8)
    inc.append((base&0x1FFFFFF,(base+0x90)&0x1FFFFFF))
    for a,w,sz,nx in blocks(m):
        inc.append((a&0x1FFFFFF,(a+(0x18 if (w>>16)==0x4246 else 0x10))&0x1FFFFFF))
        if (w>>16)==0x424D and sz and not (w&0x80):
            mg=rd(m,a+16)
            if mg not in (0x4D4F4952,0x41424B43,0x454D4954): inc.append(((a+16)&0x1FFFFFF,(a+16+sz)&0x1FFFFFF))
    for k in range(64):
        h=rd(m,0x8040BD70+4*k)
        if (h&0xFE000000)==0x80000000: ex.append(((h+0x3C)&0x1FFFFFF,(h+0x54)&0x1FFFFFF))
    ex+= [(0x49A400,0x49A410),(0x49B400,0x49BF00),(0x8043FC20&0x1FFFFFF,(0x8043FC20+8)&0x1FFFFFF)]
    pool=rd(m,0x802E9B00)
    if pool: ex.append(((pool+0x2A4)&0x1FFFFFF,(pool+0x2BC)&0x1FFFFFF))
    t=rd(m,0x800000DC); k=0
    while t and k<64:
        ex.append((t&0x1FFFFFF,(t+0x318)&0x1FFFFFF))
        if t!=0x80432D10: ex.append((rd(m,t+0x308)&0x1FFFFFF,rd(m,t+0x304)&0x1FFFFFF))
        t=rd(m,t+0x2FC); k+=1
    inc=norm(inc); ex=norm(ex)
    # subtract
    out=[]
    for lo,hi in inc:
        parts=[(lo,hi)]
        for el,eh in ex:
            nxt=[]
            for a,b in parts:
                if eh<=a or el>=b: nxt.append((a,b)); continue
                if a<el: nxt.append((a,el))
                if eh<b: nxt.append((eh,b))
            parts=nxt
        out+=parts
    return out
