#!/usr/bin/env python3
"""Aggregate pcsample rows by function via addr2line. usage: symbolize.py pc.txt"""
import subprocess, sys, collections, os
B="/home/gilankpam/Projects/drone/openipc-builder/openipc/output"
A2L=B+"/host/bin/arm-openipc-linux-gnueabihf-addr2line"
READELF=B+"/host/bin/arm-openipc-linux-gnueabihf-readelf"
SYS=B+"/host/arm-buildroot-linux-gnueabihf/sysroot"
DSO={"/usr/bin/maburd": B+"/build/mabur-custom/buildroot-build/drone/maburd"}
def local(dso):
    if dso in DSO: return DSO[dso]
    base=os.path.basename(dso)
    for d in (SYS+"/lib", SYS+"/usr/lib", B+"/target/lib", B+"/target/usr/lib"):
        p=os.path.join(d, base)
        if os.path.exists(p): return p
        # libstdc++.so.6 -> libstdc++.so.6.0.32
        for f in os.listdir(d) if os.path.isdir(d) else []:
            if f.startswith(base): return os.path.join(d,f)
    return None
def is_exec(path):
    out=subprocess.run([READELF,"-h",path],capture_output=True,text=True).stdout
    return "EXEC" in out and "DYN" not in out
def loads(path):
    """[(p_offset, p_vaddr, p_filesz)] of PT_LOAD segments."""
    out=subprocess.run([READELF,"-lW",path],capture_output=True,text=True).stdout
    segs=[]
    for l in out.splitlines():
        f=l.split()
        if len(f)>=6 and f[0]=="LOAD": segs.append((int(f[1],16),int(f[2],16),int(f[4],16)))
    return segs
def vaddr(path, off, segs):
    for o,v,sz in segs:
        if o<=off<o+sz: return off-o+v
    return off
rows=[]; hdr=""
for line in open(sys.argv[1]):
    if line.startswith("#"): hdr=line.strip(); continue
    n,ip,dso,off=line.split(); rows.append((int(n),int(ip,16),dso,int(off,16)))
total=sum(r[0] for r in rows)
print(hdr); print("user samples in table:", total)
bydso=collections.defaultdict(list)
for r in rows: bydso[r[2]].append(r)
func=collections.Counter(); funcfile=collections.Counter(); dsotot=collections.Counter()
for dso,rs in bydso.items():
    path=local(dso); dsotot[dso]=sum(r[0] for r in rs)
    if not path: 
        func[("?",dso)]+=dsotot[dso]; continue
    ex=is_exec(path); segs=loads(path)
    addrs=[hex(r[1] if ex else vaddr(path, r[3], segs)) for r in rs]
    out=subprocess.run([A2L,"-f","-C","-e",path]+addrs,capture_output=True,text=True).stdout.splitlines()
    for i,r in enumerate(rs):
        fn=out[2*i] if 2*i<len(out) else "?"; fl=out[2*i+1] if 2*i+1<len(out) else "?"
        fl=fl.split(":")[0].split("/")[-1]
        func[(fn,os.path.basename(dso))]+=r[0]; funcfile[(fn,fl)]+=r[0]
print("\n== by DSO"); 
for d,n in dsotot.most_common(): print("%6d %5.1f%%  %s" % (n,100*n/total,d))
print("\n== by function (top 40)")
for (fn,d),n in func.most_common(40): print("%6d %5.1f%%  %-60s %s" % (n,100*n/total,fn[:60],d))
