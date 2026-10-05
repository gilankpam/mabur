#!/usr/bin/env python3
"""Per-arm summary from a session dir: au.log (truncated/dropped, first->finish),
fec.log (abandoned episodes), maburgs.log (last nack line), flight.jsonl (air_pct, tx pps)."""
import json,re,sys,statistics as st
def pct(v,p):
    v=sorted(v); return v[min(len(v)-1,int(p*len(v)))] if v else 0
for d in sys.argv[1:]:
    au=[]
    for l in open(f"{d}/au.log"):
        if l.startswith("#"): continue
        f=l.split()
        if len(f)<12: continue
        au.append((int(f[0]),int(f[2]),int(f[3]),int(f[5],16),int(f[7]),int(f[8])))
    t0=au[0][0]; tsec=(au[-1][0]-t0)/1e6
    trunc=[a for a in au if not a[3]&0x80]
    gaps=0; prev=None
    for a in au:
        if prev is not None and not (a[3]&2) and a[2]>prev+1: gaps+=a[2]-prev-1
        prev=a[2]
    w=[(a[5]-a[4])/1000 for a in au if a[3]&0x80 and a[4] and a[5]>=a[4]]
    ep=[l.split() for l in open(f"{d}/fec.log") if not l.startswith("feclog") and not l.startswith("#")]
    ab=[e for e in ep if len(e)>=13 and int(e[9])>0]
    absyms=sum(int(e[9]) for e in ab)
    nack=""
    try:
        for l in open(f"{d}/maburgs.log"):
            if l.startswith("maburgs nack:"): nack=l.strip()
    except FileNotFoundError: pass
    rows=[json.loads(l) for l in open(f"{d}/flight.jsonl") if l.strip()]
    def g(r,p):
        for k in p.split("."):
            if not isinstance(r,dict) or k not in r: return None
            r=r[k]
        return r
    air=[g(r,"link.air_pct") for r in rows]; air=[x for x in air if x is not None]
    tx=[sum((c.get("tx_pps") or 0) for c in (r.get("cards") or [])) for r in rows]
    cpu=[g(r,"drone.sys.cpu_pct") or g(r,"sys.cpu_pct") for r in rows]; cpu=[x for x in cpu if x is not None]
    print(f"== {d}: {tsec:.0f}s, {len(au)} AUs ({len(au)/tsec:.1f}/s)")
    print(f"   truncated {len(trunc)} ({len(trunc)/tsec*60:.1f}/min)  base {sum(1 for a in trunc if a[1]==0)} enh {sum(1 for a in trunc if a[1]==1)}   dropped(fid gaps) {gaps} ({gaps/tsec*60:.1f}/min)")
    print(f"   complete first->finish ms p50 {pct(w,.5):.1f} p90 {pct(w,.9):.1f} p99 {pct(w,.99):.1f} max {max(w) if w else 0:.0f}  >20ms {sum(1 for x in w if x>20)}")
    print(f"   fec abandoned episodes {len(ab)} ({len(ab)/tsec*60:.1f}/min) syms {absyms}  base {sum(1 for e in ab if e[1]=='0')} enh {sum(1 for e in ab if e[1]=='1')}")
    print(f"   air_pct median {st.median(air) if air else None}  GS tx_pps median {st.median(tx) if tx else None}  drone cpu median {st.median(cpu) if cpu else None}")
    print(f"   {nack}")
