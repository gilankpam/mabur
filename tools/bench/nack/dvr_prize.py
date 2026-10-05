import json,sys,statistics as st
from collections import Counter,defaultdict
def pct(v,p):
    v=sorted(v); return v[min(len(v)-1,int(p*len(v)))] if v else None
for d in sys.argv[1:]:
    print(f"\n===== session {d}")
    # --- sideport: flight or bench?
    rows=[json.loads(l) for l in open(f"{d}/flight.jsonl") if l.strip()]
    def g(r,path):
        for k in path.split("."):
            if not isinstance(r,dict) or k not in r: return None
            r=r[k]
        return r
    rssi=[g(r,"drone.uplink.rssi_a") for r in rows]; rssi=[x for x in rssi if x is not None]
    rung=[g(r,"link.ctl.rung.idx") for r in rows]; rung=[x for x in rung if x is not None]
    ch=Counter(g(r,"link.channel") for r in rows)
    span=(rows[-1]["t_ms"]-rows[0]["t_ms"])/1000
    tr=[g(r,"link.video.truncated") for r in rows]; dr=[g(r,"link.video.dropped") for r in rows]; cl=[g(r,"link.video.clean") for r in rows]
    tr=[x for x in tr if x is not None]; dr=[x for x in dr if x is not None]; cl=[x for x in cl if x is not None]
    print(f"span {span:.0f}s  uplink rssi min/med/max {min(rssi) if rssi else None}/{st.median(rssi) if rssi else None}/{max(rssi) if rssi else None}  rung hist {Counter(rung).most_common(6)}  channels {ch.most_common(3)}")
    print(f"sideport cum: clean {cl[0]}->{cl[-1]} (+{cl[-1]-cl[0]})  truncated {tr[0]}->{tr[-1]} (+{tr[-1]-tr[0]})  dropped {dr[0]}->{dr[-1]} (+{dr[-1]-dr[0]})")
    # --- au.log
    au=[]
    for l in open(f"{d}/au.log"):
        if l.startswith("#"): continue
        f=l.split()
        if len(f)<12: continue
        au.append(dict(t=int(f[0]),sid=int(f[2]),fid=int(f[3]),len=int(f[4]),flags=int(f[5],16),tf=int(f[7]),tc=int(f[8])))
    n=len(au); comp=[a for a in au if a["flags"]&0x80]; trunc=[a for a in au if not a["flags"]&0x80]
    t0=au[0]["t"]
    tsec=(au[-1]["t"]-t0)/1e6
    print(f"au rows {n} over {tsec:.0f}s ({n/tsec:.1f}/s): complete {len(comp)}, truncated {len(trunc)} ({len(trunc)/tsec*60:.2f}/min)")
    bysid=Counter(a["sid"] for a in trunc); print("  truncated by sid:",dict(bysid), " IDR among truncated:",sum(1 for a in trunc if a['flags']&1))
    # whole-frame drops = fid gaps
    gaps=[]; prev=None
    for a in au:
        if prev is not None and not (a["flags"]&2) and a["fid"]>prev+1: gaps.append((a["t"],a["fid"]-prev-1))
        prev=a["fid"]
    print(f"  fid gaps (whole AUs never emitted): {len(gaps)} events, {sum(g for _,g in gaps)} AUs ({sum(g for _,g in gaps)/tsec*60:.2f}/min); gap-size hist {Counter(g for _,g in gaps).most_common(6)}")
    # wait of truncated AUs (t_complete - t_first)
    w=[(a["tc"]-a["tf"])/1000 for a in trunc if a["tf"] and a["tc"]>=a["tf"]]
    if w: print(f"  truncated AU wait first->finish ms: p10 {pct(w,.1):.0f} p50 {pct(w,.5):.0f} p90 {pct(w,.9):.0f} max {max(w):.0f}")
    wc=[(a["tc"]-a["tf"])/1000 for a in comp if a["tf"] and a["tc"]>=a["tf"]]
    print(f"  complete AU first->finish ms: p50 {pct(wc,.5):.1f} p99 {pct(wc,.99):.1f}; >20ms (fec-wait) {sum(1 for x in wc if x>20)} ({sum(1 for x in wc if x>20)/max(1,len(wc))*100:.2f}%)")
    # first truncation after start (warm-up)
    if trunc: print(f"  first truncation at +{(trunc[0]['t']-t0)/1e6:.1f}s; truncations after +20s: {sum(1 for a in trunc if a['t']-t0>20e6)}")
    # time clustering of truncations: events within 1 s bins
    bins=Counter(int((a["t"]-t0)/1e6) for a in trunc)
    print(f"  truncation seconds: {len(bins)} distinct seconds; busiest {bins.most_common(5)}")
    # --- fec.log
    ep=[]
    for l in open(f"{d}/fec.log"):
        if l.startswith("feclog") or l.startswith("#"): continue
        f=l.split()
        if len(f)<13: continue
        ep.append(dict(t=int(f[0]),sid=int(f[1]),mcs=int(f[2]),bw=int(f[3]),ov=float(f[4]),first=int(f[5]),span=int(f[6]),m=int(f[7]),rec=int(f[8]),ab=int(f[9]),stale=int(f[10]),r=int(f[11]),w=int(f[12])))
    tot=len(ep); ab=[e for e in ep if e["ab"]>0 and e["stale"]==0]; abst=[e for e in ep if e["ab"]>0 and e["stale"]>0]
    print(f"fec episodes {tot}; recovered-only {tot-len(ab)-len(abst)}; abandoned (non-stale) {len(ab)}; abandoned-stale {len(abst)}")
    if ab:
        print(f"  abandoned by sid {dict(Counter(e['sid'] for e in ab))}; by mcs {dict(Counter(e['mcs'] for e in ab))}")
        a=[e["ab"] for e in ab]; m=[e["m"] for e in ab]; r=[e["r"] for e in ab]
        print(f"  symbols short (ab) p50 {pct(a,.5)} p90 {pct(a,.9)} max {max(a)}; missing m p50 {pct(m,.5)} p90 {pct(m,.9)}; repairs r p50 {pct(r,.5)}")
        print(f"  ab<=8: {sum(1 for x in a if x<=8)}, <=24: {sum(1 for x in a if x<=24)}, <=64: {sum(1 for x in a if x<=64)}, >64: {sum(1 for x in a if x>64)}")
        print(f"  abandoned symbols total {sum(a)} = {sum(a)*332/1e3:.0f} kB over {tsec:.0f}s")
    m_rec=[e["m"] for e in ep if e["ab"]==0]
    if m_rec: print(f"  recovered episodes m p50 {pct(m_rec,.5)} p90 {pct(m_rec,.9)} max {max(m_rec)}")
