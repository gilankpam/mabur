#!/usr/bin/env python3
"""Per-arm summary from a session dir (docs/fec-nack.md "Bench"): au.log
(truncated/dropped, first->finish), fec.log (abandoned episodes, NACK fills
per sid), flight.jsonl (air_pct, tx pps, the last link.nack block, the
drone.nack counters).

fec.log is read by its version marker through flightreport.load_feclog
(feclog 1/2/3; rtx exists only in feclog 3, older rows read rtx 0)."""
import json, os, statistics as st, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from flightreport import load_feclog  # noqa: E402  (tools/flightreport.py)


def pct(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))] if v else 0


def g(r, p):
    for k in p.split("."):
        if not isinstance(r, dict) or k not in r:
            return None
        r = r[k]
    return r


for d in sys.argv[1:]:
    au = []
    for l in open(f"{d}/au.log"):
        if l.startswith("#"):
            continue
        f = l.split()
        if len(f) < 12:
            continue
        au.append((int(f[0]), int(f[2]), int(f[3]), int(f[5], 16), int(f[7]), int(f[8])))
    t0 = au[0][0]
    tsec = max(1e-9, (au[-1][0] - t0) / 1e6)
    trunc = [a for a in au if not a[3] & 0x80]
    gaps = 0
    prev = None
    for a in au:
        if prev is not None and not (a[3] & 2) and a[2] > prev + 1:
            gaps += a[2] - prev - 1
        prev = a[2]
    w = [(a[5] - a[4]) / 1000 for a in au if a[3] & 0x80 and a[4] and a[5] >= a[4]]

    ep = load_feclog(f"{d}/fec.log")
    ab = [e for e in ep if e["aband"] > 0]
    absyms = sum(e["aband"] for e in ab)
    rtx = {}
    for e in ep:
        rtx[e["sid"]] = rtx.get(e["sid"], 0) + e["rtx"]

    rows = [json.loads(l) for l in open(f"{d}/flight.jsonl") if l.strip()]
    air = [x for x in (g(r, "link.air_pct") for r in rows) if x is not None]
    tx = [sum((c.get("tx_pps") or 0) for c in (r.get("cards") or [])) for r in rows]
    cpu = [g(r, "drone.sys.cpu_pct") or g(r, "sys.cpu_pct") for r in rows]
    cpu = [x for x in cpu if x is not None]
    nack = next((g(r, "link.nack") for r in reversed(rows) if g(r, "link.nack")), None)
    # drone.nack is per Telem period and repeated on every record until the
    # next Telem: count each drone.tlm_seq once.
    dn = {"rx": 0, "retx_syms": 0, "retx_refused": 0}
    seen = set()
    for r in rows:
        dk, seq = g(r, "drone.nack"), g(r, "drone.tlm_seq")
        if not dk or seq is None or seq in seen:
            continue
        seen.add(seq)
        for k in dn:
            dn[k] += dk.get(k) or 0

    print(f"== {d}: {tsec:.0f}s, {len(au)} AUs ({len(au)/tsec:.1f}/s)")
    print(f"   truncated {len(trunc)} ({len(trunc)/tsec*60:.1f}/min)  base {sum(1 for a in trunc if a[1]==0)} enh {sum(1 for a in trunc if a[1]==1)}   dropped(fid gaps) {gaps} ({gaps/tsec*60:.1f}/min)")
    print(f"   complete first->finish ms p50 {pct(w,.5):.1f} p90 {pct(w,.9):.1f} p99 {pct(w,.99):.1f} max {max(w) if w else 0:.0f}  >20ms {sum(1 for x in w if x>20)}")
    print(f"   fec abandoned episodes {len(ab)} ({len(ab)/tsec*60:.1f}/min) syms {absyms}  base {sum(1 for e in ab if e['sid']==0)} enh {sum(1 for e in ab if e['sid']==1)}")
    print(f"   fec rtx (NACK-filled syms) per sid {dict(sorted(rtx.items()))}")
    print(f"   air_pct median {st.median(air) if air else None}  GS tx_pps median {st.median(tx) if tx else None}  drone cpu median {st.median(cpu) if cpu else None}")
    print(f"   drone.nack summed per tlm_seq: rx {dn['rx']} retx_syms {dn['retx_syms']} retx_refused {dn['retx_refused']}")
    print(f"   link.nack (last): {json.dumps(nack, sort_keys=True) if nack else None}")
