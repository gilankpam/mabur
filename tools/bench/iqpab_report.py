#!/usr/bin/env python3
"""iqpab_report.py <session-dir> <cycles.txt>: per-cycle metrics from airdrain.analyze()
cycle window = [t_on - 1 s, t_on + climb window]; reports the cascade (demote) reached,
its air-excess peak/settle, the IDR of every demote step and every promote in the window."""
import sys, os, statistics as st  # usage: iqpab_report.py <session-dir> <cycles.txt> [climb_window_s]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import airdrain, session as _s
sess = _s.resolve(sys.argv[1]); cyc = []
for l in open(sys.argv[2]):
    p = l.split()
    if not l.startswith('cycle'): continue
    arm = int(p[2].split('=')[1]); t_on = int(p[4]); t_off = int(p[5])
    cyc.append((int(p[1]), arm, t_on, t_off))
W = float(sys.argv[3]) * 1000 if len(sys.argv) > 3 else 40000.0
r = airdrain.analyze(sess.ctl, sess.au)
A = airdrain.load_au(sess.au); airdrain.air_excess(A)
B = [a for a in A if a['sid'] == 0]          # base only: enh carries the injected loss
def base_peak(t0, w=1500):
    v = [a['air'] for a in B if t0 <= a['tf'] <= t0 + w]
    return max(v) if v else float('nan')
rows = []
for i, arm, t_on, t_off in cyc:
    lo, hi = t_on - 1000, t_on + W
    casc = [c for c in r['cascades'] if lo <= c['t0'] <= hi]
    sing = [c for c in r['singles'] if lo <= c['t0'] <= hi]
    idrs = [x for x in r['idrs'] if lo <= x['t0'] <= hi]
    dem = [x for x in idrs if x['to'] < x['from']]; pro = [x for x in idrs if x['to'] > x['from']]
    low = min([x['to'] for x in dem], default=None)
    r0 = [x['kb'] for x in dem if x['to'] == 0 and x['kb'] is not None]
    pk = max([base_peak(x['t0']) for x in dem], default=float('nan'))       # base-only, 1.5 s after each demote
    ppk = max([base_peak(x['t0']) for x in pro], default=float('nan'))      # same after each promote
    settle = max([c['settle_ms'] for c in casc], default=float('nan'))
    dkb = [x['kb'] for x in dem if x['kb'] is not None]; pkb = [x['kb'] for x in pro if x['kb'] is not None]
    rows.append((i, arm, low, len(dem), dkb, r0, pk, settle, len(pro), pkb, ppk))
    print(f"cycle {i} arm={arm:2d} lowest_rung={low} demotes={len(dem)} demoteIDR_kB={[round(k,1) for k in dkb]} "
          f"r0IDR={[round(k,1) for k in r0]} base_air_peak demote={pk:.0f}ms promote={ppk:.0f}ms settle={settle:.0f}ms promotes={len(pro)} promoteIDR_kB={[round(k,1) for k in pkb]}")
print("\nsummary by arm (median over cycles):")
for arm in sorted(set(x[1] for x in rows)):
    R = [x for x in rows if x[1] == arm]
    dk = [k for x in R for k in x[4]]; pk_ = [k for x in R for k in x[9]]; r0 = [k for x in R for k in x[5]]
    pks = [x[6] for x in R if x[6] == x[6]]; sts = [x[7] for x in R if x[7] == x[7]]; ppks = [x[10] for x in R if x[10] == x[10]]
    f = lambda v: f"{st.median(v):.1f}/{max(v):.1f}(n={len(v)})" if v else '-'
    print(f"  min_iqp={arm}: demote IDR kB p50/max {f(dk)} | rung-0 IDR {f(r0)} | promote IDR {f(pk_)} | "
          f"base air peak after demotes ms {f(pks)} | after promotes ms {f(ppks)} | settle ms {f(sts)} | reached r0 in {sum(1 for x in R if x[2]==0)}/{len(R)} cycles")
