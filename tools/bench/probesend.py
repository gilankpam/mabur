#!/usr/bin/env python3
"""probesend.py -- where do the GS's own control sends land relative to the
probe stream? (probe-blanking fix, 2026-09-05)

Inputs, all from one maburgs run on the GS:
  gaplog   stderr of maburgs started with MABUR_GAPLOG=1 ('gstx card=..
           mono=<us> reason=<r> ...' per control send, mono us at send time)
  probe    /media/dvr/probe-NNNN_<date>.log (probelog 2: first_ms column)
  aulog    /media/dvr/log/au-NNNN.log (flightrec, '# aulog 2': t_complete)

Prints: each send relative to the preceding enh/base completion; each
probe relative to its enh AU's completion; sends relative to the probe of
the same AU; and the decisive test -- of the LOST probes (seq gaps,
attributed to the enh AUs completing between the neighbouring rows), how
many had a send within the collision window, against the same rate for
delivered probes. Bench 2026-09-05 (pinned mcs4, feedback 50): 45/55 lost
vs 22 % baseline before the fix.

Usage: python3 tools/bench/probesend.py gaplog.txt <session-dir> | probe-NNNN.log au-NNNN.log
(run from the repo root: imports tools/flightreport.py)
"""
import sys, bisect, re, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import flightreport as fr
if len(sys.argv) < 2: sys.exit(__doc__)
gap = sys.argv[1]
arg2 = sys.argv[2] if len(sys.argv) > 2 else None
if arg2 is None or os.path.isdir(arg2):
    import session as _session   # tools/ is already on sys.path here
    s = _session.resolve(arg2)
    if s.probe is None or s.au is None:
        sys.exit('session needs both probe.log and au.log')
    probe, aulog = s.probe, s.au
else:
    if len(sys.argv) != 4: sys.exit(__doc__)
    probe, aulog = sys.argv[2], sys.argv[3]
pl = fr.load_probelog(probe); au = fr.load_aulog(aulog)
sends = [int(m.group(1)) for m in re.finditer(r'gstx card=\d+ mono=(\d+)', open(gap).read())]
sends.sort()
lo = min(r['first_ms'] for r in pl['rows']) * 1000; hi = max(r['first_ms'] for r in pl['rows']) * 1000
sends = [s for s in sends if lo - 50000 <= s <= hi + 50000]
enh = sorted((r['t_complete'], r['fid']) for r in au if r['sid'] == 1 and lo - 50000 <= r['t_complete'] <= hi + 50000)
base = sorted(r['t_complete'] for r in au if r['sid'] == 0 and lo - 50000 <= r['t_complete'] <= hi + 50000)
etc = [t for t, _ in enh]
def pct(v, q):
    if not v: return float('nan')
    s = sorted(v); return s[min(len(s) - 1, int(q * len(s)))]
# A. send relative to preceding completion (which kind)
d_enh, d_base = [], []
for s in sends:
    ie = bisect.bisect_right(etc, s) - 1; ib = bisect.bisect_right(base, s) - 1
    te = etc[ie] if ie >= 0 else -1; tb = base[ib] if ib >= 0 else -1
    if te < 0 and tb < 0: continue
    if te > tb: d_enh.append((s - te) / 1000)
    else: d_base.append((s - tb) / 1000)
print(f"sends={len(sends)} after-enh={len(d_enh)} after-base={len(d_base)}")
print("send - enh completion (ms): p10 %.2f p50 %.2f p90 %.2f p99 %.2f" % tuple(pct(d_enh, q) for q in (.1, .5, .9, .99)))
print("send - base completion (ms): p10 %.2f p50 %.2f p90 %.2f p99 %.2f" % tuple(pct(d_base, q) for q in (.1, .5, .9, .99)))
# B. probe relative to completion
pairs = fr.probe_au_offset_rows(pl, au)
offs = [o for _, o in pairs]
print("probe - enh completion (ms): p10 %.2f p50 %.2f p90 %.2f p99 %.2f n=%d" % (*(pct(offs, q) for q in (.1, .5, .9, .99)), len(offs)))
# send relative to the probe of the same AU (delivered probes only)
by_fid = {}
for r, o in pairs: by_fid[r['enh_fid']] = r['first_ms'] * 1000
d_sp = []
for s in sends:
    ie = bisect.bisect_right(etc, s) - 1
    if ie < 0: continue
    te, fid = enh[ie]
    if s - te > 12000: continue
    if fid in by_fid: d_sp.append((s - by_fid[fid]) / 1000)
print("send - probe arrival, same AU (ms): n=%d p10 %.2f p50 %.2f p90 %.2f ; within [-2,+1] ms: %d" %
      (len(d_sp), *(pct(d_sp, q) for q in (.1, .5, .9)), sum(1 for d in d_sp if -2 <= d <= 1)))
# C. lost probes: seq gaps -> the enh AU(s) between neighbours; send within window?
rows = [r for r in pl['rows'] if r['first_ms'] is not None]
fid_of = {r['enh_fid']: r for r in rows}
enh_by_fid = {fid: t for t, fid in enh}
def has_send(tc, a=-2000, b=6000):
    i = bisect.bisect_left(sends, tc + a); return i < len(sends) and sends[i] <= tc + b
lost_tc = []
for p, n in zip(rows, rows[1:]):
    if n['seq'] - p['seq'] <= 1: continue
    # enh AUs completing strictly between the two neighbours' arrivals
    i0 = bisect.bisect_right(etc, p['first_ms'] * 1000); i1 = bisect.bisect_left(etc, n['first_ms'] * 1000)
    # the neighbours' own AUs complete inside that interval too (the probe
    # lands after its AU): exclude them, they were delivered
    for t, fid in enh[i0:i1]:
        if fid not in (p['enh_fid'], n['enh_fid']): lost_tc.append((t, fid, n['seq'] - p['seq'] - 1))
deliv_tc = [enh_by_fid[r['enh_fid']] for r in rows if r['enh_fid'] in enh_by_fid]
for w in ((-2000, 6000), (-1000, 4000), (0, 3000)):
    l = sum(1 for t, _, _ in lost_tc if has_send(t, *w)); d = sum(1 for t in deliv_tc if has_send(t, *w))
    print(f"window {w[0]/1000:+.0f}..{w[1]/1000:+.0f} ms after completion: lost with send {l}/{len(lost_tc)}  delivered with send {d}/{len(deliv_tc)} ({d/len(deliv_tc):.1%})")
near_all = []
for t, fid, k in lost_tc:
    i = bisect.bisect_left(sends, t - 3000)
    near_all += [(s - t) / 1000 for s in sends[i:i + 4] if s - t < 8000]
print("lost-probe AUs: sends rel completion (ms), sorted:", ' '.join('%.1f' % x for x in sorted(near_all)))
