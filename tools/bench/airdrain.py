#!/usr/bin/env python3
"""airdrain.py -- per-frame `air` excess around every rung transition: is the
post-demote latency spike a drain, and how big/long is it? (flights 20/21,
docs/probe-stream-flight-findings-2026-09-05.md section 9; the measurement
the drain-shed follow-up is A/B'd with)

Replays the player's `air` arithmetic (gs/player/src/lat_tracker.cpp) over
flightrec's AU log: per AU, adj = t_first - enc - drone_q - pts, minus a
running-min floor with PtsAnchor's 60 ppm upward leak and its 2 s pts
discontinuity reset -- so it needs no lat-NNNN.log at all, and it is per
FRAME, not per second. Then it joins the ctl log's E lines
(flightreport.find_episodes clusters demotes into episodes) and prints, per
cascade (>= 2 steps), the 250 ms-binned excess from -1.0 s to +3.0 s around
the FIRST demote with the IDRs marked, the peak / time-to-peak / settle,
the excess in the 1 s BEFORE the demote (a fade that adds latency would
show here; flights 20/21: ~4 ms, it does not), and the bytes put on air in
the first 500 ms after the demote against the NEW rung's nominal PHY rate
(>= 100 % is the over-full pipe the drain is made of). Single demotes and
promotes get peak summaries; the steady-state excess per rung (frames >= 3 s
from any transition) and standalone spike seconds close the report.

Inputs (one boot -- the ctl and au indices are DIFFERENT counters, match
them by mono span; the lat log is not needed and its index is a third
counter, see docs/observability.md):
  ctl    /media/dvr/ctl-NNNN_<date>.log   (ctllog >= 8 for the ov pairs)
  aulog  /media/dvr/log/au-NNNN.log       (flightrec '# aulog 3', air_ms
                                          column 12; aulog-2 logs still parse)

Usage: python3 tools/bench/airdrain.py ctl-NNNN_<date>.log au-NNNN.log
         [--from S --to S] [--spike MS] [--profiles] [--model]
  --from/--to  analysis window in ctl seconds (default: first E line to the
               last S line; starves open no episode)
  --spike MS   per-frame excess threshold for the spike-second list (15)
  --profiles   print every cascade's 250 ms bin row (default: summary only)
  --model      compare the drone's air-clock model (aulog-3 `air_ms`) against
               the GS-measured air+q per frame -- a through-origin slope,
               residual percentiles, per-cascade peak pairs, and the quiet
               floor; prints one line saying so instead if the log predates
               aulog 3 (docs/data-provenance.md, 2026-09-06 air clock)
(run from the repo root: imports tools/flightreport.py; tests/test_airdrain.py)
"""
import argparse, bisect, collections, os, statistics as st, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import flightreport as fr

# Nominal HT20 long-GI single-stream PHY rates, Mb/s, by MCS. "Nominal" on
# purpose: real goodput at 1400 B PPDUs is ~70 % of this, so a first-500 ms
# on-air figure of 70 % below already means the pipe is full.
NOMINAL_MBPS = {0: 6.5, 1: 13.0, 2: 19.5, 3: 26.0, 4: 39.0, 5: 52.0, 6: 58.5, 7: 65.0}
LEAK_PPM = 60.0          # PtsAnchor::kLeakPpm
RESYNC_MS = 2000.0       # PtsAnchor::kResyncUs: a bigger pts step is a drone restart
SETTLE_MS = 15.0         # "drained" = 5 consecutive frames under this
STEADY_GUARD_MS = 3000.0 # steady state = this far from any E line


def pct(v, q):
    if not v: return float('nan')
    s = sorted(v); return s[min(len(s) - 1, int(q * len(s)))]


def load_au(path):
    """Full aulog-2 rows (flightreport.load_aulog keeps only the join keys)."""
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith('#'): continue
            p = line.split()
            if len(p) < 11 or int(p[7]) == 0: continue   # t_first 0 = unknown stamp
            rows.append({'tc': int(p[8]) / 1e3, 'tf': int(p[7]) / 1e3, 'pts': int(p[1]) / 1e3,
                         'sid': int(p[2]), 'len': int(p[4]), 'idr': int(p[5], 16) & 1,
                         'enc': int(p[9]) / 1e3, 'q': int(p[10]),
                         'air_ms': int(p[11]) if len(p) >= 12 else None})
    rows.sort(key=lambda a: a['tf'])
    return rows


def air_excess(rows):
    """PtsAnchor replay in place: base = min(adj - pts), snap-down instant,
    +60 ppm upward leak, reset on a > 2 s pts step (drone restart)."""
    base = None; last_tf = None; last_pts = None
    for a in rows:
        adj = a['tf'] - a['enc'] - a['q'] - a['pts']
        if base is None or abs(a['pts'] - last_pts) > RESYNC_MS:
            base = adj
        else:
            base += (a['tf'] - last_tf) * LEAK_PPM * 1e-6
            if adj < base: base = adj
        last_tf = a['tf']; last_pts = a['pts']
        a['air'] = adj - base


def model_compare(A, casc):
    """Drone air-clock model (aulog-3 `air_ms`: the modelled backlog at the
    AU's arrival on the drone, spec 2026-09-06) vs the GS-measured excess.
    Compared as air + q: the drone books at TxQueue push, the GS floor is
    post-pop, so the queue wait sits between the two. Through-origin slope
    measured/model over frames where either side is >= 5 ms (the leaky
    floor makes both ~0 elsewhere), residual percentiles, per-cascade peak
    pairs, and the model's p99 on frames the GS saw as quiet (a phantom
    backlog would show here). None when the log predates aulog 3."""
    rows = [a for a in A if a.get('air_ms') is not None]
    if not rows: return None
    pairs = [(a['air_ms'], a['air'] + a['q']) for a in rows]
    big = [(m, x) for m, x in pairs if m >= 5 or x >= 5]
    smm = sum(m * m for m, _ in big)
    slope = sum(m * x for m, x in big) / smm if smm else float('nan')
    resid = [x - m for m, x in big]
    per_casc = []
    for c in casc:
        w = [a for a in rows if c['t0'] <= a['tf'] <= c['t0'] + 3000]
        if w:
            per_casc.append((c['t0'], max(a['air_ms'] for a in w),
                             max(a['air'] + a['q'] for a in w)))
    quiet = [m for m, x in pairs if x < 5]
    return {'n': len(rows), 'n_big': len(big), 'slope': slope,
            'resid_p50': pct(resid, .5), 'resid_p90': pct(resid, .9),
            'cascades': per_casc, 'quiet_model_p99': pct(quiet, .99)}


def analyze(ctl_path, au_path, t0=None, t1=None, spike_ms=15.0):
    """Everything the report prints, as a dict (tests assert on this)."""
    log = fr.load_ctllog(ctl_path)
    E = log['E']; S = log['S']
    rungs = fr._parse_ladder_token(log['header'].get('ladder', ''))
    if not E: raise SystemExit('no E lines: nothing transitioned')
    t0 = t0 * 1000 if t0 is not None else E[0]['t_ms']
    # To the LAST S line: a starve is link loss, not the end of the flight,
    # and find_episodes' starved episodes are dropped below. (Ending at the
    # first starved E line read flight 0031 as 72 s of 566, 2026-09-06.)
    t1 = t1 * 1000 if t1 is not None else S[-1]['t_ms']
    Ew = [e for e in E if t0 <= e['t_ms'] <= t1]

    A = load_au(au_path)
    air_excess(A)
    W = [a for a in A if t0 <= a['tf'] <= t1]
    if not W: raise SystemExit('no AU rows inside the window -- wrong au log for this ctl log?')

    def rung_of(t):
        r = 0
        for e in E:
            if e['t_ms'] <= t: r = e['to']
            else: break
        return r

    def mbps_for(rung):
        mcs = rungs[rung]['mcs'] if rung < len(rungs) else rung
        return NOMINAL_MBPS.get(mcs, float('nan'))

    def onair_kb(rung, frames):
        ovb = rungs[rung]['ov_base'] if rung < len(rungs) else 1.0
        ove = rungs[rung]['ov_enh'] if rung < len(rungs) else 0.5
        return sum(a['len'] * (1 + (ovb if a['sid'] == 0 else ove)) for a in frames) / 1000

    def post_peak(tt):
        post = [a['air'] for a in A if tt <= a['tf'] <= tt + 3000]
        return max(post) if post else 0.0

    def pre_median(tt):
        prev = [a['air'] for a in A if tt - 1000 <= a['tf'] < tt]
        return st.median(prev) if prev else 0.0

    # A starve is link loss, not a rung decision: it opens no episode here.
    eps = [p for p in fr.find_episodes(Ew) if p['first_reason'] != 'starved']
    casc = []
    for p in (x for x in eps if x['steps'] >= 2):
        tt = p['t0']; post = [a for a in A if tt <= a['tf'] <= tt + 3000]
        pk = max((a['air'] for a in post), default=0.0)
        tp = next((a['tf'] - tt for a in post if a['air'] == pk), 0.0)
        after = [a for a in post if a['tf'] >= tt + tp]
        d = next((after[i]['tf'] - tt for i in range(max(0, len(after) - 5))
                  if all(x['air'] < SETTLE_MS for x in after[i:i + 5])), 3000.0)
        first = [a for a in A if tt <= a['tf'] < tt + 500]
        new = p['path'][1]; cap_kb = mbps_for(new) * 1e6 / 8 / 1000 * 0.5
        bins = []
        for k in range(-4, 12):
            w = [a['air'] for a in A if tt + k * 250 <= a['tf'] < tt + (k + 1) * 250]
            idr = [round(a['len'] / 1000) for a in A
                   if tt + k * 250 <= a['tf'] < tt + (k + 1) * 250 and a['idr']]
            bins.append((st.median(w) if w else float('nan'), idr))
        casc.append({'t0': tt, 'from': p['path'][0], 'to': new, 'steps': p['steps'],
                     'first_reason': p['first_reason'], 'peak_ms': pk, 'time_to_peak_ms': tp,
                     'settle_ms': d, 'pre_ms': pre_median(tt),
                     'onair_pct': 100 * onair_kb(new, first) / cap_kb if cap_kb else float('nan'),
                     'bins': bins})

    singles = [{'t0': p['t0'], 'peak_ms': post_peak(p['t0']), 'pre_ms': pre_median(p['t0'])}
               for p in eps if p['steps'] == 1]
    promotes = [{'t0': e['t_ms'], 'peak_ms': post_peak(e['t_ms']), 'pre_ms': pre_median(e['t_ms'])}
                for e in Ew if e['to'] > e['from']]

    ev = sorted(e['t_ms'] for e in E)
    steady = collections.defaultdict(list)
    for a in W:
        i = bisect.bisect_left(ev, a['tf'])
        near = min([abs(ev[j] - a['tf']) for j in (i - 1, i) if 0 <= j < len(ev)] or [1e9])
        if near >= STEADY_GUARD_MS: steady[rung_of(a['tf'])].append(a['air'])

    secs = collections.defaultdict(list)
    for a in W: secs[int(a['tf'] // 1000)].append(a['air'])
    spike_secs = sum(1 for v in secs.values() if st.median(v) >= spike_ms)
    standalone = [(s, st.median(v), max(v)) for s, v in sorted(secs.items())
                  if st.median(v) >= spike_ms and min(abs(t - s * 1000 - 500) for t in ev) >= STEADY_GUARD_MS]

    return {'t0': t0, 't1': t1, 'n_au': len(W), 'excess': [a['air'] for a in W],
            'cascades': casc, 'singles': singles, 'promotes': promotes,
            'steady': dict(steady), 'n_secs': len(secs), 'spike_secs': spike_secs,
            'standalone': standalone, 'model': model_compare(W, casc)}


def print_report(r, ctl_path, au_path, profiles=False, spike_ms=15.0, model_flag=False):
    ex = r['excess']
    print(f"airdrain: {os.path.basename(ctl_path)} + {os.path.basename(au_path)}, "
          f"window {r['t0']/1000:.0f}-{r['t1']/1000:.0f} s, {r['n_au']} AUs")
    print(f"  per-frame air excess: p50={pct(ex,.5):.1f} p90={pct(ex,.9):.1f} p99={pct(ex,.99):.1f} max={max(ex):.0f} ms")
    casc = r['cascades']
    print(f"\n  cascades (>=2 steps): {len(casc)}  -- 250 ms bins of median excess, -1.0..+3.0 s around the first demote; I<kB>=IDR in bin")
    if profiles:
        for c in casc:
            bins = ' '.join(f"{b:.0f}" + (f"I{'/'.join(map(str, i))}" if i else '') for b, i in c['bins'])
            print(f"   t={c['t0']/1000:6.1f}s {c['from']}->{c['to']} steps={c['steps']} {c['first_reason']:<11} "
                  f"peak={c['peak_ms']:4.0f}ms @+{c['time_to_peak_ms']:4.0f} settle=+{c['settle_ms']:4.0f} "
                  f"pre1s={c['pre_ms']:3.0f} onair0.5s={c['onair_pct']:3.0f}% | {bins}")
    if casc:
        g = lambda k: [c[k] for c in casc]
        print(f"  cascade summary: peak p50={pct(g('peak_ms'),.5):.0f} max={max(g('peak_ms')):.0f} ms | "
              f"time-to-peak p50={pct(g('time_to_peak_ms'),.5):.0f} ms | settle p50={pct(g('settle_ms'),.5):.0f} ms | "
              f"pre-demote 1 s p50={pct(g('pre_ms'),.5):.0f} ms (>=20: {sum(1 for x in g('pre_ms') if x >= 20)}) | "
              f"first-500 ms on-air vs new rung nominal p50={pct(g('onair_pct'),.5):.0f}% max={max(g('onair_pct')):.0f}%")
    for name, sel in (('single demote', r['singles']), ('promote', r['promotes'])):
        if sel:
            pk = [x['peak_ms'] for x in sel]; pr = [x['pre_ms'] for x in sel]
            print(f"  {name:<14} n={len(sel)}: post peak p50={pct(pk,.5):.0f} p90={pct(pk,.9):.0f} max={max(pk):.0f} ms | pre-1 s p50={pct(pr,.5):.0f}")
    print("  steady state by rung (frames >=3 s from any transition), p50/p99 ms (n): " +
          ' '.join(f"r{k}={pct(v,.5):.0f}/{pct(v,.99):.0f}({len(v)})" for k, v in sorted(r['steady'].items())))
    print(f"  spike seconds (median excess >= {spike_ms:.0f} ms): {r['spike_secs']} of {r['n_secs']}; "
          f"standalone (no transition within 3 s): {len(r['standalone'])}" +
          (': ' + ' '.join(f"{s}s({m:.0f}/{mx:.0f})" for s, m, mx in r['standalone'][:12]) if r['standalone'] else ''))
    m = r.get('model')
    if m:
        print(f"\n  air-clock model (aulog-3 air_ms) vs measured air+q: n={m['n']} ({m['n_big']} >= 5 ms) | "
              f"slope measured/model={m['slope']:.2f} (1.00 = calibrated; scale air_clock.efficiency by 1/slope) | "
              f"residual p50={m['resid_p50']:.1f} p90={m['resid_p90']:.1f} ms | "
              f"quiet-frame model p99={m['quiet_model_p99']:.0f} ms")
        for t0, pm, px in m['cascades']:
            print(f"   t={t0/1000:6.1f}s peak model={pm:4.0f} measured={px:4.0f} ms")
    elif model_flag:
        print("\n  --model: no air_ms column (aulog < 3) -- nothing to compare")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('ctl', nargs='?'); ap.add_argument('aulog', nargs='?')
    ap.add_argument('--from', dest='t0', type=float); ap.add_argument('--to', dest='t1', type=float)
    ap.add_argument('--spike', type=float, default=15.0)
    ap.add_argument('--profiles', action='store_true')
    ap.add_argument('--model', action='store_true')
    args = ap.parse_args()
    if args.ctl is None or os.path.isdir(args.ctl):
        import session as _session   # tools/ is already on sys.path here
        s = _session.resolve(args.ctl)
        if s.ctl is None or s.au is None:
            sys.exit('session needs both ctl.log and au.log')
        args.ctl, args.aulog = s.ctl, s.au
    r = analyze(args.ctl, args.aulog, args.t0, args.t1, args.spike)
    print_report(r, args.ctl, args.aulog, args.profiles, args.spike, args.model)


if __name__ == '__main__':
    main()
