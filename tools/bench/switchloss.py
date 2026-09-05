#!/usr/bin/env python3
"""switchloss.py -- where does the loss burst at a rung change come from?

Places every per-card seq gap the GS saw around each ladder transition on
three timelines: the drone's actual rate switch (from the probe log: the
probe body's own MCS follows the applied op), the encoder's IDR after the
op change (from flightrec's au log), and the GS's own control-frame sends
(the self-blanking mechanism). Each gap is classified by the RX MCS and
stream id of the frames bracketing it:
  pre       both neighbours at the OLD op's rate for their stream -- the
            lost frames were old-rate frames, sent before the drone switched
  post      both neighbours at the NEW op's rate -- loss at the new rate
  straddle  old-rate before, new-rate after -- lost frames sit exactly on
            the switch (in-flight tail kill or the first new-rate frames)
  ?         an unknown MCS (255) or a non-video stream on either side
Inputs, one maburgs run on the GS started with MABUR_GAPLOG=1 and the
switchloss gaplog build (gap lines carry prev_mcs/mcs/prev_sid):
  gaplog   stderr of that run ('gap ...', 'gstx ...' lines)
  ctl      /media/dvr/ctl-NNNN_<date>.log of the same run (E lines)
  probe    /media/dvr/probe-NNNN_<date>.log of the same run
  aulog    /media/dvr/log/au-NNNN.log overlapping it (flightrec)
  [pulser] stdout of /root/s3pulser.py (on_mono/off_mono stamps) -- gaps
           inside an injection pulse are marked 'inj' and excluded
Usage: python3 tools/bench/switchloss.py gaplog.txt ctl.log probe.log au.log [pulser.txt]
All clocks are the GS CLOCK_MONOTONIC (ctl/probe ms, gaplog/au us).
"""
import re, sys, bisect, collections

if len(sys.argv) < 5:
    sys.exit(__doc__)
gap_path, ctl_path, probe_path, au_path = sys.argv[1:5]
pulser_path = sys.argv[5] if len(sys.argv) > 5 else None

PRE_MS, POST_MS = 200.0, 600.0       # window around each E line
STEADY_EXCL_MS = 3000.0              # steady-state loss = gaps this far from any E
SEND_BEFORE_US, SEND_AFTER_US = 2000, 500  # send inside [gap_start-2ms, gap_end+0.5ms]

# ---- gaplog ---------------------------------------------------------------
gaps, sends = [], []
gap_re = re.compile(r'^gap card=(\d+) seq=(\d+)\.\.(\d+) n=(\d+) dtsf=(\d+) dhost=(\d+) mono=(\d+) '
                    r'prev_len=(\d+) len=(\d+) sid=(-?\d+) prev_agg=(\d+) after_physt=(\d+)'
                    r'(?: prev_mcs=(\d+) mcs=(\d+))?(?: prev_sid=(-?\d+))?')
for line in open(gap_path, errors='replace'):
    m = gap_re.match(line)
    if m:
        g = dict(card=int(m[1]), seq0=int(m[2]), seq1=int(m[3]), n=int(m[4]), dtsf=int(m[5]),
                 dhost=int(m[6]), mono=int(m[7]), sid=int(m[10]), prev_agg=int(m[11]),
                 after_physt=int(m[12]), prev_mcs=int(m[13]) if m[13] else 255,
                 mcs=int(m[14]) if m[14] else 255, prev_sid=int(m[15]) if m[15] else -1)
        gaps.append(g)
        continue
    m = re.match(r'^gstx card=(\d+) mono=(\d+) reason=(\d+)', line)
    if m:
        sends.append(int(m[2]))
sends.sort()
gaps.sort(key=lambda g: g['mono'])
if not gaps:
    sys.exit('no gap lines in %s (MABUR_GAPLOG=1 run of the switchloss build?)' % gap_path)
has_mcs = any(g['mcs'] != 255 for g in gaps)

# ---- ctl log: ladder + E lines ---------------------------------------------
ladder, events, max_mcs = [], [], None
for line in open(ctl_path):
    p = line.split()
    if not p: continue
    if p[0] == 'ctllog':
        for tok in p[1:]:
            if tok.startswith('ladder='):
                ladder = [int(r.split('/')[0]) for r in tok[7:].split(',')]
    elif p[0] == 'E':
        events.append(dict(t=float(p[1]), frm=int(p[2]), to=int(p[3]), reason=p[4],
                           u=float(p[5]), snr=p[6]))
if not ladder:
    sys.exit('no ladder= header in %s' % ctl_path)
max_mcs = max(ladder)

def rates(rung):
    """PHY MCS per stream at a rung: same-rate fixed pairs (2026-08-30) put
    base and enh on the op MCS; the probe canary runs one above it."""
    m = ladder[rung]
    return {0: m, 1: m, 5: m + 1 if m + 1 <= max_mcs else None}

# ---- probe log: the drone's applied op, as heard ---------------------------
probe_rows = []
for line in open(probe_path):
    p = line.split()
    if not p or p[0] == 'probelog': continue
    try:
        probe_rows.append((float(p[0]), int(p[2]), float(p[10]) if len(p) >= 11 else float(p[0])))
    except ValueError:
        continue
probe_rows.sort(key=lambda r: r[2])
probe_first = [r[2] for r in probe_rows]

def drone_switch_ms(ev):
    """Estimate when the drone applied the op, from the probe stream's own MCS.
    Returns (t_ms, how) or (None, why)."""
    pn = rates(ev['to'])[5]
    po = rates(ev['frm'])[5]
    i = bisect.bisect_left(probe_first, ev['t'] - 50.0)
    if pn is not None:
        for r in probe_rows[i:]:
            if r[2] > ev['t'] + 2000: break
            if r[1] == pn:
                return r[2], 'first probe@mcs%d first_ms' % pn
        return None, 'no probe@mcs%d within 2 s' % pn
    # promote to the top rung: the probe stops; switch = after the last old-rate probe
    last = None
    for r in probe_rows[i:]:
        if r[2] > ev['t'] + 2000: break
        if r[1] == po: last = r[2]
    if last is not None:
        return last + 17.0, 'last probe@mcs%d + 1 AU' % po
    return None, 'no probe rows after E'

# ---- au log: IDRs ------------------------------------------------------------
idrs = []   # (t_first_us, t_complete_us, sid, fid, len)
version = 1
for line in open(au_path):
    p = line.split()
    if not p: continue
    if p[0] == '#':
        if len(p) >= 3 and p[1] == 'aulog': version = int(p[2])
        continue
    if version < 2 or len(p) < 11: continue
    try:
        if int(p[6]) == 32:
            idrs.append((int(p[7]), int(p[8]), int(p[2]), int(p[3]), int(p[4])))
    except ValueError:
        continue
idrs.sort()
idr_first = [x[0] for x in idrs]

# ---- pulser stamps -----------------------------------------------------------
pulses = []
if pulser_path:
    for line in open(pulser_path):
        m = re.search(r'on_mono=(\d+) off_mono=(\d+)', line)
        if m: pulses.append((int(m[1]), int(m[2])))

def in_pulse(mono_us):
    ms = mono_us / 1000.0
    return any(on - 20 <= ms <= off + 80 for on, off in pulses)

# ---- helpers -----------------------------------------------------------------
def classify(g, ev):
    if not has_mcs or g['mcs'] == 255 or g['prev_mcs'] == 255: return '?'
    old, new = rates(ev['frm']), rates(ev['to'])
    def side(sid, mcs):
        if sid not in (0, 1, 5): return '?'
        o, n = old.get(sid), new.get(sid)
        if mcs == n and mcs != o: return 'post'
        if mcs == o and mcs != n: return 'pre'
        if mcs == o == n: return 'same'   # rate shared by old and new op for this stream
        return '?'
    a, b = side(g['prev_sid'], g['prev_mcs']), side(g['sid'], g['mcs'])
    if 'same' in (a, b):
        a2 = b if a == 'same' else a
        b2 = a if b == 'same' else b
        if a2 == 'same': return 'same'
        a, b = a2, b2
    if a == b: return a
    if a == 'pre' and b == 'post': return 'straddle'
    if a == 'post' and b == 'pre': return 'reorder'
    return '?'

def send_near(g):
    lo = g['mono'] - g['dhost'] - SEND_BEFORE_US
    hi = g['mono'] + SEND_AFTER_US
    i = bisect.bisect_left(sends, lo)
    return i < len(sends) and sends[i] <= hi

def idr_at(g):
    """IDR AU whose air burst covers the gap: gap inside [t_first-5ms, t_complete+1ms]."""
    i = bisect.bisect_right(idr_first, g['mono'] + 5000) - 1
    while i >= 0 and idrs[i][0] >= g['mono'] - 400000:
        f, c, sid, fid, ln = idrs[i]
        if f - 5000 <= g['mono'] <= c + 1000:
            return idrs[i]
        i -= 1
    return None

# ---- per-transition report -------------------------------------------------
gap_mono = [g['mono'] for g in gaps]
tot = collections.Counter()
tot_by_kind = collections.defaultdict(collections.Counter)
n_ev = 0
switch_lat = []
print('transitions: %d   gaps: %d   sends: %d   IDRs: %d   pulses: %d   mcs in gap lines: %s'
      % (len(events), len(gaps), len(sends), len(idrs), len(pulses), has_mcs))
print()
for ev in events:
    if ev['reason'] in ('starved',): continue
    n_ev += 1
    sw, how = drone_switch_ms(ev)
    if sw is not None: switch_lat.append(sw - ev['t'])
    kind = 'promote' if ev['to'] > ev['frm'] else 'demote'
    lo = int((ev['t'] - PRE_MS) * 1000); hi = int((ev['t'] + POST_MS) * 1000)
    i0 = bisect.bisect_left(gap_mono, lo); i1 = bisect.bisect_right(gap_mono, hi)
    # first IDR after the E line
    j = bisect.bisect_left(idr_first, int(ev['t'] * 1000) - 50000)
    idr_txt = '-'
    if j < len(idrs) and idrs[j][0] <= hi:
        f, c, sid, fid, ln = idrs[j]
        idr_txt = 'IDR sid%d %dkB air %+.0f..%+.0f ms' % (sid, ln // 1024, f / 1000 - ev['t'], c / 1000 - ev['t'])
    sw_txt = ('drone switch %+.0f ms (%s)' % (sw - ev['t'], how)) if sw else ('drone switch ? (%s)' % how)
    print('E t=%.0f %d->%d %-15s u=%.3f snr=%s | %s | %s' % (ev['t'], ev['frm'], ev['to'], ev['reason'], ev['u'], ev['snr'], sw_txt, idr_txt))
    for g in gaps[i0:i1]:
        cls = classify(g, ev)
        inj = in_pulse(g['mono'])
        flags = []
        if inj: flags.append('inj')
        if send_near(g): flags.append('SEND')
        ia = idr_at(g)
        if ia: flags.append('IDR')
        if g['after_physt'] and g['prev_agg'] >= 0 and g['n'] >= 5: flags.append('whole-agg')
        dt = g['mono'] / 1000.0 - ev['t']
        rel_sw = ('%+.0f' % (g['mono'] / 1000.0 - sw)) if sw else '?'
        print('   %+6.0f ms  card%d n=%-2d dtsf=%-5d sid %s@mcs%s -> %s@mcs%s  %-8s vs-switch %s  %s'
              % (dt, g['card'], g['n'], g['dtsf'], g['prev_sid'], g['prev_mcs'], g['sid'], g['mcs'],
                 cls, rel_sw, ' '.join(flags)))
        if not inj and dt >= -PRE_MS:
            key = cls if dt >= 0 else 'before-E'
            tot[key] += g['n']
            tot_by_kind[kind][key] += g['n']
            if 'SEND' in flags: tot['with-send'] += g['n']; tot_by_kind[kind]['with-send'] += g['n']
            if 'IDR' in flags: tot['in-IDR'] += g['n']; tot_by_kind[kind]['in-IDR'] += g['n']
    print()

# ---- steady-state comparison -------------------------------------------------
ev_t = sorted(e['t'] for e in events)
steady_n = steady_send = steady_idr = 0
steady_span_ms = 0.0
first, last = gaps[0]['mono'] / 1000.0, gaps[-1]['mono'] / 1000.0
# steady span = total span minus +-3 s around each event (approx.)
steady_span_ms = (last - first) - len(ev_t) * 2 * STEADY_EXCL_MS
for g in gaps:
    ms = g['mono'] / 1000.0
    k = bisect.bisect_left(ev_t, ms)
    near = (k < len(ev_t) and ev_t[k] - ms < STEADY_EXCL_MS) or (k > 0 and ms - ev_t[k - 1] < STEADY_EXCL_MS)
    if near or in_pulse(g['mono']): continue
    steady_n += g['n']
    if send_near(g): steady_send += g['n']
    if idr_at(g): steady_idr += g['n']

print('==== totals (lost frames per card-gap, injection pulses excluded) ====')
for kind in ('promote', 'demote'):
    c = tot_by_kind[kind]
    print('  %-8s pre=%d straddle=%d post=%d same=%d reorder=%d ?=%d before-E=%d | with-send=%d in-IDR=%d'
          % (kind, c['pre'], c['straddle'], c['post'], c['same'], c['reorder'], c['?'], c['before-E'],
             c['with-send'], c['in-IDR']))
print('  all      pre=%d straddle=%d post=%d same=%d reorder=%d ?=%d before-E=%d | with-send=%d in-IDR=%d'
      % (tot['pre'], tot['straddle'], tot['post'], tot['same'], tot['reorder'], tot['?'], tot['before-E'],
         tot['with-send'], tot['in-IDR']))
per_ev = (tot['pre'] + tot['straddle'] + tot['post'] + tot['same'] + tot['?']) / max(n_ev, 1)
print('  lost frames per transition (0..+%.0f ms): %.1f' % (POST_MS, per_ev))
if switch_lat:
    sl = sorted(switch_lat)
    print('  drone switch after E (probe-log estimate, n=%d): p50=%.0f p90=%.0f max=%.0f ms'
          % (len(sl), sl[len(sl) // 2], sl[int(len(sl) * 0.9)], sl[-1]))
if steady_span_ms > 0:
    rate = steady_n / (steady_span_ms / 1000.0)
    print('  steady state (>%.0f s from any E): %d lost frames over %.0f s = %.2f /s  (expected in a %.0f ms window: %.2f) | with-send=%d in-IDR=%d'
          % (STEADY_EXCL_MS / 1000, steady_n, steady_span_ms / 1000, rate, POST_MS, rate * POST_MS / 1000, steady_send, steady_idr))
