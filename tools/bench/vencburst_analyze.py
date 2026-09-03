#!/usr/bin/env python3
"""Scene-change overshoot from a PASSIVE tools/bench/vencprobe capture
(`vencprobe --cycles 0 --duration MS`), for the bench plan in
docs/handover-venc-overshoot-2026-09-03.md.

The flight data (1 Hz telemetry) showed the SigmaStar CBR encoder running
10-40 % over its commanded bitrate for the first second of motion after a
quiet scene, and filling the drone TxQueue with it — but 1 s is too coarse
to say whether that was a 100 ms spike or a 1 s trickle. This reads every
frame the encoder published and reports, per quiet→motion cycle:

  quiet    trailing 1 s rate just before the stimulus (how far under the
           command the RC was sitting), and the encoder qp there
  peak     max 100 ms rate after the onset, as a multiple of the PROGRAMMED
           rate — kbps x 1024, the unit star6e_controls.c apply_bitrate()
           hands the SDK, so an exact CBR reads 1.000 here (and 1.024
           against RcAgent's decimal command)
  settle   first 100 ms window after the peak back inside +10 % of programmed
  excess   integral of (rate - programmed)+ over the ramp, in bytes — what
           the TxQueue has to hold, to compare with one IDR (~64 KB) and
           the queue cap
  idrs     IDR frames inside the ramp: an IDR in the window confounds the
           cycle (a rung change or a chain-break, not the scene)

Window rates divide the bytes by the span the frames actually took (first
frame of the window back to the frame before it), not by the nominal
window, so a 60 fps stream does not read 6 vs 7 frames per 100 ms
depending on alignment.

Cycle detection: a cycle starts (onset) when the 100 ms rate crosses the
midpoint between the quiet rate and the programmed rate after >= quiet_s
of the trailing-1 s rate sitting under quiet_frac x programmed. Cover the
lens / point at a wall for >= 5 s, then uncover / pan hard, >= 10 times.

  vencburst_analyze.py vb.csv [--cmd-kbps N] [--quiet-s S] [--window-ms W]

Acceptance for a shipped change (handover doc): peak <= 1.2x programmed
AND excess <= 64 KB on every uncover cycle, plus the standing aucadence /
ausniff gates.
"""
import argparse
import bisect
import sys
from dataclasses import dataclass
from statistics import median
from typing import List, Optional

FLAG_IDR = 0x01
FLAG_ENH = 0x04
ACCEPT_PEAK_RATIO = 1.2
ACCEPT_EXCESS_BYTES = 64 * 1024


@dataclass
class Params:
    window_ms: int = 100      # the burst window the TxQueue sees
    step_ms: int = 25         # rate-series grid
    quiet_s: float = 2.0      # trailing-1 s rate must be quiet this long before an onset
    quiet_frac: float = 0.7   # quiet := trailing-1 s rate < quiet_frac x programmed
    settle_frac: float = 1.10 # settle := 100 ms rate <= settle_frac x programmed
    ramp_max_s: float = 3.0   # horizon for peak/settle/excess after the onset
    cmd_kbps: Optional[int] = None  # override the programmed rate source (polls)


@dataclass
class Cycle:
    onset_us: int
    programmed_kbps: float
    quiet_kbps: float
    peak_kbps: float
    peak_at_ms: float
    settle_ms: Optional[float]
    excess_bytes: float
    qp_quiet: Optional[int]
    qp_peak: Optional[int]
    qp_settle: Optional[int]
    idrs: int

    @property
    def peak_ratio(self):
        return self.peak_kbps / self.programmed_kbps if self.programmed_kbps else float("nan")


def load(fh):
    """vencprobe CSV -> (frames, polls, cmds). Polls carry qp=None when the
    capture predates the 3rd column (2026-09-03)."""
    frames, polls, cmds = [], [], []
    for line in fh:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        p = line.split(',')
        if p[0] == 'f':
            frames.append(dict(t=int(p[1]), idx=int(p[2]), len=int(p[3]),
                               pts=int(p[4]), flags=int(p[5]), enc=int(p[6])))
        elif p[0] == 's':
            polls.append(dict(t=int(p[1]), kbps=int(p[2]),
                              qp=int(p[3]) if len(p) > 3 and p[3] not in ('', '-1') else None))
        elif p[0] == 'c':
            cmds.append(dict(tb=int(p[1]), ta=int(p[2]), kbps=int(p[3]), ok=int(p[4])))
    frames.sort(key=lambda f: f['t'])
    polls.sort(key=lambda s: s['t'])
    return frames, polls, cmds


def programmed_kbps(polls, cmd_kbps=None):
    """Decimal kbit/s the encoder is actually programmed with: RcAgent's
    kbps x 1024 (star6e_controls.c apply_bitrate). Median of the polls'
    req_bitrate_kbps unless overridden."""
    if cmd_kbps is not None:
        return cmd_kbps * 1.024
    vals = [s['kbps'] for s in polls if s['kbps'] > 0]
    if not vals:
        raise SystemExit("no req_bitrate polls and no --cmd-kbps: cannot scale the rates")
    return median(vals) * 1.024


class RateSeries:
    """Sliding-window byte rate over the frames, sampled on a step grid.
    rate(g, W) = bytes of frames with t in (g-W, g], over the span from the
    frame preceding the window to the last frame in it (kbit/s)."""

    def __init__(self, frames):
        self.t = [f['t'] for f in frames]
        self.cum = [0]
        for f in frames:
            self.cum.append(self.cum[-1] + f['len'])

    def rate_kbps(self, g_us, window_us):
        hi = bisect.bisect_right(self.t, g_us)          # frames with t <= g
        lo = bisect.bisect_right(self.t, g_us - window_us)  # first frame with t > g-W
        if hi - lo < 1 or lo == 0:
            return 0.0
        nbytes = self.cum[hi] - self.cum[lo]
        span_us = self.t[hi - 1] - self.t[lo - 1]
        if span_us <= 0:
            return 0.0
        return nbytes * 8.0 / (span_us / 1000.0)  # bits per ms == kbit/s


def _poll_at(polls, t_us, key):
    """Value of `key` from the last poll at or before t_us (None if none/absent)."""
    ts = [s['t'] for s in polls]
    i = bisect.bisect_right(ts, t_us) - 1
    if i < 0:
        return None
    return polls[i][key]


def find_cycles(frames, polls, prm: Params) -> List[Cycle]:
    if len(frames) < 10:
        return []
    rs = RateSeries(frames)
    step = prm.step_ms * 1000
    win = prm.window_ms * 1000
    t0, t1 = frames[0]['t'] + 1_000_000, frames[-1]['t']
    prog_default = programmed_kbps(polls, prm.cmd_kbps) if (polls or prm.cmd_kbps) else None

    def prog_at(g):
        if prm.cmd_kbps is not None or not polls:
            return prog_default
        k = _poll_at(polls, g, 'kbps')
        return k * 1.024 if k and k > 0 else prog_default

    cycles = []
    quiet_since = None
    g = t0
    while g <= t1:
        prog = prog_at(g)
        if not prog:
            g += step
            continue
        r1 = rs.rate_kbps(g, 1_000_000)
        r100 = rs.rate_kbps(g, win)
        if quiet_since is not None and (g - quiet_since) >= prm.quiet_s * 1e6:
            # Quiet rate one window BEFORE the candidate onset: the onset
            # grid point already sits a frame or two into the burst, and a
            # 45 kB burst frame inside a 1 s average of 10 kB frames would
            # inflate "quiet" by ~10 %.
            quiet_rate = rs.rate_kbps(g - win, 1_000_000)
            if r100 > quiet_rate + 0.5 * (prog - quiet_rate):
                cycles.append(_measure(rs, polls, frames, g, quiet_rate, prog, prm))
                g += int(prm.ramp_max_s * 1e6)
                quiet_since = None
                continue
        if r1 < prm.quiet_frac * prog:
            if quiet_since is None:
                quiet_since = g
        else:
            quiet_since = None
        g += step
    return cycles


def _measure(rs, polls, frames, onset, quiet_rate, prog, prm) -> Cycle:
    step = prm.step_ms * 1000
    win = prm.window_ms * 1000
    horizon = onset + int(prm.ramp_max_s * 1e6)
    peak, peak_at = 0.0, onset
    g = onset
    while g <= horizon:
        r = rs.rate_kbps(g, win)
        if r > peak:
            peak, peak_at = r, g
        g += step
    settle = None
    g = peak_at
    while g <= horizon:
        if rs.rate_kbps(g, win) <= prm.settle_frac * prog:
            settle = g
            break
        g += step
    end = settle if settle is not None else horizon
    excess_bits = 0.0
    g = onset
    while g <= end:
        excess_bits += max(0.0, rs.rate_kbps(g, win) - prog) * prm.step_ms
        g += step
    ts = [f['t'] for f in frames]
    lo, hi = bisect.bisect_left(ts, onset - win), bisect.bisect_right(ts, end)
    idrs = sum(1 for f in frames[lo:hi] if f['flags'] & FLAG_IDR)
    return Cycle(
        onset_us=onset, programmed_kbps=prog, quiet_kbps=quiet_rate,
        peak_kbps=peak, peak_at_ms=(peak_at - onset) / 1000.0,
        settle_ms=(settle - onset) / 1000.0 if settle is not None else None,
        excess_bytes=excess_bits / 8.0,
        qp_quiet=_poll_at(polls, onset - 500_000, 'qp') if polls else None,
        qp_peak=_poll_at(polls, peak_at, 'qp') if polls else None,
        qp_settle=_poll_at(polls, settle, 'qp') if (polls and settle is not None) else None,
        idrs=idrs)


def report(cycles, polls, prm: Params):
    if polls or prm.cmd_kbps is not None:
        prog = programmed_kbps(polls, prm.cmd_kbps)
        print(f"programmed {prog/1000:.3f} Mb/s (= req {prog/1.024/1000:.1f} Mb/s x 1.024)")
    print(f"{len(cycles)} cycle(s)  [window {prm.window_ms} ms, quiet >= {prm.quiet_s:.0f} s "
          f"under {prm.quiet_frac:.2f}x, settle <= {prm.settle_frac:.2f}x]\n")
    if not cycles:
        print("no quiet->motion cycle found: hold the scene static (>= 5 s, well under the "
              "command) before each stimulus, or lower --quiet-s")
        return
    for i, c in enumerate(cycles):
        qp = lambda v: f"{v:2d}" if v is not None else "--"
        settle = f"{c.settle_ms:5.0f} ms" if c.settle_ms is not None else "  never"
        ok = c.peak_ratio <= ACCEPT_PEAK_RATIO and c.excess_bytes <= ACCEPT_EXCESS_BYTES
        print(f"cycle {i}: t+{(c.onset_us - cycles[0].onset_us)/1e6:6.1f} s  "
              f"quiet {c.quiet_kbps/1000:5.2f} Mb/s ({c.quiet_kbps/c.programmed_kbps:.2f}x, qp {qp(c.qp_quiet)})  "
              f"peak {c.peak_kbps/1000:5.2f} Mb/s = {c.peak_ratio:.2f}x @+{c.peak_at_ms:.0f} ms (qp {qp(c.qp_peak)})  "
              f"settle {settle} (qp {qp(c.qp_settle)})  "
              f"excess {c.excess_bytes/1024:6.1f} KB  idr {c.idrs}  {'ok' if ok else 'FAIL'}")
    ratios = [c.peak_ratio for c in cycles]
    excess = [c.excess_bytes / 1024 for c in cycles]
    settles = [c.settle_ms for c in cycles if c.settle_ms is not None]
    clean = [c for c in cycles if c.idrs == 0]
    print(f"\nSUMMARY  peak ratio median {median(ratios):.2f}x  max {max(ratios):.2f}x   "
          f"excess median {median(excess):.0f} KB  max {max(excess):.0f} KB   "
          + (f"settle median {median(settles):.0f} ms  max {max(settles):.0f} ms" if settles
             else "settle: never within horizon")
          + f"   ({len(clean)}/{len(cycles)} cycles IDR-free)")
    fails = [i for i, c in enumerate(cycles)
             if c.peak_ratio > ACCEPT_PEAK_RATIO or c.excess_bytes > ACCEPT_EXCESS_BYTES]
    print(f"ACCEPTANCE (peak <= {ACCEPT_PEAK_RATIO}x AND excess <= {ACCEPT_EXCESS_BYTES//1024} KB): "
          + ("PASS" if not fails else f"FAIL on cycle(s) {fails}"))
    # The hypothesis' prediction 1: burst scales with the pre-burst deficit.
    if len(cycles) >= 4:
        deficits = [c.programmed_kbps - c.quiet_kbps for c in cycles]
        pairs = sorted(zip(deficits, ratios))
        lo = median(r for _, r in pairs[:len(pairs) // 2])
        hi = median(r for _, r in pairs[len(pairs) // 2:])
        print(f"deficit split: smaller-deficit half peak median {lo:.2f}x, larger-deficit half {hi:.2f}x"
              + ("  (consistent with the QP-floor hypothesis)" if hi > lo + 0.05 else
                 "  (NO deficit dependence — look elsewhere than the QP floor)"))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('csv')
    ap.add_argument('--cmd-kbps', type=int, default=None,
                    help='RcAgent command (decimal kbps) if the capture has no /venc polls')
    ap.add_argument('--quiet-s', type=float, default=Params.quiet_s)
    ap.add_argument('--window-ms', type=int, default=Params.window_ms)
    ap.add_argument('--quiet-frac', type=float, default=Params.quiet_frac)
    a = ap.parse_args(argv)
    prm = Params(window_ms=a.window_ms, quiet_s=a.quiet_s, quiet_frac=a.quiet_frac,
                 cmd_kbps=a.cmd_kbps)
    with open(a.csv) as fh:
        frames, polls, cmds = load(fh)
    if cmds:
        print(f"note: {len(cmds)} bitrate commands in this capture — this is an ACTIVE "
              f"vencprobe run; vencprobe_analyze.py is the tool for step response")
    print(f"{len(frames)} frames, {len(polls)} polls")
    report(find_cycles(frames, polls, prm), polls, prm)


if __name__ == '__main__':
    main()
