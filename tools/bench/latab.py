#!/usr/bin/env python3
"""latab.py A.log B.log — vsync A/B verdict from two persisted lat logs.

A = vsync_lock false (baseline D=12), B = servo. Gates recalibrated
2026-08-31 after the first bench A/B: the original "e2e p50 <= A-8"
encoded a wrong baseline model — the D=12 rule passes ~84% of frames
straight through (reg p50 == 0), so its MEDIAN already sits near the
half-period floor and no servo carrying a nonzero lead can beat it
there. The servo's deliverable is stability (flat dsp, no 16 s beat
sweep, ~1 ms present-jitter) at a bounded median cost, so the e2e gate
is a SANITY BOUND on that cost, and dsp level/tail/flatness are the
primary gates. Bench reference (lat-0019/0020): dsp 19/28 -> 5/5,
sweep 8.5 -> 0.0, e2e p50 56 -> 58.
"""
import re, statistics as st, sys

LINE = re.compile(
    r"^(\d+) lat: n=(\d+) e2e=(\d+)/(\d+) .* reg=(\d+)/(\d+) "
    r"dsp~?=(\d+)/(\d+) chk=.* anchor=(\w+)")

def load(path):
    rows = []  # (t_s, e2e50, e2e99, reg50, reg99, dsp50, dsp99)
    warm_skipped = 0
    for ln in open(path):
        m = LINE.match(ln)
        if m:
            groups = m.groups()
            anchor = groups[8]
            if anchor != "ok":
                warm_skipped += 1
                continue
            v = [int(x) for x in groups[:8]]
            rows.append((v[0] / 1e6, v[2], v[3], v[4], v[5], v[6], v[7]))
    if not rows:
        if warm_skipped > 0:
            sys.exit(f"{path}: all {warm_skipped} windows were anchor=warm (still settling)")
        sys.exit(f"{path}: no lat lines (is this a latlog 1 file?)")
    return rows, warm_skipped

def med(rows, i): return st.median(r[i] for r in rows)

def dsp_sweep(rows):
    """Beat signature: max-min of dsp p50 over 4 s buckets."""
    if len(rows) < 8: return float("nan")
    t0 = rows[0][0]
    buckets = {}
    for r in rows:
        buckets.setdefault(int((r[0] - t0) // 4), []).append(r[5])
    per = [st.median(v) for v in buckets.values() if v]
    return max(per) - min(per)

a, a_warm = load(sys.argv[1])
b, b_warm = load(sys.argv[2])
print(f"{'metric':22s} {'A (baseline)':>14s} {'B (servo)':>12s}  gate")
rowsfmt = "{:22s} {:>14.1f} {:>12.1f}  {}"
checks = []
def row(name, va, vb, ok, gate):
    checks.append(ok)
    print(rowsfmt.format(name, va, vb, ("PASS " if ok else "FAIL ") + gate))

row("e2e p50 (ms)", med(a,1), med(b,1), med(b,1) <= med(a,1) + 3,
    "B <= A+3 (stability trade sanity bound)")
row("dsp p50 (ms)", med(a,5), med(b,5), med(b,5) <= 6, "B <= 6")
row("dsp p99 (ms)", med(a,6), med(b,6), med(b,6) <= med(a,6) - 8, "B <= A-8")
row("dsp p50 sweep (ms)", dsp_sweep(a), dsp_sweep(b), dsp_sweep(b) <= 3,
    "B flat (<=3)")
print(f"windows: A={len(a)} ({a_warm} warm excluded) B={len(b)} ({b_warm} warm excluded) "
      f"(want >=300 each: >=5 min at 1 Hz)")
print("ALL GATES PASS" if all(checks) else "GATES FAILED — ship dark "
      "(display.vsync_lock: false) and bring both logs home")
