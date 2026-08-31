#!/usr/bin/env python3
"""latab.py A.log B.log — vsync A/B verdict from two persisted lat logs.

A = vsync_lock false (baseline D=12), B = servo. Gates from
docs/superpowers/specs/2026-08-31-vsync-locked-regulator-design.md.
"""
import re, statistics as st, sys

LINE = re.compile(
    r"^(\d+) lat: n=(\d+) e2e=(\d+)/(\d+) .* reg=(\d+)/(\d+) "
    r"dsp~?=(\d+)/(\d+) chk=")

def load(path):
    rows = []  # (t_s, e2e50, e2e99, reg50, reg99, dsp50, dsp99)
    for ln in open(path):
        m = LINE.match(ln)
        if m:
            v = [int(x) for x in m.groups()]
            rows.append((v[0] / 1e6, v[2], v[3], v[4], v[5], v[6], v[7]))
    if not rows:
        sys.exit(f"{path}: no lat lines (is this a latlog 1 file?)")
    return rows

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

a, b = load(sys.argv[1]), load(sys.argv[2])
print(f"{'metric':22s} {'A (baseline)':>14s} {'B (servo)':>12s}  gate")
rowsfmt = "{:22s} {:>14.1f} {:>12.1f}  {}"
checks = []
def row(name, va, vb, ok, gate):
    checks.append(ok)
    print(rowsfmt.format(name, va, vb, ("PASS " if ok else "FAIL ") + gate))

row("e2e p50 (ms)", med(a,1), med(b,1), med(b,1) <= med(a,1) - 8, "B <= A-8")
row("dsp p50 (ms)", med(a,5), med(b,5), med(b,5) <= 6, "B <= 6")
row("dsp p99 (ms)", med(a,6), med(b,6), med(b,6) <= med(a,6) - 8, "B <= A-8")
row("dsp p50 sweep (ms)", dsp_sweep(a), dsp_sweep(b), dsp_sweep(b) <= 3,
    "B flat (<=3)")
print(f"windows: A={len(a)} B={len(b)} "
      f"(want >=300 each: >=5 min at 1 Hz)")
print("ALL GATES PASS" if all(checks) else "GATES FAILED — ship dark "
      "(display.vsync_lock: false) and bring both logs home")
