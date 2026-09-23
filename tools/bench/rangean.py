#!/usr/bin/env python3
"""Analyse linkbench-rx --range files (linkbench-tx --range-sweep; method and
results in docs/bw40-sweep-findings-2026-09-23.md "Range test").

Per (file = one GS card at one tuning, bw, mcs): delivery vs drone TX power
index (rel to the TXAGC anchor), averaged over completed cycles (each file's
first flushed cycle is dropped: the receiver started mid-cycle). Scanning UP
from the bottom of the ramp, since the PA wall sits at the top:
  floor90 = first rel whose next 4 steps (itself included) are all >= 90 %
  p50     = first upward 50 % crossing, linearly interpolated (steadier)
Lower = reaches further; 4 rel steps ~ 1 dB. Usage:
  rangean.py FRAMES_PER_CELL card0.range[:label] card1.range[:label] ...
"""
import collections
import sys

frames = int(sys.argv[1])
files = sys.argv[2:]


def load(path):
    cyc = collections.defaultdict(dict)   # cycle -> {(bw,mcs,rel): (rx,r0,r1,s0,s1)}
    done = set()
    for line in open(path):
        p = line.split()
        if p[0] == "C":
            c, bw, mcs, rel, rx = map(int, p[1:6])
            r0, r1, s0, s1 = map(float, p[6:10])
            cyc[c][(bw, mcs, rel)] = (rx, r0, r1, s0, s1)
        elif p[0] == "F":
            done.add(int(p[1]))
    return cyc, done


def analyse(path):
    cyc, done = load(path)
    # Only cycles with data; a cell absent from a completed cycle = 0 rx.
    # The first flushed cycle is partial (the receiver started mid-cycle).
    cycles = sorted(c for c in cyc if c in done)[1:]
    rels = sorted({k[2] for c in cycles for k in cyc[c]})
    rows = sorted({(k[0], k[1]) for c in cycles for k in cyc[c]})
    out = {}
    for bw, mcs in rows:
        d, rssi = {}, {}
        for rel in rels:
            tot = sum(cyc[c].get((bw, mcs, rel), (0,))[0] for c in cycles)
            d[rel] = 100.0 * tot / (frames * len(cycles)) if cycles else 0
            r0 = sum(cyc[c].get((bw, mcs, rel), (0, 0))[1] for c in cycles)
            rssi[rel] = r0 / tot if tot else None
        # Scan UP from the bottom of the ramp (the PA wall sits at the top):
        # floor90 = first rel whose next 4 steps (itself included) are all
        # >= 90 %; p50 = first upward 50 % crossing, interpolated.
        asc = sorted(rels)
        floor90 = None
        for i in range(len(asc)):
            win = asc[i:i + 4]
            if len(win) == 4 and all(d[r] >= 90 for r in win):
                floor90 = asc[i]
                break
        p50 = None
        for lo, hi in zip(asc, asc[1:]):
            if d[lo] < 50 <= d[hi]:
                p50 = lo + (hi - lo) * (50 - d[lo]) / (d[hi] - d[lo])
                break
        out[(bw, mcs)] = (floor90, p50, d, rssi, min(rels) if rels else None)
    return out, len(cycles), rels


results = []
for spec in files:
    path, _, label = spec.partition(":")
    res, n, rels = analyse(path)
    results.append((label or path, res, n, rels))
    print(f"{label or path}: {n} completed cycles, rel {min(rels) if rels else '-'}..{max(rels) if rels else '-'}")

print("\nfloor90 / p50 (TX rel index; lower = reaches further; '<lo' = still >=90 % at the bottom of the ramp)")
hdr = "row      " + "".join(f"{lab:>22}" for lab, *_ in results)
print(hdr)
keys = sorted({k for _, res, *_ in results for k in res})
for k in keys:
    s = f"HT{k[0]} m{k[1]}"
    s = f"{s:9}"
    for _, res, _, rels in results:
        if k not in res:
            s += f"{'-':>22}"
            continue
        f90, p50, d, _, lo = res[k]
        f = "none" if f90 is None else (f"<{lo}" if f90 == lo else str(f90))
        q = "-" if p50 is None else f"{p50:.1f}"
        s += f"{f + ' / ' + q:>22}"
    print(s)
