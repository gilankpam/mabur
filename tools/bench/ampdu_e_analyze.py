#!/usr/bin/env python3
"""Verdict per cell for the 8822E A-MPDU spike (tools/bench/ampdu_e_spike.sh).

Two views per cell:
  - the ampdu_spike.sh-style line (unique fps / paggr / equal-tsfl bursts) —
    note the equal-tsfl burst marker does NOT fire on an 8812EU RX (tsfl is
    stamped per-MPDU there, not latched per-PPDU);
  - the inter-frame tsfl-delta histogram, which is the discriminator that
    works on this RX: subframes inside an aggregate arrive at pure MPDU
    airtime spacing (~216 us at MCS5/1396 B), singles pay preamble + DIFS +
    backoff on top (~309 us mgmt / ~429 us BE).

First run + interpretation: docs/dq-spike-findings-2026-08-31.md §11.
"""
import glob
import json
import os
import sys
from collections import Counter

logs = sys.argv[1] if len(sys.argv) > 1 else "./ampdu_e_logs"
payload = int(os.environ.get("PAYLOAD", "1396"))
secs = float(os.environ.get("SECS", "8"))
want_len = {payload, payload + 4}  # RX len may include the 4-byte FCS

order = ["control", "qsel0", "ampdu_rty0", "ampdu_rty0_urb",
         "mode", "mode_thr4", "ampdu_mgmt"]
paths = {os.path.basename(p)[3:-6]: p
         for p in glob.glob(os.path.join(logs, "rx_*.jsonl"))}

for cell in order:
    path = paths.get(cell)
    if not path:
        print(f"{cell:>16}: (no log)")
        continue
    frames, uniq, retry = [], set(), 0
    for line in open(path, errors="replace"):
        if not line.startswith('{"ev":"rx.frame"'):
            continue
        try:
            ev = json.loads(line)
        except json.JSONDecodeError:
            continue
        if ev.get("len") in want_len and not ev.get("crc"):
            frames.append(ev)
            if ev.get("fc1", 0) & 0x08:
                retry += 1
            body = ev.get("body", "")
            if len(body) >= 12:
                uniq.add(body[4:12])  # per-frame counter, MPDU bytes 26..29
    n = len(frames)
    if n == 0:
        print(f"{cell:>16}: NO FRAMES (queue wedged / delivery broken?)")
        continue
    paggr = sum(1 for f in frames if f.get("paggr"))

    deltas = [(b.get("tsfl", 0) - a.get("tsfl", 0)) & 0xFFFFFFFF
              for a, b in zip(frames, frames[1:])]
    deltas = [d for d in deltas if d < 5000]  # drop inter-lull gaps
    buckets = Counter(min(d // 50 * 50, 800) for d in deltas)
    nd = len(deltas)
    med = sorted(deltas)[nd // 2] if nd else 0
    hist = ", ".join(f"{k}-{k + 49}us:{100 * v // nd}%"
                     for k, v in sorted(buckets.items()) if v > nd * 0.02)

    print(f"{cell:>16}: rx={n} unique={len(uniq)} "
          f"({len(uniq) / secs:.0f} uniq fps)  paggr={100 * paggr // n}% "
          f"retry_flagged={100 * retry // max(n, 1)}%")
    print(f"{'':>16}  median_delta={med}us  {hist}")
