#!/usr/bin/env python3
"""Is the AU ring published in encode order?

The decoder is fed strictly in ring order. If frame_id ever moves backwards
(or skips) relative to the record sequence, the decoder sees pictures out of
encode order / with holes, and will report missing references even though
every AU arrived intact. Reports inversions and holes with the sid pair
involved.
"""
import argparse
import collections
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ausniff  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--ring", required=True)
ap.add_argument("--seconds", type=float, default=20.0)
a = ap.parse_args()

f, mm, slot_bytes, slot_count, epoch = ausniff.open_ring(a.ring)

recs = {}
t0 = time.time()
while time.time() - t0 < a.seconds:
    for s in range(slot_count):
        base = ausniff.HDR + s * (ausniff.SLOT_HDR + slot_bytes)
        m = ausniff.read_slot(mm, base, slot_bytes)
        if m and m["rec"] not in recs:
            recs[m["rec"]] = (m["fid"], m["sid"], m["flags"])
    time.sleep(0.002)

order = sorted(recs.items())
# Only analyse a contiguous run of record numbers: a Python reader can miss
# records, and a miss is not an inversion.
runs, cur = [], []
prev_rec = None
for rec, v in order:
    if prev_rec is not None and rec != prev_rec + 1:
        if len(cur) > 1:
            runs.append(cur)
        cur = []
    cur.append((rec, v))
    prev_rec = rec
if len(cur) > 1:
    runs.append(cur)

covered = sum(len(r) for r in runs)
inversions = []
holes = collections.Counter()
steps = collections.Counter()
for run in runs:
    for (r0, (f0, s0, _)), (r1, (f1, s1, _)) in zip(run, run[1:]):
        d = f1 - f0
        steps[d] += 1
        if d <= 0:
            inversions.append((r0, s0, f0, r1, s1, f1))
        elif d > 1:
            holes[(s0, s1, d)] += 1

print(f"records seen={len(recs)} in {len(runs)} contiguous runs "
      f"covering {covered} adjacent-pair-able records")
print(f"frame_id step histogram (delta -> count): {dict(sorted(steps.items()))}")
print(f"inversions (frame_id not increasing in ring order): {len(inversions)}")
for x in inversions[:15]:
    print(f"   rec {x[0]} sid{x[1]} fid {x[2]}  ->  rec {x[3]} sid{x[4]} fid {x[5]}")
print(f"forward holes (delta>1): {sum(holes.values())}")
for (s0, s1, d), c in sorted(holes.items(), key=lambda kv: -kv[1])[:10]:
    print(f"   sid{s0} -> sid{s1} skip {d}: {c}x")
