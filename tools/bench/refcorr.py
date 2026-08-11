#!/usr/bin/env python3
"""Correlate decoder missing-reference events with what arrived at the ring.

Tails maburplay's log (mpp's "missing ref poc N" lines carry no timestamp,
so we stamp them at arrival) while reading the AU ring, then reports what
was published in the moments before each decoder complaint. If the
parameter-set refresh on sid 0 is what breaks the decoder's reference
state, every event lands right after a sid-0 AU.
"""
import argparse
import collections
import os
import re
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ausniff  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--ring", required=True)
ap.add_argument("--log", default="/tmp/maburplay.log")
ap.add_argument("--seconds", type=float, default=120.0)
a = ap.parse_args()

events = []       # (t, poc)
stop = threading.Event()
RE = re.compile(r"missing ref poc (\d+)")


def tail():
    p = subprocess.Popen(["tail", "-n", "0", "-F", a.log],
                         stdout=subprocess.PIPE, text=True)
    for line in p.stdout:
        if stop.is_set():
            break
        m = RE.search(line)
        if m:
            events.append((time.monotonic(), int(m.group(1))))
    p.kill()


threading.Thread(target=tail, daemon=True).start()

f, mm, slot_bytes, slot_count, epoch = ausniff.open_ring(a.ring)
arrivals = []     # (t, rec, sid, fid, flags)
seen = set()
t0 = time.monotonic()
while time.monotonic() - t0 < a.seconds:
    for s in range(slot_count):
        base = ausniff.HDR + s * (ausniff.SLOT_HDR + slot_bytes)
        m = ausniff.read_slot(mm, base, slot_bytes)
        if m and m["rec"] not in seen:
            seen.add(m["rec"])
            arrivals.append((time.monotonic(), m["rec"], m["sid"], m["fid"],
                             m["flags"]))
    time.sleep(0.001)
stop.set()
time.sleep(0.3)

arrivals.sort()
sid_counts = collections.Counter(x[2] for x in arrivals)
dur = arrivals[-1][0] - arrivals[0][0] if len(arrivals) > 1 else 1
print(f"window {dur:.1f}s: AUs {dict(sorted(sid_counts.items()))} "
      f"({len(arrivals)/dur:.1f} AU/s)")
print(f"sid0 rate {sid_counts[0]/dur:.2f}/s   "
      f"missing-ref events {len(events)} ({len(events)/dur:.2f}/s)")
print()

# For each decoder complaint, what were the last few AUs published before it?
prev_sid_hist = collections.Counter()
lag_to_sid0 = []
for t, poc in events:
    before = [x for x in arrivals if x[0] <= t]
    if not before:
        continue
    prev_sid_hist[tuple(x[2] for x in before[-3:])] += 1
    last0 = [x for x in before if x[2] == 0]
    if last0:
        lag_to_sid0.append(t - last0[-1][0])

print("sid of the last 3 AUs published before each missing-ref event:")
for k, v in prev_sid_hist.most_common(8):
    print(f"   {k}: {v}x")
print()
if lag_to_sid0:
    lag_to_sid0.sort()
    n = len(lag_to_sid0)
    print(f"time from the most recent sid0 AU to the complaint: "
          f"min={lag_to_sid0[0]*1000:.0f}ms p50={lag_to_sid0[n//2]*1000:.0f}ms "
          f"max={lag_to_sid0[-1]*1000:.0f}ms")
    near = sum(1 for x in lag_to_sid0 if x < 0.25)
    print(f"   complaints within 250 ms of a sid0 AU: {near}/{n}")
    print("   (sid0 arrives every ~2 s; if the refresh were unrelated, only "
          "~12% would land in any 250 ms window)")
