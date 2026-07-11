#!/usr/bin/env python3
"""Live PHY meter: rx.frame JSONL on stdin -> per-interval MCS / RSSI / SNR.

Usage (8812EU proto-GS receiver; expect ~41 s of silence during bring-up):
  tail -f /dev/null | \
  DEVOURER_PID=0xa81a DEVOURER_CHANNEL=149 DEVOURER_STREAM_OUT=1 \
  DEVOURER_EVENTS=stdout ./build/duplex 2>/dev/null | \
  python3 tools/bench/phy_meter.py

Rate index legend (Realtek): 0-11 = legacy CCK/OFDM, 12+ = HT MCS(n-12),
so 12=MCS0, 13=MCS1, ... RSSI is the chip's raw PWDB (dBm ~= raw - 110;
not comparable across chip models). SNR is per-chain, dB.
"""
import collections
import json
import sys
import time

INTERVAL_S = 5.0


def rate_name(r):
    if r is None:
        return "?"
    return f"MCS{r - 12}" if r >= 12 else f"legacy{r}"


frames = []
t_last = time.time()
for line in sys.stdin:
    if '"rx.frame"' not in line:
        continue
    try:
        e = json.loads(line)
    except json.JSONDecodeError:
        continue
    frames.append(e)
    now = time.time()
    if now - t_last < INTERVAL_S:
        continue
    rates = collections.Counter(rate_name(f.get("rate")) for f in frames)
    r0 = [f["rssi"][0] for f in frames]
    r1 = [f["rssi"][1] for f in frames]
    s0 = [f["snr"][0] for f in frames]
    s1 = [f["snr"][1] for f in frames]
    crc_bad = sum(1 for f in frames if f.get("crc"))
    n = len(frames)
    print(
        f"[{time.strftime('%H:%M:%S')}] {n / (now - t_last):6.0f} fps  "
        f"rate={dict(rates.most_common(3))}  "
        f"rssi=[{sum(r0)/n:.1f},{sum(r1)/n:.1f}] (dBm~[{sum(r0)/n - 110:.0f},{sum(r1)/n - 110:.0f}])  "
        f"snr=[{sum(s0)/n:.1f},{sum(s1)/n:.1f}] dB  crc_bad={crc_bad}",
        flush=True,
    )
    frames.clear()
    t_last = now
