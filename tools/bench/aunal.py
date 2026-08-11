#!/usr/bin/env python3
"""Census of NAL types / temporal ids / frame_id continuity at the AU ring.

Answers, from the bitstream rather than from a counter:
  - does the delivered stream contain ANY IRAP (IDR/CRA) past session start?
  - what does the periodic sid==0 AU actually carry?
  - is frame_id contiguous per sid (i.e. did anything vanish before the ring)?
Reuses ausniff's seqlock reader.
"""
import argparse
import collections
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ausniff  # noqa: E402

NAL_NAMES = {
    0: "TRAIL_N", 1: "TRAIL_R", 2: "TSA_N", 3: "TSA_R", 4: "STSA_N",
    5: "STSA_R", 6: "RADL_N", 7: "RADL_R", 8: "RASL_N", 9: "RASL_R",
    16: "BLA_W_LP", 17: "BLA_W_RADL", 18: "BLA_N_LP", 19: "IDR_W_RADL",
    20: "IDR_N_LP", 21: "CRA", 32: "VPS", 33: "SPS", 34: "PPS",
    35: "AUD", 36: "EOS", 37: "EOB", 39: "PREFIX_SEI", 40: "SUFFIX_SEI",
}


def nals(buf):
    """Yield (nal_type, temporal_id) over an Annex-B access unit."""
    i, n = 0, len(buf)
    while i < n - 4:
        if buf[i] == 0 and buf[i + 1] == 0 and buf[i + 2] == 1:
            h = i + 3
        elif (buf[i] == 0 and buf[i + 1] == 0 and buf[i + 2] == 0
              and buf[i + 3] == 1):
            h = i + 4
        else:
            i += 1
            continue
        if h + 1 < n:
            yield ((buf[h] >> 1) & 0x3F, (buf[h + 1] & 0x07) - 1)
        i = h + 2
    return


ap = argparse.ArgumentParser()
ap.add_argument("--ring", required=True)
ap.add_argument("--seconds", type=float, default=20.0)
a = ap.parse_args()

f, mm, slot_bytes, slot_count, epoch = ausniff.open_ring(a.ring)

by_sid = collections.defaultdict(collections.Counter)
tid_by_sid = collections.defaultdict(collections.Counter)
flags_by_sid = collections.defaultdict(collections.Counter)
aus = collections.Counter()
last_fid = {}
fid_gaps = collections.Counter()
irap_seen = []
seen = set()

t0 = time.time()
while time.time() - t0 < a.seconds:
    for s in range(slot_count):
        base = ausniff.HDR + s * (ausniff.SLOT_HDR + slot_bytes)
        m = ausniff.read_slot(mm, base, slot_bytes)
        if not m or m["rec"] in seen:
            continue
        seen.add(m["rec"])
        sid = m["sid"]
        aus[sid] += 1
        types = list(nals(m["payload"]))
        for t, tid in types:
            by_sid[sid][NAL_NAMES.get(t, str(t))] += 1
            if t <= 21:
                tid_by_sid[sid][tid] += 1
            if 16 <= t <= 21:
                irap_seen.append((m["rec"], sid, NAL_NAMES.get(t, t)))
        if m["flags"] & ausniff.FLAG_IDR:
            flags_by_sid[sid]["IDR"] += 1
        if m["flags"] & ausniff.FLAG_DISCONT:
            flags_by_sid[sid]["DISCONT"] += 1
        if not m["flags"] & ausniff.FLAG_COMPLETE:
            flags_by_sid[sid]["INCOMPLETE"] += 1
        if sid in last_fid and m["fid"] != last_fid[sid] + 1:
            fid_gaps[sid] += 1
        last_fid[sid] = m["fid"]
    time.sleep(0.002)

print(f"window={a.seconds}s  AUs per sid: {dict(sorted(aus.items()))}")
print()
for sid in sorted(aus):
    print(f"sid {sid}: {aus[sid]} AUs")
    print(f"   nal types : {dict(by_sid[sid])}")
    print(f"   temporal_id: {dict(tid_by_sid[sid])}")
    print(f"   ring flags : {dict(flags_by_sid[sid]) or 'none'}")
    print(f"   frame_id discontinuities (per-sid, incl. interleave): "
          f"{fid_gaps[sid]}")
print()
print(f"IRAP NALs seen in window: {len(irap_seen)}")
for r in irap_seen[:10]:
    print("  ", r)
