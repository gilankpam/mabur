# Offline analyzer for seqdump.py output. Set-membership gap detection:
# a seq is MISSING only if neither loopback copy (lo TX + lo RX) ever
# appeared anywhere in the capture — immune to duplicate interleaving and
# transient reordering that confuse sequential-delta accounting.
# For each missing run it prints the delivery-time position, the NAL/FU/ts
# context on both sides, and whether the run sits at a frame boundary and/or
# near a keyframe — the layer-classification evidence the handoff asks for.
#   python3 seqdump_analyze.py <dumpfile>
import sys
from collections import Counter

KEYFRAME_NALS = {19, 20, 21, 32, 33, 34}  # IDR_W_RADL, IDR_N_LP, CRA, VPS, SPS, PPS

rows = []  # (us, ext_seq, ts, marker, nal, fu, paylen[, fu_rt])
last_seq = None
base = 0
with open(sys.argv[1]) as f:
    for line in f:
        parts = list(map(int, line.split()))
        us, seq, ts, mkr, nal, fu, plen = parts[:7]
        fu_rt = parts[7] if len(parts) > 7 else -1
        if last_seq is not None:
            d = (seq - last_seq) & 0xFFFF
            if d <= 0x7FFF:
                base += d
            else:
                base -= 0x10000 - d
        else:
            base = seq
        last_seq = seq
        rows.append((us, base, ts, mkr, nal, fu, plen, fu_rt))

if not rows:
    print("empty dump"); sys.exit(1)

count = Counter(r[1] for r in rows)
lo, hi = min(count), max(count)
total_expected = hi - lo + 1
copies = Counter(count.values())
missing = sorted(e for e in range(lo, hi + 1) if e not in count)
dur_s = (rows[-1][0] - rows[0][0]) / 1e6

print(f"lines={len(rows)} unique_seqs={len(count)} span={total_expected} dur={dur_s:.1f}s")
print(f"copies histogram (2=both lo halves seen): {dict(sorted(copies.items()))}")
print(f"MISSING={len(missing)} ({len(missing)*100/max(1,total_expected):.2f}%)")

# group missing into runs
runs = []
for e in missing:
    if runs and e == runs[-1][1] + 1:
        runs[-1][1] = e
    else:
        runs.append([e, e])

# index: first occurrence per ext seq for context lookup
first = {}
for i, r in enumerate(rows):
    if r[1] not in first:
        first[r[1]] = i

by_seq = sorted(first)
import bisect
def ctx(e, side, k=4):
    # k delivered seqs strictly below (side=-1) / above (side=+1) e
    i = bisect.bisect_left(by_seq, e)
    sel = by_seq[max(0, i - k):i] if side < 0 else by_seq[i:i + k]
    return [rows[first[s]] for s in sel]

CRIT = set(range(16, 24)) | {32, 33, 34}  # classify_rtp critical -> stream 0
def layer_of(nal, fu_rt):
    t = fu_rt if nal == 49 and fu_rt >= 0 else nal
    return "s0" if t in CRIT or nal == 48 else "s1+"

def fmt(r):
    us, e, ts, mkr, nal, fu, plen, fu_rt = r
    fus = {0: "", 1: "S", 2: "E", 3: "SE"}[fu]
    rt = f"/rt{fu_rt}" if fu_rt >= 0 else ""
    return (f"seq={e & 0xFFFF} nal={nal}{fus}{rt} {layer_of(nal, fu_rt)} "
            f"m={mkr} ts={ts} len={plen} t={us/1e6:.3f}")

gap_times = []
print(f"\nruns={len(runs)}  (len histogram: {dict(Counter(b - a + 1 for a, b in runs))})")
for a, b in runs[:40]:
    before = ctx(a, -1); after = ctx(b, +1)
    t_est = before[-1][0] / 1e6 if before else 0.0
    gap_times.append(t_est)
    frame_boundary = bool(before and after and before[-1][2] != after[0][2])
    kf_near = any(r[4] in KEYFRAME_NALS for r in before + after)
    print(f"\n[{a & 0xFFFF}..{b & 0xFFFF}] len={b - a + 1} t~{t_est:.3f}s "
          f"frame_boundary={frame_boundary} keyframe_near={kf_near}")
    for r in before: print("   < " + fmt(r))
    for r in after:  print("   > " + fmt(r))
if len(runs) > 40:
    print(f"... {len(runs) - 40} more runs")

if len(gap_times) > 2:
    iv = [round(b - a, 2) for a, b in zip(gap_times, gap_times[1:])]
    print("\ninter-gap intervals (s):", iv[:60])

# Reordering: a delivered packet is LATE if a higher seq was already
# delivered before its first copy. PixelPilot's depacketizer treats
# sufficiently-late packets like loss, so lateness matters even at 0 missing.
max_seen = None
max_seen_us = 0
late = []  # (ext, seq_dist, ms_after_overtake)
firsts_in_arrival = []
seen = set()
for i in range(len(rows)):
    e = rows[i][1]
    if e in seen: continue
    seen.add(e)
    firsts_in_arrival.append(rows[i])
for us, e, ts, mkr, nal, fu, plen, fu_rt in firsts_in_arrival:
    if max_seen is None or e > max_seen:
        max_seen = e; max_seen_us = us
    else:
        late.append((e, max_seen - e, (us - max_seen_us) / 1e3))
print(f"\nLATE (arrived after a higher seq): {len(late)} "
      f"({len(late)*100/max(1,len(seen)):.2f}%)")
if late:
    dist = Counter(min(d, 50) for _, d, _ in late)
    print("  seq-distance histogram (50=50+):", dict(sorted(dist.items())))
    ms = sorted(m for _, _, m in late)
    print(f"  ms-late p50={ms[len(ms)//2]:.1f} p90={ms[int(len(ms)*0.9)]:.1f} "
          f"p99={ms[int(len(ms)*0.99)]:.1f} max={ms[-1]:.1f}")
    worst = sorted(late, key=lambda x: -x[2])[:10]
    print("  worst:", [(e & 0xFFFF, d, round(m, 1)) for e, d, m in worst])
