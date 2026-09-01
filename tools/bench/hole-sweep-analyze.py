#!/usr/bin/env python3
"""Match RX json activity bursts to sweep manifest cells (launch order),
emit the mcs x sym mac_lost%% table. A burst = consecutive seconds with
frames>0, split on >=3 zero seconds."""
import csv, json, sys

jsonl, manifest = sys.argv[1], sys.argv[2]

secs = []
for line in open(jsonl):
    try:
        j = json.loads(line)
    except Exception:
        continue
    secs.append(j)

bursts, cur, zeros = [], [], 0
for j in secs:
    if j["frames"] > 0:
        if zeros >= 3 and cur:
            bursts.append(cur)
            cur = []
        cur.append(j)
        zeros = 0
    else:
        zeros += 1
if cur:
    bursts.append(cur)

cells = list(csv.DictReader(open(manifest)))
ok_cells = [c for c in cells if c["exit"] != "FAIL"]
print(f"# bursts={len(bursts)} manifest_ok={len(ok_cells)} (must match)")

rows = {}
for c, b in zip(ok_cells, bursts):
    frames = sum(j["frames"] for j in b)
    crc = sum(j["crc_bad"] for j in b)
    lost = sum(j["mac_lost"] for j in b)
    okf = frames - crc
    pct = 100.0 * lost / (okf + lost) if okf + lost else 0.0
    rows[(int(c["mcs"]), int(c["sym"]))] = (pct, frames, crc, lost,
                                            int(c["tx_frames"] or 0))

syms = sorted({s for _, s in rows})
print("mac_lost%  " + " ".join(f"s{s}" for s in syms))
for m in range(8):
    line = [f"mcs{m}     "]
    for s in syms:
        r = rows.get((m, s))
        line.append(f"{r[0]:5.2f}" if r else "  -  ")
    print(" ".join(line))
print("\n# cells with mac_lost% > 1% (candidate holes):")
for (m, s), (pct, fr, crc, lost, txf) in sorted(rows.items()):
    if pct > 1.0:
        print(f"  mcs{m} sym{s} air={4*s+103}B: {pct:.2f}% "
              f"(rx {fr} crc_bad {crc} lost {lost} / tx {txf})")
print("\n# rx-vs-tx frame coverage sanity (rx+lost)/tx, worst 5:")
cov = sorted(((fr - crc + lost) / txf if txf else 0, m, s)
             for (m, s), (pct, fr, crc, lost, txf) in rows.items())
for cvg, m, s in cov[:5]:
    print(f"  mcs{m} sym{s}: {cvg:.3f}")
