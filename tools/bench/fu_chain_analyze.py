# FU-chain integrity over a seqdump, walked in RTP-seq order on the deduped
# set — the authoritative version of rtpsniff's FU accounting (immune to
# loopback dup interleaving and sniffer socket drops). Distinguishes chains
# broken AT a missing seq (transport loss) from chains truncated with FULLY
# CONTINUOUS seqs (the packets themselves lack the chain end — source-side
# truncation or in-transport payload/header corruption).
#   python3 fu_chain_analyze.py <dumpfile>
import sys
from collections import Counter

rows = {}
last_seq = None
base = 0
with open(sys.argv[1]) as f:
    for line in f:
        p = list(map(int, line.split()))
        us, seq, ts, mkr, nal, fu, plen = p[:7]
        fu_rt = p[7] if len(p) > 7 else -1
        if last_seq is not None:
            d = (seq - last_seq) & 0xFFFF
            base += d if d <= 0x7FFF else d - 0x10000
        else:
            base = seq
        last_seq = seq
        rows.setdefault(base, (us, ts, mkr, nal, fu, plen, fu_rt))

seqs = sorted(rows)
lo, hi = seqs[0], seqs[-1]

chains = 0
ok_chains = 0
trunc_contig = 0    # new S while chain open, NO seq missing in between
trunc_at_gap = 0    # chain broken across a missing seq
orphan_frag = 0     # non-start FU with no chain open
trunc_by_rt = Counter()
trunc_times = []
open_rt = None      # inner NAL type of open chain
open_last_seq = None
gap_since_open = False

prev = None
for e in seqs:
    us, ts, mkr, nal, fu, plen, fu_rt = rows[e]
    gap_before = prev is not None and e != prev + 1
    prev = e
    if nal != 49:
        continue
    if gap_before and open_rt is not None:
        gap_since_open = True
    start, endb = fu & 1, fu & 2
    if start:
        chains += 1
        if open_rt is not None:
            if gap_since_open: trunc_at_gap += 1
            else:
                trunc_contig += 1
                trunc_by_rt[open_rt] += 1
                if len(trunc_times) < 2000: trunc_times.append(us / 1e6)
        open_rt = fu_rt
        gap_since_open = False
    elif open_rt is None:
        orphan_frag += 1
        continue
    if endb:
        if open_rt is not None: ok_chains += 1
        open_rt = None

print(f"unique={len(seqs)} span={hi-lo+1} missing={hi-lo+1-len(seqs)}")
print(f"chains={chains} ok={ok_chains} trunc_contig={trunc_contig} "
      f"trunc_at_gap={trunc_at_gap} orphan={orphan_frag}")
print("trunc_contig by inner NAL type:", dict(trunc_by_rt.most_common()))
if trunc_times:
    iv = [round(b - a, 3) for a, b in zip(trunc_times, trunc_times[1:])]
    c = Counter(round(x, 1) for x in iv)
    print("inter-truncation interval histogram (s, top12):",
          dict(c.most_common(12)))
    print("first 20 truncation times:", [round(t, 2) for t in trunc_times[:20]])
