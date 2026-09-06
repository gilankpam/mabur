#!/usr/bin/env python3
"""Live E2E decode: rxdemo STREAM_OUT JSONL -> SBI -> RS-FEC -> FRAG -> RTP/H.265.

Reports per-stream envelope counts, post-FEC loss (FRAG seq gaps per stream),
recovered RTP packets, NAL-type histogram, and payload bitrate.
"""
import json, struct, sys

import os
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.abspath(os.path.join(ROOT, "..", "devourer", "tools", "precoder")))
sys.path.insert(0, os.path.join(ROOT, "tools", "pyref"))
import sbi, stream_fec  # noqa: E402

K, SYMBOL = 8, 64
ENV_SIZE = sbi.SBI_HDR_LEN + SYMBOL
FRAG_HDR = struct.Struct("<HBB")

path = sys.argv[1]
frames = []
t_first = t_last = None
for line in open(path):
    if '"rx.frame"' not in line:
        continue
    e = json.loads(line)
    frames.append(bytes.fromhex(e["body"]))
    t = e.get("tsfl")

decs = {s: stream_fec.make_decoder(stream_fec.FecConfig(
    k=K, symbol_size=SYMBOL, overhead=1.0, scheme="rs")) for s in range(4)}

per_stream_env = {s: 0 for s in range(4)}
frag_seqs = {s: set() for s in range(4)}
reasm, reasm_n = {}, {}
recovered = []  # (sid, frag_seq, rtp_packet)
n_sbi = n_other = 0

for body in frames:
    sid = sbi.peek_stream_id(body)
    if sid is None or sid < 0 or sid > 3:
        n_other += 1
        continue
    n_sbi += 1
    res = sbi.unpack(body, ENV_SIZE)
    per_stream_env[sid] += len(res["survivors"])
    for env in res["survivors"]:
        for pkt in decs[sid].add_symbol(env):
            if len(pkt) < FRAG_HDR.size:
                continue
            seq, idx, n = FRAG_HDR.unpack_from(pkt)
            frag_seqs[sid].add(seq)
            key = (sid, seq)
            reasm.setdefault(key, {})[idx] = pkt[FRAG_HDR.size:]
            reasm_n[key] = n
            if len(reasm[key]) == n:
                recovered.append((sid, seq, b"".join(reasm[key][i] for i in range(n))))
                del reasm[key]

print(f"frames: {len(frames)} total, {n_sbi} SBI data, {n_other} other (RC/DISC)")
print(f"envelopes per stream: {per_stream_env}")

# post-FEC loss: gaps in the per-stream FRAG seq space (contiguous counter),
# interior only (drop first/last 2 values to ignore capture-boundary partials)
for s in range(4):
    seqs = sorted(frag_seqs[s])
    if len(seqs) < 8:
        if seqs:
            print(f"stream {s}: only {len(seqs)} packets (not assessed)")
        continue
    # unwrap 16-bit
    un, off = [], 0
    for i, v in enumerate(seqs):
        un.append(v)
    interior = seqs[2:-2]
    span = interior[-1] - interior[0] + 1
    missing = span - len(interior)
    incomplete = sum(1 for (sid, q) in reasm if sid == s)
    print(f"stream {s}: {len(seqs)} FRAG seqs, interior span {span}, "
          f"missing {missing} ({100.0*missing/span:.3f}%), "
          f"unreassembled leftovers {incomplete}")

# RTP sanity + NAL histogram (H.265)
nals = {}
payload_bytes = 0
bad_rtp = 0
for sid, seq, pkt in recovered:
    if len(pkt) < 14 or (pkt[0] >> 6) != 2:
        bad_rtp += 1
        continue
    payload_bytes += len(pkt)
    off = 12 + 4 * (pkt[0] & 0x0F)
    t = (pkt[off] >> 1) & 0x3F
    if t == 49:  # FU: real type in FU header
        fu = pkt[off + 2]
        label = f"FU({fu & 0x3F})" + ("S" if fu & 0x80 else "")
    elif t == 48:
        label = "AP"
    else:
        label = str(t)
    nals[label] = nals.get(label, 0) + 1

print(f"recovered RTP packets: {len(recovered)} (bad RTP hdr: {bad_rtp})")
print("NAL types:", dict(sorted(nals.items(), key=lambda kv: -kv[1])[:12]))
crit = [k for k in nals if k in ("32", "33", "34", "AP") or k.startswith("FU(19)") or k.startswith("FU(20)") or k.startswith("FU(21)")]
print("critical NALs present:", {k: nals[k] for k in crit})
print(f"recovered payload: {payload_bytes} bytes")
