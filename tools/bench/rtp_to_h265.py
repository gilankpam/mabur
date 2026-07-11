#!/usr/bin/env python3
"""Depacketize recovered RTP (from live_decode pipeline) into Annex-B H.265."""
import json, struct, sys

import os
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.abspath(os.path.join(ROOT, "..", "devourer", "tools", "precoder")))
import fec_subblock, stream_fec  # noqa: E402

K, SYMBOL = 8, 64
ENV_SIZE = 11 + SYMBOL
FRAG_HDR = struct.Struct("<HBB")

src, dst = sys.argv[1], sys.argv[2]
decs = {s: stream_fec.make_decoder(stream_fec.FecConfig(
    k=K, symbol_size=SYMBOL, overhead=1.0, scheme="rs")) for s in range(4)}
reasm, reasm_n, recovered = {}, {}, []
for line in open(src):
    if '"rx.frame"' not in line:
        continue
    body = bytes.fromhex(json.loads(line)["body"])
    sid = fec_subblock.peek_stream_id(body)
    if sid is None or sid > 3:
        continue
    for env in fec_subblock.unpack(body, ENV_SIZE).survivors:
        for pkt in decs[sid].add_symbol(env):
            if len(pkt) < FRAG_HDR.size:
                continue
            seq, idx, n = FRAG_HDR.unpack_from(pkt)
            key = (sid, seq)
            reasm.setdefault(key, {})[idx] = pkt[FRAG_HDR.size:]
            if len(reasm[key]) == n:
                recovered.append(b"".join(reasm[key][i] for i in range(n)))
                del reasm[key]

# order by RTP seq (16-bit, unwrap)
def rtp_seq(p):
    return struct.unpack_from(">H", p, 2)[0]
recovered = [p for p in recovered if len(p) >= 14 and (p[0] >> 6) == 2]
base = rtp_seq(recovered[0])
def unwrap(s):
    d = (s - base) & 0xFFFF
    return d if d < 0x8000 else d - 0x10000
recovered.sort(key=lambda p: unwrap(rtp_seq(p)))

out = bytearray()
SC = b"\x00\x00\x00\x01"
fu_buf, fu_hdr, fu_next_seq = None, None, None
n_nals = n_fu_drop = 0
for p in recovered:
    off = 12 + 4 * (p[0] & 0x0F)
    pay = p[off:]
    t = (pay[0] >> 1) & 0x3F
    if t == 49:  # FU
        fu = pay[2]
        s_bit, e_bit, ftype = fu & 0x80, fu & 0x40, fu & 0x3F
        frag = pay[3:]
        sq = unwrap(rtp_seq(p))
        if s_bit:
            hdr = bytes([(pay[0] & 0x81) | (ftype << 1), pay[1]])
            fu_buf, fu_hdr, fu_next_seq = bytearray(frag), hdr, sq + 1
        elif fu_buf is not None and sq == fu_next_seq:
            fu_buf += frag
            fu_next_seq = sq + 1
        else:
            fu_buf = None  # gap: drop this FU
            n_fu_drop += 1
            continue
        if e_bit and fu_buf is not None:
            out += SC + fu_hdr + fu_buf
            n_nals += 1
            fu_buf = None
    elif t == 48:  # AP
        q = pay[2:]
        while len(q) >= 2:
            (l,) = struct.unpack_from(">H", q)
            out += SC + q[2:2 + l]
            n_nals += 1
            q = q[2 + l:]
    else:  # single NAL
        out += SC + pay
        n_nals += 1

open(dst, "wb").write(out)
print(f"wrote {len(out)} bytes, {n_nals} NALs, {n_fu_drop} FUs dropped on gaps")
