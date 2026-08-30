#!/usr/bin/env python3
"""Streaming proto-GS viewer filter: rxdemo STREAM_OUT JSONL on stdin ->
Annex-B H.265 elementary stream on stdout.

Usage (see docs/bench-validation.md, E1):
  DEVOURER_RX_ZEROCOPY=0 DEVOURER_PID=0x8812 DEVOURER_CHANNEL=149 \
  DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=warn DEVOURER_STREAM_OUT=1 \
  ./build/rxdemo 2>/dev/null | python3 live_play.py | \
  ffplay -fflags nobuffer -flags low_delay -f hevc -i -

Decode chain per frame body: SBI unpack -> per-stream RS decoder -> FRAG
reassembly -> RTP depacketize (single NAL / AP / FU type 49) -> start-coded
NAL units, flushed as soon as each completes.
"""
import json
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.abspath(os.path.join(ROOT, "..", "devourer", "tools", "precoder")))
sys.path.insert(0, os.path.join(ROOT, "tools", "pyref"))
import sbi, stream_fec  # noqa: E402

K, SYMBOL = 8, 64
ENV_SIZE = 11 + SYMBOL
FRAG_HDR = struct.Struct("<HBB")
SC = b"\x00\x00\x00\x01"

out = sys.stdout.buffer
decs = {s: stream_fec.make_decoder(stream_fec.FecConfig(
    k=K, symbol_size=SYMBOL, overhead=1.0, scheme="rs")) for s in range(4)}
reasm = {}
# per-stream FU reassembly state: (nal_hdr, buf, last_rtp_seq)
fu = {}


def emit_rtp(sid, pkt):
    if len(pkt) < 14 or (pkt[0] >> 6) != 2:
        return
    seq = struct.unpack_from(">H", pkt, 2)[0]
    off = 12 + 4 * (pkt[0] & 0x0F)
    pay = pkt[off:]
    t = (pay[0] >> 1) & 0x3F
    if t == 49:  # FU
        f = pay[2]
        s_bit, e_bit, ftype = f & 0x80, f & 0x40, f & 0x3F
        frag = pay[3:]
        if s_bit:
            hdr = bytes([(pay[0] & 0x81) | (ftype << 1), pay[1]])
            fu[sid] = (hdr, bytearray(frag), seq)
        elif sid in fu:
            hdr, buf, last = fu[sid]
            if ((seq - last) & 0xFFFF) != 1:
                del fu[sid]  # gap inside the FU: drop the slice
                return
            buf += frag
            fu[sid] = (hdr, buf, seq)
        else:
            return
        if e_bit and sid in fu:
            hdr, buf, _ = fu.pop(sid)
            out.write(SC + hdr + buf)
            out.flush()
    elif t == 48:  # AP
        q = pay[2:]
        while len(q) >= 2:
            (l,) = struct.unpack_from(">H", q)
            out.write(SC + q[2:2 + l])
            q = q[2 + l:]
        out.flush()
    else:
        out.write(SC + pay)
        out.flush()


for line in sys.stdin:
    if '"rx.frame"' not in line:
        continue
    try:
        body = bytes.fromhex(json.loads(line)["body"])
    except (ValueError, KeyError):
        continue
    sid = sbi.peek_stream_id(body)
    if sid is None or sid < 0 or sid > 3:
        continue
    for env in sbi.unpack(body, ENV_SIZE)["survivors"]:
        for pkt in decs[sid].add_symbol(env):
            if len(pkt) < FRAG_HDR.size:
                continue
            fseq, idx, n = FRAG_HDR.unpack_from(pkt)
            key = (sid, fseq)
            reasm.setdefault(key, {})[idx] = pkt[FRAG_HDR.size:]
            if len(reasm[key]) == n:
                parts = reasm.pop(key)
                emit_rtp(sid, b"".join(parts[i] for i in range(n)))
