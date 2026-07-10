#!/usr/bin/env python3
"""Decode maburd --dry-run frames with devourer's reference decoders and
verify the recovered RTP stream against the input fixture."""
import argparse, os, random, struct, sys
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.abspath(os.path.join(ROOT, "..", "devourer", "tools", "precoder")))
import fec_subblock, stream_fec  # noqa: E402

FRAG_HDR = struct.Struct("<HBB")

def read_frames(path):
    out = []
    with open(path, "rb") as f:
        while True:
            h = f.read(4)
            if not h: break
            (l,) = struct.unpack("<I", h)
            out.append(f.read(l))
    return out

def strip_to_body(frame):
    (rl,) = struct.unpack_from("<H", frame, 2)   # radiotap it_len
    return frame[rl:rl + 2], frame[rl + 24:]     # (fc bytes, body)

def read_fixture(path):
    out = []
    with open(path, "rb") as f:
        while True:
            h = f.read(2)
            if not h: break
            (l,) = struct.unpack("<H", h)
            out.append(f.read(l))
    return out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True); ap.add_argument("--fixture", required=True)
    ap.add_argument("--k", type=int, default=8); ap.add_argument("--symbol-size", type=int, default=64)
    ap.add_argument("--drop-pct", type=float, default=0.0); ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--min-critical", type=float, default=1.0)  # delivery floor under loss
    ap.add_argument("--expect-mcs", type=int); ap.add_argument("--after", type=int, default=0)
    ap.add_argument("--stream", type=int, help="restrict --expect-mcs to this SBI stream_id")
    ap.add_argument("--max-stream", type=int, default=3,
                     help="only require fixture packets whose UEP stream "
                          "classification is <= this value to round-trip; "
                          "packets on higher (shed) streams are ignored when "
                          "computing 'want'. Defaults to 3 (no restriction). "
                          "Use 1 when no RCF/DISC was sent, since maburd's "
                          "MAX_RANGE boot default sheds streams 2/3 (T1/T2) "
                          "until an RC frame lands.")
    a = ap.parse_args()

    frames = read_frames(a.frames)
    rng = random.Random(a.seed)
    decs = {s: stream_fec.make_decoder(stream_fec.FecConfig(
        k=a.k, symbol_size=a.symbol_size, overhead=1.0, scheme="rs")) for s in range(4)}
    env_size = 11 + a.symbol_size
    reasm, reasm_n, recovered, per_stream_in = {}, {}, [], {s: 0 for s in range(4)}

    if a.expect_mcs is not None:  # RCF-application check: HT radiotap MCS byte is
        bad = checked = 0          # the last byte of the 13-byte header.
        for i, fr in enumerate(frames):
            if i < a.after: continue
            (rl,) = struct.unpack_from("<H", fr, 2)
            if rl != 13: continue                       # HT frames only
            if a.stream is not None:
                sid = fec_subblock.peek_stream_id(fr[rl + 24:])
                if sid != a.stream: continue
            checked += 1
            if fr[12] != a.expect_mcs: bad += 1
        print(f"expect-mcs: checked {checked} frames after {a.after}, {bad} mismatched")
        sys.exit(1 if bad or not checked else 0)

    for fr in frames:
        _, body = strip_to_body(fr)
        sid = fec_subblock.peek_stream_id(body)
        if sid is None or sid > 3: continue
        per_stream_in[sid] += 1
        if a.drop_pct and rng.random() * 100 < a.drop_pct: continue
        for env in fec_subblock.unpack(body, env_size).survivors:
            for pkt in decs[sid].add_symbol(env):
                if len(pkt) < FRAG_HDR.size: continue
                seq, idx, n = FRAG_HDR.unpack_from(pkt)
                key = (sid, seq); reasm.setdefault(key, {})[idx] = pkt[FRAG_HDR.size:]
                reasm_n[key] = n
                if len(reasm[key]) == n:
                    recovered.append(b"".join(reasm[key][i] for i in range(n)))
                    del reasm[key]

    fixture = read_fixture(a.fixture)
    def rtp_seq(p): return struct.unpack_from(">H", p, 2)[0]
    got = {rtp_seq(p): p for p in recovered}
    # UEP stream classification mirrors common/src/nal.cpp's classify_rtp:
    # critical (VPS/SPS/PPS/IDR-FU) -> stream 0; otherwise 1 + min(tid, 2).
    def classify_stream(p):
        off = 12 + 4 * (p[0] & 0x0F); t = (p[off] >> 1) & 0x3F
        tid = max(0, (p[off + 1] & 0x07) - 1)
        real = p[off + 2] & 0x3F if t == 49 else t
        crit = t == 48 or (16 <= real <= 23) or (32 <= real <= 34)
        return 0 if crit else 1 + min(tid, 2)
    all_want = {rtp_seq(p): p for p in fixture}
    # --max-stream restricts "want" to streams that are actually reachable
    # under the exercised link state: maburd's MAX_RANGE boot default sheds
    # streams 2/3 (T1/T2) until an RCF/DISC lands (drone/src/rc_agent.cpp),
    # so a run with no --rc-in can never deliver those packets.
    want = {s: p for s, p in all_want.items() if classify_stream(p) <= a.max_stream}
    exact = sum(1 for s, p in want.items() if got.get(s) == p)
    # critical = fixture packets that classify to stream 0 (VPS/SPS/PPS/IDR-FU)
    crit = [s for s, p in want.items() if classify_stream(p) == 0]
    crit_ok = sum(1 for s in crit if got.get(s) == want[s])
    print(f"recovered {exact}/{len(want)} RTP packets; critical {crit_ok}/{len(crit)}; "
          f"bodies per stream {per_stream_in}")
    if a.drop_pct == 0:
        sys.exit(0 if exact == len(want) else 1)
    sys.exit(0 if crit_ok >= a.min_critical * len(crit) else 1)

if __name__ == "__main__":
    main()
