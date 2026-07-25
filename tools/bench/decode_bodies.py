#!/usr/bin/env python3
"""Decode maburd --dry-run radio frames with devourer's reference decoders and
verify the recovered frame units against the input fixture.

Recovered unit = FrameHdr (8 B) + Annex-B frame, reassembled from the 6-byte
FRAG fragments each layer's sliding-window decoder delivers — the same
assembly maburgs' FrameStream does on air."""
import argparse, os, random, struct, sys
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.abspath(os.path.join(ROOT, "..", "devourer", "tools", "precoder")))
import fec_subblock  # noqa: E402
sys.path.insert(0, os.path.join(ROOT, "tools", "pyref"))
import sw_fec  # noqa: E402

FRAG_HDR = struct.Struct("<HHH")     # seq, idx, count
FRAME_HDR = struct.Struct("<HBBI")   # frame_id, flags, codec, pts_us
FLAG_IDR = 0x01
FLAG_DISCONT = 0x02

def parse_symbol_size(text):
    """Accepts a single int (shared by all 4 streams) or a comma-separated
    4-list, one per UEP stream (e.g. "164,1312,1312,1312"), mirroring the
    GS/drone bundle's scalar-or-array fec.symbol_size."""
    parts = [p.strip() for p in text.split(",")]
    if len(parts) == 1:
        return [int(parts[0])] * 4
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            "--symbol-size must be a single int or a comma-separated 4-list")
    return [int(p) for p in parts]

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
    """tests/fixtures/frame_stream.bin: u32-LE len | VencFrameMeta | Annex-B."""
    out = []
    with open(path, "rb") as f:
        while True:
            h = f.read(4)
            if len(h) < 4: break
            (n,) = struct.unpack("<I", h)
            rec = f.read(n)
            if len(rec) < n: sys.exit(f"{path}: truncated record")
            pts, codec, flags, _ = struct.unpack_from("<IBBH", rec, 0)
            out.append({"pts": pts, "codec": codec, "flags": flags,
                        "annexb": rec[8:]})
    return out

def classify_frame(annexb):
    """Mirror of mabur classify_frame (common/src/nal.cpp)."""
    if len(annexb) < 5: return 0
    sid, i = None, 0
    while i + 4 < len(annexb):
        if annexb[i:i + 3] != b"\x00\x00\x01":
            i += 1
            continue
        t = (annexb[i + 3] >> 1) & 0x3F
        tid = max((annexb[i + 4] & 0x07) - 1, 0)
        if (16 <= t <= 23) or (32 <= t <= 34): return 0
        if sid is None and t < 16: sid = 1 + min(tid, 2)
        i += 3
    return 0 if sid is None else sid

def expected_unit(rec, frame_id, discont):
    """The wire unit maburd builds for this fixture frame (drone
    frame_pipeline.cpp): FrameHdr stamped over the producer meta."""
    flags = (FLAG_IDR if rec["flags"] & FLAG_IDR else 0) | (FLAG_DISCONT if discont else 0)
    return FRAME_HDR.pack(frame_id, flags, rec["codec"], rec["pts"]) + rec["annexb"]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True); ap.add_argument("--fixture", required=True)
    ap.add_argument("--symbol-size", type=parse_symbol_size, default=[64, 64, 64, 64])
    ap.add_argument("--drop-pct", type=float, default=0.0); ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--min-critical", type=float, default=1.0)  # delivery floor under loss
    ap.add_argument("--expect-mcs", type=int); ap.add_argument("--after", type=int, default=0)
    ap.add_argument("--stream", type=int, help="restrict --expect-mcs to this SBI stream_id")
    ap.add_argument("--max-stream", type=int, default=3,
                     help="only require fixture frames whose UEP stream "
                          "classification is <= this value to round-trip; "
                          "frames on higher (shed) streams are ignored when "
                          "computing 'want'. Defaults to 3 (no restriction). "
                          "Use 1 when no RCF/DISC was sent, since maburd's "
                          "MAX_RANGE boot default sheds streams 2/3 (T1/T2) "
                          "until an RC frame lands.")
    a = ap.parse_args()

    frames = read_frames(a.frames)
    rng = random.Random(a.seed)
    decs = {s: sw_fec.SwDecoder(symbol_size=a.symbol_size[s]) for s in range(4)}
    env_size = {s: sw_fec.SW_HDR_LEN + a.symbol_size[s] for s in range(4)}
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
        for env in fec_subblock.unpack(body, env_size[sid]).survivors:
            for frag in decs[sid].add_symbol(env):
                if len(frag) < FRAG_HDR.size: continue
                seq, idx, n = FRAG_HDR.unpack_from(frag)
                if n == 0: continue
                key = (sid, seq); reasm.setdefault(key, {})[idx] = frag[FRAG_HDR.size:]
                reasm_n[key] = n
                if len(reasm[key]) == n:
                    recovered.append(b"".join(reasm[key][i] for i in range(n)))
                    del reasm[key]

    fixture = read_fixture(a.fixture)
    # Recovered units are keyed by the frame_id maburd stamped: a global
    # counter across layers, so it is the fixture's frame index.
    got = {}
    for unit in recovered:
        if len(unit) < FRAME_HDR.size: continue
        got[FRAME_HDR.unpack_from(unit)[0]] = unit
    all_want = {i: expected_unit(rec, i, discont=(i == 0))
                for i, rec in enumerate(fixture)}
    # --max-stream restricts "want" to streams that are actually reachable
    # under the exercised link state: maburd's MAX_RANGE boot default sheds
    # streams 2/3 (T1/T2) until an RCF/DISC lands (drone/src/rc_agent.cpp),
    # so a run with no --rc-in can never deliver those frames.
    def stream_of(i):
        rec = fixture[i]
        return 0 if rec["flags"] & FLAG_IDR else classify_frame(rec["annexb"])
    want = {i: u for i, u in all_want.items() if stream_of(i) <= a.max_stream}
    exact = sum(1 for i, u in want.items() if got.get(i) == u)
    # critical = fixture frames on stream 0 (IDR / parameter sets)
    crit = [i for i in want if stream_of(i) == 0]
    crit_ok = sum(1 for i in crit if got.get(i) == want[i])
    print(f"recovered {exact}/{len(want)} frames; critical {crit_ok}/{len(crit)}; "
          f"bodies per stream {per_stream_in}")
    if a.drop_pct == 0:
        sys.exit(0 if exact == len(want) else 1)
    sys.exit(0 if crit_ok >= a.min_critical * len(crit) else 1)

if __name__ == "__main__":
    main()
