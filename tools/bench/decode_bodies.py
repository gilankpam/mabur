#!/usr/bin/env python3
"""Decode maburd --dry-run radio frames with devourer's reference decoders and
verify the recovered frame units against the input fixture.

Recovered unit = FrameHdr (8 B) + Annex-B frame, reassembled from the 6-byte
FRAG fragments each layer's sliding-window decoder delivers — the same
assembly maburgs' FrameStream does on air."""
import argparse, os, random, struct, sys
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "pyref"))
import sbi, sw_fec  # noqa: E402

FRAG_HDR = struct.Struct("<HHH")     # seq, idx, count
FRAME_HDR = struct.Struct("<HBBI")   # frame_id, flags, codec, pts_us
FLAG_IDR = 0x01
FLAG_DISCONT = 0x02
FLAG_ENHANCE = 0x04

def parse_symbol_size(text):
    """Accepts a single int (shared by both streams) or a comma-separated
    2-list, one per UEP stream (e.g. "164,1312"), mirroring the GS/drone
    bundle's scalar-or-array fec.symbol_size (2-stream space, spec
    2026-08-29-airtime-balance-uep)."""
    parts = [p.strip() for p in text.split(",")]
    if len(parts) == 1:
        return [int(parts[0])] * 2
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(
            "--symbol-size must be a single int or a comma-separated 2-list")
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
    fc = frame[rl:rl + 1]  # frame control byte (slice for truncation safety)
    header_len = 26 if fc == b"\x88" else 24  # QoS-Data vs legacy probe-req
    return frame[rl:rl + 2], frame[rl + header_len:]  # (fc bytes, body)

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
    """Mirror of mabur classify_frame (common/src/nal.cpp), extended to also
    report TRAIL_N-ness (mabur's frame_is_trail_n): returns (sid, trail_n).
    2-stream space (spec 2026-08-29-airtime-balance-uep): critical NALs
    (VPS/SPS/PPS 32-34, IRAP 16-23) and any non-TRAIL_N VCL -> sid 0 (base);
    TRAIL_N (type 0) -> sid 1 (enh). trail_n is only meaningful when
    sid == 1, since sid 1 is reached exclusively via TRAIL_N under this
    rule (tid no longer participates in routing)."""
    if len(annexb) < 5: return 0, False
    sid, trail_n, i = None, False, 0
    while i + 4 < len(annexb):
        if annexb[i:i + 3] != b"\x00\x00\x01":
            i += 1
            continue
        t = (annexb[i + 3] >> 1) & 0x3F
        if (16 <= t <= 23) or (32 <= t <= 34): return 0, False
        if sid is None and t < 16:
            sid = 1 if t == 0 else 0
            trail_n = t == 0
        i += 3
    return (0, False) if sid is None else (sid, trail_n)

def expected_unit(rec, frame_id):
    """The wire unit maburd builds for this fixture frame (drone
    frame_pipeline.cpp): FrameHdr stamped over the producer meta. FLAG_DISCONT
    is stripped by mask_discont() before comparing: the flag rides on every
    frame for kDiscontStickyMs after start, so which frames carry it depends
    on wall-clock timing, not fixture content."""
    flags = FLAG_IDR if rec["flags"] & FLAG_IDR else 0
    return FRAME_HDR.pack(frame_id, flags, rec["codec"], rec["pts"]) + rec["annexb"]

def mask_discont(unit):
    fid, flags, codec, pts = FRAME_HDR.unpack_from(unit)
    return FRAME_HDR.pack(fid, flags & ~FLAG_DISCONT, codec, pts) + unit[FRAME_HDR.size:]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True); ap.add_argument("--fixture", required=True)
    ap.add_argument("--symbol-size", type=parse_symbol_size, default=[64, 64])
    ap.add_argument("--drop-pct", type=float, default=0.0); ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--min-critical", type=float, default=1.0)  # delivery floor under loss
    ap.add_argument("--expect-mcs", type=int); ap.add_argument("--after", type=int, default=0)
    ap.add_argument("--stream", type=int, help="restrict --expect-mcs to this SBI stream_id")
    ap.add_argument("--max-stream", type=int, default=1,
                     help="only require fixture frames whose UEP stream "
                          "classification is <= this value to round-trip; "
                          "frames on the enh stream are ignored when "
                          "computing 'want' if it is shed. Defaults to 1 "
                          "(no restriction: the 2-stream space tops out at "
                          "sid 1). Use 0 when no RCF/DISC was sent, since "
                          "maburd's MAX_RANGE boot default sheds the enh "
                          "layer (sid 1) until an RC frame lands.")
    a = ap.parse_args()

    frames = read_frames(a.frames)
    rng = random.Random(a.seed)
    decs = {s: sw_fec.SwDecoder(symbol_size=a.symbol_size[s]) for s in range(2)}
    env_size = {s: sw_fec.SW_HDR_LEN + a.symbol_size[s] for s in range(2)}
    reasm, reasm_n, recovered, per_stream_in = {}, {}, [], {s: 0 for s in range(2)}

    if a.expect_mcs is not None:  # RCF-application check: HT radiotap MCS byte is
        bad = checked = 0          # the last byte of the 13-byte header.
        for i, fr in enumerate(frames):
            if i < a.after: continue
            (rl,) = struct.unpack_from("<H", fr, 2)
            if rl != 13: continue                       # HT frames only
            if a.stream is not None:
                fc = fr[rl:rl + 1]  # frame control byte (slice for truncation safety)
                header_len = 26 if fc == b"\x88" else 24  # QoS-Data vs legacy probe-req
                sid = sbi.peek_stream_id(fr[rl + header_len:])
                if sid != a.stream: continue
            checked += 1
            if fr[12] != a.expect_mcs: bad += 1
        print(f"expect-mcs: checked {checked} frames after {a.after}, {bad} mismatched")
        sys.exit(1 if bad or not checked else 0)

    for fr in frames:
        _, body = strip_to_body(fr)
        sid = sbi.peek_stream_id(body)
        if sid is None or sid < 0 or sid > 1: continue
        per_stream_in[sid] += 1
        if a.drop_pct and rng.random() * 100 < a.drop_pct: continue
        r = sbi.unpack(body, env_size[sid])
        for env in r["survivors"]:
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
    # Recovered units are keyed by the frame_id maburd stamped. maburd's
    # FramePipeline now allocates frame_id AFTER the shed check (spec
    # 2026-07-26 svct-enable: "shed before frame_id"), so a shed frame burns
    # no id -- frame_id is no longer the fixture index, it is the frame's
    # position among the frames maburd actually encodes (see "reach" below).
    got = {}
    discont_on_first = False
    for unit in recovered:
        if len(unit) < FRAME_HDR.size: continue
        fid, flags = FRAME_HDR.unpack_from(unit)[:2]
        if fid == 0 and flags & FLAG_DISCONT: discont_on_first = True
        got[fid] = mask_discont(unit)
    # --max-stream restricts "want" to streams that are actually reachable
    # under the exercised link state: maburd's MAX_RANGE boot default sheds
    # the enh layer (sid 1) until an RCF/DISC lands (drone/src/rc_agent.cpp),
    # so a run with no --rc-in can never deliver those frames. stream_of
    # mirrors FramePipeline::encode's full routing: classify_frame's raw sid,
    # then the same producer-flag/TRAIL_N agreement demotion (disagreement
    # protects any sid down to 0, base) before shedding is considered.
    def stream_of(i):
        rec = fixture[i]
        if rec["flags"] & FLAG_IDR: return 0
        sid, trail_n = classify_frame(rec["annexb"])
        if trail_n != bool(rec["flags"] & FLAG_ENHANCE):
            sid = 0
        return sid
    # enumerate(reach)'s k is fid only if the actual shed boundary matches
    # --max-stream exactly (MAX_RANGE default sheds sid 1, or --max-stream 1
    # sheds nothing); other --max-stream values would mis-key against got.
    reach = [i for i in range(len(fixture)) if stream_of(i) <= a.max_stream]
    want = {k: expected_unit(fixture[i], k) for k, i in enumerate(reach)}
    exact = sum(1 for k, u in want.items() if got.get(k) == u)
    # critical = fixture frames on stream 0 (IDR / parameter sets)
    crit = [k for k, i in enumerate(reach) if stream_of(i) == 0]
    crit_ok = sum(1 for k in crit if got.get(k) == want[k])
    print(f"recovered {exact}/{len(want)} frames; critical {crit_ok}/{len(crit)}; "
          f"bodies per stream {per_stream_in}")
    if 0 in got and not discont_on_first:
        print("frame 0 recovered without FLAG_DISCONT: start-of-stream signal missing")
        sys.exit(1)
    if a.drop_pct == 0:
        sys.exit(0 if exact == len(want) else 1)
    sys.exit(0 if crit_ok >= a.min_critical * len(crit) else 1)

if __name__ == "__main__":
    main()
