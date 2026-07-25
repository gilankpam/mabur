#!/usr/bin/env python3
"""Compare the RTP maburgs re-built (u16-LE length-prefixed) with the frame
fixture maburd was fed.

The GS no longer forwards the drone's packets: it reassembles whole frames and
packetizes them itself (RFC 7798). So the invariant is per FRAME, not per
packet — depacketize the output back to a NAL list and require it to equal the
fixture frame's NAL list. Start-code widths are not compared (the packetizer
strips them by design); everything inside a NAL must be byte-exact.

Per-stream accounting uses the same classifier as the drone (a mirror of
mabur classify_frame).
"""
import argparse
import struct
import sys


def read_lp16(path):
    out = []
    with open(path, "rb") as f:
        while True:
            h = f.read(2)
            if len(h) < 2:
                break
            (n,) = struct.unpack("<H", h)
            p = f.read(n)
            if len(p) < n:
                sys.exit(f"{path}: truncated record")
            out.append(p)
    return out


def read_frame_fixture(path):
    """u32-LE len | VencFrameMeta (pts u32, codec u8, flags u8, rsv u16) | Annex-B."""
    out = []
    with open(path, "rb") as f:
        while True:
            h = f.read(4)
            if len(h) < 4:
                break
            (n,) = struct.unpack("<I", h)
            rec = f.read(n)
            if len(rec) < n:
                sys.exit(f"{path}: truncated record")
            pts, _codec, flags, _ = struct.unpack_from("<IBBH", rec, 0)
            out.append({"pts": pts, "flags": flags, "nals": split_annexb(rec[8:])})
    return out


def split_annexb(data):
    """Annex-B byte stream -> list of NAL payloads (start codes stripped)."""
    def next_sc(pos):
        while pos + 3 <= len(data):
            if data[pos:pos + 3] == b"\x00\x00\x01":
                return pos
            pos += 1
        return -1

    nals = []
    sc = next_sc(0)
    while sc >= 0:
        start = sc + 3
        nxt = next_sc(start)
        end = len(data) if nxt < 0 else nxt
        # A 4-byte start code is a 3-byte code with one extra leading 0x00, so
        # trim exactly one zero off the NAL that precedes it. (HEVC emulation
        # prevention rules out a payload legitimately ending 00 before 00 00 01
        # — that byte sequence IS a 4-byte start code.)
        if nxt >= 0 and end > start and data[end - 1] == 0x00:
            end -= 1
        nals.append(data[start:end])
        sc = nxt
    return nals


def classify(nals):  # mirror of mabur classify_frame
    sid = None
    for n in nals:
        if len(n) < 2:
            continue
        t = (n[0] >> 1) & 0x3F
        tid = max((n[1] & 0x07) - 1, 0)
        if (16 <= t <= 23) or (32 <= t <= 34):
            return 0
        if sid is None and t < 16:
            sid = 1 + min(tid, 2)
    return 0 if sid is None else sid


def depacketize(packets):
    """RTP (RFC 7798) -> [{"ts", "nals", "complete"}], grouped per frame.

    A frame is one run of packets sharing an RTP timestamp; it is complete only
    if its last packet carries the marker bit (RtpPacketizer withholds the
    marker AND the FU end bit on a truncated frame)."""
    frames, cur = [], None
    for p in packets:
        if len(p) < 14:
            sys.exit("recovered: RTP packet shorter than a header")
        marker = (p[1] >> 7) & 1
        ts = struct.unpack_from(">I", p, 4)[0]
        payload = p[12:]
        if cur is None or ts != cur["ts"]:
            cur = {"ts": ts, "nals": [], "complete": False, "fu": None}
            frames.append(cur)
        t = (payload[0] >> 1) & 0x3F
        if t == 49:  # FU
            start = (payload[2] >> 7) & 1
            end = (payload[2] >> 6) & 1
            inner = payload[2] & 0x3F
            if start:
                hdr = bytes([(inner << 1) | (payload[0] & 0x81), payload[1]])
                cur["fu"] = bytearray(hdr)
            if cur["fu"] is not None:
                cur["fu"] += payload[3:]
                if end:
                    cur["nals"].append(bytes(cur["fu"]))
                    cur["fu"] = None
        else:
            cur["nals"].append(bytes(payload))
        if marker:
            cur["complete"] = True
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recovered")
    ap.add_argument("fixture")
    ap.add_argument("--require-all", action="store_true",
                    help="every fixture frame must be recovered")
    ap.add_argument("--min-stream0", type=float,
                    help="required stream-0 delivery fraction (e.g. 1.0)")
    a = ap.parse_args()

    packets = read_lp16(a.recovered)
    fix = read_frame_fixture(a.fixture)
    got = depacketize(packets)

    # Frames are matched by content (each fixture frame's payload is unique),
    # so a recovered frame counts only if every NAL came through byte-exact and
    # the frame closed with the marker bit.
    got_keys = {}
    for g in got:
        if g["complete"]:
            got_keys.setdefault(tuple(g["nals"]), 0)
            got_keys[tuple(g["nals"])] += 1
    truncated = sum(1 for g in got if not g["complete"])

    ok = True
    by_sid = {}
    for fr in fix:
        by_sid.setdefault(classify(fr["nals"]), []).append(tuple(fr["nals"]))
    for sid in sorted(by_sid):
        recovered_n = sum(1 for k in by_sid[sid] if k in got_keys)
        tot = len(by_sid[sid])
        print(f"stream {sid}: {recovered_n}/{tot} frames recovered")
        if a.require_all and recovered_n != tot:
            print(f"FAIL: stream {sid} incomplete")
            ok = False
        if sid == 0 and a.min_stream0 is not None and recovered_n < a.min_stream0 * tot:
            print("FAIL: stream 0 below floor")
            ok = False

    fix_keys = {k for keys in by_sid.values() for k in keys}
    alien = [k for k in got_keys if k not in fix_keys]
    if alien:
        print(f"FAIL: {len(alien)} recovered frame(s) not in the fixture")
        ok = False
    dup = sum(n - 1 for n in got_keys.values() if n > 1)
    if dup:
        print(f"FAIL: {dup} duplicate recovered frame(s)")
        ok = False
    # RTP timestamps must be non-decreasing: FrameStream emits frames in
    # frame_id order and RtpPacketizer derives ts from each frame's pts.
    ts_seq = [g["ts"] for g in got]
    if any(b < x for x, b in zip(ts_seq, ts_seq[1:])):
        print("FAIL: RTP timestamps not monotonic")
        ok = False
    print(f"frames out: {len(got)} ({truncated} truncated), rtp packets: {len(packets)}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
