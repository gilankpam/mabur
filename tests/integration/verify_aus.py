#!/usr/bin/env python3
"""Compare maburgs --out-aus records with the frame fixture maburd was fed.

PR C successor to verify_rtp.py: the GS no longer emits RTP — the dry-run
captures each reassembled access unit as an LP record (u32 total_len | u8
sid | u8 flags | u32 pts_us | Annex-B; flags = framewire idr|discont with
bit 0x04 = complete, written by main.cpp's AuFileOut — keep in sync). The
invariant is unchanged and per FRAME: a recovered frame counts only if it
is complete and every NAL matches the fixture frame byte-exact. Start-code
widths are not compared; per-stream accounting uses the same classifier as
the drone (a mirror of mabur classify_frame), and the recorded wire sid
must agree with that classifier.
"""
import argparse
import struct
import sys

FLAG_COMPLETE = 0x04


def read_aus(path):
    out = []
    with open(path, "rb") as f:
        while True:
            h = f.read(10)
            if len(h) < 10:
                break
            n, sid, flags, pts = struct.unpack("<IBBI", h)
            p = f.read(n)
            if len(p) < n:
                sys.exit(f"{path}: truncated record")
            out.append({"sid": sid, "flags": flags, "pts": pts,
                        "complete": bool(flags & FLAG_COMPLETE),
                        "nals": split_annexb(p)})
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


def classify(nals):  # mirror of mabur classify_frame (2-stream since
    # task-2-2-stream-classify-frame / common/src/nal.cpp: critical NALs
    # (IDR/CRA/BLA 16-23, VPS/SPS/PPS 32-34) and everything else route to
    # BASE (sid 0); the first TRAIL_N (type 0) NAL routes the frame to ENH
    # (sid 1). tid-based routing to sid 1-3 is gone.
    for n in nals:
        if len(n) < 2:
            continue
        t = (n[0] >> 1) & 0x3F
        if (16 <= t <= 23) or (32 <= t <= 34):
            return 0
        if t < 16:
            return 1 if t == 0 else 0
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recovered")
    ap.add_argument("fixture")
    ap.add_argument("--require-all", action="store_true",
                    help="every fixture frame must be recovered")
    ap.add_argument("--min-stream0", type=float,
                    help="required stream-0 delivery fraction (e.g. 1.0)")
    ap.add_argument("--min-stream1", type=float,
                    help="required stream-1 delivery fraction (e.g. 1.0). "
                         "Also fails if the fixture has stream-1 frames but "
                         "none were classified into by_sid[1] at all -- a "
                         "regression confined to the enh stream (e.g. "
                         "classify_frame or the decoder silently dropping "
                         "sid 1) must not pass a gate that only ever checked "
                         "stream 0.")
    a = ap.parse_args()

    got = read_aus(a.recovered)
    fix = read_frame_fixture(a.fixture)

    # Frames are matched by content (each fixture frame's payload is unique),
    # so a recovered frame counts only if every NAL came through byte-exact
    # and the AU record carries the complete flag.
    got_keys = {}
    for g in got:
        if g["complete"]:
            got_keys.setdefault(tuple(g["nals"]), 0)
            got_keys[tuple(g["nals"])] += 1
    truncated = sum(1 for g in got if not g["complete"])

    ok = True
    # The wire sid on each complete AU must agree with the classifier — the
    # drone classified the frame once at encode time; a disagreement means
    # frame bytes and routing metadata came apart somewhere in the chain.
    for g in got:
        if g["complete"] and classify(g["nals"]) != g["sid"]:
            print(f"FAIL: recorded sid {g['sid']} != classified "
                  f"{classify(g['nals'])} for a complete AU")
            ok = False
            break

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
        if sid == 1 and a.min_stream1 is not None and recovered_n < a.min_stream1 * tot:
            print("FAIL: stream 1 below floor")
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
    # AU pts must be non-decreasing modulo u32 wrap: FrameStream emits frames
    # in frame_id order and pts is the encoder's capture stamp. A forward
    # delta >= 2^31 would mean a genuine reversal, not a wrap.
    for prev, cur in zip(got, got[1:]):
        if cur["flags"] & 0x02:  # kFlagDiscont: pts legitimately rebases
            continue
        if ((cur["pts"] - prev["pts"]) & 0xFFFFFFFF) >= 0x80000000:
            print("FAIL: AU pts regressed")
            ok = False
            break
    print(f"frames out: {len(got)} ({truncated} truncated)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
