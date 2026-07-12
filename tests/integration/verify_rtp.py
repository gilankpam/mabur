#!/usr/bin/env python3
"""Compare maburgs-recovered RTP (u16-LE length-prefixed) with the fixture.

Per-stream accounting uses the same classifier as the drone (gen_vectors.py's
mirror of mabur classify_rtp). Recovered packets are matched by exact bytes —
RTP seq numbers make every fixture packet unique.
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


def classify(pkt):  # mirror of mabur classify_rtp (see tools/genvectors/gen_vectors.py)
    if len(pkt) < 14 or (pkt[0] >> 6) != 2:
        return 0
    off = 12 + 4 * (pkt[0] & 0x0F)
    p = pkt[off:]
    t = (p[0] >> 1) & 0x3F
    if t == 49:
        real, tid = p[2] & 0x3F, (p[1] & 7) - 1
    elif t == 48:
        return 0
    else:
        real, tid = t, (p[1] & 7) - 1
    tid = max(tid, 0)
    crit = (16 <= real <= 23) or (32 <= real <= 34)
    return 0 if crit else 1 + min(tid, 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recovered")
    ap.add_argument("fixture")
    ap.add_argument("--require-all", action="store_true",
                    help="every fixture packet must be recovered")
    ap.add_argument("--min-stream0", type=float,
                    help="required stream-0 delivery fraction (e.g. 1.0)")
    a = ap.parse_args()

    rec = read_lp16(a.recovered)
    fix = read_lp16(a.fixture)
    rec_set = set(rec)
    fix_set = set(fix)

    ok = True
    by_sid = {}
    for p in fix:
        by_sid.setdefault(classify(p), []).append(p)
    for sid in sorted(by_sid):
        got = sum(1 for p in by_sid[sid] if p in rec_set)
        tot = len(by_sid[sid])
        print(f"stream {sid}: {got}/{tot} recovered")
        if a.require_all and got != tot:
            print(f"FAIL: stream {sid} incomplete")
            ok = False
        if sid == 0 and a.min_stream0 is not None and got < a.min_stream0 * tot:
            print("FAIL: stream 0 below floor")
            ok = False

    alien = [p for p in rec if p not in fix_set]
    if alien:
        print(f"FAIL: {len(alien)} recovered packet(s) not in the fixture")
        ok = False
    dup = len(rec) - len(rec_set)
    if dup:
        print(f"FAIL: {dup} duplicate recovered packet(s)")
        ok = False
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
