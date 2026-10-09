#!/usr/bin/env python3
"""Cut tests/fixtures/slices4_1080p.h265 out of a 4-slice 1080p capture.

The source is the bench drone's cap4 (2026-10-09, spike-venc-slices
MABUR_SPIKE_DUMP, by-frame + slices=4), kept at
sbc-groundstations-gilankpam/output/gs-p1-backup-2026-10-09/cap4.h265.
Pictures 0,1,2 (IDR + two P) and 30..33 (refresh start + three 4-slice P):
small, and every case the slice tests need.

usage: make_slice_fixture.py CAP4 OUT
"""
import re
import sys

PICS = [0, 1, 2, 30, 31, 32, 33]


def main():
    src, out = sys.argv[1], sys.argv[2]
    b = open(src, "rb").read()
    pos = [m.start() - 1 for m in re.finditer(b"\x00\x00\x01", b)] + [len(b)]
    pics, pre = [], []
    for i in range(len(pos) - 1):
        s, e = pos[i], pos[i + 1]
        t = (b[s + 4] >> 1) & 63
        if t <= 21 and b[s + 6] >> 7:
            pics.append(pre)
            pre = []
        (pics[-1] if t <= 21 else pre).append((s, e))
    with open(out, "wb") as f:
        for k in PICS:
            for s, e in pics[k]:
                f.write(b[s:e])
    print(f"{out}: pictures {PICS}")


if __name__ == "__main__":
    main()
