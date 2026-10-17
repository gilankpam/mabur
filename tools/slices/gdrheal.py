#!/usr/bin/env python3
"""Refresh-picture structure and drop-one-base healing from a gdrcap capture.

  gdrheal.py summary CAP
      Per-AU table (fid, sid, len, NAL types, VCL slice count) and a
      summary: which AUs are refresh starts (VPS-led, or IRAP), their slice
      count and size, the refresh cadence, and the AU size by frames since
      the refresh start -- which shows whether the intra stripe runs on
      consecutive frames after the start or on every frame.

  gdrheal.py heal CAP [--gop N] [--k K] [--frames M]
      Builds an annex-B stream from the N-th refresh start, decodes it
      with ffmpeg (luma), then drops the K-th sid-0 P picture after that
      start and decodes again. Prints, per frame from the drop on, a map
      of the 32-px rows whose luma differs from the full decode ('.' same,
      'o' max diff < 8, '#' larger) and the count. A row the intra stripe
      refreshed in a referenced frame reads '.' from then on; one refreshed
      in a non-referenced frame reads '.' for that frame only.

ffmpeg will not start on a VPS-led P picture by default (no IRAP): the
decode runs with -flags2 +showall -flags +output_corrupt -err_detect
ignore_err -threads 1, which emits every frame (all flagged corrupt, since
the first picture's references are synthesized -- its intra rows are still
exact). Needs numpy (NixOS: nix-shell -p python3Packages.numpy).
"""
import argparse
import os
import statistics
import subprocess
import sys

W, H = 1920, 1080
R32 = (H + 31) // 32


def nals(buf):
    """(nal_type, offset) for every annex-B NAL in buf."""
    starts = []
    i = 0
    while True:
        j = buf.find(b"\x00\x00\x01", i)
        if j < 0:
            break
        starts.append(j + 3)
        i = j + 3
    return [((buf[s] >> 1) & 0x3F, s) for s in starts if s < len(buf)]


def load(cap):
    data = open(cap + ".bin", "rb").read()
    rows = []
    for line in open(cap + ".idx"):
        f = list(map(int, line.split()))
        rec, fid, pts, sid, ln, flags, nslices, tc, off = f[:9]
        enc_us = f[10] if len(f) >= 12 else -1  # idx written before enc_us was kept
        r = dict(rec=rec, fid=fid, pts=pts, sid=sid, len=ln, flags=flags,
                 nslices=nslices, tc=tc, off=off, enc_us=enc_us, au=None,
                 nal0=-1, types=[], vcl=0, irap=False)
        if off >= 0:
            au = data[off:off + ln]
            ts = [t for t, _ in nals(au)]
            r.update(au=au, nal0=ts[0] if ts else -1, types=ts,
                     vcl=sum(1 for t in ts if t < 32),
                     irap=any(16 <= t <= 23 for t in ts))
        rows.append(r)
    return rows


def is_start(r):
    return r["nal0"] == 32 or r["irap"]


def summary(cap):
    rows = load(cap)
    print("fid sid len nal0 vcl types")
    for r in rows:
        mark = " <VPS" if r["nal0"] == 32 else (" <IRAP" if r["irap"] else "")
        print("%d %d %d %d %d %s%s" % (r["fid"], r["sid"], r["len"], r["nal0"],
                                       r["vcl"], ",".join(map(str, r["types"][:8])), mark))
    comp = [r for r in rows if r["au"] is not None]
    rs = [r for r in comp if is_start(r)]
    other = [r for r in comp if not is_start(r)]
    print("---- summary")
    print("aus=%d complete=%d refresh_start=%d" % (len(rows), len(comp), len(rs)))
    if rs:
        print("refresh-start: vcl=%s sid=%s len p50=%d enc_us p50=%d" % (
            sorted(set(r["vcl"] for r in rs)), sorted(set(r["sid"] for r in rs)),
            statistics.median(r["len"] for r in rs),
            statistics.median(r["enc_us"] for r in rs)))
        fids = [r["fid"] for r in rs]
        print("refresh-start fid spacing: %s" % [b - a for a, b in zip(fids, fids[1:])])
    for sid in (0, 1):
        o = [r for r in other if r["sid"] == sid]
        if o:
            lens = sorted(r["len"] for r in o)
            print("sid%d P: n=%d vcl=%s len p50=%d p90=%d max=%d enc_us p50=%d" % (
                sid, len(o), sorted(set(r["vcl"] for r in o)),
                statistics.median(lens), lens[int(0.9 * len(lens))], lens[-1],
                statistics.median(r["enc_us"] for r in o)))
    if rs and other:
        last = None
        bucket = {}
        for r in comp:
            if is_start(r):
                last = r["fid"]
            if last is not None:
                bucket.setdefault(r["fid"] - last, []).append(r["len"])
        print("len p50 by frames-since-refresh-start:")
        print(" ".join("%d:%d" % (d, statistics.median(v))
                       for d, v in sorted(bucket.items())[:34]))


def decode(path):
    import numpy as np
    p = subprocess.run(["ffmpeg", "-v", "error", "-threads", "1", "-flags2", "+showall",
                        "-flags", "+output_corrupt", "-err_detect", "ignore_err",
                        "-i", path, "-fps_mode", "passthrough",
                        "-f", "rawvideo", "-pix_fmt", "gray", "-"],
                       capture_output=True)
    y = np.frombuffer(p.stdout, dtype=np.uint8)
    n = len(y) // (W * H)
    return y[:n * W * H].reshape(n, H, W)


def heal(cap, gop, k, frames):
    import numpy as np
    rows = [r for r in load(cap) if r["au"] is not None]
    starts = [i for i, r in enumerate(rows) if is_start(r)]
    seq = rows[starts[gop]:starts[gop] + frames]
    cnt = 0
    drop = None
    for i, r in enumerate(seq):
        if i and r["sid"] == 0 and not is_start(r):
            cnt += 1
            if cnt == k:
                drop = i
                break
    if drop is None:
        sys.exit("no %d-th base P after refresh start %d" % (k, gop))
    full, cut = cap + ".heal_full.h265", cap + ".heal_cut.h265"
    open(full, "wb").write(b"".join(r["au"] for r in seq))
    open(cut, "wb").write(b"".join(r["au"] for i, r in enumerate(seq) if i != drop))
    yf, yc = decode(full), decode(cut)
    print("drop idx %d (fid %d, len %d); full %d frames, cut %d frames; rows of 32 px, top->bottom"
          % (drop, seq[drop]["fid"], seq[drop]["len"], len(yf), len(yc)))
    for i in range(drop + 1, min(len(seq), len(yc) + 1)):
        d = np.abs(yf[i].astype(np.int16) - yc[i - 1].astype(np.int16))
        mx = [int(d[r * 32:(r + 1) * 32].max()) for r in range(R32)]
        m = "".join("." if v == 0 else ("o" if v < 8 else "#") for v in mx)
        r = seq[i]
        tag = "VPS" if r["nal0"] == 32 else ("IRAP" if r["irap"] else "   ")
        print("%3d sid%d %s vcl%d %s %d" % (i, r["sid"], tag, r["vcl"], m, m.count("#")))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("summary")
    s.add_argument("cap")
    h = sub.add_parser("heal")
    h.add_argument("cap")
    h.add_argument("--gop", type=int, default=1, help="start at the N-th refresh start")
    h.add_argument("--k", type=int, default=5, help="drop the K-th sid-0 P after it")
    h.add_argument("--frames", type=int, default=100)
    a = ap.parse_args()
    if a.cmd == "summary":
        summary(a.cap)
    else:
        heal(a.cap, a.gop, a.k, a.frames)


if __name__ == "__main__":
    main()
