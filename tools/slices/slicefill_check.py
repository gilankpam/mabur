#!/usr/bin/env python3
"""ffmpeg oracle for slice-salvage fills (spec 2026-10-10-h265-slices §5.7).

For each picture/mode: rewrite one picture with build/slicefill, decode the
original and the rewritten stream with ffmpeg (-err_detect explode), and
check (1) the rewritten stream decodes with no error and the same frame
count, (2) every frame before the picture is bit-exact, (3) the picture is
bit-exact outside the filled band (+-8 rows for deblocking at the edges),
(4) the filled band is a plausible motion-compensated copy (PSNR >= 40 dB
against the intact decode on this static bench scene; calibrated on cap4,
see BAND_MIN_DB).

usage: slicefill_check.py --cli build/tests/slicefill --in CAP4.h265
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile

W, H = 1920, 1080
# Calibrated on cap4: planted CABAC bugs passed single modes at 31.7-34.2 dB,
# correct fills measure >= 42.2 dB. Recalibrate for a different capture.
BAND_MIN_DB = 40.0


def decode_y(path):
    r = subprocess.run(["ffmpeg", "-v", "error", "-err_detect", "explode", "-i", path,
                        "-f", "rawvideo", "-pix_fmt", "gray", "-"], capture_output=True)
    frames = [r.stdout[i:i + W * H] for i in range(0, len(r.stdout), W * H)]
    return r.returncode, r.stderr.decode().strip(), frames


def psnr(a, b):
    mse = sum((x - y) ** 2 for x, y in zip(a[::7], b[::7])) / max(1, len(a[::7]))
    return 99.0 if mse == 0 else 10 * math.log10(255 * 255 / mse)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cli", required=True)
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--pictures", default="40,101,302")
    a = ap.parse_args()
    rc, err, ref = decode_y(a.inp)
    if rc or err:
        sys.exit(f"original does not decode cleanly: {err}")
    fails = 0
    with tempfile.TemporaryDirectory() as d:
        for pic in [int(x) for x in a.pictures.split(",")]:
            for mode in ("first", "middle", "last", "tail2"):
                out = os.path.join(d, f"{pic}_{mode}.h265")
                r = subprocess.run([a.cli, a.inp, out, str(pic), mode], capture_output=True, text=True)
                line = r.stdout.strip().split()
                if r.returncode or len(line) != 4:
                    print(f"FAIL {pic} {mode}: slicefill: {r.stderr.strip()} {r.stdout.strip()}")
                    fails += 1
                    continue
                y0, y1 = int(line[2]), int(line[3])
                rc, err, got = decode_y(out)
                ok = rc == 0 and not err and len(got) == len(ref)
                ok = ok and all(got[i] == ref[i] for i in range(pic))
                lo, hi = max(0, y0 - 8), min(H, y1 + 8)
                g, f = got[pic], ref[pic]
                ok = ok and g[:lo * W] == f[:lo * W] and g[hi * W:] == f[hi * W:]
                band = psnr(g[y0 * W:y1 * W], f[y0 * W:y1 * W])
                ok = ok and band >= BAND_MIN_DB
                print(f"{'PASS' if ok else 'FAIL'} picture {pic} {mode}: rows {y0}-{y1} band {band:.1f} dB"
                      + (f" ffmpeg: {err}" if err else ""))
                fails += not ok
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
