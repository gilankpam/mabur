#!/usr/bin/env python3
"""Round-trips tools/msp/gen_font.py: PNG (both encodings) -> .mfont."""
import os, struct, subprocess, sys, tempfile, zlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEN = os.path.join(ROOT, "tools", "msp", "gen_font.py")
MAGIC = 0x544E464D


def chunk(typ, data):
    c = typ + data
    return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c))


def write_rgba_png(path, w, h, px):
    """px: list of (r,g,b,a) tuples, row-major, len == w*h."""
    raw = b""
    for y in range(h):
        raw += b"\x00" + bytes(v for x in range(w) for v in px[y * w + x])
    body = (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw))
            + chunk(b"IEND", b""))
    open(path, "wb").write(body)


def write_indexed_png(path, w, h, idx):
    """idx: list of palette indices 0..1, row-major. Palette: 0=transparent, 1=opaque red."""
    raw = b""
    for y in range(h):
        line = bytearray()
        for x in range(0, w, 2):
            hi = idx[y * w + x] & 0xF
            lo = idx[y * w + x + 1] & 0xF if x + 1 < w else 0
            line.append((hi << 4) | lo)
        raw += b"\x00" + bytes(line)
    body = (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 4, 3, 0, 0, 0))
            + chunk(b"PLTE", bytes([0, 0, 0, 255, 0, 0]))
            + chunk(b"tRNS", bytes([0, 255]))
            + chunk(b"IDAT", zlib.compress(raw))
            + chunk(b"IEND", b""))
    open(path, "wb").write(body)


def read_mfont(path):
    d = open(path, "rb").read()
    magic, ver, gw, gh, n = struct.unpack("<5I", d[:20])
    assert magic == MAGIC, hex(magic)
    assert ver == 1, ver
    assert len(d) == 32 + gw * gh * n * 4, (len(d), gw, gh, n)
    return gw, gh, n, d[32:]


def glyph_px(pixels, gw, gh, gi, x, y):
    off = (gi * gw * gh + y * gw + x) * 4
    return struct.unpack("<I", pixels[off:off + 4])[0]


def run(*args):
    subprocess.run([sys.executable, GEN, *args], check=True)


def main():
    tmp = tempfile.mkdtemp()
    # Atlas layout: width = glyph_w * 4 pages, height = glyph_h * 256 chars.
    gw, gh = 2, 3
    W, H = gw * 4, gh * 256

    # --- 8-bit RGBA source: glyph (char=1, page=0) fully opaque green.
    px = [(0, 0, 0, 0)] * (W * H)
    for y in range(gh):
        for x in range(gw):
            px[(1 * gh + y) * W + (0 * gw + x)] = (0, 255, 0, 255)
    rgba = os.path.join(tmp, "rgba.png")
    write_rgba_png(rgba, W, H, px)
    out = os.path.join(tmp, "rgba.mfont")
    run(rgba, out)
    ogw, ogh, n, pixels = read_mfont(out)
    assert (ogw, ogh, n) == (gw, gh, 1024), (ogw, ogh, n)
    assert glyph_px(pixels, gw, gh, 1, 0, 0) == 0xFF00FF00
    assert glyph_px(pixels, gw, gh, 2, 0, 0) == 0x00000000

    # --- 4-bit indexed source: same glyph, opaque red.
    idx = [0] * (W * H)
    for y in range(gh):
        for x in range(gw):
            idx[(1 * gh + y) * W + (0 * gw + x)] = 1
    ipng = os.path.join(tmp, "idx.png")
    write_indexed_png(ipng, W, H, idx)
    out2 = os.path.join(tmp, "idx.mfont")
    run(ipng, out2)
    ogw, ogh, n, pixels = read_mfont(out2)
    assert (ogw, ogh, n) == (gw, gh, 1024)
    assert glyph_px(pixels, gw, gh, 1, 0, 0) == 0xFFFF0000

    # --- --glyph-size downscale changes the header dims and the payload size.
    out3 = os.path.join(tmp, "small.mfont")
    run(rgba, out3, "--glyph-size", "1x1")
    ogw, ogh, n, pixels = read_mfont(out3)
    assert (ogw, ogh, n) == (1, 1, 1024)
    assert glyph_px(pixels, 1, 1, 1, 0, 0) == 0xFF00FF00

    # --- synthetic mode needs no input PNG.
    out4 = os.path.join(tmp, "syn.mfont")
    run("--synthetic", out4, "--glyph-size", "4x6")
    ogw, ogh, n, _ = read_mfont(out4)
    assert (ogw, ogh, n) == (4, 6, 1024)

    print("OK test_gen_font")


main()
