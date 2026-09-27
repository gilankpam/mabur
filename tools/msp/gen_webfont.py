#!/usr/bin/env python3
"""Convert the player's `.mfont` MSP OSD atlas into the web GS's PNG atlas.

Usage:
  gen_webfont.py [<in.mfont> [<out.png>]]

Defaults: gs/player/bundle/font_btfl.mfont -> web/ui/public/font_btfl.png.

Layout: 32 glyphs per atlas row; glyph gi = char | (page << 8) sits at
column gi % 32, row gi // 32 (the shipped 1024 glyphs of 36x54 give a
1152x1728 image, a safe canvas size on phones). 8-bit RGBA with STRAIGHT
alpha, which is what canvas drawImage expects (.mfont words are
premultiplied). web/ui/src/lib/osd.js hardcodes 36x54 and 32 per row --
web/tests/osd.test.mjs checks this file's size against those constants.

Ends with a round-trip check: re-decode the PNG with gen_font's decoder
(which re-premultiplies) and compare every pixel to the .mfont within 1 LSB
per channel. Stdlib only.
"""
import os
import struct
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gen_font import decode_rgba_png  # noqa: E402

ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
MAGIC = 0x544E464D
COLS = 32


def read_mfont(path):
    d = open(path, "rb").read()
    magic, ver, gw, gh, n = struct.unpack("<5I", d[:20])
    assert magic == MAGIC and ver == 1, "not an .mfont v1: %s" % path
    words = struct.unpack("<%dI" % (n * gw * gh), d[32:32 + 4 * n * gw * gh])
    return gw, gh, n, words


def unpremult(w):
    a = w >> 24
    if a == 0:
        return (0, 0, 0, 0)
    ch = [(w >> 16) & 255, (w >> 8) & 255, w & 255]
    return tuple(min(255, (c * 255 + a // 2) // a) for c in ch) + (a,)


def chunk(typ, data):
    return (struct.pack(">I", len(data)) + typ + data
            + struct.pack(">I", zlib.crc32(typ + data) & 0xFFFFFFFF))


def write_png(path, w, h, img):
    stride = w * 4
    raw = b"".join(b"\x00" + bytes(img[y * stride:(y + 1) * stride]) for y in range(h))
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(chunk(b"IEND", b""))


def main(argv):
    src = argv[1] if len(argv) > 1 else os.path.join(ROOT, "gs/player/bundle/font_btfl.mfont")
    dst = argv[2] if len(argv) > 2 else os.path.join(ROOT, "web/ui/public/font_btfl.png")
    gw, gh, n, words = read_mfont(src)
    w, h = COLS * gw, ((n + COLS - 1) // COLS) * gh
    img = bytearray(w * h * 4)
    for gi in range(n):
        ox, oy, base = (gi % COLS) * gw, (gi // COLS) * gh, gi * gw * gh
        for y in range(gh):
            o = ((oy + y) * w + ox) * 4
            for x in range(gw):
                img[o + 4 * x:o + 4 * x + 4] = bytes(unpremult(words[base + y * gw + x]))
    write_png(dst, w, h, img)

    w2, h2, px = decode_rgba_png(dst)
    assert (w2, h2) == (w, h), "round trip: size %dx%d != %dx%d" % (w2, h2, w, h)
    bad = 0
    for gi in range(n):
        ox, oy, base = (gi % COLS) * gw, (gi // COLS) * gh, gi * gw * gh
        for y in range(gh):
            for x in range(gw):
                a, b = words[base + y * gw + x], px[(oy + y) * w + ox + x]
                if any(abs(((a >> s) & 255) - ((b >> s) & 255)) > 1 for s in (0, 8, 16, 24)):
                    bad += 1
    assert bad == 0, "round trip: %d pixels differ by more than 1 LSB" % bad
    print("wrote %s: %dx%d, %d glyphs of %dx%d, %d bytes; round-trip OK"
          % (dst, w, h, n, gw, gh, os.path.getsize(dst)))


if __name__ == "__main__":
    main(sys.argv)
