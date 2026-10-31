#!/usr/bin/env python3
"""Convert `.mfont` MSP OSD fonts into the web GS's PNG atlases and font list.

Usage:
  gen_webfont.py <file-or-dir>... [--out web/ui/public/fonts]

Every font goes through the same path, Betaflight included:

  gen_webfont.py gs/player/bundle/font_btfl.mfont   # rebuilds fonts/btfl.png

Each `<name>.mfont` becomes `<out>/<id>.png` and an entry in `<out>/index.json`,
the list the page reads its fonts from. The id is the file name lower-cased
(anything outside a-z 0-9 _ - becomes `_`) with a leading `font_` stripped, so
`font_btfl.mfont` gives `btfl`, the id saved configs already use. A directory
argument converts every `*.mfont` in it; at least one input is required.

index.json is merged, not rewritten: fonts already listed that are not among
the inputs are kept as they are (their ids are printed), a font passed again is
regenerated under the same id and keeps its listed name, and new fonts are
appended (a new `btfl` goes first, named "Betaflight"). To remove a font,
delete its entry and its PNG by hand.

Layout: 32 glyphs per atlas row; glyph gi = char | (page << 8) sits at
column gi % 32, row gi // 32 (the shipped 1024 glyphs of 36x54 give a
1152x1728 image, a safe canvas size on phones). 8-bit RGBA with STRAIGHT
alpha, which is what canvas drawImage expects (.mfont words are
premultiplied). web/ui/src/lib/osd.js hardcodes 36x54 and 32 per row --
web/tests/osd.test.mjs checks the Betaflight atlas's size against those
constants.

The `.mfont` spec says premultiplied alpha, but some tools write straight
alpha. A pixel with a colour channel above its alpha cannot be premultiplied,
so each font is checked: if any pixel violates that, the colours are taken as
straight (and a message is printed); otherwise they are un-premultiplied.

Every font ends with a round-trip check: re-decode the PNG with gen_font's
decoder (which re-premultiplies) and compare every pixel to the .mfont within
1 LSB per channel (a straight-alpha font is compared as its premultiplied
form). Stdlib only.
"""
import glob
import json
import os
import re
import struct
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gen_font import decode_rgba_png  # noqa: E402

ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
MAGIC = 0x544E464D
COLS = 32
USAGE = "usage: gen_webfont.py <file-or-dir>... [--out web/ui/public/fonts]"


def read_mfont(path):
    d = open(path, "rb").read()
    magic, ver, gw, gh, n = struct.unpack("<5I", d[:20])
    assert magic == MAGIC and ver == 1, "not an .mfont v1: %s" % path
    words = struct.unpack("<%dI" % (n * gw * gh), d[32:32 + 4 * n * gw * gh])
    return gw, gh, n, words


def is_straight(words):
    for v in words:
        a = v >> 24
        if (v >> 16) & 255 > a or (v >> 8) & 255 > a or v & 255 > a:
            return True
    return False


def unpremult(w):
    a = w >> 24
    if a == 0:
        return (0, 0, 0, 0)
    ch = [(w >> 16) & 255, (w >> 8) & 255, w & 255]
    return tuple(min(255, (c * 255 + a // 2) // a) for c in ch) + (a,)


def to_rgba(v, straight):
    a = v >> 24
    if a == 0:
        return (0, 0, 0, 0)
    if straight:
        return ((v >> 16) & 255, (v >> 8) & 255, v & 255, a)
    return unpremult(v)


def premult_word(v):
    """A straight-alpha word as the premultiplied word the PNG decoder returns."""
    a = v >> 24
    return (a << 24) | sum((((((v >> s) & 255) * a + 127) // 255) << s) for s in (16, 8, 0))


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


def slugify(name):
    s = re.sub(r"[^a-z0-9_-]+", "_", name.lower()).strip("_")
    return (s or "font")[:32]


def font_id(base):
    s = slugify(base)
    t = s[len("font_"):] if s.startswith("font_") else s
    return t or s


def display_name(base, fid):
    if fid == "btfl":
        return "Betaflight"
    return (base[len("font_"):] if base.lower().startswith("font_") and len(base) > 5 else base).replace("_", " ")


def convert(src, dst):
    gw, gh, n, words = read_mfont(src)
    if (gw, gh, n) != (36, 54, 1024):
        sys.exit("%s: %dx%d x%d glyphs; the web page needs 36x54 x1024" % (src, gw, gh, n))
    straight = is_straight(words)
    w, h = COLS * gw, ((n + COLS - 1) // COLS) * gh
    img = bytearray(w * h * 4)
    for gi in range(n):
        ox, oy, base = (gi % COLS) * gw, (gi // COLS) * gh, gi * gw * gh
        for y in range(gh):
            o = ((oy + y) * w + ox) * 4
            for x in range(gw):
                img[o + 4 * x:o + 4 * x + 4] = bytes(to_rgba(words[base + y * gw + x], straight))
    write_png(dst, w, h, img)

    w2, h2, px = decode_rgba_png(dst)
    assert (w2, h2) == (w, h), "round trip: size %dx%d != %dx%d" % (w2, h2, w, h)
    bad = 0
    for gi in range(n):
        ox, oy, base = (gi % COLS) * gw, (gi // COLS) * gh, gi * gw * gh
        for y in range(gh):
            for x in range(gw):
                a = words[base + y * gw + x]
                if straight:
                    a = premult_word(a)
                b = px[(oy + y) * w + ox + x]
                if any(abs(((a >> s) & 255) - ((b >> s) & 255)) > 1 for s in (0, 8, 16, 24)):
                    bad += 1
    assert bad == 0, "round trip: %d pixels differ by more than 1 LSB" % bad
    return straight


def load_manifest(path):
    if not os.path.exists(path):
        return []
    try:
        with open(path) as f:
            data = json.load(f)
    except ValueError as e:
        sys.exit("%s is not valid JSON (%s); fix or remove it first" % (path, e))
    if not isinstance(data, list):
        sys.exit("%s is not a JSON list; fix or remove it first" % path)
    return data


def merge(existing, entries):
    """Same-id entries are replaced in place (keeping the listed name); new ones are
    appended, except a new `btfl`, which goes first."""
    merged = list(existing)
    pos = {e["id"]: i for i, e in enumerate(merged) if isinstance(e, dict) and "id" in e}
    for e in entries:
        if e["id"] in pos:
            old = merged[pos[e["id"]]]
            merged[pos[e["id"]]] = dict(e, name=old.get("name", e["name"]))
        elif e["id"] == "btfl":
            merged.insert(0, e)
            pos = {x["id"]: i for i, x in enumerate(merged) if isinstance(x, dict) and "id" in x}
        else:
            merged.append(e)
            pos[e["id"]] = len(merged) - 1
    return merged


def main(argv):
    args, out = [], os.path.join(ROOT, "web/ui/public/fonts")
    it = iter(argv[1:])
    for a in it:
        if a == "--out":
            out = next(it, None)
            if out is None:
                sys.exit("--out needs a directory\n" + USAGE)
        else:
            args.append(a)
    if not args:
        sys.exit("no inputs given\n" + USAGE)
    files = []
    for a in args:
        files += sorted(glob.glob(os.path.join(a, "*.mfont"))) if os.path.isdir(a) else [a]
    if not files:
        sys.exit("no .mfont files found")
    os.makedirs(out, exist_ok=True)
    manifest = os.path.join(out, "index.json")
    existing = load_manifest(manifest)   # fail on a broken manifest before writing any PNG
    entries, used = [], set()
    for src in files:
        base = os.path.splitext(os.path.basename(src))[0]
        fid, k = font_id(base), 2
        while fid in used:
            fid, k = "%s_%d" % (font_id(base)[:28], k), k + 1
        used.add(fid)
        straight = convert(src, os.path.join(out, fid + ".png"))
        if straight:
            print("%s: colours stored as straight alpha (taken as-is)" % base)
        print("wrote %s.png (round-trip OK)" % fid)
        entries.append({"id": fid, "name": display_name(base, fid), "file": "fonts/%s.png" % fid})
    merged = merge(existing, entries)
    passed = {e["id"] for e in entries}
    kept = [e["id"] for e in existing if isinstance(e, dict) and "id" in e and e["id"] not in passed]
    with open(manifest, "w") as f:
        json.dump(merged, f, indent=2)
        f.write("\n")
    print("wrote index.json (%d fonts%s)" % (len(merged), "; kept " + ", ".join(kept) if kept else ""))


if __name__ == "__main__":
    main(sys.argv)
