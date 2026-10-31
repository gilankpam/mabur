#!/usr/bin/env python3
"""Convert `.mfont` OSD fonts into the web GS's selectable fonts.

Usage:
  gen_webfonts.py <dir-or-files...> [--out web/ui/public/fonts]

At least one input is required: a `.mfont` file, or a folder whose `*.mfont`
files are all converted. Each `<name>.mfont` becomes `<out>/<slug>.png` (same
36x54, 32-per-row atlas as gen_webfont.py), and `<out>/index.json` lists them
for the "OSD font" picker in the Config form's Display group. The shipped
Betaflight atlas (font_btfl.png) is always offered by the page and is not
listed here.

index.json is merged, not rewritten: fonts already listed that are not among
the inputs are kept as they are (their ids are printed), a font that is passed
again is regenerated under the same id and keeps its listed name, and new
fonts are appended. To remove a font, delete its entry from index.json and its
PNG by hand.

The `.mfont` spec says premultiplied alpha, but some tools write straight
alpha. A pixel with a colour channel above its alpha cannot be premultiplied,
so each font is checked: if any pixel violates that, the colours are taken as
straight (and a warning is printed); otherwise they are un-premultiplied as
gen_webfont.py does. Stdlib only.
"""
import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_webfont as g  # noqa: E402

ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
COLS = g.COLS
USAGE = "usage: gen_webfonts.py <dir-or-files...> [--out web/ui/public/fonts]"


def is_straight(words):
    for v in words:
        a = v >> 24
        if (v >> 16) & 255 > a or (v >> 8) & 255 > a or v & 255 > a:
            return True
    return False


def to_rgba(v, straight):
    a = v >> 24
    if a == 0:
        return (0, 0, 0, 0)
    if straight:
        return ((v >> 16) & 255, (v >> 8) & 255, v & 255, a)
    return g.unpremult(v)


def slugify(name):
    s = re.sub(r"[^a-z0-9_-]+", "_", name.lower()).strip("_")
    return (s or "font")[:32]


def convert(src, dst):
    gw, gh, n, words = g.read_mfont(src)
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
    g.write_png(dst, w, h, img)
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
    """New entries replace same-id ones in place (keeping the listed name); others are appended."""
    merged = list(existing)
    pos = {e["id"]: i for i, e in enumerate(merged) if isinstance(e, dict) and "id" in e}
    for e in entries:
        if e["id"] in pos:
            old = merged[pos[e["id"]]]
            merged[pos[e["id"]]] = dict(e, name=old.get("name", e["name"]))
        else:
            merged.append(e)
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
    entries, used = [], {"btfl"}
    for src in files:
        base = os.path.splitext(os.path.basename(src))[0]
        slug, k = slugify(base), 2
        while slug in used:
            slug, k = "%s_%d" % (slugify(base)[:28], k), k + 1
        used.add(slug)
        straight = convert(src, os.path.join(out, slug + ".png"))
        if straight:
            print("%s: colours stored as straight alpha (taken as-is)" % base)
        print("wrote %s.png" % slug)
        entries.append({"id": slug, "name": base.replace("_", " "), "file": "fonts/%s.png" % slug})
    merged = merge(existing, entries)
    kept = [e["id"] for e in existing
            if isinstance(e, dict) and "id" in e and e["id"] not in {n["id"] for n in entries}]
    with open(manifest, "w") as f:
        json.dump(merged, f, indent=2)
        f.write("\n")
    print("wrote index.json (%d fonts%s)" % (len(merged), "; kept " + ", ".join(kept) if kept else ""))


if __name__ == "__main__":
    main(sys.argv)
