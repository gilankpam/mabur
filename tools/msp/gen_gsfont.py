#!/usr/bin/env python3
"""Bake a monospace TTF into a `.gfont` alpha-mask atlas for the GS
link-status OSD (docs/superpowers/specs/2026-08-04-gs-osd-essential-design.md).

Usage:
  gen_gsfont.py <font.ttf> <out.gfont> [--sizes 19,21,22,24,26,34,38,56]
  gen_gsfont.py --synthetic <out.gfont> --sizes 8,12

Unlike the MSP `.mfont` (pre-coloured ARGB glyphs), a `.gfont` glyph stores
TWO 8-bit channels per pixel -- coverage and shadow -- so ONE atlas per size
serves every design token colour, and the handoff's universal text shadow
(`0 1px 3px rgba(0,0,0,0.92)`) is baked at generation time rather than
blurred on maburplay's 2 ms main loop.

`.gfont` format, all little-endian:

  header, 32 B: {u32 magic 'GFNT' = 0x544E4647, u32 version = 1,
                 u32 n_sizes, u32 reserved[5] = 0}
  size directory, n_sizes x 36 B, ascending by px, right after the header:
                {u32 px, u32 glyph_w, u32 glyph_h, u32 advance_x,
                 u32 baseline, u32 n_glyphs, u32 pad = 0, u64 offset}
  glyph block at `offset`:
                u32 codepoints[n_glyphs]  (ascending -- GsFont bisects)
                u8  pixels[n_glyphs * glyph_w * glyph_h * 2]
                    glyph-major, row-major, byte0 = coverage, byte1 = shadow

glyph_w/glyph_h INCLUDE a 4 px pad on every side so the shadow has room.

Regenerating the SHIPPED asset (`gs/player/bundle/gs_osd.gfont`) --
JetBrains Mono **Medium** (the design's weight 500), all 30 sizes:

  nix-shell -p jetbrains-mono python3Packages.freetype-py --run \\
    "tools/msp/gen_gsfont.py \\
       \\$(nix-build --no-out-link -E 'with import <nixpkgs> {}; jetbrains-mono'\\
         )/share/fonts/truetype/JetBrainsMono-Medium.ttf \\
       gs/player/bundle/gs_osd.gfont \\
       --sizes 13,14,15,16,17,19,21,22,23,24,25,26,28,29,32,34,35,37,38,42,\\
44,45,48,51,52,56,68,75,76,112"

Expect **13,219,312 bytes** and ~5 minutes of pure-Python box blur (this is
a build-time tool; it is not on any hot path). A materially different byte
count means something changed -- a different weight or family, a different
size list, a different subset -- and should be explained before committing.

The 30 sizes are the eight design sizes at x2/3, x1, x4/3 and x2, which is
what makes every type role land on an EXACT atlas size at 720p, 1080p,
1440p and 2160p with no runtime scaling. The list is duplicated (with its
derivation) in `tests/gs_scaled_sizes.h`, and `tests/test_gs_asset.cpp`
asserts the committed asset bakes exactly it -- change one, change both.

freetype and the font itself are DEV-HOST dependencies only: the generated
asset is committed, so a deploy needs no font toolchain. `--synthetic`
needs stdlib alone and is what the C++ tests use.
"""
import struct, sys

MAGIC = 0x544E4647
VERSION = 1
PAD = 4                     # shadow room on every side
SHADOW_ALPHA = 0.92
SHADOW_DY = 1
DEFAULT_SIZES = [19, 21, 22, 24, 26, 34, 38, 56]

# Printable ASCII plus the five non-ASCII glyphs the design uses:
# U+2014 EM DASH (the "——" pair that renders a never-received value --
# gs_overlay.h's kEmDashPair), U+2212 MINUS SIGN (a true minus, not a
# hyphen), U+2192 RIGHTWARDS ARROW, U+25CF BLACK CIRCLE (the recording dot)
# and U+25CB WHITE CIRCLE -- the latter no longer drawn by anything since
# the armed REC field went blank, but kept baked so the committed asset
# needn't be regenerated for a glyph that costs a few hundred bytes.
#
# U+2014 was missing from this list until 2026-08-04 and the omission was
# invisible: draw_text advances the pen for a codepoint the atlas lacks, so
# "——" rendered as correctly-sized EMPTY SPACE. Every test passed, because
# they assert the STRING a field formats, not the ink it puts down. A card
# that had never been heard would have shown a blank where the design calls
# for a dash pair -- absence reading as nothing rather than as absence,
# which is precisely what the design's "never substitute zero" rule exists
# to prevent. Any new glyph gs_overlay.h names must be added here too.
SUBSET = [c for c in range(0x20, 0x7F)] + [0x2014, 0x2212, 0x2192, 0x25CF, 0x25CB]


def box_blur(buf, w, h, radius):
    """Separable box blur, in place on a copy. Three passes at radius 1
    approximate a Gaussian closely enough for a drop shadow."""
    out = list(buf)
    tmp = [0] * (w * h)
    for y in range(h):
        row = y * w
        for x in range(w):
            acc = n = 0
            for dx in range(-radius, radius + 1):
                xx = x + dx
                if 0 <= xx < w:
                    acc += out[row + xx]; n += 1
            tmp[row + x] = acc // n
    for x in range(w):
        for y in range(h):
            acc = n = 0
            for dy in range(-radius, radius + 1):
                yy = y + dy
                if 0 <= yy < h:
                    acc += tmp[yy * w + x]; n += 1
            out[y * w + x] = acc // n
    return out


def make_shadow(cov, w, h):
    """Blur, offset +SHADOW_DY in y, scale by SHADOW_ALPHA."""
    b = cov
    for _ in range(3):
        b = box_blur(b, w, h, 1)
    out = [0] * (w * h)
    for y in range(h):
        sy = y - SHADOW_DY
        if sy < 0 or sy >= h:
            continue
        for x in range(w):
            out[y * w + x] = int(b[sy * w + x] * SHADOW_ALPHA)
    return out


def render_ttf(ttf_path, px):
    """-> (glyph_w, glyph_h, advance_x, baseline, {cp: coverage list})"""
    import freetype
    face = freetype.Face(ttf_path)
    face.set_pixel_sizes(0, px)
    advance = face.size.max_advance >> 6
    ascender = face.size.ascender >> 6
    descender = -(face.size.descender >> 6)
    gw = advance + 2 * PAD
    gh = ascender + descender + 2 * PAD
    baseline = PAD + ascender
    glyphs = {}
    for cp in SUBSET:
        face.load_char(chr(cp), freetype.FT_LOAD_RENDER)
        bmp = face.glyph.bitmap
        cov = [0] * (gw * gh)
        ox = PAD + face.glyph.bitmap_left
        oy = baseline - face.glyph.bitmap_top
        for r in range(bmp.rows):
            yy = oy + r
            if yy < 0 or yy >= gh:
                continue
            for c in range(bmp.width):
                xx = ox + c
                if 0 <= xx < gw:
                    cov[yy * gw + xx] = bmp.buffer[r * bmp.pitch + c]
        glyphs[cp] = cov
    return gw, gh, advance, baseline, glyphs


def synthetic(px):
    """Deterministic stdlib-only atlas: every glyph is a filled box inset by
    PAD, with a codepoint-dependent notch so glyphs are distinguishable."""
    advance = max(2, px * 3 // 5)
    ascender = px
    descender = max(1, px // 4)
    gw = advance + 2 * PAD
    gh = ascender + descender + 2 * PAD
    baseline = PAD + ascender
    glyphs = {}
    for cp in SUBSET:
        cov = [0] * (gw * gh)
        for y in range(PAD, gh - PAD):
            for x in range(PAD, gw - PAD):
                on = ((x + y + cp) % 3) != 0
                cov[y * gw + x] = 255 if on else 0
        glyphs[cp] = cov
    return gw, gh, advance, baseline, glyphs


def build(sizes, render):
    """render(px) -> (gw, gh, advance, baseline, {cp: coverage})"""
    blocks, dirents = [], []
    hdr_size = struct.calcsize("<3I5I")  # magic, version, n_sizes, reserved[5]
    dirent_size = struct.calcsize("<7IQ")
    offset = hdr_size + dirent_size * len(sizes)
    for px in sizes:
        gw, gh, advance, baseline, glyphs = render(px)
        cps = sorted(glyphs)
        body = bytearray()
        for cp in cps:
            body += struct.pack("<I", cp)
        for cp in cps:
            cov = glyphs[cp]
            sha = make_shadow(cov, gw, gh)
            for i in range(gw * gh):
                body.append(cov[i]); body.append(sha[i])
        dirents.append(struct.pack("<7IQ", px, gw, gh, advance, baseline,
                                   len(cps), 0, offset))
        blocks.append(bytes(body))
        offset += len(body)
    out = bytearray(struct.pack("<3I", MAGIC, VERSION, len(sizes)))
    reserved_size = hdr_size - struct.calcsize("<3I")
    out += b"\x00" * reserved_size          # reserved[5]
    for d in dirents:
        out += d
    for b in blocks:
        out += b
    return bytes(out)


def main(argv):
    args = list(argv[1:])
    sizes = DEFAULT_SIZES
    if "--sizes" in args:
        i = args.index("--sizes")
        sizes = sorted(int(v) for v in args[i + 1].split(","))
        del args[i:i + 2]

    if args and args[0] == "--synthetic":
        if len(args) != 2:
            print(__doc__); return 2
        data = build(sizes, synthetic)
        open(args[1], "wb").write(data)
    else:
        if len(args) != 2:
            print(__doc__); return 2
        ttf, out = args
        data = build(sizes, lambda px: render_ttf(ttf, px))
        open(out, "wb").write(data)
        print("wrote %s: %d sizes, %d bytes" % (out, len(sizes), len(data)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
