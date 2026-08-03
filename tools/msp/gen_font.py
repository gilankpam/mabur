#!/usr/bin/env python3
"""Decode an MSP OSD font atlas PNG (4-bit indexed or 8-bit RGBA) into a
`.mfont` binary the player mmaps at runtime.

Usage:
  gen_font.py <font.png> <out.mfont> [--glyph-size WxH]
  gen_font.py --synthetic <out.mfont> --glyph-size WxH

`.mfont` format: 32-byte little-endian header
`{u32 magic 'MFNT' = 0x544E464D, u32 version = 1, u32 glyph_w, u32 glyph_h,
u32 n_glyphs, u32 reserved[3] = 0}` followed by `n_glyphs * glyph_w * glyph_h`
little-endian `0xAARRGGBB` words, glyph-major, row-major within a glyph,
premultiplied. Glyph index `gi = char | (page << 8)`.

Atlas layout: width = glyph_w * 4 pages, height = glyph_h * 256 chars.
Glyph (char,page) occupies atlas cols [page*gw, page*gw+gw), rows
[char*gh, char*gh+gh). Stdlib only (no Pillow, no numpy).
"""
import sys, zlib, struct

def decode_indexed_png(path):
    d = open(path, "rb").read()
    assert d[:8] == b"\x89PNG\r\n\x1a\n", "not a PNG"
    pos = 8; W = H = bd = ct = 0; plte = b""; trns = b""; idat = b""
    while pos < len(d):
        ln = struct.unpack(">I", d[pos:pos+4])[0]; typ = d[pos+4:pos+8]
        data = d[pos+8:pos+8+ln]; pos += 12 + ln
        if typ == b"IHDR": W, H, bd, ct = struct.unpack(">IIBB", data[:10])
        elif typ == b"PLTE": plte = data
        elif typ == b"tRNS": trns = data
        elif typ == b"IDAT": idat += data
        elif typ == b"IEND": break
    assert ct == 3 and bd == 4, "expected 4-bit indexed PNG (ct=3 bd=4), got ct=%d bd=%d" % (ct, bd)
    raw = zlib.decompress(idat)
    sb = (W * bd + 7) // 8  # scanline bytes
    def paeth(a, b, c):
        p = a + b - c; pa = abs(p-a); pb = abs(p-b); pc = abs(p-c)
        return a if pa <= pb and pa <= pc else (b if pb <= pc else c)
    out = bytearray(); prev = bytearray(sb); o = 0
    for _ in range(H):
        ft = raw[o]; o += 1; line = bytearray(raw[o:o+sb]); o += sb
        for i in range(sb):
            a = line[i-1] if i >= 1 else 0; b = prev[i]; c = prev[i-1] if i >= 1 else 0
            if ft == 1: line[i] = (line[i] + a) & 255
            elif ft == 2: line[i] = (line[i] + b) & 255
            elif ft == 3: line[i] = (line[i] + ((a + b) >> 1)) & 255
            elif ft == 4: line[i] = (line[i] + paeth(a, b, c)) & 255
        out += line; prev = line
    def index(x, y):
        byte = out[y*sb + x//2]
        return (byte >> 4) if x % 2 == 0 else (byte & 0xF)
    def rgba(i):
        r, g, b = plte[i*3], plte[i*3+1], plte[i*3+2]
        a = trns[i] if i < len(trns) else 255
        return r, g, b, a
    # Palette entries are already opaque or fully transparent, so
    # premultiplication is a no-op here; still route through it for
    # consistency with decode_rgba_png's contract.
    pixels = []
    for y in range(H):
        for x in range(W):
            pixels.append(premult_argb(*rgba(index(x, y))))
    return W, H, pixels

def decode_rgba_png(path):
    """Decodes an 8-bit RGBA PNG (ct=6 bd=8) to (W, H, [ARGB32 ints])."""
    d = open(path, "rb").read()
    assert d[:8] == b"\x89PNG\r\n\x1a\n", "not a PNG"
    pos = 8; W = H = bd = ct = 0; idat = b""
    while pos < len(d):
        ln = struct.unpack(">I", d[pos:pos+4])[0]; typ = d[pos+4:pos+8]
        data = d[pos+8:pos+8+ln]; pos += 12 + ln
        if typ == b"IHDR": W, H, bd, ct = struct.unpack(">IIBB", data[:10])
        elif typ == b"IDAT": idat += data
        elif typ == b"IEND": break
    assert ct == 6 and bd == 8, "expected 8-bit RGBA PNG, got ct=%d bd=%d" % (ct, bd)
    raw = zlib.decompress(idat)
    sb = W * 4
    def paeth(a, b, c):
        p = a + b - c; pa = abs(p-a); pb = abs(p-b); pc = abs(p-c)
        return a if pa <= pb and pa <= pc else (b if pb <= pc else c)
    out = []; prev = bytearray(sb); o = 0
    for _ in range(H):
        ft = raw[o]; o += 1; line = bytearray(raw[o:o+sb]); o += sb
        for i in range(sb):
            a = line[i-4] if i >= 4 else 0; b = prev[i]; c = prev[i-4] if i >= 4 else 0
            if ft == 1: line[i] = (line[i] + a) & 255
            elif ft == 2: line[i] = (line[i] + b) & 255
            elif ft == 3: line[i] = (line[i] + ((a + b) >> 1)) & 255
            elif ft == 4: line[i] = (line[i] + paeth(a, b, c)) & 255
        for x in range(W):
            r, g, b_, al = line[x*4], line[x*4+1], line[x*4+2], line[x*4+3]
            # premultiply
            r = r * al // 255; g = g * al // 255; b_ = b_ * al // 255
            out.append((al << 24) | (r << 16) | (g << 8) | b_)
        prev = line
    return W, H, out

def premult_argb(r, g, b, a):
    return (a << 24) | (((r*a)//255) << 16) | (((g*a)//255) << 8) | ((b*a)//255)

def box_resample(glyph, gw, gh, tw, th):
    """Area-average a premultiplied ARGB32 glyph to tw x th."""
    out = []
    for y in range(th):
        y0 = y * gh // th; y1 = max(y0 + 1, (y + 1) * gh // th)
        for x in range(tw):
            x0 = x * gw // tw; x1 = max(x0 + 1, (x + 1) * gw // tw)
            a = r = g = b = n = 0
            for sy in range(y0, y1):
                for sx in range(x0, x1):
                    p = glyph[sy * gw + sx]
                    a += p >> 24; r += (p >> 16) & 255; g += (p >> 8) & 255; b += p & 255; n += 1
            out.append(((a//n) << 24) | ((r//n) << 16) | ((g//n) << 8) | (b//n))
    return out


def write_mfont(path, gw, gh, glyphs):
    """glyphs: list of 1024 lists of gw*gh ARGB32 ints."""
    with open(path, "wb") as f:
        f.write(struct.pack("<8I", 0x544E464D, 1, gw, gh, len(glyphs), 0, 0, 0))
        for g in glyphs:
            f.write(struct.pack("<%dI" % len(g), *g))


def atlas_to_glyphs(W, H, px, gw, gh):
    """Atlas layout: width = gw*4 pages, height = gh*256 chars; index = char|page<<8."""
    assert W == gw * 4 and H == gh * 256, "unexpected atlas dims %dx%d" % (W, H)
    glyphs = []
    for gi in range(1024):
        char = gi & 0xFF; page = (gi >> 8) & 0x3
        g = []
        for y in range(gh):
            row = (char * gh + y) * W + page * gw
            g.extend(px[row:row + gw])
        glyphs.append(g)
    return glyphs


def synthetic_glyphs(gw, gh):
    """Deterministic test atlas: glyph gi draws a gi-dependent opaque pattern."""
    glyphs = []
    for gi in range(1024):
        g = []
        for y in range(gh):
            for x in range(gw):
                on = ((x + y + gi) % 3) == 0
                g.append(0xFF000000 | (gi & 0xFFFF) if on else 0)
        glyphs.append(g)
    return glyphs


def main(argv):
    args = list(argv[1:])
    size = None
    if "--glyph-size" in args:
        i = args.index("--glyph-size"); size = args[i+1]; del args[i:i+2]
    tw = th = None
    if size:
        tw, th = (int(v) for v in size.lower().split("x"))

    if args and args[0] == "--synthetic":
        if len(args) != 2 or tw is None:
            sys.exit("usage: gen_font.py --synthetic <out.mfont> --glyph-size WxH")
        write_mfont(args[1], tw, th, synthetic_glyphs(tw, th))
        return

    if len(args) != 2:
        sys.exit("usage: gen_font.py <font.png> <out.mfont> [--glyph-size WxH]")
    src, dst = args
    head = open(src, "rb").read(26)
    ct = head[25]
    W, H, px = decode_rgba_png(src) if ct == 6 else decode_indexed_png(src)
    gw, gh = W // 4, H // 256
    glyphs = atlas_to_glyphs(W, H, px, gw, gh)
    if tw is not None and (tw, th) != (gw, gh):
        glyphs = [box_resample(g, gw, gh, tw, th) for g in glyphs]
        gw, gh = tw, th
    write_mfont(dst, gw, gh, glyphs)


main(sys.argv)
