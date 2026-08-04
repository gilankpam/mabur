#!/usr/bin/env python3
"""Round-trips tools/msp/gen_gsfont.py --synthetic and checks the .gfont
binary layout the C++ GsFont loader depends on."""
import os, struct, subprocess, sys, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEN = os.path.join(ROOT, "tools", "msp", "gen_gsfont.py")
MAGIC = 0x544E4647
HDR = 32
DIRENT = 36


def read_gfont(path):
    d = open(path, "rb").read()
    magic, ver, n_sizes = struct.unpack("<3I", d[:12])
    assert magic == MAGIC, hex(magic)
    assert ver == 1, ver
    sizes = []
    for i in range(n_sizes):
        o = HDR + i * DIRENT
        px, gw, gh, adv, base, ng, _pad, off = struct.unpack("<7IQ", d[o:o + DIRENT])
        cps = struct.unpack("<%dI" % ng, d[off:off + 4 * ng])
        pix_off = off + 4 * ng
        pix = d[pix_off:pix_off + ng * gw * gh * 2]
        sizes.append(dict(px=px, gw=gw, gh=gh, adv=adv, base=base,
                          n=ng, cps=cps, pix=pix))
    return d, sizes


def main():
    tmp = tempfile.mkdtemp()
    out = os.path.join(tmp, "syn.gfont")
    subprocess.check_call([sys.executable, GEN, "--synthetic", out,
                           "--sizes", "8,12"])
    d, sizes = read_gfont(out)

    assert len(sizes) == 2, len(sizes)
    assert [s["px"] for s in sizes] == [8, 12], sizes

    for s in sizes:
        # Codepoints ascending and unique -- GsFont binary-searches them.
        assert list(s["cps"]) == sorted(set(s["cps"])), s["cps"][:8]
        # The subset the overlay needs. Every non-ASCII codepoint named in
        # gs_overlay.h belongs here -- U+2014 was missing from SUBSET for a
        # while and nothing noticed, because draw_text advances the pen for
        # a glyph the atlas lacks, so the never-received "——" rendered as
        # empty space of exactly the right width.
        for cp in [0x20, 0x30, 0x39, 0x41, 0x5A, 0x25, 0x2F, 0x3A, 0x2E,
                   0x2014, 0x2212, 0x2192, 0x25CF, 0x25CB]:
            assert cp in s["cps"], (hex(cp), s["px"])
        # Two bytes per pixel, exact size, no truncation.
        assert len(s["pix"]) == s["n"] * s["gw"] * s["gh"] * 2, s["px"]
        # Geometry sanity: the 4px pad on each side means the cell is
        # strictly wider than the advance.
        assert s["gw"] > s["adv"] > 0, s
        assert 0 < s["base"] < s["gh"], s

    # Both coverage and shadow channels must be populated. Without this,
    # a golden hash would match an empty render just as happily, and a
    # regression that kills shadow generation entirely would ship unnoticed.
    s = sizes[0]
    cov = sum(s["pix"][i] for i in range(0, len(s["pix"]), 2))
    sha = sum(s["pix"][i] for i in range(1, len(s["pix"]), 2))
    assert cov > 0, "synthetic atlas has no coverage"
    assert sha > 0, "synthetic atlas has no shadow"

    # Determinism: the same invocation twice is byte-identical, which is
    # what lets the committed asset and the e2e golden hash be stable.
    out2 = os.path.join(tmp, "syn2.gfont")
    subprocess.check_call([sys.executable, GEN, "--synthetic", out2,
                           "--sizes", "8,12"])
    assert open(out2, "rb").read() == d, "generator is not deterministic"

    print("OK gen_gsfont: %d sizes, %d glyphs each" % (len(sizes), sizes[0]["n"]))


if __name__ == "__main__":
    main()
