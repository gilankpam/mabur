#!/usr/bin/env python3
"""Convert maburplay's startup splash image to the raw XRGB8888 asset.

The player has no image decoder on purpose -- the conversion happens here, on
the dev host, and the result is committed like the two font assets. Pillow is
a DEV-HOST dependency only; nothing on the GS needs it.

  nix-shell -p python3Packages.pillow --run \\
    "python3 tools/gen_splash.py ~/Pictures/fpv-feed.jpg gs/player/bundle/splash.bin"

Output layout (little-endian), matching gs/player/src/splash_image.h:
  [0..3]   magic "MSPL"
  [4..7]   uint32 width
  [8..11]  uint32 height
  [12..15] uint32 reserved, zero
  [16..]   width*height pixels, 0xFFRRGGBB (bytes B,G,R,0xFF), rows packed

The alpha byte is forced to 0xFF. The FB the player allocates is XRGB8888, so
the top byte is ignored on scanout -- but the resampler averages all four
channels, and a uniform 0xFF keeps a scaled pixel opaque if the buffer is ever
read as ARGB.
"""

import struct
import sys

from PIL import Image


def main() -> int:
    if len(sys.argv) != 3:
        sys.stderr.write("usage: gen_splash.py <input-image> <output.bin>\n")
        return 2
    src_path, out_path = sys.argv[1], sys.argv[2]

    im = Image.open(src_path).convert("RGB")
    w, h = im.size
    im.putalpha(255)
    # PIL's "BGRA" raw mode is byte order B,G,R,A -- read back as a
    # little-endian uint32 that is exactly 0xAARRGGBB.
    body = im.tobytes("raw", "BGRA")
    assert len(body) == w * h * 4, f"unexpected body length {len(body)}"

    with open(out_path, "wb") as f:
        f.write(b"MSPL")
        f.write(struct.pack("<III", w, h, 0))
        f.write(body)

    print(f"{out_path}: {w}x{h}, {16 + len(body)} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
