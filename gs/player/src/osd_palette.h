#ifndef MABUR_PLAYER_OSD_PALETTE_H_
#define MABUR_PLAYER_OSD_PALETTE_H_

#include <cstdint>
#include <vector>

#include "osd_font.h"    // GlyphAtlas
#include "osd_raster.h"  // Surface

namespace maburplay {

// A palette in the exact layout MppEncOSDPltVal::val wants.
//
// WARNING: rk_venc_cmd.h declares that union's bitfields as {v,u,y,alpha}
// from the LSB, which is WRONG on little-endian -- its own
// MPP_ENC_OSD_PLT_* constants and the hardware both put Y in the low byte.
// Writing by the field names renders white glyphs pink. Byte order here is
// byte0=Y, byte1=U, byte2=V, byte3=alpha, BT.601 limited range.
struct OsdPalette {
  uint32_t entry[256] = {0};
  int n = 0;             // entries in use; entry[0] is always transparent
};

// Palette-indexed bitmap for one encoder OSD region. RASTER layout with
// stride mb_w*16 bytes -- NOT macroblock-tiled. (Tiling passes the HAL's
// size check and renders as scan-line noise.)
struct OsdIndexMap {
  std::vector<uint8_t> px;
  int mb_w = 0, mb_h = 0;
  int stride() const { return mb_w * 16; }
};

// Median-cut over the atlas's colours, weighted by pixel frequency, with
// index 0 reserved for fully transparent. One fixed palette for the whole
// session is sufficient (measured: mean error 3.20 vs 1.73 for a per-screen
// fit, visually indistinguishable), so this runs once at startup.
OsdPalette build_palette(const GlyphAtlas& atlas);

// Quantizes `s` against `pal`. Sizes `out` to ceil(w/16) x ceil(h/16)
// macroblocks; pixels outside the surface become index 0.
void quantize(const Surface& s, const OsdPalette& pal, OsdIndexMap* out);

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_PALETTE_H_
