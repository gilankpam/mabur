#ifndef MABUR_PLAYER_OSD_PALETTE_H_
#define MABUR_PLAYER_OSD_PALETTE_H_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
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

// Session-lifetime ARGB-word -> palette-index memo, plus an MRU of one.
// Optional, but any caller quantizing more than once wants it: a fixed glyph
// atlas produces a bounded set of distinct words (~1 k for the shipped
// font), and WITHOUT the memo every call pays the 256-entry nearest-
// neighbour scan for each of them all over again -- which is what dominates
// an incremental update, not the pixels it writes (measured: 0.24 ms -> 0.01
// ms for a 5-cell update, tools/bench/quantize_bench.cpp).
//
// Valid for ONE palette. Call clear() if the palette is ever rebuilt.
struct QuantizeCache {
  std::unordered_map<uint32_t, uint8_t> map;
  uint32_t last_argb = 0;  // MRU; seeded with the pair that is always true
  uint8_t last_idx = 0;    // (a fully transparent word is index 0)
  void clear() {
    map.clear();
    last_argb = 0;
    last_idx = 0;
  }
};

// Median-cut over the atlas's colours, weighted by pixel frequency, with
// index 0 reserved for fully transparent. One fixed palette for the whole
// session is sufficient (measured: mean error 3.20 vs 1.73 for a per-screen
// fit, visually indistinguishable), so this runs once at startup.
OsdPalette build_palette(const GlyphAtlas& atlas);

// Quantizes `s` against `pal`. Sizes `out` to ceil(w/16) x ceil(h/16)
// macroblocks; pixels outside the surface become index 0.
//
// COST: this walks every pixel -- 2.07 M of them at 1080p. It is NOT cheap
// enough for maburplay's 2 ms main loop (measured: 3.4 ms on an x86 host,
// ~25 ms on the RK3566 A55 before the MRU below; see tools/bench/
// quantize_bench.cpp). Callers on that loop must keep a persistent map and
// use quantize_rects() for updates; full quantize is for the first screen,
// a resize, or a blank.
void quantize(const Surface& s, const OsdPalette& pal, OsdIndexMap* out,
              QuantizeCache* cache = nullptr);

// Incremental quantize: re-quantizes only `rects` (clipped to the surface)
// into an `out` that a previous quantize() already sized for `s`, leaving
// every other byte alone. Returns false -- touching nothing -- if `out` is
// not sized for `s`, which is the caller's signal to run a full quantize().
//
// Rects come from OsdRaster::diff() against a ShadowGrid dedicated to this
// map; in steady state an MSP snapshot changes a handful of cells, so this
// is three orders of magnitude cheaper than the full pass.
bool quantize_rects(const Surface& s, const OsdPalette& pal, const DirtyRect* rects,
                    size_t n_rects, OsdIndexMap* out, QuantizeCache* cache = nullptr);

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_PALETTE_H_
