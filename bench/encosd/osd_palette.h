#ifndef MABUR_BENCH_ENCOSD_OSD_PALETTE_H_
#define MABUR_BENCH_ENCOSD_OSD_PALETTE_H_

#include <cstdint>
#include <vector>

// Palette quantizer for the vepu54x encoder-side OSD.
//
// The hardware OSD takes an 8-bit index bitmap plus a 256-entry palette whose
// entries are {v:8, u:8, y:8, alpha:8} -- YUV, NOT RGB (see MppEncOSDPltVal in
// rk_venc_cmd.h). maburplay's rasterizer produces premultiplied ARGB32, so the
// conversion is: un-premultiply -> BT.601 limited-range YUV -> quantize the
// 4-D (y,u,v,a) cloud with median cut.
//
// Median cut rather than "the 256 most frequent values" on purpose: a 36x54
// antialiased glyph atlas spends most of its distinct values on edge alphas,
// and a frequency histogram keeps the 256 flattest interior colors while
// throwing away exactly the edge ramp the eye judges. Median cut spends
// entries where the cloud is spread out.
namespace encosd {

// One packed palette entry, byte-identical to MppEncOSDPltVal::val.
//
// WARNING: the bitfield member names in MppEncOSDPltVal (rk_venc_cmd.h) are a
// LIE on little-endian. They declare v:8 first (i.e. at the LSB), then u, then
// y, then alpha -- but MPP's own palette constants in the same header say
// otherwise: MPP_ENC_OSD_PLT_WHITE is (255<<24)|(128<<16)|(128<<8)|235, and
// 235/128/128 is BT.601 limited-range WHITE with Y at the LSB. Every one of
// the eight constants (RED 81/90/240, BLUE 41/240/110, GREEN 145/54/34, ...)
// checks out as Y,U,V,A from the LSB up. The hardware agrees: writing the
// palette by the bitfield names renders white glyphs pink (verified on the
// bench GS, see the spike report). So: byte0 = Y, byte1 = U, byte2 = V,
// byte3 = alpha -- ignore the struct's field names, use ::val.
inline uint32_t pack_yuva(uint8_t y, uint8_t u, uint8_t v, uint8_t a) {
  return (uint32_t)y | ((uint32_t)u << 8) | ((uint32_t)v << 16) | ((uint32_t)a << 24);
}

struct Yuva {
  uint8_t y = 0, u = 128, v = 128, a = 0;
};

inline Yuva unpack_yuva(uint32_t p) {
  Yuva c;
  c.y = (uint8_t)(p & 0xFF);
  c.u = (uint8_t)((p >> 8) & 0xFF);
  c.v = (uint8_t)((p >> 16) & 0xFF);
  c.a = (uint8_t)((p >> 24) & 0xFF);
  return c;
}

// Premultiplied ARGB32 (0xAARRGGBB, host order) -> YUVA, BT.601 limited range
// (Y 16..235, C 16..240) to match MPP's own palette constants
// (MPP_ENC_OSD_PLT_WHITE is y=235 u=v=128).
Yuva argb_premul_to_yuva(uint32_t argb);

// Inverse of the above, for the reconstruction dump.
uint32_t yuva_to_argb_premul(const Yuva& c);

struct Palette {
  // Exactly 256 entries, packed as MppEncOSDPltVal::val. Index 0 is always
  // fully transparent -- reserved, never produced by the quantizer -- so a
  // cleared index bitmap is a blank OSD.
  std::vector<uint32_t> entries;
  int used = 0;  // entries actually assigned (1 + quantizer output)
};

// Builds the palette from a premultiplied-ARGB surface and, in the same pass,
// the 8-bit index bitmap (raster, one byte per pixel, row stride == w).
//
// max_colors caps the quantizer output (<= 255; index 0 is the reserved
// transparent entry). alpha_weight scales the alpha axis in the distance
// metric and in the median-cut extent: alpha error on a glyph edge shows up
// as a hard jaggy, so it is worth more than a shade of grey.
struct QuantResult {
  Palette plt;
  std::vector<uint8_t> index;  // w*h
  int distinct_colors = 0;     // distinct YUVA values present in the input
  double mean_err = 0.0;       // mean weighted L2 distance, quantized pixels only
  double max_err = 0.0;
};

QuantResult quantize(const uint32_t* argb, int w, int h, int stride_px, int max_colors,
                     double alpha_weight);

// Indexes a surface against a palette built from something ELSE -- the whole
// glyph atlas rather than one rendered screen.
//
// This is the shape Phase 2 actually wants: one palette computed offline from
// the atlas, shipped as a constant, uploaded once, and never touched again --
// so a screen whose glyph mix changes never needs a palette re-upload. The
// per-screen palette is the best case and only tells you the floor; this
// tells you what a fixed palette costs.
QuantResult map_to_palette(const uint32_t* argb, int w, int h, int stride_px, const Palette& plt,
                           double alpha_weight);

}  // namespace encosd

#endif  // MABUR_BENCH_ENCOSD_OSD_PALETTE_H_
