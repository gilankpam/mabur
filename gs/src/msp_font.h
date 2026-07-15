#pragma once
#include <cstdint>
namespace maburgs {

// A baked-in MSP OSD glyph atlas. Glyph g occupies pixels[g*glyph_w*glyph_h ..],
// row-major, premultiplied Cairo ARGB32 (0xAARRGGBB native words). Index
// g = char | (page<<8): char = cell & 0xFF, page = (cell>>8) & 0x3.
struct MspFont {
  int glyph_w;
  int glyph_h;
  int n_glyphs;
  const uint32_t* pixels;
};

// Betaflight HD font (generated in msp_font_btfl.cpp from font_btfl_hd.png).
extern const MspFont kMspFontBtfl;

}  // namespace maburgs
