#ifndef MABUR_PLAYER_GS_DRAW_H_
#define MABUR_PLAYER_GS_DRAW_H_

#include <cstdint>

#include "gs_font.h"
#include "osd_raster.h"  // Surface, DirtyRect

namespace maburplay {

// Premultiplied ARGB word from an opaque 0xRRGGBB token colour and an
// 8-bit alpha. The OSD plane blends premultiplied, so every write on this
// surface must be premultiplied -- writing straight alpha renders the OSD
// washed out over bright video and correct over black, which is the sort
// of bug that only shows up in flight.
uint32_t premul(uint32_t rgb, uint8_t a);

// Opaque axis-aligned fill, clipped to the surface. `alpha` exists for the
// meter track, which the design draws at full opacity but which a future
// level may want to soften; every current caller passes 255.
void fill_rect(const Surface& s, int x, int y, int w, int h, uint32_t rgb,
               uint8_t alpha = 255);

// Zeroes a rect (fully transparent), clipped. This is what a field redraw
// does before it draws, which is why draw_text can write rather than blend.
void clear_region(const Surface& s, const DirtyRect& r);

// Total pen advance for a UTF-8 string. Codepoints the atlas lacks still
// advance, so a missing glyph leaves a gap instead of shifting the line.
// This is what sizes every field box -- see GsOverlay::layout().
int text_width(const MaskAtlas& a, const char* utf8);

// Draws `utf8` with the pen starting at (pen_x, baseline_y) and returns the
// advance consumed. The region is assumed already cleared, so each pixel is
// a WRITE, not a blend:
//
//   a   = cov + shadow*(1-cov)
//   rgb = colour * cov          (the shadow is black -- no colour of its own)
//
// Pixels with a == 0 are skipped, leaving the surface transparent for the
// plane's own blend against video.
int draw_text(const Surface& s, const MaskAtlas& a, int pen_x, int baseline_y,
              const char* utf8, uint32_t rgb);

// Decodes one UTF-8 codepoint at *p, advancing *p. Returns 0xFFFD on a
// malformed sequence and still advances one byte, so a bad string cannot
// spin. Exposed for testing.
uint32_t utf8_next(const char** p);

}  // namespace maburplay

#endif  // MABUR_PLAYER_GS_DRAW_H_
