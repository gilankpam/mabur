#ifndef MABUR_PLAYER_OSD_LAYOUT_H_
#define MABUR_PLAYER_OSD_LAYOUT_H_

#include "osd_font.h"  // ScaleMode

namespace maburplay {

// Where the character grid lands on the surface. Cell pitch IS the draw
// size: glyphs tile contiguously so multi-cell artwork (horizon lines,
// sidebars) stays unbroken, and the whole grid is centred on screen.
struct OsdLayout {
  int draw_w = 0;    // glyph draw width in px (0 = no usable layout)
  int draw_h = 0;
  int origin_x = 0;  // top-left of cell (0,0) on the surface
  int origin_y = 0;
  int cols = 0;
  int rows = 0;
  bool operator==(const OsdLayout&) const = default;
};

// Sharp mode: the draw size is the largest INTEGER multiple of the atlas
// glyph that fits one cell of screen/cols x screen/rows; if the atlas glyph
// is larger than the cell, it shrinks with the aspect ratio preserved
// (shrinking stays sharp, unlike a fractional upscale, which is what makes
// the pre-existing renderer look blurry). Fill mode takes the whole cell.
OsdLayout compute_layout(int screen_w, int screen_h, int cols, int rows,
                         int glyph_w, int glyph_h, ScaleMode mode);

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_LAYOUT_H_
