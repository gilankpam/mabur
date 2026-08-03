#ifndef MABUR_PLAYER_OSD_LAYOUT_H_
#define MABUR_PLAYER_OSD_LAYOUT_H_

#include <cstdint>

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

// The DRM plane stacking order. Two topologies use it:
//
//   OSD available  -- the OSD is on the CRTC's PRIMARY plane, above the
//                     video, and there is NO backdrop (have_backdrop
//                     false): the OSD's own transparent buffer is what the
//                     primary scans out, and the video already covers the
//                     whole CRTC, so nothing is left to fill. On the bench
//                     vop2 (every plane [0,7]) that is video 6 / osd 7.
//   OSD absent     -- the pre-OSD arrangement: video on top of a black
//                     backdrop on the primary (video vmax, backdrop pmin).
//
// The backdrop parameters survive because the middle case is real: if the
// primary is not linear-ARGB but some other plane is, the OSD goes there
// and the primary keeps its backdrop, giving backdrop < video < osd.
//
// Lives here rather than inside DrmPresenter's pimpl because it is pure
// arithmetic over six integers with no ioctls in it -- and an untestable
// version of exactly this shipped a feature that disabled itself at init on
// the real vop2, where all planes advertise the SAME zpos range.
struct ZposPlan {
  uint64_t backdrop = 0;  // meaningful only when have_backdrop
  uint64_t video = 0;     // meaningful only when have_video_prop
  uint64_t osd = 0;       // meaningful only when osd_ok
  bool osd_ok = false;    // false => run video-only, do not touch the OSD plane
};

// have_osd / have_backdrop / have_video_prop mean "the plane exists AND has
// a mutable zpos property"; anything else disqualifies the OSD (there is
// then no way to prove it lands above the video). have_backdrop is false
// whenever the OSD occupies the primary -- there is no backdrop buffer at
// all in that topology.
//
// Policy: the OSD takes the TOP of its own range and the video sits one
// step below it, clamped into the video's range; the backdrop, when there
// is one, takes the bottom of its own. When there is no OSD the video keeps
// the hardware-validated top-of-range value untouched. If the resulting
// values do not come out strictly ordered (backdrop <) video < osd, osd_ok
// is false and the video/backdrop values revert to the no-OSD ones -- the
// OSD must never perturb the video's stacking (a video/backdrop tie is the
// vop2 black screen this whole block exists to prevent).
//
// Only the range endpoints the policy actually consumes are parameters:
// the backdrop's max and the OSD's min are never reachable choices (the
// backdrop always takes pmin, the OSD always takes omax), so they are
// deliberately not passed rather than accepted and ignored.
ZposPlan plan_zpos(bool have_osd, bool have_video_prop, bool have_backdrop, uint64_t vmin,
                   uint64_t vmax, uint64_t pmin, uint64_t omax);

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_LAYOUT_H_
