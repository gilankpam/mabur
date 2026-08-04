#ifndef MABUR_PLAYER_OSD_RASTER_H_
#define MABUR_PLAYER_OSD_RASTER_H_

#include <cstdint>
#include <vector>

#include "mabur/msp_dp.h"
#include "osd_font.h"
#include "osd_layout.h"

namespace maburplay {

// A plain 32-bit ARGB drawing target. Backed by a heap buffer on the host
// and by a DRM dumb buffer on hardware -- OsdRaster does not care which.
struct Surface {
  uint32_t* pixels = nullptr;
  int width = 0;
  int height = 0;
  int stride_px = 0;  // row pitch in PIXELS, not bytes
};

// Per-buffer memory of what is currently drawn, so an update only touches
// changed cells. One ShadowGrid per surface: they alternate with the
// double-buffered OSD surface and must not be shared.
struct ShadowGrid {
  OsdLayout layout;
  std::vector<uint16_t> cells;
};

// A rectangle of a Surface, in pixels. Produced by OsdRaster::diff() and
// consumed by quantize_rects() (osd_palette.h): the burned-DVR path's
// equivalent of the ShadowGrid cell diff.
struct DirtyRect {
  int x = 0, y = 0, w = 0, h = 0;
};

// Rasterizes an MspScreen onto a Surface. Every blit is 1:1 from an atlas
// built at exactly the draw size (see OsdFont::atlas_at), so no filtering
// happens here at all.
class OsdRaster {
 public:
  OsdRaster(OsdFont& font, ScaleMode mode) : font_(font), mode_(mode) {}

  // Returns the number of cells redrawn. On a full redraw -- shadow ==
  // nullptr, or the layout/grid dimensions changed -- clears the grid rect
  // and redraws every cell; otherwise redraws only cells that differ from
  // the shadow. The cleared rect is the union of the new grid and the
  // previous one on record, so a layout change doesn't strand stale pixels
  // outside the new grid. With shadow == nullptr there is no previous grid
  // on record, so ONLY the new grid is cleared -- unlike clear(), draw()
  // never paints outside its own grid in the first place, so with nothing
  // to account for there is nothing else that could need erasing.
  int draw(const mabur::MspScreen& screen, const Surface& s, ShadowGrid* shadow);

  // Blanks the grid the shadow records as drawn -- NOT the whole surface:
  // the GS link-status overlay shares this surface and must survive an MSP
  // stale-blank. Invalidates the shadow, so the next draw is a full one.
  // With shadow == nullptr there is no record of what was drawn and the
  // whole surface is blanked instead.
  void clear(const Surface& s, ShadowGrid* shadow);

  // draw()'s change detection WITHOUT touching the surface: appends to `out`
  // the pixel rects whose cells differ from `shadow`, and updates `shadow` to
  // match `screen`. Horizontally adjacent changed cells are merged into one
  // rect. A full repaint (null/stale shadow, layout change) yields exactly
  // one rect covering the whole surface. Returns the number of cells changed.
  //
  // This exists because the burned DVR's index-map quantizer is a SECOND
  // consumer of the same raster and needs its OWN change lineage. draw()'s
  // shadows are per DRM buffer, so they describe the change since two renders
  // ago -- which is not a superset of the change since the last one (a cell
  // going X -> Y -> X is invisible to them, and stale for anyone updated on
  // every render). One extra ShadowGrid, owned by that consumer, is the fix.
  int diff(const mabur::MspScreen& screen, const Surface& s, ShadowGrid* shadow,
           std::vector<DirtyRect>* out);

  const OsdLayout& layout() const { return layout_; }

 private:
  OsdFont& font_;
  ScaleMode mode_;
  OsdLayout layout_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_RASTER_H_
