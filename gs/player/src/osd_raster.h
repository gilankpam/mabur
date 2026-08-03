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

// Rasterizes an MspScreen onto a Surface. Every blit is 1:1 from an atlas
// built at exactly the draw size (see OsdFont::atlas_at), so no filtering
// happens here at all.
class OsdRaster {
 public:
  OsdRaster(OsdFont& font, ScaleMode mode) : font_(font), mode_(mode) {}

  // Returns the number of cells redrawn. With shadow == nullptr, or when
  // the layout or grid dimensions changed, clears the surface and redraws
  // every cell; otherwise redraws only cells that differ from the shadow.
  int draw(const mabur::MspScreen& screen, const Surface& s, ShadowGrid* shadow);

  // Blanks the whole surface and invalidates the shadow (next draw is full).
  void clear(const Surface& s, ShadowGrid* shadow);

  const OsdLayout& layout() const { return layout_; }

 private:
  OsdFont& font_;
  ScaleMode mode_;
  OsdLayout layout_;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_RASTER_H_
