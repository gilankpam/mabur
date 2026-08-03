#include "osd_layout.h"

#include <algorithm>

namespace maburplay {

OsdLayout compute_layout(int screen_w, int screen_h, int cols, int rows,
                         int glyph_w, int glyph_h, ScaleMode mode) {
  OsdLayout l;
  if (screen_w <= 0 || screen_h <= 0 || cols <= 0 || rows <= 0 || glyph_w <= 0 || glyph_h <= 0)
    return l;

  const int avail_w = screen_w / cols, avail_h = screen_h / rows;
  if (avail_w <= 0 || avail_h <= 0) return l;

  if (mode == ScaleMode::kFill) {
    l.draw_w = avail_w;
    l.draw_h = avail_h;
  } else {
    const int k = std::min(avail_w / glyph_w, avail_h / glyph_h);
    if (k >= 1) {
      l.draw_w = glyph_w * k;
      l.draw_h = glyph_h * k;
    } else if (avail_w * glyph_h <= avail_h * glyph_w) {  // width-limited
      l.draw_w = avail_w;
      l.draw_h = std::max(1, glyph_h * avail_w / glyph_w);
    } else {
      l.draw_h = avail_h;
      l.draw_w = std::max(1, glyph_w * avail_h / glyph_h);
    }
  }
  l.cols = cols;
  l.rows = rows;
  l.origin_x = (screen_w - l.draw_w * cols) / 2;
  l.origin_y = (screen_h - l.draw_h * rows) / 2;
  return l;
}

namespace {
uint64_t clamp_u64(uint64_t v, uint64_t lo, uint64_t hi) {
  if (hi < lo) return lo;
  return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

ZposPlan plan_zpos(bool have_osd, bool have_video_prop, bool have_backdrop, uint64_t vmin,
                   uint64_t vmax, uint64_t pmin, uint64_t omax) {
  ZposPlan p;
  // The no-OSD assignment, hardware-validated long before the OSD existed:
  // video at the top of its range, backdrop at the bottom of its own. It is
  // also what we fall back to when the OSD cannot be stacked -- in which
  // case the caller re-allocates the backdrop it had skipped and asks again
  // with have_osd false.
  p.backdrop = have_backdrop ? pmin : 0;
  p.video = have_video_prop ? vmax : 0;
  if (!have_osd || !have_video_prop) return p;

  // OSD on top of its own range; video one step below, clamped into the
  // video plane's OWN range (a value outside it is not assignable, and
  // clamping is what turns "OSD range sits below video's" into a detectable
  // ordering violation instead of an illegal value).
  const uint64_t osd = omax;
  const uint64_t video = clamp_u64(omax == 0 ? 0 : omax - 1, vmin, vmax);
  const bool backdrop_ok = !have_backdrop || p.backdrop < video;
  if (!backdrop_ok || !(video < osd)) return p;  // keep the no-OSD values

  p.video = video;
  p.osd = osd;
  p.osd_ok = true;
  return p;
}

}  // namespace maburplay
