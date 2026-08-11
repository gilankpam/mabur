#ifndef MABUR_PLAYER_SPLASH_IMAGE_H_
#define MABUR_PLAYER_SPLASH_IMAGE_H_

#include <cstdint>
#include <string>

#include "osd_raster.h"  // Surface

namespace maburplay {

// The startup splash asset, produced by tools/gen_splash.py.
//
// File layout, all little-endian:
//   [0..3]   magic "MSPL"
//   [4..7]   uint32 width
//   [8..11]  uint32 height
//   [12..15] uint32 reserved, must be 0
//   [16..]   width*height pixels, 0xFFRRGGBB, rows tightly packed
//
// Deliberately trivial: the whole point of a build-time raw asset is that the
// player needs no image decoder at all. Every structural field is validated
// against the file length before a single pixel is read, so no blit here can
// run off the mapping -- the rule GsFont::load follows for the .gfont.
constexpr int kSplashHeaderBytes = 16;

// The one shipped splash. NOT configurable, by explicit design decision:
// there is exactly one splash image and no config key selects it.
constexpr const char* kSplashPath = "/usr/local/share/mabur/splash.bin";

// Paints the asset at `path` into `dst` with a COVER fit: scaled until both
// axes are covered, overflow centre-cropped, so `dst` is always painted edge
// to edge with no bars. Exact size match is a straight copy; downscale is
// box-average; upscale is bilinear.
//
// On ANY failure -- missing file, bad magic, short file, implausible
// dimensions -- `dst` is filled with opaque black, *err (when non-null) gets
// the reason, and it returns false. The caller logs and carries on: the
// picture is lost, the display is still lit.
bool paint_splash(const std::string& path, const Surface& dst, std::string* err);

// Cover-fit resample of a tightly-packed source into `dst`. Exposed so the
// geometry and filtering can be tested without a file; paint_splash() is the
// only production caller. `src` must hold at least src_w*src_h pixels.
void resample_cover(const uint32_t* src, int src_w, int src_h, const Surface& dst);

}  // namespace maburplay

#endif  // MABUR_PLAYER_SPLASH_IMAGE_H_
