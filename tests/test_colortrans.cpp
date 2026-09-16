#include "mtest.h"
#include "colortrans.h"
#include <cmath>
#include <cstdint>
using namespace maburplay;

static bool near3(Vec3 v, float r, float g, float b, float tol = 2e-4f) {
  return std::fabs(v.r - r) <= tol && std::fabs(v.g - g) <= tol && std::fabs(v.b - b) <= tol;
}

// Reference values: a NumPy transcription of ../colortrans/colortrans3.glsl
// with the kColorTrans3 constants, computed 2026-09-16 (spec section 1).
// Black and white pin the clamps, mid grey pins the grade (no saturation
// effect on grey), the two colours pin the matrix and the saturation mix.
TEST(forward_matches_the_glsl_reference) {
  ColorTrans t;
  CHECK(near3(t.forward({0.f, 0.f, 0.f}), 0.f, 0.f, 0.f));
  CHECK(near3(t.forward({1.f, 1.f, 1.f}), 1.f, 1.f, 1.f));
  CHECK(near3(t.forward({0.5f, 0.5f, 0.5f}), 0.853268f, 0.853268f, 0.853268f));
  CHECK(near3(t.forward({0.8f, 0.2f, 0.2f}), 1.f, 0.084757f, 0.201865f));
  CHECK(near3(t.forward({0.3f, 0.6f, 0.9f}), 0.60007f, 1.f, 1.f));
}

// Every OSD colour (the seven GS tokens, white, black) lands inside the
// unclipped region, so invert() is exact there: forward(invert(c)) == c
// to within one 8-bit step after two quantisations.
TEST(invert_round_trips_the_osd_palette) {
  ColorTrans t;
  const uint32_t toks[] = {0xF2F3F5u, 0xB0B4B8u, 0x93989Du, 0x43474Bu,
                           0x3FC99Au, 0xDFA63Au, 0xE5484Du, 0xFFFFFFu, 0x000000u};
  for (uint32_t tok : toks) {
    const uint32_t back = forward_rgb8(t, invert_rgb8(t, tok));
    for (int sh = 0; sh <= 16; sh += 8) {
      const int a = (tok >> sh) & 0xFF, b = (back >> sh) & 0xFF;
      CHECK(std::abs(a - b) <= 1);
    }
  }
  // White is drawn as mid grey on the surface; the LUT lifts it back.
  CHECK(invert_rgb8(t, 0xFFFFFFu) == 0x8F8F8Fu);
}

TEST(invert_is_the_inverse_on_a_grid_of_unclipped_colours) {
  ColorTrans t;
  int checked = 0;
  for (int r = 0; r <= 10; ++r)
    for (int g = 0; g <= 10; ++g)
      for (int b = 0; b <= 10; ++b) {
        const Vec3 c{r / 10.f, g / 10.f, b / 10.f};
        const Vec3 y = t.forward(c);
        // Only colours the forward map did not crush are recoverable.
        if (y.r <= 0.f || y.r >= 1.f || y.g <= 0.f || y.g >= 1.f || y.b <= 0.f || y.b >= 1.f)
          continue;
        CHECK(near3(t.invert(y), c.r, c.g, c.b, 1e-3f));
        ++checked;
      }
  CHECK(checked > 100);
}

TEST(premultiplied_helpers_keep_alpha_and_transparency) {
  ColorTrans t;
  CHECK(invert_premul_argb(t, 0x00000000u) == 0u);
  CHECK(invert_premul_argb(t, 0xFFFFFFFFu) == 0xFF8F8F8Fu);
  // Half-alpha white: straight colour is still white -> 0x8F8F8F, premultiplied by 0x80.
  const uint32_t half = invert_premul_argb(t, 0x80808080u);
  CHECK((half >> 24) == 0x80u);
  CHECK(((half >> 16) & 0xFF) == 0x47u || ((half >> 16) & 0xFF) == 0x48u);
  CHECK(forward_premul_argb(t, 0xFF8F8F8Fu) == 0xFFFFFFFFu ||
        forward_premul_argb(t, 0xFF8F8F8Fu) == 0xFFFEFEFEu);
}
MTEST_MAIN
