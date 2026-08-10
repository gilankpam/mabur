#include "mtest.h"
#include "splash_image.h"
#include "scratch.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace maburplay;

namespace {

// A destination surface with a sentinel-filled backing store, so a test can
// prove the resampler never wrote outside [0,width) x [0,height).
struct Buf {
  std::vector<uint32_t> px;
  Surface s;
  Buf(int w, int h, int stride = 0)
      : px(static_cast<size_t>(stride ? stride : w) * static_cast<size_t>(h), 0xDEADBEEFu) {
    s.pixels = px.data();
    s.width = w;
    s.height = h;
    s.stride_px = stride ? stride : w;
  }
  uint32_t at(int x, int y) const {
    return px[static_cast<size_t>(y) * static_cast<size_t>(s.stride_px) + static_cast<size_t>(x)];
  }
};

std::vector<uint8_t> make_asset(int w, int h, const std::vector<uint32_t>& pixels) {
  std::vector<uint8_t> f(16 + static_cast<size_t>(w) * h * 4);
  std::memcpy(f.data(), "MSPL", 4);
  auto put = [&](int off, uint32_t v) {
    f[off] = v & 0xff;
    f[off + 1] = (v >> 8) & 0xff;
    f[off + 2] = (v >> 16) & 0xff;
    f[off + 3] = (v >> 24) & 0xff;
  };
  put(4, static_cast<uint32_t>(w));
  put(8, static_cast<uint32_t>(h));
  put(12, 0);
  if (!pixels.empty())
    std::memcpy(f.data() + 16, pixels.data(), pixels.size() * 4);
  return f;
}

void write_file(const char* path, const std::vector<uint8_t>& bytes) {
  FILE* fp = std::fopen(path, "wb");
  REQUIRE(fp != nullptr);
  REQUIRE(std::fwrite(bytes.data(), 1, bytes.size(), fp) == bytes.size());
  std::fclose(fp);
}

bool all_opaque_black(const Buf& b) {
  for (int y = 0; y < b.s.height; ++y)
    for (int x = 0; x < b.s.width; ++x)
      if (b.at(x, y) != 0xff000000u) return false;
  return true;
}

}  // namespace

TEST(exact_size_match_is_a_straight_copy) {
  std::vector<uint32_t> src(16);
  for (int i = 0; i < 16; ++i) src[i] = 0xff000000u | static_cast<uint32_t>(i * 0x010203);
  // Wider stride than width: the sentinel columns prove no row overrun.
  Buf d(4, 4, 8);
  resample_cover(src.data(), 4, 4, d.s);
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 4; ++x) CHECK(d.at(x, y) == src[y * 4 + x]);
  for (int y = 0; y < 4; ++y)
    for (int x = 4; x < 8; ++x) CHECK(d.at(x, y) == 0xDEADBEEFu);
}

TEST(cover_crops_the_overflowing_axis_and_fills_the_surface) {
  // 8x8 source into a 4x8 destination: the source is wider than the target
  // aspect, so the crop keeps full height and 4 centred columns (2..5).
  // Columns 0,1,6,7 are red and must not appear anywhere in the output.
  const uint32_t kRed = 0xffff0000u, kGreen = 0xff00ff00u;
  std::vector<uint32_t> src(64);
  for (int y = 0; y < 8; ++y)
    for (int x = 0; x < 8; ++x) src[y * 8 + x] = (x >= 2 && x <= 5) ? kGreen : kRed;
  Buf d(4, 8);
  resample_cover(src.data(), 8, 8, d.s);
  for (int y = 0; y < 8; ++y)
    for (int x = 0; x < 4; ++x) CHECK(d.at(x, y) == kGreen);
}

TEST(downscale_box_averages_within_each_destination_pixel) {
  // 4x4 of four solid 2x2 quadrants into 2x2: each destination pixel is the
  // average of one uniform quadrant, so it must equal that quadrant exactly.
  const uint32_t tl = 0xff102030u, tr = 0xff405060u, bl = 0xff708090u, br = 0xffa0b0c0u;
  std::vector<uint32_t> src(16);
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 4; ++x)
      src[y * 4 + x] = y < 2 ? (x < 2 ? tl : tr) : (x < 2 ? bl : br);
  Buf d(2, 2);
  resample_cover(src.data(), 4, 4, d.s);
  CHECK(d.at(0, 0) == tl);
  CHECK(d.at(1, 0) == tr);
  CHECK(d.at(0, 1) == bl);
  CHECK(d.at(1, 1) == br);
}

TEST(upscale_is_bilinear_and_preserves_the_corners) {
  const uint32_t a = 0xff000000u, b = 0xffff0000u, c = 0xff00ff00u, e = 0xff0000ffu;
  const std::vector<uint32_t> src = {a, b, c, e};
  Buf d(8, 8);
  resample_cover(src.data(), 2, 2, d.s);
  // Destination corner centres clamp onto the source corners exactly.
  CHECK(d.at(0, 0) == a);
  CHECK(d.at(7, 0) == b);
  CHECK(d.at(0, 7) == c);
  CHECK(d.at(7, 7) == e);
  for (int y = 0; y < 8; ++y)
    for (int x = 0; x < 8; ++x) CHECK(d.at(x, y) != 0xDEADBEEFu);
}

TEST(upscaling_a_uniform_source_stays_uniform) {
  const uint32_t k = 0xff123456u;
  const std::vector<uint32_t> src = {k, k, k, k};
  Buf d(7, 5);  // deliberately not a multiple of the source size
  resample_cover(src.data(), 2, 2, d.s);
  for (int y = 0; y < 5; ++y)
    for (int x = 0; x < 7; ++x) CHECK(d.at(x, y) == k);
}

TEST(a_valid_asset_paints_and_reports_no_error) {
  const uint32_t k = 0xff204060u;
  ScratchFile f("test_splash_image", ".bin");
  write_file(f.c_str(), make_asset(2, 2, {k, k, k, k}));
  Buf d(4, 4);
  std::string err = "untouched";
  CHECK(paint_splash(f.path, d.s, &err));
  CHECK(err.empty());
  CHECK(d.at(0, 0) == k);
  CHECK(d.at(3, 3) == k);
}

TEST(bad_magic_is_rejected_and_the_surface_goes_black) {
  std::vector<uint8_t> bytes = make_asset(2, 2, {1, 2, 3, 4});
  bytes[0] = 'X';
  ScratchFile f("test_splash_image", ".bin");
  write_file(f.c_str(), bytes);
  Buf d(4, 4);
  std::string err;
  CHECK(!paint_splash(f.path, d.s, &err));
  CHECK(!err.empty());
  CHECK(all_opaque_black(d));
}

TEST(a_truncated_body_is_rejected) {
  std::vector<uint8_t> bytes = make_asset(4, 4, std::vector<uint32_t>(16, 0xff112233u));
  bytes.resize(bytes.size() - 8);  // header still claims 4x4
  ScratchFile f("test_splash_image", ".bin");
  write_file(f.c_str(), bytes);
  Buf d(4, 4);
  std::string err;
  CHECK(!paint_splash(f.path, d.s, &err));
  CHECK(all_opaque_black(d));
}

TEST(zero_dimensions_are_rejected) {
  ScratchFile f("test_splash_image", ".bin");
  write_file(f.c_str(), make_asset(0, 0, {}));
  Buf d(4, 4);
  std::string err;
  CHECK(!paint_splash(f.path, d.s, &err));
  CHECK(all_opaque_black(d));
}

TEST(a_nonzero_reserved_word_is_rejected) {
  std::vector<uint8_t> bytes = make_asset(2, 2, {1, 2, 3, 4});
  bytes[12] = 1;
  ScratchFile f("test_splash_image", ".bin");
  write_file(f.c_str(), bytes);
  Buf d(4, 4);
  std::string err;
  CHECK(!paint_splash(f.path, d.s, &err));
  CHECK(all_opaque_black(d));
}

TEST(a_missing_file_is_rejected) {
  Buf d(4, 4);
  std::string err;
  CHECK(!paint_splash("/nonexistent/splash.bin", d.s, &err));
  CHECK(!err.empty());
  CHECK(all_opaque_black(d));
}

MTEST_MAIN
