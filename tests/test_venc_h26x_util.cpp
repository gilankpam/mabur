/* ported from waybeam_venc f956a52:tests/test_h26x_util.c */
#include <cstddef>
#include <cstdint>

// h26x_util.h has no extern "C" guard (it's only ever included from C
// translation units in production); wrap it here so the C symbols link
// against this C++ test binary without name mangling.
extern "C" {
#include "h26x_util.h"
}
#include "mtest.h"

TEST(strip_start_code_three_byte) {
  const uint8_t buf[] = {0x00, 0x00, 0x01, 0x65, 0xAA};
  const uint8_t *ptr = buf;
  size_t len = sizeof(buf);
  h26x_util_strip_start_code(&ptr, &len);
  CHECK(ptr == &buf[3]);
  CHECK(len == 2);
}

TEST(strip_start_code_four_byte) {
  const uint8_t buf[] = {0x00, 0x00, 0x00, 0x01, 0x42, 0x01};
  const uint8_t *ptr = buf;
  size_t len = sizeof(buf);
  h26x_util_strip_start_code(&ptr, &len);
  CHECK(ptr == &buf[4]);
  CHECK(len == 2);
}

TEST(strip_start_code_none) {
  const uint8_t buf[] = {0x26, 0x01};
  const uint8_t *ptr = buf;
  size_t len = sizeof(buf);
  h26x_util_strip_start_code(&ptr, &len);
  CHECK(ptr == buf);
  CHECK(len == sizeof(buf));
}

TEST(nalu_type_extraction) {
  const uint8_t h264[] = {0x65};
  CHECK(h26x_util_h264_nalu_type(h264, 1) == 5);
  CHECK(h26x_util_h264_nalu_type(nullptr, 0) == 0);

  const uint8_t hevc[] = {0x26, 0x01};
  CHECK(h26x_util_hevc_nalu_type(hevc, 2) == 19);
  CHECK(h26x_util_hevc_nalu_type(nullptr, 0) == 0);
}

TEST(annexb_next_iterates_access_unit) {
  const uint8_t access_unit[] = {0x00, 0x00, 0x00, 0x01, 0x40, 0x01, 0xaa,
                                  0x00, 0x00, 0x01, 0x26, 0x01, 0xbb, 0x00,
                                  0x00};
  size_t cursor = 0;
  const uint8_t *nal = nullptr;
  size_t nal_len = 0;

  CHECK(h26x_util_annexb_next(access_unit, sizeof(access_unit), &cursor,
                               &nal, &nal_len) == 1);
  CHECK(nal == &access_unit[4]);
  CHECK(nal_len == 3);

  CHECK(h26x_util_annexb_next(access_unit, sizeof(access_unit), &cursor,
                               &nal, &nal_len) == 1);
  CHECK(nal == &access_unit[10]);
  CHECK(nal_len == 3);

  CHECK(h26x_util_annexb_next(access_unit, sizeof(access_unit), &cursor,
                               &nal, &nal_len) == 0);

  size_t bad_cursor = sizeof(access_unit) + 1;
  CHECK(h26x_util_annexb_next(access_unit, sizeof(access_unit), &bad_cursor,
                               &nal, &nal_len) == 0);
}

TEST(annexb_next_skips_empty_nal_from_consecutive_start_codes) {
  const uint8_t consecutive_codes[] = {0x00, 0x00, 0x01, 0x00, 0x00,
                                        0x00, 0x01, 0x26, 0x01, 0xcc};
  size_t cursor = 0;
  const uint8_t *nal = nullptr;
  size_t nal_len = 0;

  CHECK(h26x_util_annexb_next(consecutive_codes, sizeof(consecutive_codes),
                               &cursor, &nal, &nal_len) == 1);
  CHECK(nal == &consecutive_codes[7]);
  CHECK(nal_len == 3);
  CHECK(h26x_util_annexb_next(consecutive_codes, sizeof(consecutive_codes),
                               &cursor, &nal, &nal_len) == 0);
}

TEST(hevc_patch_trail_r_to_n) {
  uint8_t access_unit[] = {
      0x00, 0x00, 0x00, 0x01, 0x40, 0x01, 0xaa,  // VPS
      0x00, 0x00, 0x01, 0x02, 0x01, 0xbb,        // TRAIL_R
      0x00, 0x00, 0x01, 0x00, 0x01, 0xcc,        // TRAIL_N (already)
      0x00, 0x00, 0x01, 0x03, 0x01, 0xdd,        // layer-id msb set
      0x00, 0x00, 0x01, 0x02, 0x09, 0xee         // layer-id low bit set
  };

  CHECK(h26x_util_hevc_patch_trail_r_to_n(access_unit, sizeof(access_unit)) ==
        1);
  CHECK(access_unit[10] == 0x00);    // TRAIL_R header rewritten to TRAIL_N
  CHECK(access_unit[4] == 0x40);     // VPS untouched
  CHECK(access_unit[16] == 0x00);    // pre-existing TRAIL_N untouched
  CHECK(access_unit[22] == 0x03);    // layered NAL (msb set) untouched
  CHECK(access_unit[28] == 0x02);    // layered NAL (low bit set) untouched

  // Idempotent: nothing left to rewrite on a second pass.
  CHECK(h26x_util_hevc_patch_trail_r_to_n(access_unit, sizeof(access_unit)) ==
        0);
  CHECK(h26x_util_hevc_patch_trail_r_to_n(nullptr, 0) == 0);
}

MTEST_MAIN
