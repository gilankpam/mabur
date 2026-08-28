/* ported from waybeam_venc f956a52:tests/test_h26x_param_sets.c */
#include <cstring>

// h26x_param_sets.h has no extern "C" guard (it's only ever included from C
// translation units in production); wrap it here so the C symbols link
// against this C++ test binary without name mangling.
extern "C" {
#include "h26x_param_sets.h"
}
#include "mtest.h"

TEST(update_null_safe) {
  H26xParamSets sets = {};
  const uint8_t nal[] = {0x01};

  h26x_param_sets_update(nullptr, PT_H265, 32, nal, sizeof(nal));
  h26x_param_sets_update(&sets, PT_H265, 32, nullptr, sizeof(nal));
  CHECK(sets.vps_len == 0);
}

TEST(update_hevc_stores_vps_sps_pps) {
  H26xParamSets sets = {};
  const uint8_t vps[] = {0x40, 0x01, 0xAA};
  const uint8_t sps[] = {0x42, 0x01, 0xBB, 0xCC};
  const uint8_t pps[] = {0x44, 0x01, 0xDD};

  h26x_param_sets_update(&sets, PT_H265, 32, vps, sizeof(vps));
  h26x_param_sets_update(&sets, PT_H265, 33, sps, sizeof(sps));
  h26x_param_sets_update(&sets, PT_H265, 34, pps, sizeof(pps));

  CHECK(sets.vps_len == sizeof(vps) &&
        std::memcmp(sets.vps, vps, sizeof(vps)) == 0);
  CHECK(sets.sps_len == sizeof(sps) &&
        std::memcmp(sets.sps, sps, sizeof(sps)) == 0);
  CHECK(sets.pps_len == sizeof(pps) &&
        std::memcmp(sets.pps, pps, sizeof(pps)) == 0);
}

TEST(update_h264_stores_sps_pps_leaves_vps) {
  H26xParamSets sets = {};
  const uint8_t sps[] = {0x67, 0x64, 0x00};
  const uint8_t pps[] = {0x68, 0xEE};

  h26x_param_sets_update(&sets, PT_H264, 7, sps, sizeof(sps));
  h26x_param_sets_update(&sets, PT_H264, 8, pps, sizeof(pps));

  CHECK(sets.sps_len == sizeof(sps) &&
        std::memcmp(sets.sps, sps, sizeof(sps)) == 0);
  CHECK(sets.pps_len == sizeof(pps) &&
        std::memcmp(sets.pps, pps, sizeof(pps)) == 0);
  CHECK(sets.vps_len == 0);
}

TEST(update_oversize_ignored) {
  H26xParamSets sets = {};
  uint8_t oversize[512];
  std::memset(oversize, 0xAB, sizeof(oversize));

  h26x_param_sets_update(&sets, PT_H265, 32, oversize, sizeof(oversize));
  h26x_param_sets_update(&sets, PT_H264, 7, oversize, sizeof(oversize));

  CHECK(sets.vps_len == 0);
  CHECK(sets.sps_len == 0);
}

TEST(prepend_hevc_returns_vps_sps_pps_in_order) {
  H26xParamSets sets = {};
  H26xParamSetRef refs[3];
  const uint8_t vps[] = {0x40, 0x01, 0xAA};
  const uint8_t sps[] = {0x42, 0x01, 0xBB};
  const uint8_t pps[] = {0x44, 0x01, 0xCC};

  h26x_param_sets_update(&sets, PT_H265, 32, vps, sizeof(vps));
  h26x_param_sets_update(&sets, PT_H265, 33, sps, sizeof(sps));
  h26x_param_sets_update(&sets, PT_H265, 34, pps, sizeof(pps));
  size_t count = h26x_param_sets_get_prepend(&sets, PT_H265, 19, refs,
                                              sizeof(refs) / sizeof(refs[0]));

  CHECK(count == 3);
  CHECK(refs[0].len == sizeof(vps) && refs[1].len == sizeof(sps) &&
        refs[2].len == sizeof(pps) &&
        std::memcmp(refs[0].data, vps, sizeof(vps)) == 0 &&
        std::memcmp(refs[1].data, sps, sizeof(sps)) == 0 &&
        std::memcmp(refs[2].data, pps, sizeof(pps)) == 0);
}

TEST(prepend_h264_returns_sps_pps_in_order) {
  H26xParamSets sets = {};
  H26xParamSetRef refs[2];
  const uint8_t sps[] = {0x67, 0x64, 0x00};
  const uint8_t pps[] = {0x68, 0xEE};

  h26x_param_sets_update(&sets, PT_H264, 7, sps, sizeof(sps));
  h26x_param_sets_update(&sets, PT_H264, 8, pps, sizeof(pps));
  size_t count = h26x_param_sets_get_prepend(&sets, PT_H264, 5, refs,
                                              sizeof(refs) / sizeof(refs[0]));

  CHECK(count == 2);
  CHECK(refs[0].len == sizeof(sps) && refs[1].len == sizeof(pps) &&
        std::memcmp(refs[0].data, sps, sizeof(sps)) == 0 &&
        std::memcmp(refs[1].data, pps, sizeof(pps)) == 0);
}

TEST(prepend_non_idr_and_null_are_empty) {
  H26xParamSets sets = {};
  H26xParamSetRef refs[3];
  const uint8_t sps[] = {0x67, 0x64, 0x00};

  h26x_param_sets_update(&sets, PT_H264, 7, sps, sizeof(sps));

  CHECK(h26x_param_sets_get_prepend(&sets, PT_H264, 1, refs,
                                     sizeof(refs) / sizeof(refs[0])) == 0);
  CHECK(h26x_param_sets_get_prepend(nullptr, PT_H265, 19, refs,
                                     sizeof(refs) / sizeof(refs[0])) == 0);
}

MTEST_MAIN
