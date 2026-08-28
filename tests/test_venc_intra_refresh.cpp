/* ported from waybeam_venc f956a52:tests/test_intra_refresh.c */
#include <cmath>
#include <cstring>

// intra_refresh.h has no extern "C" guard (it's only ever included from C
// translation units in production); wrap it here so the C symbols link
// against this C++ test binary without name mangling.
extern "C" {
#include "intra_refresh.h"
}
#include "mtest.h"

namespace {
bool approx(double a, double b) { return std::fabs(a - b) < 1e-6; }
}  // namespace

TEST(parse_mode_names) {
  CHECK(intra_refresh_parse_mode("off") == INTRA_MODE_OFF);
  CHECK(intra_refresh_parse_mode("fast") == INTRA_MODE_FAST);
  CHECK(intra_refresh_parse_mode("balanced") == INTRA_MODE_BALANCED);
  CHECK(intra_refresh_parse_mode("robust") == INTRA_MODE_ROBUST);
  CHECK(intra_refresh_parse_mode("BALANCED") == INTRA_MODE_BALANCED);
  CHECK(intra_refresh_parse_mode("bogus") == INTRA_MODE_OFF);
  CHECK(intra_refresh_parse_mode(nullptr) == INTRA_MODE_OFF);
  CHECK(intra_refresh_parse_mode("") == INTRA_MODE_OFF);
}

TEST(mode_name_round_trip) {
  CHECK(std::strcmp(intra_refresh_mode_name(INTRA_MODE_OFF), "off") == 0);
  CHECK(std::strcmp(intra_refresh_mode_name(INTRA_MODE_FAST), "fast") == 0);
  CHECK(std::strcmp(intra_refresh_mode_name(INTRA_MODE_BALANCED),
                     "balanced") == 0);
  CHECK(std::strcmp(intra_refresh_mode_name(INTRA_MODE_ROBUST), "robust") ==
        0);
}

TEST(off_mode_zeros_output) {
  IntraRefreshDerived ir;
  intra_refresh_compute(INTRA_MODE_OFF, 1080, 60, 0, 0, 0.0, &ir);
  CHECK(ir.mode == INTRA_MODE_OFF && ir.lines == 0 && ir.gop_frames == 0 &&
        ir.req_iqp == 0 && ir.target_ms == 0);
}

TEST(h265_1080p60_fast) {
  // total_rows = ceil(1080/32) = 34
  // refresh_frames = round(60 * 150 / 1000) = 9; lines = ceil(34/9) = 4
  // gop_frames = ceil(34/4) = 9 -> 9/60 s
  IntraRefreshDerived ir;
  intra_refresh_compute(INTRA_MODE_FAST, 1080, 60, 0, 0, 0.0, &ir);
  CHECK(ir.total_rows == 34);
  CHECK(ir.lines == 4);
  CHECK(ir.gop_frames == 9);
  CHECK(approx(ir.gop_sec, 9.0 / 60.0));
  CHECK(ir.req_iqp == 36);
  CHECK(ir.target_ms == 150);
  CHECK(ir.lines_clamped == 0);
  CHECK(ir.gop_overridden == 0);
}

TEST(h265_1080p60_balanced) {
  // refresh_frames = round(60 * 500 / 1000) = 30; lines = ceil(34/30) = 2
  // gop_frames = ceil(34/2) = 17 -> 17/60 s
  IntraRefreshDerived ir;
  intra_refresh_compute(INTRA_MODE_BALANCED, 1080, 60, 0, 0, 0.0, &ir);
  CHECK(ir.lines == 2);
  CHECK(ir.gop_frames == 17);
}

TEST(h265_1080p60_robust) {
  // refresh_frames = round(60 * 1000 / 1000) = 60; lines = ceil(34/60) = 1
  // gop_frames = ceil(34/1) = 34 -> 34/60 s
  IntraRefreshDerived ir;
  intra_refresh_compute(INTRA_MODE_ROBUST, 1080, 60, 0, 0, 0.0, &ir);
  CHECK(ir.lines == 1);
  CHECK(ir.gop_frames == 34);
}

TEST(h265_720p60_fast) {
  // total_rows = ceil(720/32) = 23; lines = ceil(23/9) = 3
  IntraRefreshDerived ir;
  intra_refresh_compute(INTRA_MODE_FAST, 720, 60, 0, 0, 0.0, &ir);
  CHECK(ir.total_rows == 23);
  CHECK(ir.lines == 3);
}

TEST(override_lines_wins_auto_gop_recomputes) {
  IntraRefreshDerived ir;
  intra_refresh_compute(INTRA_MODE_BALANCED, 1080, 60, 4, 0, 0.0, &ir);
  CHECK(ir.lines == 4);
  CHECK(ir.gop_frames == 9);  // ceil(34/4)
  CHECK(ir.lines_clamped == 0);
}

TEST(override_lines_clamped_to_total_rows) {
  IntraRefreshDerived ir;
  intra_refresh_compute(INTRA_MODE_BALANCED, 1080, 60, 999, 0, 0.0, &ir);
  CHECK(ir.lines == 34);
  CHECK(ir.lines_clamped == 1);
  CHECK(ir.gop_frames == 1);
}

TEST(override_qp_wins_over_codec_default) {
  IntraRefreshDerived ir;
  intra_refresh_compute(INTRA_MODE_BALANCED, 1080, 60, 0, 30, 0.0, &ir);
  CHECK(ir.req_iqp == 30);
}

TEST(override_gop_suppresses_auto_gop) {
  IntraRefreshDerived ir;
  intra_refresh_compute(INTRA_MODE_BALANCED, 1080, 60, 0, 0, 2.0, &ir);
  CHECK(ir.gop_overridden == 1);
  CHECK(ir.gop_frames == 0);
  CHECK(ir.lines == 2);       // auto lines still computed
  CHECK(ir.req_iqp == 32);    // balanced default still applied
}

TEST(per_mode_qp_defaults) {
  IntraRefreshDerived ir;
  intra_refresh_compute(INTRA_MODE_FAST, 1080, 60, 0, 0, 0.0, &ir);
  CHECK(ir.req_iqp == 36);
  intra_refresh_compute(INTRA_MODE_BALANCED, 1080, 60, 0, 0, 0.0, &ir);
  CHECK(ir.req_iqp == 32);
  intra_refresh_compute(INTRA_MODE_ROBUST, 1080, 60, 0, 0, 0.0, &ir);
  CHECK(ir.req_iqp == 28);
}

TEST(zero_height_or_fps_treated_as_off) {
  IntraRefreshDerived ir;
  intra_refresh_compute(INTRA_MODE_BALANCED, 0, 60, 0, 0, 0.0, &ir);
  CHECK(ir.mode == INTRA_MODE_OFF);
  intra_refresh_compute(INTRA_MODE_BALANCED, 1080, 0, 0, 0, 0.0, &ir);
  CHECK(ir.mode == INTRA_MODE_OFF);
}

TEST(null_out_does_not_crash) {
  intra_refresh_compute(INTRA_MODE_BALANCED, 1080, 60, 0, 0, 0.0, nullptr);
  CHECK(true);
}

MTEST_MAIN
