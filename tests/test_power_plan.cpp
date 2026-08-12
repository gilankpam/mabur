#include "mtest.h"
#include "power_plan.h"
#include <array>

// Wall-equalization plan: diff[r] = walls[r] - m - base_ref_idx, so every
// rate's effective index (base_ref_idx + diff[r]) equals walls[r] - m —
// every rate parked at wall-minus-margin, where m converts margin_db into
// the chip's 0.25 dB index steps. Nothing moves power afterwards (runtime
// TX-power control was deleted 2026-08-12), so this park is the radiated
// operating point for the life of the process.

TEST(measured_unit_plan) {
  // This unit's measured walls (docs/txagc-calibration.md), margin 1 dB,
  // base ref 53. m = round(1.0*4) = 4 steps.
  std::array<int, 8> walls = {91, 91, 91, 91, 73, 56, 51, 49};
  auto p = mabur::make_power_plan(walls, /*legacy_wall=*/91,
                                  /*base_ref=*/53, /*margin_db=*/1.0);
  // diff[r] = walls[r] - 4 - 53: effective[r] = walls[r] - 4.
  CHECK(p.mcs[0] == 34);   // 91-4-53
  CHECK(p.mcs[4] == 16);   // 73-4-53
  CHECK(p.mcs[5] == -1);   // 56-4-53
  CHECK(p.mcs[6] == -6);   // 51-4-53
  CHECK(p.mcs[7] == -8);   // 49-4-53
  CHECK(p.legacy == 34);
  CHECK(p.cck == 34);      // v1: cck rides the legacy wall
}

TEST(margin_scales) {
  std::array<int, 8> walls = {91, 91, 91, 91, 73, 56, 51, 49};
  auto p = mabur::make_power_plan(walls, 91, 53, 2.0);  // m = 8
  CHECK(p.mcs[7] == -12);  // 49-8-53
}

// Belt-and-braces clamp: the 8822E's diff field is 7-bit two's complement
// (devourer's pack_rate_diff_word masks & 0x7f), so the clamp target is the
// FIELD range [-64, 63], not the full int8 [-128, 127]. A miscalibrated
// config (e.g. base_ref_idx left at 0) can drive the raw diff to +127; the
// clamp must saturate at 63, never wrap through the field's two's-complement
// boundary.
TEST(clamp_saturates_to_field_range_not_int8) {
  std::array<int, 8> walls = {127, 127, 127, 127, 127, 127, 127, 127};
  auto p = mabur::make_power_plan(walls, /*legacy_wall=*/127,
                                  /*base_ref=*/0, /*margin_db=*/0.0);
  // Raw diff = 127-0-0 = 127, which would wrap to -1 under 7-bit two's
  // complement on air if left unclamped. Must clamp to the field max, 63.
  for (int r = 0; r < 8; ++r) CHECK(p.mcs[r] == 63);
  CHECK(p.legacy == 63);
  CHECK(p.cck == 63);

  std::array<int, 8> walls_lo = {0, 0, 0, 0, 0, 0, 0, 0};
  auto p_lo = mabur::make_power_plan(walls_lo, /*legacy_wall=*/0,
                                     /*base_ref=*/127, /*margin_db=*/0.0);
  // Raw diff = 0-0-127 = -127; must clamp to the field min, -64.
  for (int r = 0; r < 8; ++r) CHECK(p_lo.mcs[r] == -64);
  CHECK(p_lo.legacy == -64);
}

MTEST_MAIN
