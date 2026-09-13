#include "mtest.h"
#include "power_plan.h"
#include <array>

// Relative walls (spec 2026-09-13): diff[r] = rel[r] - m, where rel is the
// measured wall as an index RELATIVE to the chip's own per-channel anchor,
// and m converts margin_db into 0.25 dB index steps. The chip adds the
// anchor for whichever channel it is on, so one table is valid everywhere.

TEST(measured_unit_plan) {
  // 2026-09-13 ch136 means, margin 1 dB -> m = 4.
  std::array<int, 8> rel = {63, 63, 62, 45, 22, 6, 9, 5};
  auto p = mabur::make_power_plan(rel, /*legacy_rel=*/63, /*margin_db=*/1.0,
                                  /*anchor=*/39);
  CHECK(p.mcs[0] == 59);   // 63-4
  CHECK(p.mcs[3] == 41);   // 45-4
  CHECK(p.mcs[6] == 5);    // 9-4
  CHECK(p.mcs[7] == 1);    // 5-4
  CHECK(p.legacy == 59);
  CHECK(p.cck == 59);      // cck rides the legacy wall
}

TEST(margin_scales_and_negative_rel_is_ordinary) {
  std::array<int, 8> rel = {63, 63, 63, 42, 20, 1, -2, -4};
  auto p = mabur::make_power_plan(rel, 63, 2.0, 53);  // m = 8
  CHECK(p.mcs[6] == -10);  // -2-8
  CHECK(p.mcs[7] == -12);  // -4-8
}

TEST(clamp_saturates_to_the_diff_field) {
  std::array<int, 8> lo = {-64, -64, -64, -64, -64, -64, -64, -64};
  auto p = mabur::make_power_plan(lo, -64, 6.0, 53);  // -64-24 -> -64
  for (int r = 0; r < 8; ++r) CHECK(p.mcs[r] == -64);
  CHECK(p.legacy == -64);
}

TEST(anchor_cap_keeps_reference_plus_diff_at_or_below_127) {
  // A blank-efuse card (devourer fallback anchor 75) must not be driven to
  // ref + diff = 138: the cap is 127 - anchor = 52. On this unit (anchors
  // 39-57) the cap never binds.
  std::array<int, 8> rail = {63, 63, 63, 63, 63, 63, 63, 63};
  auto capped = mabur::make_power_plan(rail, 63, 0.0, 75);
  for (int r = 0; r < 8; ++r) CHECK(capped.mcs[r] == 52);
  CHECK(capped.legacy == 52);
  auto free = mabur::make_power_plan(rail, 63, 0.0, 57);
  for (int r = 0; r < 8; ++r) CHECK(free.mcs[r] == 63);
  // Unknown anchor (<= 0): no cap.
  auto unknown = mabur::make_power_plan(rail, 63, 0.0, 0);
  CHECK(unknown.mcs[0] == 63);
}

MTEST_MAIN
