#include "mtest.h"
#include "power_plan.h"
#include <array>

// Wall-equalization plan: diff[r] = walls[r] - m - base_ref_idx, so at
// offset 0 every rate's effective index (base_ref_idx + diff[r]) equals
// walls[r] - m — every rate parked at wall-minus-margin. max_offset_qdb is
// always 0 in this formulation (the controller only ever backs off from
// there); commanded offsets are <= 0 and scale every rate down uniformly.

TEST(measured_unit_plan) {
  // This unit's measured walls (docs/txagc-calibration.md), margin 1 dB,
  // base ref 53. m = round(1.0*4) = 4 steps.
  std::array<int, 8> walls = {91, 91, 91, 91, 73, 56, 51, 49};
  auto p = mabur::make_power_plan(walls, /*legacy_wall=*/91,
                                  /*base_ref=*/53, /*margin_db=*/1.0);
  // diff[r] = walls[r] - 4 - 53: effective[r] at offset 0 = walls[r] - 4.
  CHECK(p.mcs[0] == 34);   // 91-4-53
  CHECK(p.mcs[4] == 16);   // 73-4-53
  CHECK(p.mcs[5] == -1);   // 56-4-53
  CHECK(p.mcs[6] == -6);   // 51-4-53
  CHECK(p.mcs[7] == -8);   // 49-4-53
  CHECK(p.legacy == 34);
  CHECK(p.cck == 34);      // v1: cck rides the legacy wall
  CHECK(p.max_offset_qdb == 0);
}

TEST(margin_scales) {
  std::array<int, 8> walls = {91, 91, 91, 91, 73, 56, 51, 49};
  auto p = mabur::make_power_plan(walls, 91, 53, 2.0);  // m = 8
  CHECK(p.mcs[7] == -12);  // 49-8-53
}

MTEST_MAIN
