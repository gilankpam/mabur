#include "mtest.h"
#include "cal_plan.h"
#include "mabur/cal_wire.h"

using namespace maburgs;

TEST(coarse_plan_covers_all_eight_rates) {
  const auto c = make_coarse_plan(7, 99);
  CHECK(c.vtx_id == 7);
  CHECK(c.nonce == 99);
  CHECK(c.phase == mabur::cal::kPhaseCoarse);
  CHECK(c.windows.size() == 8);
  for (uint8_t r = 0; r < 8; ++r) {
    CHECK(c.windows[r].rate == r);
    CHECK(c.windows[r].idx_lo == 0);
    CHECK(c.windows[r].idx_hi >= 124);
    CHECK(c.windows[r].idx_step == 4);
  }
  CHECK(c.frames_per_cell == 20);
}

TEST(coarse_plan_is_256_cells) {
  // 8 rates x 32 indices (0,4,...124). This count drives the duration
  // estimate the GS uses to know when the drone stops transmitting.
  const auto c = make_coarse_plan(1, 1);
  int cells = 0;
  for (const auto& w : c.windows)
    for (int i = w.idx_lo; i <= w.idx_hi; i += w.idx_step) ++cells;
  CHECK(cells == 256);
}

TEST(fine_plan_only_covers_rows_that_dipped) {
  std::array<RateWall, 8> coarse{};
  for (int r = 0; r < 3; ++r) {           // mcs0-2: no dip, knee-derived
    coarse[static_cast<size_t>(r)].wall = 91;
    coarse[static_cast<size_t>(r)].flags = kCalNoDip;
  }
  for (int r = 3; r < 7; ++r) {           // mcs3-6: real dips
    coarse[static_cast<size_t>(r)].wall = 60 - r;
    coarse[static_cast<size_t>(r)].flags = 0;
  }
  coarse[7].wall = -1;                    // mcs7: undetermined
  coarse[7].flags = kCalUndetermined;

  const auto f = make_fine_plan(2, 3, coarse);
  CHECK(f.phase == mabur::cal::kPhaseFine);
  CHECK(f.windows.size() == 4);           // only mcs3-6
  for (const auto& w : f.windows) {
    CHECK(w.rate >= 3 && w.rate <= 6);
    CHECK(w.idx_step == 1);
  }
  CHECK(f.frames_per_cell == 100);
}

TEST(fine_window_brackets_the_coarse_wall) {
  std::array<RateWall, 8> coarse{};
  coarse[5].wall = 56;
  coarse[5].flags = 0;
  const auto f = make_fine_plan(1, 1, coarse);
  REQUIRE(f.windows.size() == 1);
  CHECK(f.windows[0].idx_lo == 56 - kFineHalfWidth);
  CHECK(f.windows[0].idx_hi == 56 + kFineHalfWidth);
}

TEST(fine_window_clamps_to_the_seven_bit_range) {
  std::array<RateWall, 8> coarse{};
  coarse[0].wall = 3;     // near the bottom
  coarse[1].wall = 125;   // near the top
  const auto f = make_fine_plan(1, 1, coarse);
  REQUIRE(f.windows.size() == 2);
  CHECK(f.windows[0].idx_lo == 0);
  CHECK(f.windows[1].idx_hi == 127);
}

TEST(fine_plan_with_no_dips_is_empty) {
  std::array<RateWall, 8> coarse{};
  for (auto& w : coarse) w.flags = kCalNoDip;
  const auto f = make_fine_plan(1, 1, coarse);
  CHECK(f.windows.empty());
}

TEST(verify_plan_is_one_cell_per_parked_rate) {
  std::array<int, 8> park = {87, 87, 87, 91, 69, 50, 47, 45};
  const auto v = make_verify_plan(1, 1, park);
  CHECK(v.phase == mabur::cal::kPhaseVerify);
  CHECK(v.windows.size() == 8);
  CHECK(v.windows[4].idx_lo == 69);
  CHECK(v.windows[4].idx_hi == 69);
  CHECK(v.windows[4].idx_step == 1);
}

TEST(verify_plan_skips_negative_park_indices) {
  // An undetermined rate has no parked index to verify.
  std::array<int, 8> park = {87, -1, 87, 91, 69, 50, 47, 45};
  const auto v = make_verify_plan(1, 1, park);
  CHECK(v.windows.size() == 7);
}

TEST(duration_counts_settle_and_airtime) {
  mabur::rc::CalCmd c;
  c.frames_per_cell = 20;
  c.settle_ms = 100;
  c.gap_us = 2000;
  c.windows = {{0, 0, 12, 4}};   // 4 cells: 0,4,8,12
  // 4 * (100 ms settle + 20 frames * 2 ms) = 4 * 140 = 560 ms
  CHECK(plan_duration_ms(c) == 560);
}

MTEST_MAIN
