#include "mtest.h"
#include "cal_plan.h"
#include "mabur/cal_park.h"
#include "mabur/cal_proto.h"

using namespace maburgs;

TEST(power_sweep_is_full_resolution_twenty_frames) {
  const auto c = make_power_sweep(42, 3, 20);
  CHECK(c.nonce == 42);
  CHECK(c.seq == 3);
  CHECK(c.kind == mabur::cal::Kind::Power);
  CHECK(c.bw == 20);
  CHECK(c.frames_per_cell == kPowerFrames);
  CHECK(c.settle_ms == kPowerSettleMs);
  CHECK(c.gap_us == kGapUs);
  CHECK(c.idx_lo == kSweepLo);
  CHECK(c.idx_hi == kSweepHi);
  // 8 rates * 105 indices (-41..63 inclusive) = 840 cells, each
  // 5 ms settle + 20 frames * 1 ms gap = 25 ms -> 21000 ms.
  CHECK(sweep_duration_ms(c) == 21000);
}

TEST(capacity_sweep_is_eight_two_second_points) {
  const auto c = make_capacity_sweep(5, 1, 40);
  CHECK(c.nonce == 5);
  CHECK(c.seq == 1);
  CHECK(c.kind == mabur::cal::Kind::Capacity);
  CHECK(c.bw == 40);
  CHECK(c.settle_ms == kCapSettleMs);
  CHECK(c.dwell_ms == kCapDwellMs);
  CHECK(c.mcs_mask == 0xFF);
  // 8 points * (300 ms settle + 2000 ms dwell) = 18400 ms.
  CHECK(sweep_duration_ms(c) == 18400);
}

TEST(verify_cells_skip_undetermined) {
  std::array<int, 8> park = {59, mabur::cal::kWallLeave, 59, 38, 16, -3, -6, -8};
  const auto cells = verify_cells(park);
  CHECK(cells.size() == 7);
  for (const auto& rp : cells) CHECK(rp.first != 1);
  CHECK(cells[0].first == 0 && cells[0].second == 59);
  CHECK(cells.back().first == 7 && cells.back().second == -8);
  // 7 cells * (5 ms settle + 20 frames * 1 ms gap) = 175 ms.
  CHECK(verify_duration_ms(cells.size()) == 175);
}

TEST(the_grid_ends_on_the_rail) {
  CHECK(kSweepHi == mabur::rc::kRailRel);
  const auto c = make_power_sweep(1, 0, 20);
  CHECK(c.idx_hi == mabur::rc::kRailRel);
}

// Controller ruling: the drone parks a verify cell at clamp(wall - margin,
// kRelMin, kRelMax), skipping kWallLeave (drone/src/cal_link.cpp
// verify_parks, via mabur::park_rel_for in mabur/cal_park.h). The GS must
// predict exactly the same cell from the same walls/margin.
TEST(verify_cells_includes_a_clamped_park) {
  std::array<int8_t, 8> walls{};
  walls.fill(mabur::cal::kWallLeave);
  walls[2] = -64;  // wall - margin_steps(1.0) = -68, clamped to kRelMin -64
  const auto park = mabur::park_rel_for(walls, 1.0);
  CHECK(park[2] == -64);  // clamped, not -68

  const auto cells = verify_cells(park);
  REQUIRE(cells.size() == 1);
  CHECK(cells[0].first == 2);
  CHECK(cells[0].second == -64);
}

MTEST_MAIN
