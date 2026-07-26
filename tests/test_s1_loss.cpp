#include <cmath>

#include "s1_loss.h"
#include "mtest.h"

using namespace maburgs;

namespace {

// Helper to check if two doubles are approximately equal
bool approx_eq(double a, double b, double eps = 1e-6) {
  return std::abs(a - b) < eps;
}

}  // namespace

TEST(constant_rate_with_deficit) {
  // Feed monotonic counters at a constant rate with 3% deficit.
  // Expected = arrival rate * (1 / 0.97) to get 3% loss.
  S1LossWindow lw(500);  // 500 ms window

  double t = 0;
  uint64_t expected = 0;
  uint64_t arrived = 0;

  // Feed samples at 50 ms intervals for 1000 ms total.
  // Each sample: arrived increases by 97, expected by 100 (3% loss).
  for (int i = 0; i < 20; ++i) {
    arrived += 97;
    expected += 100;
    lw.add(expected, arrived, t);
    t += 50;
  }

  // Sample at end: should show approximately 3% loss
  auto sample = lw.sample(t - 1);
  CHECK(sample.valid);
  CHECK(approx_eq(sample.loss, 0.03, 1e-3));
}

TEST(zero_traffic_invalid) {
  S1LossWindow lw(500);

  // Add entries with zero deltas (same counters)
  lw.add(0, 0, 0);
  lw.add(0, 0, 50);
  lw.add(0, 0, 100);

  auto sample = lw.sample(100);
  CHECK(!sample.valid);
  CHECK(sample.loss == 0.0);
}

TEST(counter_reset_flushes_window) {
  S1LossWindow lw(500);

  double t = 0;
  uint64_t expected = 0;
  uint64_t arrived = 0;

  // Feed normal increasing counters
  for (int i = 0; i < 5; ++i) {
    expected += 100;
    arrived += 100;  // 0% loss
    lw.add(expected, arrived, t);
    t += 50;
  }

  auto sample_before = lw.sample(t - 1);
  CHECK(sample_before.valid);
  CHECK(sample_before.loss == 0.0);

  // Now simulate counter reset: totals go backwards
  // After reset, sample() should be invalid until we accumulate fresh deltas
  expected = 50;  // went backwards!
  arrived = 50;
  lw.add(expected, arrived, t);
  t += 50;

  // Now sample should be invalid because dExpected is zero
  auto sample_after_reset = lw.sample(t - 1);
  CHECK(!sample_after_reset.valid);

  // Add one more entry with positive delta
  expected = 150;
  arrived = 150;
  lw.add(expected, arrived, t);
  t += 50;

  // Now we have fresh deltas, but dExpected still spans the reset.
  // The window should only reflect deltas AFTER the reset.
  // With only one add() after reset, sample is still invalid until at least
  // two adds establish the window.
  auto sample_after_one_add = lw.sample(t - 1);
  CHECK(!sample_after_one_add.valid);

  // Add one more to establish proper window
  expected = 250;
  arrived = 250;
  lw.add(expected, arrived, t);

  auto sample_after_two_adds = lw.sample(t - 1);
  CHECK(sample_after_two_adds.valid);
  CHECK(sample_after_two_adds.loss == 0.0);
}

TEST(old_entries_age_out) {
  S1LossWindow lw(500);  // 500 ms window

  double t = 0;
  uint64_t expected = 0;
  uint64_t arrived = 0;

  // Feed samples for 1000 ms: each sample adds 100 expected, 80 arrived (20% loss)
  for (int i = 0; i < 20; ++i) {
    expected += 100;
    arrived += 80;
    lw.add(expected, arrived, t);
    t += 50;
  }

  // At t=950, sample should reflect ~20% loss from the 500 ms window
  auto sample_before_aging = lw.sample(950);
  CHECK(sample_before_aging.valid);
  CHECK(approx_eq(sample_before_aging.loss, 0.20, 1e-3));

  // Jump forward 600 ms. Old entries should age out.
  // All entries from before t=450 should be pruned when we sample at t=1550.
  // Feed a high-quality sample at new time
  expected += 100;
  arrived += 100;  // 0% loss on the new sample
  t = 1550;
  lw.add(expected, arrived, t);

  auto sample_after_aging = lw.sample(t);
  // The old 20% loss entries have aged out; the new 0% loss sample dominates.
  // Loss should be close to 0% now.
  CHECK(sample_after_aging.valid);
  CHECK(approx_eq(sample_after_aging.loss, 0.0, 1e-3));
}

TEST(arrived_greater_than_expected_clamps_to_zero) {
  S1LossWindow lw(500);

  double t = 0;

  // Unusual but possible: arrived > expected (e.g., duplicate arrivals)
  lw.add(100, 150, t);  // arrived > expected
  t += 50;
  lw.add(200, 300, t);

  auto sample = lw.sample(t);
  CHECK(sample.valid);
  // Loss should clamp to 0.0, not go negative
  CHECK(sample.loss >= 0.0);
  CHECK(approx_eq(sample.loss, 0.0, 1e-6));
}

MTEST_MAIN
