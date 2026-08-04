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

  // Now we have fresh deltas AFTER the reset.
  // A single add() with positive delta is valid (one-entry window).
  auto sample_after_one_add = lw.sample(t - 1);
  CHECK(sample_after_one_add.valid);
  CHECK(sample_after_one_add.loss == 0.0);

  // Add another to verify multi-entry window still works
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

TEST(single_add_then_sample_is_valid) {
  // After a single add() with positive delta from zero baseline,
  // sample() should be valid with correct loss.
  S1LossWindow lw(500);

  double t = 0;
  // First add: 100 expected, 97 arrived (3% loss)
  lw.add(100, 97, t);

  auto sample = lw.sample(t);
  CHECK(sample.valid);
  CHECK(approx_eq(sample.loss, 0.03, 1e-6));
}

TEST(reset_with_nondegen_loss) {
  // Critical test: after a reset followed by real loss,
  // sample() must report that loss once valid (not 0% or garbage).
  S1LossWindow lw(500);

  double t = 0;
  uint64_t expected = 0;
  uint64_t arrived = 0;

  // Build up some initial data
  for (int i = 0; i < 3; ++i) {
    expected += 100;
    arrived += 100;
    lw.add(expected, arrived, t);
    t += 50;
  }

  // Verify initial state: valid, 0% loss
  auto sample_before_reset = lw.sample(t - 1);
  CHECK(sample_before_reset.valid);
  CHECK(sample_before_reset.loss == 0.0);

  // Reset: totals go backwards
  expected = 10;
  arrived = 10;
  lw.add(expected, arrived, t);
  t += 50;

  // After reset, window is empty: invalid
  auto sample_after_reset = lw.sample(t - 1);
  CHECK(!sample_after_reset.valid);

  // Add first entry post-reset: ~50% loss (100 expected, 50 arrived)
  expected = 110;
  arrived = 60;
  lw.add(expected, arrived, t);
  t += 50;

  // Single entry post-reset: should be valid (dExpected > 0)
  auto sample_one_entry = lw.sample(t - 1);
  CHECK(sample_one_entry.valid);
  CHECK(approx_eq(sample_one_entry.loss, 0.50, 1e-3));  // ~50% loss
}

TEST(long_run_deque_pruning) {
  // After many adds spanning >> window_ms, window_.size() stays bounded.
  // Loss reflects only the recent interval.
  S1LossWindow lw(100);  // 100 ms window for faster pruning

  double t = 0;
  uint64_t expected = 0;
  uint64_t arrived = 0;

  // Add 1000 entries over 5000 ms (50 ms intervals, 100 ms window)
  // ~2 entries in window at a time. Window should stay small.
  for (int i = 0; i < 1000; ++i) {
    expected += 10;
    arrived += 9;  // 10% loss
    lw.add(expected, arrived, t);
    t += 5;  // 5 ms per update

    // Periodically check that window size stays bounded
    if (i % 100 == 0 && i > 0) {
      int size = lw.size();
      // At t >= 500, should have at most ~20 entries (100 ms window / 5 ms per entry)
      if (t >= 500) {
        CHECK(size <= 30);  // Conservative bound
      }
    }
  }

  // At the end, window should have only recent entries (100 ms window at t=5000)
  int final_size = lw.size();
  CHECK(final_size <= 30);  // ~20 entries for 100 ms window at 5 ms intervals

  // Loss should still reflect ~10%
  auto sample = lw.sample(t - 1);
  CHECK(sample.valid);
  CHECK(approx_eq(sample.loss, 0.10, 1e-3));
}

TEST(expected_in_window_counts_window_only) {
  maburgs::S1LossWindow w(500);
  w.add(0, 0, 0.0);          // baseline
  w.add(100, 90, 100.0);     // 100 expected in-window
  w.add(250, 240, 400.0);    // 150 more
  CHECK(w.expected_in_window(400.0) == 250);
  CHECK(w.expected_in_window(2000.0) == 0);   // everything aged out
}

MTEST_MAIN
