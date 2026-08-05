#pragma once

#include <cstdint>
#include <deque>
#include <algorithm>

namespace maburgs {

// Sliding window over MONOTONIC per-stream FEC counters. Feed absolute
// totals; emits windowed loss. expected/arrived semantics: expected =
// symbols the SBI/seq framing says should have arrived; arrived = symbols
// actually received (pre-repair).
class S1LossWindow {
 public:
  explicit S1LossWindow(int window_ms = 500)
      : window_ms_(window_ms),
        prev_expected_(0),
        prev_arrived_(0),
        have_first_sample_(false) {}

  void add(uint64_t expected_total, uint64_t arrived_total, double now_ms) {
    // Detect counter reset: if totals go backwards, flush the window and
    // re-baseline prev_expected_/prev_arrived_ without pushing an entry.
    bool reset_detected = false;
    if (have_first_sample_ &&
        (expected_total < prev_expected_ || arrived_total < prev_arrived_)) {
      window_.clear();
      reset_detected = true;
    }

    // If reset was detected, just update the baseline and return (no entry).
    if (reset_detected) {
      prev_expected_ = expected_total;
      prev_arrived_ = arrived_total;
      have_first_sample_ = true;
      return;
    }

    // Compute deltas from previous state.
    uint64_t d_expected = expected_total - prev_expected_;
    uint64_t d_arrived = arrived_total - prev_arrived_;

    // Only add entries with positive deltas.
    if (d_expected > 0 || d_arrived > 0) {
      window_.push_back({now_ms, d_expected, d_arrived});
    }

    // Prune entries older than window_ms to keep deque bounded.
    const double cutoff_time = now_ms - window_ms_;
    while (!window_.empty() && window_.front().t < cutoff_time) {
      window_.pop_front();
    }

    prev_expected_ = expected_total;
    prev_arrived_ = arrived_total;
    have_first_sample_ = true;
  }

  // Loss over the window: (dExpected - dArrived) / dExpected.
  // valid=false when dExpected == 0 (no traffic).
  struct Sample {
    bool valid;
    double loss;
  };

  Sample sample(double now_ms) const {
    // Prune entries outside the window: older than window_ms AND in the future.
    const double cutoff_time = now_ms - window_ms_;

    // Compute totals from entries within [cutoff_time, now_ms].
    uint64_t total_expected = 0;
    uint64_t total_arrived = 0;
    for (const auto& entry : window_) {
      // Skip entries older than the window cutoff or after the sample time.
      if (entry.t < cutoff_time || entry.t > now_ms) {
        continue;
      }
      total_expected += entry.d_expected;
      total_arrived += entry.d_arrived;
    }

    // Return invalid if no traffic in the window.
    // Valid iff dExpected > 0.
    if (total_expected == 0) {
      return {false, 0.0};
    }

    // Compute loss and clamp to [0, 1].
    // Cast to double first to avoid uint64_t underflow when arrived > expected.
    double loss_val =
        (static_cast<double>(total_expected) - static_cast<double>(total_arrived)) / total_expected;
    loss_val = std::max(0.0, std::min(1.0, loss_val));

    return {true, loss_val};
  }

  // Sum of d_expected over entries in [now_ms - window_ms, now_ms].
  // Used for the s3 availability floor (probe_s3_min_syms).
  uint64_t expected_in_window(double now_ms) const {
    const double cutoff_time = now_ms - window_ms_;
    uint64_t total = 0;
    for (const auto& e : window_) {
      if (e.t < cutoff_time || e.t > now_ms) continue;
      total += e.d_expected;
    }
    return total;
  }

  // Observer for testing: current number of entries in deque.
  int size() const { return static_cast<int>(window_.size()); }

 private:
  struct Entry {
    double t;
    uint64_t d_expected;
    uint64_t d_arrived;
  };

  int window_ms_;
  uint64_t prev_expected_;
  uint64_t prev_arrived_;
  bool have_first_sample_;
  std::deque<Entry> window_;
};

}  // namespace maburgs
