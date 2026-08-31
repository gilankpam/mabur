#ifndef MABUR_PLAYER_VBLANK_ESTIMATOR_H_
#define MABUR_PLAYER_VBLANK_ESTIMATOR_H_

#include <cmath>
#include <cstdint>

namespace maburplay {

// Predicts the panel's vblank grid from the kernel page-flip timestamps
// the presenter already captures (DRM_CAP_TIMESTAMP_MONOTONIC). Pure
// arithmetic: no clocks, no threads, no DRM types. Exact flips only --
// an inexact timestamp is a poll-receipt stamp and would poison phase.
//
// Spec: docs/superpowers/specs/2026-08-31-vsync-locked-regulator-design.md §1.
class VblankEstimator {
 public:
  static constexpr double kSeedPeriodUs = 16'667.0;
  static constexpr int kWarmFlips = 8;
  static constexpr int kStalePeriods = 32;

  void on_flip(uint64_t flip_us, bool exact) {
    if (!exact) return;
    if (exact_flips_ == 0) {
      phase_us_ = flip_us;
      last_exact_us_ = flip_us;
      exact_flips_ = 1;
      return;
    }
    const double delta = static_cast<double>(flip_us - phase_us_);
    const double k = std::round(delta / period_us_);
    if (k < 1.0) return;  // duplicate / backwards jitter: drop
    if (k == 1.0 && std::fabs(delta - period_us_) <= 0.02 * period_us_) {
      period_us_ += (delta - period_us_) / 16.0;
      const double lo = kSeedPeriodUs * 0.99, hi = kSeedPeriodUs * 1.01;
      if (period_us_ < lo) period_us_ = lo;
      if (period_us_ > hi) period_us_ = hi;
    }
    // Any k >= 1 refreshes phase: skipped vsyncs during stalls must not
    // strand the grid in the past.
    phase_us_ = flip_us;
    last_exact_us_ = flip_us;
    if (exact_flips_ < kWarmFlips) ++exact_flips_;
  }

  bool valid(uint64_t now_us) const {
    if (exact_flips_ < kWarmFlips) return false;
    // Signed: a last flip "in the future" relative to now (pathological
    // phase) is trivially recent -- validity must not underflow away.
    // The regulator's safety clamp is what actually guards that state.
    const double age =
        static_cast<double>(static_cast<int64_t>(now_us - last_exact_us_));
    return age <= kStalePeriods * period_us_;
  }

  struct Release {
    uint64_t vblank_us;
    uint64_t release_us;
  };

  // Earliest v = phase + n*period with v - lead >= now. Precondition:
  // valid(now_us). release_us >= now_us by construction.
  Release next_release(uint64_t now_us, uint64_t lead_us) const {
    const double target = static_cast<double>(now_us + lead_us);
    const double base = static_cast<double>(phase_us_);
    double n = std::ceil((target - base) / period_us_);
    if (n < 1.0) n = 1.0;
    const uint64_t v = phase_us_ + static_cast<uint64_t>(n * period_us_);
    return {v, v - lead_us};
  }

  double period_us() const { return period_us_; }
  int exact_flips() const { return exact_flips_; }

 private:
  double period_us_ = kSeedPeriodUs;
  uint64_t phase_us_ = 0;
  uint64_t last_exact_us_ = 0;
  int exact_flips_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_VBLANK_ESTIMATOR_H_
