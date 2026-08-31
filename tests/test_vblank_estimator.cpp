#include <cstdint>

#include "mtest.h"
#include "vblank_estimator.h"

using maburplay::VblankEstimator;

namespace {
constexpr uint64_t kT0 = 10'000'000;   // arbitrary mono µs
constexpr uint64_t kP = 16'667;        // nominal period
}  // namespace

TEST(cold_and_warmup) {
  VblankEstimator e;
  CHECK(!e.valid(kT0));
  // 7 exact flips: still warming (kWarmFlips = 8).
  for (int i = 0; i < 7; ++i) e.on_flip(kT0 + i * kP, true);
  CHECK(!e.valid(kT0 + 7 * kP));
  e.on_flip(kT0 + 7 * kP, true);  // 8th
  CHECK(e.valid(kT0 + 7 * kP));
}

TEST(period_locks_on_noisy_flips) {
  VblankEstimator e;
  // Alternate 16660/16674 (±7 µs jitter around 16667).
  uint64_t t = kT0;
  for (int i = 0; i < 64; ++i) {
    e.on_flip(t, true);
    t += (i % 2) ? 16'660 : 16'674;
  }
  CHECK(e.period_us() > 16'662.0);
  CHECK(e.period_us() < 16'672.0);
}

TEST(skip_multiples_update_phase_not_period) {
  VblankEstimator e;
  for (int i = 0; i < 16; ++i) e.on_flip(kT0 + i * kP, true);
  const double p_before = e.period_us();
  // 3-period gap (display stall, flips resumed): phase must follow,
  // period must not fold in delta/1.
  const uint64_t resumed = kT0 + 15 * kP + 3 * kP;
  e.on_flip(resumed, true);
  CHECK(e.period_us() == p_before);
  // Phase followed: the next vblank after `resumed` is resumed + kP.
  const auto r = e.next_release(resumed + 1'000, 3'000);
  CHECK(r.vblank_us == resumed + kP);
}

TEST(inexact_flips_ignored) {
  VblankEstimator e;
  for (int i = 0; i < 64; ++i) e.on_flip(kT0 + i * 20'000, false);
  CHECK(!e.valid(kT0 + 64 * 20'000));
  CHECK(e.exact_flips() == 0);
}

TEST(stale_after_32_periods) {
  VblankEstimator e;
  for (int i = 0; i < 16; ++i) e.on_flip(kT0 + i * kP, true);
  const uint64_t last = kT0 + 15 * kP;
  CHECK(e.valid(last + 31 * kP));
  CHECK(!e.valid(last + 33 * kP));
  // Re-warms after 8 fresh exact flips.
  uint64_t t = last + 100 * kP;
  for (int i = 0; i < 8; ++i) e.on_flip(t + i * kP, true);
  CHECK(e.valid(t + 7 * kP));
}

TEST(next_release_earliest_catchable) {
  VblankEstimator e;
  for (int i = 0; i < 16; ++i) e.on_flip(kT0 + i * kP, true);
  const uint64_t phase = kT0 + 15 * kP;
  const uint64_t lead = 3'000;
  // now leaves 4 µs more than lead before the next vblank: catch it.
  auto r = e.next_release(phase + kP - lead - 4, lead);
  CHECK(r.vblank_us == phase + kP);
  CHECK(r.release_us == r.vblank_us - lead);
  CHECK(r.release_us >= phase + kP - lead - 4);
  // now inside the lead window: too risky, target the one after.
  r = e.next_release(phase + kP - lead + 1, lead);
  CHECK(r.vblank_us == phase + 2 * kP);
}

TEST(period_clamped_to_one_percent) {
  VblankEstimator e;
  // A drift the EMA would chase past the clamp must be capped.
  uint64_t t = kT0;
  for (int i = 0; i < 512; ++i) { e.on_flip(t, true); t += 16'834; }  // +1%+
  CHECK(e.period_us() <= 16'667.0 * 1.01 + 0.5);
}

MTEST_MAIN
