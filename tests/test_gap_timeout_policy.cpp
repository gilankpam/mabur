#include <cstdint>

#include "gap_timeout_policy.h"
#include "mtest.h"

using maburgs::GapTimeoutPolicy;

// floor 50, cap 150 unless a test says otherwise. w = 32 (sw::kMaxWindow),
// margin 15 ms — all inside the policy.

namespace {
// Feed `secs` one-second samples at `rate` syms/s starting from (seq0, t0);
// returns the final (seq, t).
std::pair<uint64_t, uint64_t> feed(GapTimeoutPolicy& p, int sid, double rate,
                                   int secs, uint64_t seq0, uint64_t t0) {
  uint64_t seq = seq0, t = t0;
  for (int i = 0; i < secs; ++i) {
    t += 1000;
    seq += static_cast<uint64_t>(rate);
    p.update(sid, seq, 32, t);
  }
  return {seq, t};
}
}  // namespace

TEST(unseeded_and_first_sample_stay_at_floor) {
  GapTimeoutPolicy p(50, 150);
  CHECK(p.timeout_ms(0) == 50);
  p.update(0, 1'000'000, 32, 1000);  // seed only — no rate yet
  CHECK(p.timeout_ms(0) == 50);
}

TEST(high_rate_clamps_to_floor) {
  GapTimeoutPolicy p(50, 150);
  p.update(0, 1'000'000, 32, 0);
  feed(p, 0, 1880.0, 10, 1'000'000, 0);  // mcs5-ish: span ~17ms + 15 << 50
  CHECK(p.timeout_ms(0) == 50);
}

TEST(low_rate_stretches_toward_window_span) {
  GapTimeoutPolicy p(50, 150);
  p.update(0, 1'000'000, 32, 0);
  feed(p, 0, 470.0, 10, 1'000'000, 0);  // ~1 Mb/s stream: 32/470 = 68ms + 15
  const int ms = p.timeout_ms(0);
  CHECK(ms >= 75 && ms <= 95);
}

TEST(fade_decays_to_cap) {
  GapTimeoutPolicy p(50, 150);
  p.update(0, 1'000'000, 32, 0);
  auto [seq, t] = feed(p, 0, 470.0, 10, 1'000'000, 0);
  feed(p, 0, 0.0, 10, seq, t);  // silence: rate EWMA decays, span explodes
  CHECK(p.timeout_ms(0) == 150);
}

TEST(cap_zero_disables_rate_awareness) {
  GapTimeoutPolicy p(50, 0);
  p.update(0, 1'000'000, 32, 0);
  auto [seq, t] = feed(p, 0, 470.0, 10, 1'000'000, 0);
  CHECK(p.timeout_ms(0) == 50);
  feed(p, 0, 0.0, 10, seq, t);
  CHECK(p.timeout_ms(0) == 50);
}

TEST(backward_seq_reanchors_without_blowup) {
  GapTimeoutPolicy p(50, 150);
  p.update(0, 5'000'000, 32, 0);
  auto [seq, t] = feed(p, 0, 470.0, 10, 5'000'000, 0);
  (void)seq;
  p.update(0, 1'000, 32, t + 1000);  // decoder reset: newest jumps far down
  const int ms = p.timeout_ms(0);
  CHECK(ms >= 50 && ms <= 150);
  // keeps tracking from the new anchor
  feed(p, 0, 1880.0, 10, 1'000, t + 1000);
  CHECK(p.timeout_ms(0) == 50);
}

TEST(sids_are_independent) {
  GapTimeoutPolicy p(50, 150);
  p.update(0, 1'000'000, 32, 0);
  p.update(1, 9'000'000, 32, 0);
  feed(p, 0, 470.0, 10, 1'000'000, 0);
  feed(p, 1, 1880.0, 10, 9'000'000, 0);
  CHECK(p.timeout_ms(0) > 50);
  CHECK(p.timeout_ms(1) == 50);
}

MTEST_MAIN
