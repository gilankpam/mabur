#include "mtest.h"
#include "../drone/src/air_balancer.h"
using namespace mabur;

static void feed_frames(AirBalancer& b, double len_b, double mult_b,
                        double len_e, double mult_e, int n = 64) {
  for (int i = 0; i < n; ++i) {
    b.on_frame(0, (size_t)len_b, (size_t)(len_b * mult_b));
    b.on_frame(1, (size_t)len_e, (size_t)(len_e * mult_e));
  }
}

TEST(unseeded_returns_commanded_both) {
  AirBalancer b(nullptr);
  auto s = b.solve(39.0, 52.0, 0.5);
  CHECK(s.ov_base == 0.5);
  CHECK(s.ov_enh == 0.5);
}

TEST(equal_rates_compensates_len_asymmetry) {
  AirBalancer b(nullptr);
  // enh frames 2x base (spike-1 scene), same rate, framing == nominal.
  feed_frames(b, 1000, 1.5, 2000, 1.5);
  auto s = b.solve(52.0, 52.0, 0.5);
  // base must get MORE overhead (stretch), enh LESS (shrink):
  CHECK(s.ov_base > 0.5);
  CHECK(s.ov_enh < 0.5);
  // budget invariant: len_b*ov_b + len_e*ov_e == (len_b+len_e)*ov_cmd
  CHECK(std::abs(1000 * s.ov_base + 2000 * s.ov_enh - 3000 * 0.5) < 1.0);
}

TEST(split_rates_equal_len_balances_air) {
  AirBalancer b(nullptr);
  feed_frames(b, 2000, 1.5, 2000, 1.5);       // equal sizes
  auto s = b.solve(39.0, 52.0, 0.5);          // base slower
  // solved air must match within 1%:
  double air_b = 2000 * (1.5 + (s.ov_base - 0.5)) / 39.0;
  double air_e = 2000 * (1.5 + (s.ov_enh - 0.5)) / 52.0;
  CHECK(std::abs(air_b - air_e) / air_e < 0.01);
}

TEST(anchoring_cancels_framing_bias) {
  AirBalancer b(nullptr);
  // base stream secretly emits 30% more than nominal (framing/quantization)
  feed_frames(b, 2000, 1.5 * 1.3, 2000, 1.5);
  auto s1 = b.solve(52.0, 52.0, 0.5);
  // apply, then measure again with the same hidden bias and re-solve:
  feed_frames(b, 2000, (1.0 + s1.ov_base) * 1.3, 2000, 1.0 + s1.ov_enh);
  auto s2 = b.solve(52.0, 52.0, 0.5);
  double air_b = 2000 * ((1.0 + s2.ov_base) * 1.3) / 52.0;
  double air_e = 2000 * (1.0 + s2.ov_enh) / 52.0;
  CHECK(std::abs(air_b - air_e) / air_e < 0.05);  // converged despite bias
}

TEST(rails_clamp_and_resolve) {
  AirBalancer b(nullptr);
  feed_frames(b, 500, 1.5, 4000, 1.5);  // extreme asymmetry
  auto s = b.solve(52.0, 52.0, 0.5);
  CHECK(s.ov_base <= 1.0 + 1e-9);   // 2x rail
  CHECK(s.ov_base >= 0.25 - 1e-9);  // 0.5x rail
  CHECK(s.ov_enh <= 1.0 + 1e-9);
  CHECK(s.ov_enh >= 0.25 - 1e-9);
}

TEST(publishes_feed) {
  BalancerFeed f;
  AirBalancer b(&f);
  feed_frames(b, 1000, 1.6, 2000, 1.5);
  auto s = b.solve(52.0, 52.0, 0.5);
  CHECK(std::abs(f.share_base.load() - 1000.0 / 3000.0) < 0.01);
  CHECK(f.excess_base.load() > 0.05f);            // 1.6 - 1.5
  CHECK(std::abs(f.ov_base.load() - s.ov_base) < 1e-3);
}

MTEST_MAIN
