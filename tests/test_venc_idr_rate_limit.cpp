/* ported from waybeam_venc f956a52:tests/test_idr_rate_limit.c */
#include <cstdint>
#include <ctime>

#include "idr_rate_limit.h"
#include "mtest.h"

namespace {

// The gate uses wb_monotonic_us() under the hood (real CLOCK_MONOTONIC, no
// test seam in timing.c — see drone/venc/timing.c) — so the one case that
// needs to cross the spacing boundary sleeps past it with nanosleep()
// rather than mocking the clock. Every other case stays inside the
// lockout window and needs no sleep at all, so this is the only sleep in
// the whole file.
void sleep_us(uint64_t us) {
  struct timespec ts;
  ts.tv_sec = static_cast<time_t>(us / 1000000ULL);
  ts.tv_nsec = static_cast<long>((us % 1000000ULL) * 1000ULL);
  nanosleep(&ts, nullptr);
}

}  // namespace

TEST(idr_first_call_honored) {
  idr_rate_limit_reset();
  CHECK(idr_rate_limit_allow(0) == 1);
  CHECK(idr_rate_limit_honored(0) == 1);
  CHECK(idr_rate_limit_dropped(0) == 0);
}

TEST(idr_burst_coalesced_inside_window) {
  idr_rate_limit_reset();
  CHECK(idr_rate_limit_allow(0) == 1);  // first call always honored
  CHECK(idr_rate_limit_allow(0) == 0);
  CHECK(idr_rate_limit_allow(0) == 0);
  CHECK(idr_rate_limit_allow(0) == 0);
  CHECK(idr_rate_limit_honored(0) == 1);
  CHECK(idr_rate_limit_dropped(0) == 3);
}

TEST(idr_per_channel_independence) {
  idr_rate_limit_reset();
  CHECK(idr_rate_limit_allow(0) == 1);
  // Channel 1 has never fired: its own first call is honored regardless
  // of channel 0's state.
  CHECK(idr_rate_limit_allow(1) == 1);
  CHECK(idr_rate_limit_honored(1) == 1);
  CHECK(idr_rate_limit_honored(0) == 1);
}

TEST(idr_out_of_range_channels_bypass) {
  idr_rate_limit_reset();
  // Out-of-range channel indices always honor (safer fallback than
  // silently dropping a request the caller thinks is real).
  CHECK(idr_rate_limit_allow(-1) == 1);
  CHECK(idr_rate_limit_allow(IDR_RATE_LIMIT_MAX_CHANNELS) == 1);
  CHECK(idr_rate_limit_allow(9999) == 1);
}

TEST(idr_after_spacing_interval_honored) {
  idr_rate_limit_reset();
  CHECK(idr_rate_limit_allow(0) == 1);
  CHECK(idr_rate_limit_allow(0) == 0);
  CHECK(idr_rate_limit_allow(0) == 0);
  CHECK(idr_rate_limit_allow(0) == 0);
  // 100 ms spacing + 10 ms slack for scheduling noise on the test host.
  sleep_us(IDR_RATE_LIMIT_MIN_SPACING_US + 10000);
  CHECK(idr_rate_limit_allow(0) == 1);
  CHECK(idr_rate_limit_honored(0) == 2);
  CHECK(idr_rate_limit_dropped(0) == 3);
}

TEST(idr_reset_clears_counters) {
  idr_rate_limit_reset();
  idr_rate_limit_allow(0);  // honored
  idr_rate_limit_allow(0);  // dropped
  idr_rate_limit_reset();
  CHECK(idr_rate_limit_honored(0) == 0);
  CHECK(idr_rate_limit_dropped(0) == 0);
  CHECK(idr_rate_limit_allow(0) == 1);
}

MTEST_MAIN
