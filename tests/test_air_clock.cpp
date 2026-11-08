// AirClock: the drone's per-frame virtual air-serialization model. Every
// number below is one worked example: rung 2 = 19.5 Mb/s, efficiency 1.0, 60 fps (16 667 µs period).
#include <thread>

#include "../drone/src/air_clock.h"
#include "mtest.h"

using mabur::AirClock;

namespace {
constexpr uint64_t kPeriodUs = 16667;
constexpr size_t kIdrBytes = 292500;   // 292500 B × 0.410256 µs/B = 120 000 µs
constexpr size_t kBaseOld = 24000;     // encoder still at the OLD rung's rate
constexpr size_t kEnhOld = 12000;

// Frames 1..100 arrive every period after an IDR booked at t=0. Returns
// the backlog each frame saw BEFORE its own booking, and the first frame
// index that found the air free (backlog 0), or -1.
struct Run {
  std::vector<uint32_t> seen;
  int first_free = -1;
};
Run drive(bool keep_enh) {
  AirClock c;
  c.set_rates(19.5, 19.5, 0.0, 0);
  c.book(0, kIdrBytes, 0);
  Run r;
  for (int k = 1; k <= 100; ++k) {
    const uint64_t t = k * kPeriodUs;
    const uint32_t b = c.backlog_us(t);
    r.seen.push_back(b);
    if (b == 0 && r.first_free < 0) r.first_free = k;
    c.book(t, kBaseOld, 0);
    if (keep_enh) c.book(t, kEnhOld, 1);
  }
  return r;
}
}  // namespace

TEST(air_clock_idr_books_120ms_at_rung2) {
  AirClock c;
  c.set_rates(19.5, 19.5, 0.0, 0);
  CHECK(c.backlog_us(0) == 0);
  c.book(0, kIdrBytes, 0);
  const uint32_t b = c.backlog_us(0);
  CHECK(b >= 119990 && b <= 120010);
}

TEST(air_clock_worked_example_enh_kept_drains_in_56_periods) {
  // 14 769 µs booked per 16 667 µs period: net drain ~1.9 ms/period.
  // frame k sees 120000 - k*16667 + (k-1)*14769.
  Run r = drive(/*keep_enh=*/true);
  CHECK(r.seen[0] >= 103300 && r.seen[0] <= 103400);   // frame 1: 103 333
  CHECK(r.seen[5] >= 93700 && r.seen[5] <= 94000);     // frame 6: 93 843
  CHECK(r.first_free == 56);
}

TEST(air_clock_worked_example_enh_dropped_drains_in_17_periods) {
  // base only, 9 846 µs per period: net drain ~6.8 ms/period.
  Run r = drive(/*keep_enh=*/false);
  CHECK(r.seen[0] >= 103300 && r.seen[0] <= 103400);
  CHECK(r.seen[4] >= 76000 && r.seen[4] <= 76300);     // frame 5: 76 049
  CHECK(r.seen[5] >= 69100 && r.seen[5] <= 69400);     // frame 6: 69 228
  CHECK(r.first_free == 17);
}

TEST(air_clock_leaks_to_zero_and_restarts_from_now) {
  AirClock c;
  c.set_rates(19.5, 19.5, 0.0, 0);
  c.book(0, 1000, 0);                 // 410 µs
  CHECK(c.backlog_us(0) == 410);
  CHECK(c.backlog_us(10000) == 0);    // idle past free_at: clamped, no debt
  c.book(10000, 1000, 0);             // starts at NOW, not at old free_at
  CHECK(c.backlog_us(10000) == 410);
}

TEST(air_clock_reprices_only_later_bodies) {
  AirClock c;
  c.set_rates(52.0, 52.0, 0.0, 0);   // 0.1538 µs/B
  c.book(0, 10000, 0);                    // 1538 µs
  c.set_rates(6.5, 6.5, 0.0, 0);     // demote to 1.2308 µs/B
  c.book(0, 10000, 0);                    // +12308
  const uint32_t b = c.backlog_us(0);
  CHECK(b >= 13840 && b <= 13850);
}

TEST(air_clock_efficiency_and_body_us_scale_the_cost) {
  AirClock c;
  c.set_rates(9.75, 9.75, 0.0, 100);   // half the rate (delivered), +100 µs per body
  c.book(0, 1000, 1);                       // 1000 × 0.8205 + 100 = 920.5 -> 921
  const uint32_t b = c.backlog_us(0);
  CHECK(b >= 920 && b <= 921);
}

TEST(air_clock_probe_uses_probe_rate_and_ignores_unknown_rate) {
  AirClock c;
  c.set_rates(19.5, 19.5, 0.0, 0);   // probe off: rate 0
  c.book(0, 1000, AirClock::kProbeSid);
  CHECK(c.backlog_us(0) == 0);            // not booked
  c.set_rates(19.5, 19.5, 6.5, 0);   // probe at mcs0
  c.book(0, 1000, AirClock::kProbeSid);   // 1231 µs
  const uint32_t b = c.backlog_us(0);
  CHECK(b >= 1230 && b <= 1232);
  c.book(0, 1000, 7);                     // bad sid: ignored
  CHECK(c.backlog_us(0) == b);
}

TEST(air_clock_backlog_saturates_u32) {
  AirClock c;
  c.set_rates(19.5, 19.5, 0.0, 4000000000u);   // absurd body_us
  c.book(0, 1, 0);
  c.book(0, 1, 0);                                  // ~8e9 µs ahead
  CHECK(c.backlog_us(0) == 0xFFFFFFFFu);
}

// fec-nack: the RX thread books retransmit bodies
// while the hot thread books video and reads the backlog. free_at_us_ is
// mutex-guarded; this is the shape (run it under -fsanitize=thread to see
// the race the lock closes).
TEST(air_clock_book_is_safe_from_two_threads) {
  AirClock c;
  c.set_rates(19.5, 19.5, 0.0, 0);
  std::thread t([&] { for (int i = 0; i < 10000; ++i) c.book(1000 + i, 1400, 0); });
  uint64_t seen = 0;   // consumed, so the reads cannot be optimized away
  for (int i = 0; i < 10000; ++i) seen += c.backlog_us(1000 + i);
  t.join();
  CHECK(c.backlog_us(0) > 0);
  CHECK(seen < 0xFFFFFFFFull * 10000);
}

// MCS0 at 20 MHz with the bench efficiency, 256 bodies of 1328 B: the queue
// holds 475 ms of air. The numbers below follow from that.
TEST(air_clock_backlog_is_bounded_by_what_the_queue_holds) {
  constexpr size_t kBody = 1328;
  AirClock bounded(256);
  AirClock unbounded;
  bounded.set_rates(5.72, 5.72, 0.0, 0);
  unbounded.set_rates(5.72, 5.72, 0.0, 0);
  // Overload: 100 000 bodies pushed at once, most of them dropped by the queue.
  for (int i = 0; i < 100000; ++i) {
    bounded.book(0, kBody, 0);
    unbounded.book(0, kBody, 0);
  }
  CHECK(unbounded.backlog_us(0) > 100000000u);
  const uint32_t b = bounded.backlog_us(0);
  CHECK(b >= 475000 && b <= 476000);
  // The air is really free once the queue has drained: the enh gate reopens.
  CHECK(bounded.backlog_us(476000) == 0);
  CHECK(unbounded.backlog_us(476000) > 100000000u);
}

TEST(air_clock_bound_leaves_a_backlog_under_it_alone) {
  constexpr size_t kBody = 1328;
  AirClock bounded(256);
  AirClock unbounded;
  bounded.set_rates(5.72, 5.72, 0.0, 0);
  unbounded.set_rates(5.72, 5.72, 0.0, 0);
  for (int i = 0; i < 200; ++i) {
    bounded.book(0, kBody, 0);
    unbounded.book(0, kBody, 0);
  }
  CHECK(bounded.backlog_us(0) == unbounded.backlog_us(0));
}

TEST(air_clock_a_probe_body_does_not_pull_the_bound_in) {
  constexpr size_t kBody = 1328;
  AirClock c(256);
  c.set_rates(5.72, 5.72, 65.0, 0);
  for (int i = 0; i < 1000; ++i) c.book(0, kBody, 0);
  const uint32_t full = c.backlog_us(0);
  c.book(0, 64, AirClock::kProbeSid);
  CHECK(c.backlog_us(0) >= full);
}

MTEST_MAIN
