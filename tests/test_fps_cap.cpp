#include "mtest.h"
#include "fps_cap.h"
#include <cstdint>
#include <vector>
using namespace maburplay;

// Feed arrivals (us) through a cap and count admits. Returns admits in the
// LAST `window_us` of the run, so start-up transients do not count.
static int run(FpsCap& cap, const std::vector<int64_t>& arrivals, int64_t window_us, int* total) {
  int n = 0, tail = 0;
  const int64_t t_end = arrivals.empty() ? 0 : arrivals.back();
  for (int64_t t : arrivals) {
    if (cap.admit(t)) {
      ++n;
      if (t > t_end - window_us) ++tail;
    }
  }
  if (total) *total = n;
  return tail;
}

// A 60 fps source whose decoded frames arrive in pairs: one on time, the
// next 10 ms later instead of 16.7 -- the pattern the GS pump produces. The
// old last-admit rule (interval/8 slack) rejected every early half.
static std::vector<int64_t> paired_60fps(int64_t seconds) {
  std::vector<int64_t> v;
  for (int64_t i = 0; i < seconds * 30; ++i) {
    const int64_t base = i * 33334;
    v.push_back(base);
    v.push_back(base + 10000);
  }
  return v;
}

TEST(cap_60_admits_every_frame_of_a_paired_60fps_source) {
  FpsCap cap;
  cap.reset(60);
  int total = 0;
  run(cap, paired_60fps(10), 1000000, &total);
  CHECK(total == 600);
}

TEST(cap_60_admits_every_frame_of_a_regular_59_94_source) {
  FpsCap cap;
  cap.reset(60);
  std::vector<int64_t> v;
  for (int i = 0; i < 600; ++i) v.push_back(i * 16683);
  int total = 0;
  run(cap, v, 1000000, &total);
  CHECK(total == 600);
}

// The original slack motivation: 30 fps cap on a 59.94 fps source must give
// 30, not 20 (a strict >= interval rule waits an extra source frame).
TEST(cap_30_on_59_94_source_holds_30) {
  FpsCap cap;
  cap.reset(30);
  std::vector<int64_t> v;
  for (int i = 0; i < 1200; ++i) v.push_back(i * 16683);
  const int tail = run(cap, v, 1000000, nullptr);
  CHECK(tail >= 29 && tail <= 31);
}

// Proportional-slack motivation: cap 50 on 59.94 must NOT run the encoder
// at 59.94. Bounded burst at start is fine; the long-run rate is the cap.
TEST(cap_50_on_59_94_source_holds_50_not_60) {
  FpsCap cap;
  cap.reset(50);
  std::vector<int64_t> v;
  for (int i = 0; i < 1200; ++i) v.push_back(i * 16683);
  int total = 0;
  const int tail = run(cap, v, 1000000, &total);
  CHECK(tail >= 49 && tail <= 51);
  CHECK(total <= 50 * 20 + 2);
}

TEST(cap_60_on_120fps_source_holds_60) {
  FpsCap cap;
  cap.reset(60);
  std::vector<int64_t> v;
  for (int i = 0; i < 2400; ++i) v.push_back(i * 8333);
  int total = 0;
  const int tail = run(cap, v, 1000000, &total);
  CHECK(tail >= 59 && tail <= 61);
  CHECK(total <= 60 * 20 + 2);
}

TEST(source_slower_than_cap_is_untouched) {
  FpsCap cap;
  cap.reset(60);
  std::vector<int64_t> v;
  for (int i = 0; i < 300; ++i) v.push_back(i * 33333);
  int total = 0;
  run(cap, v, 1000000, &total);
  CHECK(total == 300);
}

// A stream gap must not bank credit: after a 2 s hole, a 120 fps burst is
// still capped at 60 (plus the bounded start-up transient).
TEST(a_gap_does_not_bank_credit) {
  FpsCap cap;
  cap.reset(60);
  std::vector<int64_t> v;
  for (int i = 0; i < 120; ++i) v.push_back(i * 16667);
  const int64_t resume = 120 * 16667 + 2000000;
  for (int i = 0; i < 240; ++i) v.push_back(resume + i * 8333);
  int admitted_after = 0;
  for (int64_t t : v)
    if (cap.admit(t) && t >= resume) ++admitted_after;
  CHECK(admitted_after <= 120 + 2);
  CHECK(admitted_after >= 119);
}

MTEST_MAIN
