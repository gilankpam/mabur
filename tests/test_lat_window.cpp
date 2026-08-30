#include <cstdint>

#include "lat_window.h"
#include "mtest.h"

using maburgs::LatWindow;

namespace {

// Formula under test (lat_window.h percentile()): idx = min(n-1, n*pct/100)
// on the sorted values via nth_element. For n=100 samples valued 0..99
// (already a linear ramp, so the formula is easy to hand-verify):
//   p50: idx = 100*50/100 = 50  -> value 50
//   p99: idx = 100*99/100 = 99  -> value 99
constexpr int kN = 100;

}  // namespace

// (a) 100 known samples per segment (scaled by a distinct factor so a
// transposed segment index fails loudly), assert p50/p99 per segment.
TEST(known_values_p50_p99) {
  LatWindow w;
  for (int i = 0; i < kN; ++i) {
    w.add(static_cast<uint32_t>(i),       // enc: 0..99
          static_cast<uint32_t>(i * 2),   // dq:  0..198
          static_cast<uint32_t>(i * 3),   // air: 0..297
          static_cast<uint32_t>(i * 4));  // fec: 0..396
  }
  const auto out = w.flush();
  CHECK(out.n == kN);

  CHECK(out.p50[0] == 50);
  CHECK(out.p99[0] == 99);
  CHECK(out.p50[1] == 100);
  CHECK(out.p99[1] == 198);
  CHECK(out.p50[2] == 150);
  CHECK(out.p99[2] == 297);
  CHECK(out.p50[3] == 200);
  CHECK(out.p99[3] == 396);
}

// (b) flush() clears: a second flush with no intervening add() reports
// n==0 and all-zero percentiles.
TEST(flush_clears_window) {
  LatWindow w;
  for (int i = 0; i < kN; ++i)
    w.add(static_cast<uint32_t>(i), static_cast<uint32_t>(i), static_cast<uint32_t>(i),
          static_cast<uint32_t>(i));
  w.flush();

  const auto out2 = w.flush();
  CHECK(out2.n == 0);
  for (int i = 0; i < 4; ++i) {
    CHECK(out2.p50[i] == 0);
    CHECK(out2.p99[i] == 0);
  }
}

// (b2) clear() discards accumulated samples without computing anything --
// the session-reset path (main.cpp, alongside PtsAnchor::reset()) relies on
// this so stale pre-reset samples never mix into the first post-reset
// flush(). No intervening add() after clear(): flush() reports n==0.
TEST(clear_discards_without_flush) {
  LatWindow w;
  for (int i = 0; i < kN; ++i)
    w.add(static_cast<uint32_t>(i), static_cast<uint32_t>(i), static_cast<uint32_t>(i),
          static_cast<uint32_t>(i));
  w.clear();

  const auto out = w.flush();
  CHECK(out.n == 0);
  for (int i = 0; i < 4; ++i) {
    CHECK(out.p50[i] == 0);
    CHECK(out.p99[i] == 0);
  }
}

// (c) revert-check target: a shuffled arrival order must still produce the
// same order statistics as (a) -- nth_element, not insertion order, decides
// the percentile.
TEST(shuffled_arrival_same_percentiles) {
  LatWindow w;
  // Feed descending instead of ascending: same value set as (a)'s enc
  // segment, different arrival order.
  for (int i = kN - 1; i >= 0; --i)
    w.add(static_cast<uint32_t>(i), 0, 0, 0);
  const auto out = w.flush();
  CHECK(out.n == kN);
  CHECK(out.p50[0] == 50);
  CHECK(out.p99[0] == 99);
}

// (d) the cap: adding beyond kCap samples is a silent no-op, not growth or
// a crash -- n never exceeds kCap.
TEST(cap_bounds_the_window) {
  LatWindow w;
  for (std::size_t i = 0; i < LatWindow::kCap + 500; ++i)
    w.add(1, 1, 1, 1);
  const auto out = w.flush();
  CHECK(out.n == static_cast<int>(LatWindow::kCap));
}

MTEST_MAIN
