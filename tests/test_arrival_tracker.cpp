// ArrivalTracker (common/include/mabur/arrival_tracker.h): pre-FEC loss
// booked at arrival time in the decoder's virtual-seq space. Spec
// docs/superpowers/specs/2026-09-05-arrival-loss-design.md §3.
#include "mabur/arrival_tracker.h"
#include "mtest.h"
using mabur::ArrivalTracker;

namespace {
// Feed seqs [0, n) in order, skipping any in `drop`, then advance to n-1.
void feed(ArrivalTracker& t, uint64_t n, std::initializer_list<uint64_t> drop,
          uint64_t stale_end = 0) {
  for (uint64_t v = 0; v < n; ++v) {
    bool d = false;
    for (auto x : drop) d = d || x == v;
    if (!d) t.on_source(v, stale_end);
    t.advance(v, stale_end);
  }
}
}  // namespace

TEST(clean_stream_books_everything_arrived) {
  ArrivalTracker t(32);
  feed(t, 100, {});
  // newest 99, guard 32: seqs 0..67 settled.
  CHECK(t.expected() == 68);
  CHECK(t.arrived() == 68);
  CHECK(t.expected_stale() == 0);
  CHECK(t.late() == 0);
}

TEST(nothing_booked_inside_the_guard) {
  ArrivalTracker t(32);
  feed(t, 32, {});  // newest 31: 31 - 32 < 0, nothing settled
  CHECK(t.expected() == 0);
  feed(t, 33, {});  // re-feeding 0..32 is idempotent; newest 32 settles seq 0
  CHECK(t.expected() == 1);
  CHECK(t.arrived() == 1);
}

TEST(single_and_burst_loss) {
  ArrivalTracker t(32);
  feed(t, 100, {3, 40, 41, 42});
  CHECK(t.expected() == 68);
  CHECK(t.arrived() == 64);
}

TEST(reorder_inside_guard_is_arrived_not_late) {
  ArrivalTracker t(32);
  for (uint64_t v = 0; v < 50; ++v) {
    if (v == 10) continue;
    t.on_source(v, 0);
    t.advance(v, 0);
  }
  t.on_source(10, 0);  // newest 49: seqs 0..17 booked, seq 10 among them -> late
  CHECK(t.late() == 1);
  ArrivalTracker u(32);
  for (uint64_t v = 0; v < 40; ++v) {
    if (v == 10) continue;
    u.on_source(v, 0);
    u.advance(v, 0);
  }
  u.on_source(10, 0);  // newest 39: seqs 0..7 booked, seq 10 NOT yet booked -> counts arrived
  u.advance(60, 0);
  CHECK(u.late() == 0);
  CHECK(u.expected() == 29);  // 0..28
  CHECK(u.arrived() == 29);
}

TEST(stale_split_open_boundary_books_all_stale) {
  ArrivalTracker t(32);
  feed(t, 100, {5, 6}, ~0ull);
  CHECK(t.expected() == 68);
  CHECK(t.arrived() == 66);
  CHECK(t.expected_stale() == 68);
  CHECK(t.arrived_stale() == 66);
}

TEST(stale_split_closed_watermark) {
  ArrivalTracker t(32);
  // wm = 19 -> stale_end 20: seqs 0..19 stale, 20.. current. Drop 5 (stale) and 30 (current).
  feed(t, 100, {5, 30}, 20);
  CHECK(t.expected() == 68);
  CHECK(t.arrived() == 66);
  CHECK(t.expected_stale() == 20);
  CHECK(t.arrived_stale() == 19);
  // current-only view: 48 expected, 47 arrived
  CHECK(t.expected() - t.expected_stale() == 48);
  CHECK(t.arrived() - t.arrived_stale() == 47);
}

TEST(repair_only_advance_books_expectation_without_arrival) {
  ArrivalTracker t(32);
  feed(t, 10, {});
  t.advance(60, 0);  // a repair's wend-1 announces seqs up to 60 were sent
  CHECK(t.expected() == 29);  // 0..28 settled
  CHECK(t.arrived() == 10);   // only 0..9 heard
}

TEST(reset_keeps_counters_monotonic) {
  ArrivalTracker t(32);
  feed(t, 100, {3});
  const auto e = t.expected(), a = t.arrived();
  t.reset(5000);
  CHECK(t.expected() == e);
  CHECK(t.arrived() == a);
  // in-flight seqs at the reset are never booked; the new anchor starts clean
  for (uint64_t v = 5000; v < 5100; ++v) { t.on_source(v, 0); t.advance(v, 0); }
  CHECK(t.expected() == e + 68);
  CHECK(t.arrived() == a + 68);
  CHECK(t.late() == 0);
}

TEST(jump_beyond_ring_settles_forward_without_aliasing) {
  ArrivalTracker t(32, 1024);
  feed(t, 10, {});
  // 3000 is further ahead than the ring holds: on_source first settles the
  // line to 1977 (booking 0..1976, reading the ten heard bits correctly and
  // clearing them so nothing aliases), then sets 3000's bit.
  t.on_source(3000, 0);
  t.advance(3000, 0);  // settles to 2969: seqs 0..2968 booked
  CHECK(t.expected() == 2969);
  CHECK(t.arrived() == 10);
  CHECK(t.late() == 0);
  // 3000 itself is inside the guard and still heard: settle past it.
  t.advance(3040, 0);
  CHECK(t.expected() == 3009);
  CHECK(t.arrived() == 11);
}

MTEST_MAIN
