#include <map>
#include <optional>
#include <vector>
#include "mabur/nack_tracker.h"
#include "mtest.h"
using namespace mabur;
using S = SwDecoder::SourceState;

namespace {
struct World {
  std::vector<uint32_t> missing;
  std::map<uint32_t, S> st;
  std::optional<NackTailView> tail;
  double util = 0.0;
  NackInputs in() {
    NackInputs i;
    i.missing = [this] { return missing; };
    i.state = [this](uint32_t s) { auto it = st.find(s); return it == st.end() ? S::kDirect : it->second; };
    i.tail = [this] { return tail; };
    i.util = [this] { return util; };
    i.down_util = 0.35;
    i.gap_timeout_ms = 50;
    return i;
  }
  void lose(std::initializer_list<uint32_t> seqs) {
    for (auto s : seqs) { missing.push_back(s); st[s] = S::kUnknown; }
  }
};
NackCfg cfg_on() { NackCfg c; c.enable = true; return c; }
}  // namespace

TEST(disabled_tracker_is_inert) {
  World w; w.lose({10, 11});
  NackTracker t(NackCfg{});
  CHECK(!t.poll(1000, w.in()).has_value());
  CHECK(!t.poll(1100, w.in()).has_value());
  CHECK(t.stats().requests == 0 && t.outstanding() == 0);
  NackCfg c = cfg_on(); c.max_tries = 0;
  NackTracker o(c);
  CHECK(!o.poll(1000, w.in()).has_value());
  CHECK(!o.poll(1100, w.in()).has_value());
  CHECK(o.stats().requests == 0);
}

TEST(gap_trigger_waits_settle_then_requests_runs_base_first) {
  World w; w.lose({100, 101, 140});
  NackTracker t(cfg_on());                 // settle seeded 12
  CHECK(!t.poll(1000, w.in()).has_value());
  CHECK(!t.poll(1011, w.in()).has_value());
  auto n = t.poll(1012, w.in());
  REQUIRE(n.has_value());
  CHECK(n->sid == 0 && n->n == 2 && n->flags == 0 && n->counter == 1);
  CHECK(n->e[0].first_seq == 100 && n->e[0].bitmap == 0x3u);
  CHECK(n->e[1].first_seq == 140 && n->e[1].bitmap == 0x1u);
  CHECK(t.stats().requests == 1 && t.stats().syms_requested == 3);
}

TEST(repeat_sets_flag_and_stops_at_max_tries) {
  World w; w.lose({100});
  NackTracker t(cfg_on());
  CHECK(!t.poll(1000, w.in()).has_value());
  REQUIRE(t.poll(1012, w.in()).has_value());
  CHECK(!t.poll(1027, w.in()).has_value());          // repeat_ms 16 not yet
  auto r = t.poll(1028, w.in());
  REQUIRE(r.has_value());
  CHECK(r->flags == rc::kNackFlagRepeat && r->counter == 2);
  CHECK(t.stats().repeats == 1);
  CHECK(!t.poll(1044, w.in()).has_value());          // max_tries 2 reached
}

TEST(fill_attribution_filled_late_wasted) {
  World w; w.lose({100, 101, 102});
  NackTracker t(cfg_on());
  CHECK(!t.poll(1000, w.in()).has_value());
  REQUIRE(t.poll(1012, w.in()).has_value());
  w.st[100] = S::kRetx; w.st[101] = S::kDirect; w.st[102] = S::kRecovered;
  w.missing.clear();
  t.poll(1025, w.in());
  CHECK(t.stats().filled == 1 && t.stats().late_fill == 1 && t.stats().wasted == 1);
  auto win = t.take_window();
  REQUIRE(win.fill_ms.size() == 1);
  CHECK(win.fill_ms[0] == 13);
  CHECK(t.outstanding() == 0);
}

TEST(deadline_drops_without_retry) {
  World w; w.lose({100});
  NackTracker t(cfg_on());
  CHECK(!t.poll(1000, w.in()).has_value());
  REQUIRE(t.poll(1012, w.in()).has_value());
  CHECK(!t.poll(1100, w.in()).has_value());          // > gap_timeout 50 since first missing
  CHECK(t.stats().dropped_deadline == 1 && t.outstanding() == 0);
}

TEST(stop_rule_suppresses_while_util_high) {
  World w; w.lose({100}); w.util = 0.5;
  NackTracker t(cfg_on());
  CHECK(!t.poll(1000, w.in()).has_value());
  CHECK(!t.poll(1012, w.in()).has_value());
  CHECK(t.stats().suppressed >= 1 && t.stats().requests == 0);
  w.util = 0.1;
  w.lose({200});
  CHECK(!t.poll(1013, w.in()).has_value());          // 200 admitted now, settles 12 ms
  auto n = t.poll(1025, w.in());
  REQUIRE(n.has_value());
  CHECK(n->e[0].first_seq == 200);
}

TEST(stop_killed_repeat_still_books_deadline) {
  World w; w.lose({100});
  NackTracker t(cfg_on());
  CHECK(!t.poll(1000, w.in()).has_value());
  REQUIRE(t.poll(1012, w.in()).has_value());           // tries 1
  w.util = 0.5;
  CHECK(!t.poll(1028, w.in()).has_value());            // repeat due, suppressed -> dead
  CHECK(t.stats().suppressed == 1 && t.stats().requests == 1);
  CHECK(!t.poll(1100, w.in()).has_value());            // past the deadline
  CHECK(t.stats().dropped_deadline == 1);
  w.st[100] = S::kBelowFloor; w.missing.clear();
  t.poll(1200, w.in());                                // terminal: not booked twice
  CHECK(t.stats().dropped_deadline == 1 && t.outstanding() == 0);
}

TEST(adaptive_settle_tracks_natural_lateness) {
  World w;
  NackCfg c = cfg_on(); c.settle_min_samples = 3;
  NackTracker t(c);
  CHECK(t.settle_ms() == 12);
  // three seqs that show up by themselves after 20 ms -> settle = 22
  for (uint32_t s = 300; s < 303; ++s) { w.lose({s}); t.poll(1000, w.in()); w.st[s] = S::kDirect; w.missing.clear(); t.poll(1020, w.in()); }
  CHECK(t.settle_ms() == 22);
  CHECK(t.take_window().late_ms_max == 20);
  // clamp: 40 ms lateness -> settle_max 24
  w.lose({400}); t.poll(2000, w.in()); w.st[400] = S::kDirect; w.missing.clear(); t.poll(2040, w.in());
  CHECK(t.settle_ms() == 24);
}

TEST(tail_trigger_requests_only_inside_count) {
  World w;
  w.tail = NackTailView{6, 2, 902, 1000};     // 6 fragments, highest heard idx 2 at seq 902
  for (uint32_t s = 903; s <= 906; ++s) w.st[s] = S::kUnknown;   // unheard past newest
  NackTracker t(cfg_on());
  CHECK(!t.poll(1005, w.in()).has_value());
  auto n = t.poll(1012, w.in());
  REQUIRE(n.has_value());
  CHECK(n->n == 1 && n->e[0].first_seq == 903 && n->e[0].bitmap == 0x7u);   // 903,904,905 only
  CHECK(t.stats().tail_requests == 3);
  // The retx fills them while the tail view is stale (same slot, max_idx
  // not advanced): no re-admission, no re-request.
  for (uint32_t s = 903; s <= 905; ++s) w.st[s] = S::kRetx;
  CHECK(!t.poll(1013, w.in()).has_value());
  CHECK(!t.poll(1030, w.in()).has_value());
  CHECK(t.stats().filled == 3 && t.stats().tail_requests == 3 && t.stats().requests == 1);
}

TEST(tail_trigger_needs_header) {
  World w; w.tail = std::nullopt;
  NackTracker t(cfg_on());
  CHECK(!t.poll(1012, w.in()).has_value());
  CHECK(t.stats().tail_requests == 0);
}

TEST(clear_drops_entries_but_keeps_counter) {
  // A frame_wire edge (e.g. a >= 1 s video dropout) clears entries, but the
  // drone still holds the last counter under the same vtx nonce: the counter
  // must keep climbing or every NACK after the rejoin is refused.
  World w; w.lose({100});
  NackTracker t(cfg_on());
  CHECK(!t.poll(1000, w.in()).has_value());
  REQUIRE(t.poll(1012, w.in()).has_value());
  t.clear();
  CHECK(t.outstanding() == 0);
  w.missing.clear(); w.st.clear();
  w.lose({500});
  CHECK(!t.poll(2000, w.in()).has_value());
  auto n = t.poll(2012, w.in());
  REQUIRE(n.has_value());
  CHECK(n->counter == 2);
  CHECK(n->n == 1 && n->e[0].first_seq == 500 && n->e[0].bitmap == 0x1u);
}

TEST(restart_counter_starts_at_one) {
  World w; w.lose({100});
  NackTracker t(cfg_on());
  CHECK(!t.poll(1000, w.in()).has_value());
  REQUIRE(t.poll(1012, w.in()).has_value());
  REQUIRE(t.poll(1028, w.in()).has_value());           // counter 2
  t.restart_counter();                                 // new vtx nonce
  w.missing.clear(); w.st.clear();
  w.lose({500});
  t.clear();
  CHECK(!t.poll(2000, w.in()).has_value());
  auto n = t.poll(2012, w.in());
  REQUIRE(n.has_value());
  CHECK(n->counter == 1);
}

TEST(fill_ms_window_is_capped_when_never_drained) {
  // NACK on + sideport off: nobody calls take_window(). The fill_ms sample
  // vector stops appending at kMaxFillSamples; the filled counters still
  // count every fill.
  World w;
  NackTracker t(cfg_on());
  uint64_t now = 1000;
  uint32_t base = 1000;
  uint64_t fills = 0;
  while (fills < NackWindow::kMaxFillSamples + 500) {
    w.missing.clear(); w.st.clear();
    for (uint32_t k = 0; k < 128; ++k) w.lose({base + k});
    CHECK(!t.poll(now, w.in()).has_value());
    REQUIRE(t.poll(now + 12, w.in()).has_value());
    for (uint32_t k = 0; k < 128; ++k) w.st[base + k] = S::kRetx;
    w.missing.clear();
    t.poll(now + 13, w.in());
    fills += 128; base += 128; now += 100;
  }
  CHECK(t.stats().filled == fills);
  auto win = t.take_window();
  CHECK(win.filled == fills);
  CHECK(win.fill_ms.size() == NackWindow::kMaxFillSamples);
}

MTEST_MAIN
