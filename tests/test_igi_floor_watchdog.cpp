// IgiFloorWatchdog: raises a Jaguar3's IGI floor in steps while its 40 MHz
// sends stall, keeps the first level that clears them, and undoes a climb
// that never did.
#include <vector>
#include "mtest.h"
#include "igi_floor_watchdog.h"
using namespace mabur;
using E = IgiFloorWatchdog::Event;

constexpr uint64_t kSlow = 50000, kFast = 2000;  // us

struct Rig {
  IgiFloorParams p; uint64_t now = 1000; IgiFloorWatchdog w;
  std::vector<IgiFloorWatchdog::Result> ev;
  explicit Rig(uint8_t base = 0x22, uint8_t cap = 0x35)
      : p{igi_floor_params(base, nullptr, nullptr)}, w{(p.cap = cap, p), now} {}
  void ticks(int n, bool eligible, uint64_t us, uint8_t q = 0, int busy = -1) {
    for (int i = 0; i < n; ++i) {
      now += 100;
      auto r = w.step(now, eligible, us, q, busy);
      if (r.event != IgiFloorWatchdog::Event::None) ev.push_back(r);
    }
  }
  void secs(int n, bool eligible, uint64_t us, uint8_t q = 0, int busy = -1) {
    ticks(10 * n, eligible, us, q, busy);
  }
};

static bool is(const IgiFloorWatchdog::Result& r, E e, uint8_t level) {
  return r.event == e && r.level == level;
}

TEST(one_slow_window_does_not_trigger) {
  Rig r; r.secs(1, true, kSlow); r.secs(5, true, kFast);
  CHECK(r.ev.empty()); CHECK(r.w.level() == 0x22);
}

TEST(two_of_three_slow_starts_a_climb) {
  Rig r; r.secs(1, true, kSlow); r.secs(1, true, kFast); r.secs(1, true, kSlow);
  REQUIRE(r.ev.size() == 1);
  CHECK(r.ev[0].event == E::Raised); CHECK(r.ev[0].from == 0x22);
  CHECK(r.ev[0].level == 0x24); CHECK(r.ev[0].longest_ms == 50);
}

TEST(slow_counts_across_ignored_windows) {
  Rig r; r.secs(1, true, kSlow); r.secs(10, false, kSlow); r.secs(1, true, kSlow);
  REQUIRE(r.ev.size() == 1); CHECK(is(r.ev[0], E::Raised, 0x24));
}

TEST(slow_older_than_30s_is_forgotten) {
  Rig r; r.secs(1, true, kSlow); r.secs(31, false, kSlow); r.secs(1, true, kSlow);
  CHECK(r.ev.empty());
  r.secs(1, true, kSlow);
  REQUIRE(r.ev.size() == 1); CHECK(is(r.ev[0], E::Raised, 0x24));
}

TEST(climb_steps_and_failed_episode) {
  Rig r; r.secs(8, true, kSlow);
  REQUIRE(r.ev.size() == 5);
  const uint8_t up[] = {0x24, 0x28, 0x30, 0x35};
  for (int i = 0; i < 4; ++i) CHECK(is(r.ev[i], E::Raised, up[i]));
  CHECK(r.ev[4].event == E::Reverted); CHECK(r.ev[4].from == 0x35);
  CHECK(r.ev[4].level == 0x22); CHECK(r.ev[4].hold_s == 30);
  CHECK(r.w.level() == 0x22);
}

TEST(climb_steps_cu) {
  Rig r(0x26); r.secs(8, true, kSlow);
  REQUIRE(r.ev.size() == 5);
  const uint8_t up[] = {0x28, 0x2c, 0x34, 0x35};
  for (int i = 0; i < 4; ++i) CHECK(is(r.ev[i], E::Raised, up[i]));
  CHECK(is(r.ev[4], E::Reverted, 0x26));
}

TEST(two_clean_windows_keep_the_level) {
  Rig r; r.secs(2, true, kSlow); r.secs(4, true, kFast);
  REQUIRE(r.ev.size() == 2);
  CHECK(is(r.ev[0], E::Raised, 0x24)); CHECK(is(r.ev[1], E::Kept, 0x24));
  r.secs(20, true, kFast);
  CHECK(r.ev.size() == 2); CHECK(r.w.level() == 0x24);
}

TEST(next_episode_starts_at_plus_two) {
  Rig r; r.secs(2, true, kSlow); r.secs(3, true, kFast); r.secs(20, true, kFast);
  r.secs(2, true, kSlow);
  REQUIRE(r.ev.size() == 3);
  CHECK(r.ev[2].event == E::Raised); CHECK(r.ev[2].from == 0x24); CHECK(r.ev[2].level == 0x26);
}

TEST(clean_then_slow_keeps_climbing) {
  Rig r; r.secs(2, true, kSlow); r.ticks(13, true, kFast); r.ticks(10, true, kSlow);
  REQUIRE(r.ev.size() == 2);
  CHECK(is(r.ev[0], E::Raised, 0x24)); CHECK(is(r.ev[1], E::Raised, 0x28));
}

TEST(episode_without_windows_is_abandoned) {
  Rig r; r.secs(2, true, kSlow); r.ticks(31, false, kFast);
  REQUIRE(r.ev.size() == 2);
  CHECK(is(r.ev[1], E::Reverted, 0x22)); CHECK(r.ev[1].hold_s == 30);
}

TEST(hold_blocks_and_doubles) {
  Rig r; r.secs(8, true, kSlow);
  REQUIRE(r.ev.size() == 5);
  CHECK(r.ev[4].event == E::Reverted); CHECK(r.ev[4].hold_s == 30);
  r.secs(25, true, kSlow);                          // inside the 30 s hold
  CHECK(r.ev.size() == 5);
  r.secs(800, true, kSlow);
  std::vector<uint32_t> holds;
  for (size_t i = 5; i < r.ev.size(); ++i)
    if (r.ev[i].event == E::Reverted) holds.push_back(r.ev[i].hold_s);
  CHECK((holds == std::vector<uint32_t>{60, 120, 240, 300, 300}));
}

TEST(success_resets_the_hold) {
  Rig r; r.secs(8, true, kSlow);                   // failed episode, hold 30 s
  REQUIRE(r.ev.size() == 5);
  r.secs(31, true, kFast); r.ticks(5, true, kFast); // past the hold, on a window edge
  r.secs(2, true, kSlow); r.secs(4, true, kFast);
  REQUIRE(r.ev.size() == 7);
  CHECK(is(r.ev[5], E::Raised, 0x24)); CHECK(is(r.ev[6], E::Kept, 0x24));
  r.secs(5, true, kFast);
  r.secs(8, true, kSlow);
  REQUIRE(r.ev.size() == 12);
  CHECK(is(r.ev.back(), E::Reverted, 0x24)); CHECK(r.ev.back().hold_s == 30);
}

TEST(not_eligible_never_triggers) {
  Rig r; r.secs(60, false, kSlow); CHECK(r.ev.empty());
}

TEST(at_cap_slow_windows_do_nothing) {
  Rig r(0x33); r.secs(2, true, kSlow);
  r.ticks(13, true, kFast); r.ticks(10, true, kFast); r.ticks(10, true, kFast);
  r.secs(60, true, kSlow);
  REQUIRE(r.ev.size() == 2);
  CHECK(is(r.ev[0], E::Raised, 0x35)); CHECK(is(r.ev[1], E::Kept, 0x35));
}

TEST(watchdog_off_when_cap_equals_base) {
  Rig r(0x35, 0x35); r.secs(60, true, kSlow);
  CHECK(r.ev.empty()); CHECK(r.w.level() == 0x35);
}

TEST(late_tick_is_one_ignored_window) {
  Rig r; r.now += 5000;
  CHECK(r.w.step(r.now, true, kSlow, 0, -1).event == E::None);
  r.secs(1, true, kSlow);
  CHECK(r.ev.empty());
}

TEST(queue_backlog_alone_starts_a_climb) {
  Rig r; r.secs(2, true, kFast, 90);
  REQUIRE(r.ev.size() == 1);
  CHECK(r.ev[0].event == E::Raised); CHECK(r.ev[0].from == 0x22);
  CHECK(r.ev[0].level == 0x24); CHECK(r.ev[0].backlog_pct == 90);
  CHECK(r.ev[0].longest_ms == 2);
}

TEST(queue_below_threshold_is_clean) {
  Rig r; r.secs(60, true, kFast, 59); CHECK(r.ev.empty());
}

TEST(queue_backlog_blocks_the_keep) {
  Rig r; r.secs(2, true, kFast, 90);
  r.ticks(23, true, kFast, 70);   // 70 % is still slow: climbs on, never keeps
  REQUIRE(r.ev.size() == 2);
  CHECK(is(r.ev[0], E::Raised, 0x24)); CHECK(is(r.ev[1], E::Raised, 0x28));
}

TEST(one_backlog_tick_makes_the_window_slow) {
  Rig r;
  for (int s = 0; s < 2; ++s) { r.ticks(9, true, kFast, 0); r.ticks(1, true, kFast, 95); }
  REQUIRE(r.ev.size() == 1); CHECK(is(r.ev[0], E::Raised, 0x24));
}

TEST(queue_signal_off_when_pct_zero) {
  Rig r; r.p.queue_slow_pct = 0; r.w = IgiFloorWatchdog(r.p, r.now);
  r.secs(60, true, kFast, 99); CHECK(r.ev.empty());
}

TEST(busy_channel_never_starts_an_episode) {
  Rig r; r.secs(60, true, kSlow, 0, 50); CHECK(r.ev.empty());
}

TEST(busy_below_gate_still_triggers) {
  Rig r; r.secs(2, true, kSlow, 0, 19);
  REQUIRE(r.ev.size() == 1);
  CHECK(is(r.ev[0], E::Raised, 0x24)); CHECK(r.ev[0].busy_pct == 19);
}

TEST(busy_during_climb_abandons) {
  Rig r; r.secs(2, true, kSlow); r.secs(4, true, kSlow, 0, 50);
  REQUIRE(r.ev.size() == 2);
  CHECK(is(r.ev[0], E::Raised, 0x24));
  CHECK(is(r.ev[1], E::Reverted, 0x22)); CHECK(r.ev[1].hold_s == 30);
}

TEST(two_clean_windows_do_not_keep) {
  Rig r; r.secs(2, true, kSlow); r.ticks(23, true, kFast);
  REQUIRE(r.ev.size() == 1); CHECK(is(r.ev[0], E::Raised, 0x24));
}

TEST(three_clean_windows_keep) {
  Rig r; r.secs(2, true, kSlow); r.ticks(33, true, kFast);
  REQUIRE(r.ev.size() == 2); CHECK(is(r.ev[1], E::Kept, 0x24));
}

TEST(murky_resets_the_clean_run) {
  Rig r; r.secs(2, true, kSlow);
  r.ticks(23, true, kFast, 0); r.secs(1, true, kFast, 45); r.secs(2, true, kFast, 0);
  for (const auto& e : r.ev) CHECK(e.event != E::Kept);
}

TEST(murky_alone_abandons) {
  Rig r; r.secs(2, true, kSlow); r.secs(4, true, kFast, 45);
  REQUIRE(r.ev.size() == 2); CHECK(is(r.ev[1], E::Reverted, 0x22));
}

TEST(busy_gate_off_when_zero) {
  Rig r; r.p.busy_max_pct = 0; r.w = IgiFloorWatchdog(r.p, r.now);
  r.secs(2, true, kSlow, 0, 90);
  REQUIRE(r.ev.size() == 1); CHECK(is(r.ev[0], E::Raised, 0x24));
}

TEST(params_from_env) {
  auto p = igi_floor_params(0x22, nullptr, nullptr);  CHECK(p.base == 0x22); CHECK(p.cap == 0x35);
  p = igi_floor_params(0x22, "0x1e", nullptr);        CHECK(p.base == 0x1e); CHECK(p.cap == 0x35);
  p = igi_floor_params(0x22, "30", "0x28");           CHECK(p.base == 30);   CHECK(p.cap == 0x28);
  p = igi_floor_params(0x22, "0x30", "0x28");         CHECK(p.base == 0x30); CHECK(p.cap == 0x30);
  for (const char* bad : {"abc", "", "0x80", "-1", "0x22z"}) {
    p = igi_floor_params(0x22, bad, bad);             CHECK(p.base == 0x22); CHECK(p.cap == 0x35);
  }
}

MTEST_MAIN
