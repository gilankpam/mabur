#include "mtest.h"
#include "player_idr_latch.h"

using maburplay::PlayerIdrLatch;
using R = maburplay::PlayerIdrLatch::Reason;

TEST(starts_clear) {
  PlayerIdrLatch l;
  CHECK(!l.want());
  CHECK(l.episodes() == 0);
  CHECK(l.reason() == R::kNone);
}

TEST(flush_sets_once_but_counts_every_event) {
  PlayerIdrLatch l;
  CHECK(l.on_break(R::kFlush, 1000));   // clear -> set edge
  CHECK(l.want());
  CHECK(!l.on_break(R::kFlush, 1100));  // already set: no new episode
  CHECK(l.episodes() == 1);
  CHECK(l.count(R::kFlush) == 2);       // but both events are counted
}

TEST(kNone_is_not_a_trigger) {
  PlayerIdrLatch l;
  CHECK(!l.on_break(R::kNone, 1000));
  CHECK(!l.want());
  CHECK(l.count(R::kNone) == 0);
}

TEST(only_a_complete_idr_clears) {
  PlayerIdrLatch l;
  l.on_break(R::kFlush, 1000);
  CHECK(!l.on_au_submitted(true, false, 1100));   // complete, not IDR
  CHECK(l.want());
  CHECK(!l.on_au_submitted(false, true, 1200));   // IDR, not complete
  CHECK(l.want());
  CHECK(l.on_au_submitted(true, true, 1300));     // both
  CHECK(!l.want());
  CHECK(l.last_wait_ms() == 300);
}

TEST(flush_then_join_coalesce_into_one_episode) {
  PlayerIdrLatch l;
  CHECK(l.on_break(R::kFlush, 1000));
  CHECK(!l.on_break(R::kJoin, 1010));
  CHECK(l.episodes() == 1);
  CHECK(l.count(R::kFlush) == 1);
  CHECK(l.count(R::kJoin) == 1);
  CHECK(l.reason() == R::kFlush);  // the reason that OPENED the episode
}

TEST(relatches_after_a_clear) {
  PlayerIdrLatch l;
  l.on_break(R::kFlush, 1000);
  l.on_au_submitted(true, true, 1100);
  CHECK(l.on_break(R::kWatchdog, 2000));
  CHECK(l.want());
  CHECK(l.episodes() == 2);
  CHECK(l.reason() == R::kWatchdog);
}

TEST(clock_going_backwards_does_not_underflow) {
  // Same hazard f9c898b fixed in IdrRequester: a caller clock sampled
  // before the latch stamp must not wrap the unsigned subtraction.
  PlayerIdrLatch l;
  l.on_break(R::kFlush, 5000);
  l.on_au_submitted(true, true, 4000);
  CHECK(l.last_wait_ms() == 0);
}

MTEST_MAIN
