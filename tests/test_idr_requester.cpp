#include "mtest.h"
#include "idr_requester.h"
using maburgs::IdrRequester;

// sid 3 is non-reference enhance (TRAIL_N): loss dips fps, never smears.
// REVERT CHECK: fails if the sid <= 2 gate is removed.
TEST(sid3_loss_never_latches) {
  IdrRequester q;
  CHECK(!q.on_frame_lost(3, 100));
  CHECK(!q.want());
  CHECK(q.episodes() == 0);
}

// The latch is a LEVEL: set on the first reference loss, held through
// further losses and non-IDR emissions, cleared only by a COMPLETE
// IDR-flagged frame. REVERT CHECKS: dropping the `complete` condition makes
// the truncated-IDR check fail; dropping the `idr` condition makes the
// complete-non-IDR check fail; counting episodes per-loss instead of
// per-edge makes the episodes checks fail.
TEST(reference_loss_latches_once_until_idr_clears) {
  IdrRequester q;
  CHECK(q.on_frame_lost(1, 100));               // edge: clear -> set
  CHECK(q.want());
  CHECK(!q.on_frame_lost(0, 150));              // already latched: no new edge
  CHECK(q.episodes() == 1);
  CHECK(q.wait_ms(600) == 500);
  CHECK(!q.on_frame_emitted(true, false, 700)); // complete non-IDR: holds
  CHECK(!q.on_frame_emitted(false, true, 800)); // truncated IDR: holds
  CHECK(q.want());
  CHECK(q.on_frame_emitted(true, true, 900));   // complete IDR: cleared
  CHECK(!q.want());
  CHECK(q.last_wait_ms() == 800);               // 900 - 100
  CHECK(q.wait_ms(1000) == 0);
  CHECK(q.on_frame_lost(2, 1100));              // next glitch: new episode
  CHECK(q.episodes() == 2);
}

// An IDR arriving with NO latch set (e.g. the drone's own entering-LINKED
// IDR) is a no-op, not a spurious "cleared" edge.
TEST(idr_without_latch_is_noop) {
  IdrRequester q;
  CHECK(!q.on_frame_emitted(true, true, 100));
  CHECK(q.episodes() == 0);
}

// The sin fill passes a loop-top clock that can lag the latch stamp taken
// mid-drain by a fresh mono_ms() — wait_ms must clamp, not underflow.
// REVERT CHECK: fails (huge uint64) if the now_ms > since_ms_ clamp is
// removed from wait_ms.
TEST(wait_ms_clamps_when_caller_clock_lags_latch_stamp) {
  IdrRequester q;
  CHECK(q.on_frame_lost(0, 1000));
  CHECK(q.wait_ms(990) == 0);   // caller clock behind the stamp: clamp to 0
  CHECK(q.wait_ms(1000) == 0);  // equal: still 0, no underflow
  CHECK(q.wait_ms(1500) == 500);
}

TEST(player_break_latches_with_its_own_counter) {
  maburgs::IdrRequester r;
  CHECK(r.on_player_break(1000));
  CHECK(r.want());
  CHECK(r.episodes_player() == 1);
  CHECK(r.episodes() == 0);  // wire-loss counter untouched
}

TEST(player_break_does_not_relatch_while_set) {
  maburgs::IdrRequester r;
  CHECK(r.on_player_break(1000));
  CHECK(!r.on_player_break(1100));
  CHECK(r.episodes_player() == 1);
}

TEST(player_break_is_cleared_by_the_same_idr_path) {
  maburgs::IdrRequester r;
  r.on_player_break(1000);
  CHECK(!r.on_frame_emitted(true, false, 1100));  // not an IDR
  CHECK(r.want());
  CHECK(r.on_frame_emitted(true, true, 1300));
  CHECK(!r.want());
  CHECK(r.last_wait_ms() == 300);
}

TEST(the_two_triggers_share_one_latch) {
  // Documented consequence: whichever fires first owns the episode; the
  // other does not open a second one while the latch is up.
  maburgs::IdrRequester r;
  CHECK(r.on_frame_lost(1, 1000));
  CHECK(!r.on_player_break(1010));
  CHECK(r.episodes() == 1);
  CHECK(r.episodes_player() == 0);
}

MTEST_MAIN
