#include "hop_blank.h"
#include "mtest.h"
using namespace maburgs;

static VerdictOut W(double t_ms, bool frozen) {
  VerdictOut o;
  o.t_start_ms = t_ms - 150;
  o.t_ms = t_ms;
  o.ref_frozen = frozen;
  return o;
}

// Spec section 4: the rung store's EWMAs are blanked "from the first
// impaired window", not from the hop order. As built the blank started in
// main.cpp's HopAction::Order handler, which is 2-3 persistence windows
// (300-450 ms) later -- so every detection window, including the demotes
// the spec explicitly expects ("a demote or two, each an IDR"), was
// written into the per-rung store against the interfered channel. That is
// exactly the channel-specific pollution blank_store() exists to prevent.
TEST(no_blank_while_the_link_is_clean) {
  CHECK(!hop_store_blank_until(W(1000, /*frozen=*/false), 150).has_value());
}

TEST(blank_opens_at_the_first_impaired_window) {
  const auto until = hop_store_blank_until(W(1000, /*frozen=*/true), 150);
  REQUIRE(until.has_value());
  CHECK(*until == 1000 + 150 + kHopSettleBlankMs);
}

// A rolling deadline: each frozen window pushes it a window plus the
// settle ahead, so the blank never lapses between windows...
TEST(deadline_rolls_forward_and_never_lapses_between_windows) {
  double prev = *hop_store_blank_until(W(1000, true), 150);
  for (double t = 1150; t <= 1600; t += 150) {
    const double now = *hop_store_blank_until(W(t, true), 150);
    CHECK(now > prev);
    CHECK(prev >= t);   // the previous window's deadline still covers this one
    prev = now;
  }
}

// ...and ends on its own once the references thaw (3 healthy windows, or
// HopVerdict::reset() after a hop's verify window ends): nothing extends
// it, so it expires one settle past the last frozen window. No separate
// "unblank" call, which blank_store() could not honour anyway -- it keeps
// the LATER of the deadlines it is given.
TEST(blank_expires_a_settle_after_the_last_frozen_window) {
  const double until = *hop_store_blank_until(W(1000, true), 150);
  CHECK(!hop_store_blank_until(W(1150, /*frozen=*/false), 150).has_value());
  CHECK(until == 1300);   // 1000 + window 150 + settle 150
}

// window_ms is a config knob (50-2000); the lead has to follow it or a
// long window leaves an unblanked gap between windows.
TEST(lead_follows_the_configured_window) {
  CHECK(*hop_store_blank_until(W(5000, true), 2000) == 5000 + 2000 + kHopSettleBlankMs);
}
MTEST_MAIN
