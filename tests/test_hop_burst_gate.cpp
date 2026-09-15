#include "hop_burst_gate.h"
#include "mtest.h"
using namespace maburgs;

// The four properties task-15-brief's fix round 1 found wrong at various
// points in this plan: Idle/Hold allowed, Ordered/Verifying excluded, the
// dwell_period_ms rate limit enforced across ticks, and the very first
// burst not delayed by the caller's initial last_burst_ms == -1e18.

TEST(idle_and_hold_with_trigger_and_no_prior_burst_are_due) {
  CHECK(hop_burst_due(HopState::Idle, /*trigger=*/true, /*now_ms=*/0,
                      /*last_burst_ms=*/-1e18, /*dwell_period_ms=*/333));
  CHECK(hop_burst_due(HopState::Hold, true, 0, -1e18, 333));
}

TEST(ordered_and_verifying_are_never_due) {
  CHECK(!hop_burst_due(HopState::Ordered, true, 0, -1e18, 333));
  CHECK(!hop_burst_due(HopState::Verifying, true, 0, -1e18, 333));
}

TEST(no_trigger_is_never_due_even_when_state_and_timing_allow) {
  CHECK(!hop_burst_due(HopState::Idle, /*trigger=*/false, 100000, -1e18, 333));
  CHECK(!hop_burst_due(HopState::Hold, /*trigger=*/false, 100000, -1e18, 333));
}

TEST(rate_limit_enforced_across_successive_calls) {
  // A sustained Hold re-enters this gate every core-loop tick (~10 ms) with
  // the trigger latched true (fix round 3's finding) -- only the
  // dwell_period_ms spacing against the caller's own last_burst_ms is what
  // stops it firing back to back.
  const double last_burst_ms = 1000.0;
  const int dwell_period_ms = 333;
  CHECK(!hop_burst_due(HopState::Hold, true, 1010.0, last_burst_ms, dwell_period_ms));
  CHECK(!hop_burst_due(HopState::Hold, true, 1332.0, last_burst_ms, dwell_period_ms));
  CHECK(hop_burst_due(HopState::Hold, true, 1333.0, last_burst_ms, dwell_period_ms));
  CHECK(hop_burst_due(HopState::Hold, true, 5000.0, last_burst_ms, dwell_period_ms));
}

TEST(first_burst_not_delayed_by_initial_last_burst_ms) {
  // The core loop's own last_burst_ms starts at -1e18 (main.cpp's
  // declaration), so `now_ms - last_burst_ms` is astronomically large on
  // tick one -- it must clear the dwell_period_ms floor immediately, not
  // wait one dwell_period_ms from process start.
  CHECK(hop_burst_due(HopState::Idle, true, /*now_ms=*/0.0, -1e18, 333));
  CHECK(hop_burst_due(HopState::Idle, true, /*now_ms=*/10.0, -1e18, 333));
}
// Bench 2026-09-15: nothing in the hop block may run before the link is in
// SESSION with the boot scout's cards released (see hop_active's comment).
TEST(hop_block_is_inactive_outside_session_or_while_boot_scout_owns_a_card) {
  CHECK(hop_active(/*in_session=*/true, /*scout_joined=*/true));
  CHECK(!hop_active(false, true));
  CHECK(!hop_active(true, false));
  CHECK(!hop_active(false, false));
}
// Bench 2026-09-15: the TX selector must not switch onto the lead card
// while a hop is in flight (see tx_selection_frozen's comment).
TEST(tx_selection_is_frozen_during_a_dwell_or_an_in_flight_hop) {
  CHECK(!tx_selection_frozen(/*dwell_busy=*/false, /*hopping=*/false));
  CHECK(tx_selection_frozen(true, false));
  CHECK(tx_selection_frozen(false, true));
  CHECK(tx_selection_frozen(true, true));
}
MTEST_MAIN
