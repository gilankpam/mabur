#pragma once
#include "hop_controller.h"

namespace maburgs {

// Pure: whether the core loop's synchronous freshness burst (spec section
// 3, HopController's ONLY source of ranking data on a one-card GS -- see
// the call site's own comment in main.cpp) should run this tick.
// Extracted (Task 15 fix round 1) so the four properties that were wrong
// at various points in this plan have a direct unit test instead of only
// ever running inside the hardware-touching burst body (inflight_mu,
// fronts[], RadioFrontend, ranker, scan_log), which is NOT extracted and
// stays exactly where it is in run_radio():
//   - "no hop in flight" is Idle OR Hold (Ordered/Verifying excluded: a
//     hop is actually in progress and the radio must not wander);
//   - the trigger must be live this tick;
//   - rate-limited to at most one burst per dwell_period_ms, checked
//     against the caller's own last_burst_ms (fix round 3: a sustained
//     Hold re-enters on every core-loop tick with trigger latched true,
//     and neither cooldown_ms nor max_hops_per_min paces a burst that
//     never results in an order);
//   - last_burst_ms's caller-side initial value (-1e18) must NOT delay
//     the very first burst.
// Pure: whether the in-session hop block as a whole -- verdict window,
// freshness burst, controller tick -- may run this tick. Found on the
// 2026-09-15 bench: unconditional, the block ran during the boot
// rendezvous, where the s1 loss window reads 80-90 % while the link is
// still coming up and the boot scout's own 250 ms dwells read as `raised`
// -- `interfered` by construction. The shadow controller then ordered
// every candidate in turn and exhausted before the boot pick had even
// committed, and the freshness burst retuned the boot scout's card out
// from under it mid-dwell (that boot scan took 17 rounds instead of 6).
// `scout_joined` is the same "boot scout owns no card" predicate the
// periodic in-flight scout thread starts on; `in_session` is
// VrxState::SESSION. The caller resets HopVerdict on the falling edge so
// nothing measured while inactive can latch a trigger.
inline bool hop_active(bool in_session, bool scout_joined) {
  return in_session && scout_joined;
}

// Pure: whether the core loop must keep its current TX card this tick
// instead of letting TxSelector::update() re-pick. Two reasons, same
// shape: a card the in-flight scout has off on a candidate (dwell_busy),
// and -- found on the 2026-09-15 bench -- a hop's lead card while the hop
// is in flight (ChannelPlan::hopping(), Order until Confirm/Withdraw).
// Unfrozen, the selector switched onto the lead card within 200 ms of
// three of the four orders in the first co-channel-jam run, which moved
// the RCF uplink -- the frames CARRYING the order -- to the target
// channel where the drone was not yet listening. Every one of those
// orders withdrew at confirm_ms; the one whose TX card stayed put
// confirmed in 139 ms. After lead_confirm the trailing card follows
// (hop_follow), hopping() clears, and the selector is free again.
inline bool tx_selection_frozen(bool dwell_busy, bool hopping) {
  return dwell_busy || hopping;
}

inline bool hop_burst_due(HopState state, bool trigger, double now_ms,
                          double last_burst_ms, int dwell_period_ms) {
  const bool hop_free = state == HopState::Idle || state == HopState::Hold;
  return hop_free && trigger && (now_ms - last_burst_ms >= dwell_period_ms);
}

}  // namespace maburgs
