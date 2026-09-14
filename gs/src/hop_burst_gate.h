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
inline bool hop_burst_due(HopState state, bool trigger, double now_ms,
                          double last_burst_ms, int dwell_period_ms) {
  const bool hop_free = state == HopState::Idle || state == HopState::Hold;
  return hop_free && trigger && (now_ms - last_burst_ms >= dwell_period_ms);
}

}  // namespace maburgs
