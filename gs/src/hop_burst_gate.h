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
// `in_session` is VrxState::SESSION. The caller resets HopVerdict on the
// falling edge so nothing measured while inactive can latch a trigger.
// (The 2026-09-15 fix above gated this on a `scout_joined` boot-scout
// predicate too; superseded 2026-10-03 -- see below.)
//
// `cal_running` (CalSession::running()) also deactivates it: a calibration
// run stops video on purpose, which the verdict engine reads as a dead
// channel -- bench 2026-10-03 ordered a hop_lead seconds into a sweep and
// took card 1 off the channel the walls were being measured on.
//
// The boot scout owning a card no longer deactivates it: the pick stays
// open after link-up (spec 2026-10-03 §5) and the boot hop needs the
// verdict engine; the scout card is excluded per-card by
// verdict_card_usable's mid_dwell instead.
inline bool hop_active(bool in_session, bool cal_running) {
  return in_session && !cal_running;
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

// Pure: whether card i contributes to this window's op-channel verdict.
// The verdict describes the OP channel, so a card tuned anywhere else is
// skipped exactly like one mid-dwell. Found with Task 12 (f): a hop's lead
// card sits on the target from the Order to the Confirm, and its readings
// (clean target, no own frames) were mixed into the op verdict -- a latent
// bug that, through blocked's MIN-across-cards rule, cleared kEvBlocked
// within ~3 windows of every order. During Ordered the op verdict now runs
// on the TX card alone; a one-card GS after its OneCardRetune has no valid
// card at all (Verdict::Unknown, no kEvBlocked).
inline bool verdict_card_usable(bool ready, bool mid_dwell, uint8_t card_ch, uint8_t op_ch) {
  return ready && !mid_dwell && card_ch == op_ch;
}

inline bool hop_burst_due(HopState state, bool trigger, double now_ms,
                          double last_burst_ms, int dwell_period_ms) {
  const bool hop_free = state == HopState::Idle || state == HopState::Hold;
  return hop_free && trigger && (now_ms - last_burst_ms >= dwell_period_ms);
}

// Holds the link-pairing move edge (VrxController::take_move_edge()) for
// the length of a calibration run. The GS still beacons DISC between sweep
// phases, so a re-pair -- and its move edge -- can land mid-run, and acting
// on it would retune both cards off the channel being measured. The drone
// defers its own retunes while cal_active and replays them when the run
// ends; this replays the GS side at the same point. `hold` is the caller's
// hold condition: cal running OR `plan.hopping()`, so an edge landing mid-hop
// replays after the hop resolves instead of being dropped (2026-10-04).
class CalMoveEdgeHold {
 public:
  bool take(bool edge, bool hold) {
    held_ = held_ || edge;
    if (hold) return false;
    const bool out = held_;
    held_ = false;
    return out;
  }

 private:
  bool held_ = false;
};

// Pure: whether a due T_CAL_CMD may go out this tick. `radio_silent` is
// CalSession::radio_silent() (a sweep phase is airing). `dwell_busy` is the
// in-flight scout's flag: the scout idles while CalSession::running(), but
// it only checks that once per dwell_period_ms, so a dwell begun just
// before `maburcal start` is still off-channel for up to dwell_ms (250 ms)
// after it -- against a settle of 30 ms and 20-frame coarse cells, that
// card misses the first row's opening cells. Holding the command until the
// card is back costs at most one dwell out of the 3 s ack timeout. Once the
// run is going no new dwell starts, so the hold only ever bites at start.
inline bool cal_cmd_clear(bool radio_silent, bool dwell_busy) {
  return !radio_silent && !dwell_busy;
}

}  // namespace maburgs
