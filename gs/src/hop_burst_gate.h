#pragma once
#include "hop_controller.h"

namespace maburgs {

// Pure: whether the core loop's synchronous freshness burst
// (HopController's ONLY source of ranking data on a one-card GS -- see the
// call site's own comment in channel_core.cpp) should run this tick.
// Extracted so the four properties below, each of which has been wrong at
// some point, have a direct unit test instead of only ever running inside
// the hardware-touching burst body (inflight_mu, fronts[], RadioFrontend,
// ranker, scan_log), which is NOT extracted and stays in ChannelCore:
//   - "no hop in flight" is Idle OR Hold (Ordered/Verifying excluded: a
//     hop is actually in progress and the radio must not wander);
//   - the trigger must be live this tick;
//   - rate-limited to at most one burst per burst period
//     (hop_burst_period_ms(), below), checked
//     against the caller's own last_burst_ms (a sustained
//     Hold re-enters on every core-loop tick with trigger latched true,
//     and neither cooldown_ms nor max_hops_per_min paces a burst that
//     never results in an order);
//   - last_burst_ms's caller-side initial value (-1e18) must NOT delay
//     the very first burst.
// Pure: whether the in-session hop block as a whole -- verdict window,
// freshness burst, controller tick -- may run this tick. Found on the
// bench: unconditional, the block ran during the boot
// rendezvous, where the s1 loss window reads 80-90 % while the link is
// still coming up and the boot scout's own 250 ms dwells read as `raised`
// -- `interfered` by construction. The shadow controller then ordered
// every candidate in turn and exhausted before the boot pick had even
// committed, and the freshness burst retuned the boot scout's card out
// from under it mid-dwell (that boot scan took 17 rounds instead of 6).
// `in_session` is VrxState::SESSION. The caller resets HopVerdict on the
// falling edge so nothing measured while inactive can latch a trigger.
//
// `cal_running` (CalSession::running()) also deactivates it: a calibration
// run stops video on purpose, which the verdict engine reads as a dead
// channel -- on the bench it ordered a hop_lead seconds into a sweep and
// took card 1 off the channel the walls were being measured on.
//
// The boot scout owning a card does not deactivate it: the pick stays
// open after link-up and the boot hop needs the
// verdict engine; the scout card is excluded per-card by
// verdict_card_usable's mid_dwell instead.
inline bool hop_active(bool in_session, bool cal_running) {
  return in_session && !cal_running;
}

// Pure: whether the core loop must keep its current TX card this tick
// instead of letting TxSelector::update() re-pick. Two reasons, same
// shape: a card the in-flight scout has off on a candidate (dwell_busy),
// and -- found on the bench -- a hop's lead card while the hop
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
// skipped exactly like one mid-dwell. A hop's lead card sits on the target
// from the Order to the Confirm; mixed into the op verdict, its readings
// (clean target, no own frames) would -- through blocked's MIN-across-cards
// rule -- clear kEvBlocked within ~3 windows of every order. During Ordered
// the op verdict runs on the TX card alone; a one-card GS after its
// OneCardRetune has no valid card at all (Verdict::Unknown, no kEvBlocked).
inline bool verdict_card_usable(bool ready, bool mid_dwell, uint8_t card_ch, uint8_t op_ch) {
  return ready && !mid_dwell && card_ch == op_ch;
}

inline bool hop_burst_due(HopState state, bool trigger, double now_ms,
                          double last_burst_ms, int burst_period_ms) {
  const bool hop_free = state == HopState::Idle || state == HopState::Hold;
  return hop_free && trigger && (now_ms - last_burst_ms >= burst_period_ms);
}

// Pure: the freshness burst's pacing, the period hop_burst_due() checks.
// A relay sweeps asynchronously and blanks itself for ~280 ms per SCAN:
// relay_burst_period_ms in any state. A USB burst runs synchronously on
// the core thread and takes its card off op for every candidate dwell
// (dwell_observe_ms plus a retune away and back, per candidate), so it
// paces on burst_period_ms while it is what stands between the trigger
// and an order: in Idle, and in a Hold entered because some non-op
// candidate is not ranked yet (HopRanker marks one ranked at two fresh
// visits, so the cold-ranker path is burst -> hold_exhausted -> burst ->
// order). Once every non-op candidate is ranked (`ranking_complete`) and
// the controller still holds, everything read blocked or backed off and
// the trigger can stay live indefinitely on a link that still carries
// video (an occupied op): re-bursting at the Idle cadence would make the
// burst itself the outage, so that Hold paces on dwell_period_ms, the
// spare card's own periodic cadence.
inline int hop_burst_period_ms(HopState state, bool relay_burst, bool ranking_complete,
                               const HopCfg& hop) {
  if (relay_burst) return hop.relay_burst_period_ms;
  return (state == HopState::Hold && ranking_complete) ? hop.dwell_period_ms : hop.burst_period_ms;
}

// Pure: whether a due freshness burst is worth running. A burst on a spare
// USB card (not the TX card) re-measures what the periodic dwell thread
// already keeps ranked on that same card -- one candidate per
// dwell_period_ms -- and it runs synchronously, stalling the core loop
// (AU publishing included) for every candidate dwell. So once every non-op
// candidate is ranked, a spare-card burst is skipped and the hop acts on
// the periodic dwells' ranking (newest visit at most one dwell round old).
// A one-card GS's burst is its only ranking source, and a relay sweeps
// asynchronously: both run regardless. An incomplete ranking (cold start,
// a candidate whose dwells fail, visits aged past rank_max_age_ms) still
// bursts, which is what brings a stalled periodic dwell's ranking back.
inline bool hop_burst_wanted(bool spare_usb_burst, bool ranking_complete) {
  return !(spare_usb_burst && ranking_complete);
}

// Holds the link-pairing move edge (VrxController::take_move_edge()) for
// the length of a calibration run. The GS still beacons DISC between sweep
// phases, so a re-pair -- and its move edge -- can land mid-run, and acting
// on it would retune both cards off the channel being measured. The drone
// defers its own retunes while cal_active and replays them when the run
// ends; this replays the GS side at the same point. `hold` is the caller's
// hold condition: cal running OR `plan.hopping()`, so an edge landing mid-hop
// replays after the hop resolves instead of being dropped.
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

// Pure: whether a due calibration SweepCmd may go out this tick. `radio_silent` is
// CalSession::radio_silent() (a sweep is airing). `dwell_busy` is the
// in-flight scout's flag: the scout idles while CalSession::running(), but
// it only checks that once per dwell_period_ms, so a dwell begun just
// before `maburcal start` is still off-channel for up to dwell_ms (250 ms)
// after it -- against a sweep's few-millisecond settle and 20-frame cells,
// that card misses the first row's opening cells. Holding the command until the
// card is back costs at most one dwell out of the 3 s ack timeout. Once the
// run is going no new dwell starts, so the hold only ever bites at start.
inline bool cal_cmd_clear(bool radio_silent, bool dwell_busy) {
  return !radio_silent && !dwell_busy;
}

}  // namespace maburgs
