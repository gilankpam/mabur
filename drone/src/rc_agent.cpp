#include "rc_agent.h"

#include <algorithm>
#include <cmath>

#include "mabur/rc_proto.h"
#include "mabur/uep_encoder.h"

namespace mabur {

using rc::Disc;
using rc::DiscAck;
using rc::LayerTxSpec;
using rc::PhyMode;
using rc::Rcf;

namespace {

int round_to_100(double v) { return static_cast<int>(std::lround(v / 100.0) * 100); }

}  // namespace

RcAgent::RcAgent(const Config& cfg, Actuator& act, AirFeedOut* feed)
    : cfg_(cfg), act_(act), feed_(feed) {}

void RcAgent::note_chain_break() {
  chain_break_pending_.store(true, std::memory_order_relaxed);
}

// One pacer for every IDR producer (spec 2026-08-28 venc-foldin §4). 100 ms
// floor for all of them — an IDR is the most expensive frame on the link and
// two of them back to back buy nothing; chain-break requests also respect a
// 1 s holdoff from the previous chain IDR, because one IDR heals a break and
// a ring that has no consumer needs none at all (upstream measurement,
// waybeam f956a52 venc_frame_ring.h). A refused request is DROPPED, not
// queued: the next real break re-raises it.
bool RcAgent::idr_due(uint64_t now_ms, bool chain) {
  if (have_last_idr_ && now_ms - last_idr_ms_ < 100) return false;
  if (chain && have_last_chain_idr_ && now_ms - last_chain_idr_ms_ < 1000)
    return false;
  last_idr_ms_ = now_ms;
  have_last_idr_ = true;
  if (chain) {
    last_chain_idr_ms_ = now_ms;
    have_last_chain_idr_ = true;
  }
  return true;
}

// Applies the MAX_RANGE operating point (profile_table()[MAX_RANGE_PROFILE]
// via ladder_from). This is a state-transition apply (BOOT's initial op, or
// a LINKED->FAILSAFE entry), so it forces the bitrate policy to the robust
// MCS0 floor bitrate immediately, bypassing the steady-state throttle and
// hysteresis — the spec mandates failsafe = robust MCS + floor bitrate, and
// a degraded radio link must never keep flooding at the last LINKED rate.
void RcAgent::apply_max_range(uint64_t now_ms) {
  auto ladder = rc::ladder_from(PhyMode::HT, 0, 20);
  probe3_active_ = false;  // FAILSAFE/boot never probes

  // Forced shed while MAX_RANGE is the operating point (BOOT/RENDEZVOUS or a
  // LINKED->FAILSAFE entry) — held sticky in failsafe_shed_ until an
  // RCF/DISC takes the agent back to LINKED (see apply_ladder_op), so a
  // later congestion reapply's recompute of shed[1] (which is otherwise
  // driven by shed_level_ alone) can't silently clobber it.
  failsafe_shed_ = true;

  applied_.ladder = ladder;
  // 2.0, not 1.0: the pre-Task-1 wire scale doubled every commanded
  // overhead on its way into the budget formula (uep_layer_overhead's ref
  // scale); RC_VERSION 4 made fec_overhead a literal actual-overhead value
  // with no translation left to apply it, so this hardcoded MAX_RANGE
  // constant — the one caller with no wire value to carry the doubling for
  // it — must apply the old ×2 itself to keep the MCS0 floor bitrate
  // unchanged (carried review finding, Task 3 review). Both layers get the
  // same 2.0: MAX_RANGE has no per-stream pair to carry (Task 6), and enh
  // is shed here anyway.
  applied_.fec_ov_base = 2.0;
  applied_.fec_ov_enh = 2.0;
  applied_.shed[0] = false;
  applied_.shed[1] = true;  // enh layer shed in MAX_RANGE, per spec
  ++applied_.generation;
  act_.apply_op(applied_);
  run_bitrate_policy(now_ms, /*force=*/true);
}

// Applies a resolved (DISC row or RCF-decoded) ladder/FEC operating point.
// ov_base/ov_enh are the per-stream pair as-is (Task 6, RC_VERSION 5) — no
// translation, applied directly to the UEP layers by apply_op_to_uep. Does
// NOT run the bitrate policy itself — callers on the RCF path invoke
// run_bitrate_policy() explicitly afterwards, per the spec.
void RcAgent::apply_ladder_op(const std::array<LayerTxSpec, 2>& ladder,
                              double ov_base, double ov_enh) {
  // A resolved DISC/RCF op is only ever applied on a path that (re)enters
  // LINKED (see on_rc_frame), so the sticky MAX_RANGE forced-shed from a
  // prior RENDEZVOUS/FAILSAFE no longer applies — clear it here rather than
  // in on_rc_frame so it's cleared atomically with the op that ends it.
  failsafe_shed_ = false;

  applied_.ladder = ladder;
  applied_.fec_ov_base = ov_base;
  applied_.fec_ov_enh = ov_enh;
  applied_.shed[0] = false;
  // shed_level_ still counts 0..3 (congestion semantics untouched — see
  // run_congestion_guard), but with the reserved layer gone there is only
  // one droppable layer left, so any level >= 1 sheds it; there is no
  // second layer for level >= 2 to additionally shed.
  applied_.shed[1] = shed_level_ >= 1;
  ++applied_.generation;
  act_.apply_op(applied_);
}

// Reapplies the current commanded op (ladder/fec unchanged) with the
// current congestion-shed level folded in. Used by the congestion guard,
// which doesn't change the ladder or bump generation on its own —
// generation tracks *new operating points* (BOOT/DISC/RCF/failsafe entry),
// not shed adjustments to the current one, so this function publishes via
// act_.apply_op() WITHOUT touching applied_.generation. Consumers (the
// hot-thread loops in main.cpp) must therefore detect "a new AppliedOp was
// published" by identity-comparing the shared_ptr they last applied against
// the one the atomic handoff currently holds, NOT by checking whether
// generation changed — otherwise a congestion shed would be computed here
// but silently never reach the encoder.
//
// shed[1] ORs together every independent reason the enh layer must be
// shed: failsafe_shed_ (sticky for as long as MAX_RANGE is the operating
// point — see apply_max_range/apply_ladder_op) and the local congestion-
// shed level. Without the OR, this recompute-from-shed_level_-alone would
// clobber a forced failsafe shed the moment a congestion tick runs while in
// FAILSAFE.
void RcAgent::reapply_with_shed() {
  applied_.shed[1] = failsafe_shed_ || (shed_level_ >= 1);
  act_.apply_op(applied_);
}

// Runs whenever a new op is applied. The formula is always recomputed, but
// whether an actual set_bitrate_kbps() call happens depends on `force` and
// on the DIRECTION of the change:
//
//  - a DECREASE vs the last value actually sent always goes out, on the
//    same tick that applied the MCS. The radio capacity has already
//    dropped when the RCF lands; deferring the shed lets the encoder flood
//    a smaller pipe and TxQueue drop-oldest kills whole FEC bodies — a
//    multi-rung demote cascade under a gated shed is the 2026-08-09
//    freeze-crash (docs/shed-lag-findings-2026-08-09.md).
//  - force=false, non-decrease (steady-state LINKED RCFs): the call is
//    throttled to at most once per 1000ms, so repeated RCFs within one
//    second collapse to a single call (scenario: bitrate hysteresis) and a
//    late quality INCREASE stays harmless and lazy. The dedup test is an
//    exact inequality vs the last value actually sent: a target that
//    differs at all is a real operating-point change and must eventually
//    reach the encoder. It was a +-10% magnitude deadband until
//    2026-08-28, which measured as a permanent undershoot — the deadband
//    filtered an ABSOLUTE target with no accumulator, so an error smaller
//    than the band could never grow into it and the last applied value
//    became a fixed point. Prod (airtime_budget 0.60, bitrate_max_kbps
//    10000) hit it at the top of the ladder: rung4 commands 9400, rung5's
//    12480 clamps to 10000, and |10000-9400| = 600 < 940 meant the promote
//    was discarded forever — the link ran mcs5 while the encoder stayed on
//    the mcs4 bitrate, 6% low, on the rung the link occupies almost all
//    the time. Note it takes the CLAMP to create the trap: unclamped, the
//    rung gap is far wider than 10%. Decreases were always exempt, so the
//    error only ever accumulated downward.
//  - force=true (BOOT's initial MAX_RANGE apply, every LINKED->FAILSAFE
//    entry, and RCFs that transition into LINKED from RENDEZVOUS/FAILSAFE):
//    the throttle and dedup gates are bypassed entirely — the new
//    operating point's bitrate (e.g. the MCS0 floor on failsafe entry)
//    takes effect immediately, every time. The value is still clamped and
//    rounded to 100 as usual. The throttle timestamp is updated afterwards
//    either way, so a steady-state RCF arriving shortly after a forced call
//    is throttled normally.
//
// The blend (spec 2026-08-29-airtime-balance-uep §2 bitrate): with two
// streams at different PHY rates, a single-rate budget target (T0's rate
// alone) is wrong the moment the encoder isn't splitting the video 50/50
// across them, so the target is built from both rates weighted by the
// live measured share. fb/exb/exe default to 0.5/0/0 (an even split, no
// measured framing excess) whenever feed_ is null (tests, or before Task 7
// wires the feed up) — see run_bitrate_policy's core comment below.
void RcAgent::run_bitrate_policy(uint64_t now_ms, bool force) {
  last_policy_ms_ = now_ms;
  have_last_policy_ = true;
  const double rate_b = rc::phy_rate_mbps(applied_.ladder[0]);
  const double rate_e = rc::phy_rate_mbps(applied_.ladder[1]);
  // The commanded pair (Task 6, RC_VERSION 5) IS the source now — no single
  // ov to fan out to both terms.
  double ovb = applied_.fec_ov_base, ove = applied_.fec_ov_enh;
  double fb = 0.5, exb = 0.0, exe = 0.0;
  if (feed_) {
    fb = std::clamp<double>(feed_->share_base.load(std::memory_order_relaxed), 0.05, 0.95);
    exb = feed_->excess_base.load(std::memory_order_relaxed);
    exe = feed_->excess_enh.load(std::memory_order_relaxed);
  }
  // Debug-HTTP per-layer override active: the layers fly THESE overheads
  // (not the commanded pair), so the budget target must be built from them.
  // Still gated by feed_ (tests construct RcAgent with none): the ternary's
  // -1 short-circuits the `>= 0` check exactly as a null feed_ did before.
  const int ob = feed_ ? feed_->ovr_base_pct.load(std::memory_order_relaxed) : -1;
  const int oe = feed_ ? feed_->ovr_enh_pct.load(std::memory_order_relaxed) : -1;
  if (ob >= 0 && oe >= 0) { ovb = ob / 100.0; ove = oe / 100.0; }
  // Blended airtime: V * [fb*mult_b/rate_b + (1-fb)*mult_e/rate_e] = budget.
  // mult uses the COMMANDED pair (ovb/ove, the RCF's literal
  // fec_overhead_base/enh) plus MEASURED framing excess (exb/exe), never
  // AirFeed's live per-stream ov (AirFeedOut::ov_base/ov_enh,
  // telemetry-only) — that value is repair-byte-neutral and must not feed
  // back into this target (spec §2 bitrate).
  const double denom = fb * (1.0 + ovb + exb) / rate_b +
                       (1.0 - fb) * (1.0 + ove + exe) / rate_e;
  double kbps = 1000.0 * cfg_.encoder.airtime_budget / denom;
  // NOTE the encoder has its own floor below this one: venc's apply_bitrate
  // rails at VENC_BITRATE_MIN_KBPS (1000), so a configured bitrate_min_kbps
  // under 1000 is silently re-clamped there and this policy's "structural
  // minimum" would not be the one in force.
  kbps = std::clamp(kbps, static_cast<double>(cfg_.encoder.bitrate_min_kbps),
                     static_cast<double>(cfg_.encoder.bitrate_max_kbps));
  int kbps_i = round_to_100(kbps);

  bool decrease = have_last_bitrate_ && kbps_i < last_bitrate_kbps_;
  bool changed = !have_last_bitrate_ || kbps_i != last_bitrate_kbps_;
  bool throttled =
      have_last_bitrate_eval_ && now_ms - last_bitrate_eval_ms_ < 1000;
  // Every piece of "what the encoder is currently running" state is latched
  // ONLY when the verb reports success — including the throttle timestamp,
  // so a failed apply is not merely remembered as pending but is retried at
  // the first opportunity rather than one second later. A failed call
  // therefore leaves the agent exactly as it was, and the ordinary
  // changed/decrease logic re-issues the same value on the next policy
  // tick (i.e. the next RCF). Latching unconditionally would recreate the
  // waybeam wedge in-process: one dropped MI call and the encoder stays on
  // the old rate for the rest of the flight, because `changed` reads false
  // forever after.
  bool failed = false;
  if (force || decrease || (changed && !throttled)) {
    if (act_.set_bitrate_kbps(kbps_i)) {
      last_bitrate_kbps_ = kbps_i;
      have_last_bitrate_ = true;
      last_bitrate_eval_ms_ = now_ms;
      have_last_bitrate_eval_ = true;
    } else {
      failed = true;
    }
  }

  bool now_low = kbps_i < cfg_.encoder.roi_threshold_kbps;
  if (now_low != roi_low_) {
    // Same rule, same reason: flipping roi_low_ before knowing the encoder
    // took the QP would make the transition un-repeatable.
    if (act_.set_roi_qp(now_low ? cfg_.encoder.roi_qp_low : cfg_.encoder.roi_qp_normal))
      roi_low_ = now_low;
    else
      failed = true;
  }
  // Latched for tick()'s retry path. Cleared, not merely left alone, on a
  // clean run: the re-assert must go back to its 5 s cadence once the
  // encoder is taking values again, not retry on every tick forever.
  verb_apply_failed_ = failed;
}

// Escalates shed_level_ (0..3) by one step the instant tx_drops rises since
// the last tick (congestion is assumed the moment drops increase — no
// debounce on the way up), and decays it by exactly one step down per 2s
// clean window (2000ms with no further drop-rise); each step-down restarts
// the 2s window, so recovering from level 3 to level 0 takes three separate
// 2s clean windows in a row, not one. shed_level_ still ranges 0..3 for its
// own escalate/decay cadence, but the 2-stream space (spec
// 2026-08-29-airtime-balance-uep) leaves only one droppable layer (sid 1,
// enh) -- any level >= 1 sheds it and there is no second layer for level
// >= 2 to additionally shed (see reapply_with_shed()). Any level change
// reapplies the current op (shed folded in) so the change reaches the
// actuator immediately.
//
// Reaching level 3 also cut the encoder bitrate by 30% until 2026-08-28.
// That cut was removed: it wrote act_.set_bitrate_kbps(last * 0.7) straight
// to the actuator, bypassing run_bitrate_policy's
// clamp(bitrate_min_kbps, bitrate_max_kbps), and it compounded because it
// overwrote last_bitrate_kbps_ with its own output -- so each re-entry into
// level 3 multiplied the already-cut value (1300 -> 910 -> 637 -> 446 ->
// 312). An RCF repairs that within ~1s in LINKED, but tick() never calls
// run_bitrate_policy, so in FAILSAFE/RENDEZVOUS -- where the op is
// MAX_RANGE = mcs0 -- nothing restored it and the ratchet ran unopposed
// under the configured floor. run_bitrate_policy is now the ONLY writer of
// the encoder bitrate, so bitrate_min_kbps is a structural minimum
// (test 10e). Shedding layers remains the guard's whole job: bounded 0..3,
// self-decaying, no ratchet.
//
// NOTE the trigger is health.tx_drops = TxStats::failed (main.cpp), which
// counts USB bulk-OUT submission/completion FAILURES, not queue
// backpressure -- TxQueue drop-oldest (txq.drops) and RadioTx::drops() are
// separate counters and neither feeds this. So the guard responds to a sick
// dongle, not to over-driving the air. Known mis-wiring, left as-is.
void RcAgent::run_congestion_guard(uint64_t now_ms, const RadioHealth& health) {
  bool drops_rose = have_last_tx_drops_ && health.tx_drops > last_tx_drops_;
  if (!have_last_tx_drops_) drops_rose = health.tx_drops > 0;
  have_last_tx_drops_ = true;
  last_tx_drops_ = health.tx_drops;

  bool changed = false;
  if (drops_rose) {
    last_drop_rise_ms_ = now_ms;
    have_last_drop_rise_ = true;
    if (shed_level_ < 3) {
      ++shed_level_;
      changed = true;
    }
  } else if (have_last_drop_rise_ && now_ms - last_drop_rise_ms_ >= 2000) {
    if (shed_level_ > 0) {
      --shed_level_;
      changed = true;
      last_drop_rise_ms_ = now_ms;  // restart the 2s window for the next step-down
    }
  }

  if (changed) reapply_with_shed();
}

void RcAgent::on_rc_frame(const uint8_t* body, size_t len, uint64_t now_ms) {
  int type = rc::frame_type(body, len);
  if (type == rc::T_DISC) {
    auto d = rc::parse_disc(body, len);
    if (!d.has_value() || d->vtx_id != cfg_.link.vtx_id) return;

    if (state_ == State::LINKED) {
      // Ack-only: a rebooted GS starts in SESSION with peer_caps_=0 and
      // its video tail gated off; its ~1 Hz keep-alive DISC is the only
      // way it can re-learn chip_caps (stale-caps deadlock, 2026-08-12).
      // Reply, but change NOTHING else — the init-profile apply, state
      // transition and watchdog refresh stay LINKED-entry-only (op-thrash
      // fix, 2026-07-12).
      act_.send_control(rc::pack_disc_ack(make_disc_ack(d->vrx_nonce, d->seq)));
      return;
    }

    // Echo the drone's ACTUAL operating channel/width, not the GS-requested
    // d->op_channel/op_width — the drone doesn't retune in v1 (its channel
    // is fixed from its own config at startup), so acking back the GS's
    // request would claim an agreement that never happened whenever the
    // two disagree. Reporting the true op point lets the GS detect and
    // handle a mismatch instead of being told (incorrectly) that its
    // request was honored.
    act_.send_control(rc::pack_disc_ack(make_disc_ack(d->vrx_nonce, d->seq)));

    int row_idx = std::clamp<int>(d->init_profile, 0,
                                   static_cast<int>(rc::profile_table().size()) - 1);
    auto ladder = rc::ladder_for_row(row_idx);
    const auto& row = rc::profile_table()[static_cast<size_t>(row_idx)];
    apply_ladder_op(ladder, row.ov_base, row.ov_enh);

    if (state_ != State::FAILSAFE) link_established_ = true;
    state_ = State::LINKED;
    last_fb_ms_ = now_ms;
    have_last_fb_ = true;
    // A DISC establishes a (new) GS session — same session-boundary seq
    // reset as failsafe entry above.
    have_last_seq_ = false;

    // Same rationale as RCF's entering_linked force: DISC always
    // (re)establishes LINKED from RENDEZVOUS/FAILSAFE, so the newly resolved
    // op's bitrate must take effect immediately rather than waiting for the
    // steady-state throttle/hysteresis gate (or, worse, never running at all
    // — DISC apply previously never called run_bitrate_policy(), leaving
    // the encoder stuck at whatever bitrate was last set, e.g. the MAX_RANGE
    // floor, until the first post-DISC RCF arrived).
    run_bitrate_policy(now_ms, /*force=*/true);
    return;
  }

  if (type == rc::T_RCF) {
    auto r = rc::parse_rcf(body, len);
    if (!r.has_value() || r->vtx_id != cfg_.link.vtx_id) return;

    bool fresh;
    if (!have_last_seq_) {
      fresh = true;
    } else {
      uint16_t delta = static_cast<uint16_t>(r->seq - last_seq_);
      fresh = delta >= 1 && delta <= 32767;
    }
    if (!fresh) return;
    last_seq_ = r->seq;
    have_last_seq_ = true;
    ++rcf_accepted_;

    PhyMode mode;
    uint8_t mcs, bw;
    rc::decode_profile(r->profile, mode, mcs, bw);
    auto ladder = rc::ladder_from(mode, mcs, bw);

    // s3 probe (spec 2026-08-05): when the RCF carries probe3, the enh
    // layer (ladder[1], the old s3) transmits at probe_profile's MCS while
    // mode/bw stay the base profile's — MCS-only probe, the base layer
    // (ladder[0]) rides the normal ladder. Recomputed on every accepted
    // RCF, so a follow-up frame without the flag reverts the enh layer to
    // the base profile's mcs.
    probe3_active_ = false;
    if (r->probe3) {
      PhyMode pmode;
      uint8_t pmcs, pbw;
      rc::decode_profile(r->probe_profile, pmode, pmcs, pbw);
      ladder[1].mcs = pmcs;
      probe3_active_ = true;
    }

    State prev_state = state_;
    apply_ladder_op(ladder, r->fec_overhead_base, r->fec_overhead_enh);

    if (prev_state == State::BOOT || prev_state == State::RENDEZVOUS)
      link_established_ = true;
    state_ = State::LINKED;
    last_fb_ms_ = now_ms;
    have_last_fb_ = true;

    bool entering_linked = prev_state == State::FAILSAFE || prev_state == State::RENDEZVOUS;
    // Through the shared pacer (§4) like every other IDR producer. The
    // 100 ms floor cannot suppress this in practice — re-entering LINKED
    // requires having spent at least failsafe_ms (>= 1 s) outside it — but
    // routing it here is what makes "RcAgent is the only IDR authority"
    // true rather than aspirational.
    if (entering_linked && idr_due(now_ms, /*chain=*/false)) {
      act_.request_idr();
    }

    // RCFs that transition into LINKED (from RENDEZVOUS or FAILSAFE) force
    // the bitrate policy so the new operating point's bitrate takes effect
    // immediately; steady-state LINKED RCFs go through the normal
    // throttle+hysteresis gate.
    run_bitrate_policy(now_ms, /*force=*/entering_linked);
    return;
  }
  // Anything else (DISC_ACK, unknown/corrupt) is ignored.
}

void RcAgent::tick(uint64_t now_ms, const RadioHealth& health) {
  if (state_ == State::BOOT) {
    apply_max_range(now_ms);
    state_ = State::RENDEZVOUS;
    return;
  }

  // Chain-break intake, evaluated against the state as of this tick's ENTRY
  // — deliberately ahead of the failsafe/rendezvous timers below. The
  // encoder broke its reference chain while the link was up, and the tick
  // that notices a missed feedback deadline is the tick most likely to be
  // carrying that break; deferring the heal until after the state machine
  // has moved us to FAILSAFE would silently discard exactly the IDR that
  // matters. The LINKED gate stays, so a break raised while genuinely
  // unlinked is consumed and dropped rather than queued — affordable only
  // because the encoder's own GOP is the backstop: at the shipped
  // venc.gop_s = 2.0 an unhealed chain break self-clears within ~2 s. Raise
  // gop_s and that safety net stretches with it, at which point dropping
  // refused requests instead of deferring them needs re-arguing.
  if (chain_break_pending_.exchange(false, std::memory_order_relaxed) &&
      state_ == State::LINKED && idr_due(now_ms, /*chain=*/true)) {
    act_.request_idr();
  }

  if (state_ == State::LINKED) {
    if (have_last_fb_ && now_ms - last_fb_ms_ >= static_cast<uint64_t>(cfg_.link.failsafe_ms)) {
      apply_max_range(now_ms);
      state_ = State::FAILSAFE;
      // Session boundary: forget the RCF seq baseline. The Python VTX has
      // NO stale-seq check at all (adaptive_link.py applies every valid
      // RCF); the port added replay protection, which locked out a
      // RESTARTED GS (its seq restarts at 0 -> every RCF reads stale for
      // up to 32k seqs, ~28 min at 20 Hz — bench 2026-07-12: metronomic
      // 3 s LINKED / 1 s FAILSAFE with a healthy air link). Resetting at
      // failsafe keeps in-session replay protection with no lockout.
      have_last_seq_ = false;
      // Rebase the rendezvous_ms timer from the moment failsafe was
      // entered (not the last real feedback), so a link silent since t=0
      // with failsafe_ms=1000/rendezvous_ms=30000 falls back to
      // RENDEZVOUS at t=1000+30000=31000, not t=30000.
      last_fb_ms_ = now_ms;
    }
  } else if (state_ == State::FAILSAFE) {
    if (have_last_fb_ &&
        now_ms - last_fb_ms_ >= static_cast<uint64_t>(cfg_.link.rendezvous_ms)) {
      state_ = State::RENDEZVOUS;
    }
  }

  // Periodic re-assert of the encoder verbs (kReassertMs). run_bitrate_policy
  // otherwise runs ONLY on an RCF, a DISC, or a max-range entry, which makes
  // "retry on the next policy tick" a promise the agent cannot keep in the
  // states where it matters most: in FAILSAFE there are no RCFs by
  // definition, so a verb that failed on the failsafe entry itself stayed
  // failed for up to rendezvous_ms (30 s) with the encoder flooding a
  // mcs0-sized pipe at the previous rung's rate. The same gap let anything
  // that wrote the encoder behind RcAgent's back — the debug endpoint's
  // POST /venc/set?bitrate= above all — persist until the ladder happened to
  // change rung.
  //
  // force=true is required, not incidental: with force=false an unchanged
  // computed target is a deliberate no-op (the changed/decrease gate), so a
  // re-assert would send nothing at all — which is the entire bug. force
  // re-applies the CURRENT computed target, including FAILSAFE's floor
  // (applied_ is the max-range op while in FAILSAFE, so the value stamped
  // over is the correct one for the state, never a stale LINKED rate).
  //
  // Cadence: kReassertMs since the last bitrate the encoder actually ACCEPTED
  // (last_bitrate_eval_ms_ is latched only inside the success branch), so a
  // busy LINKED link that keeps genuinely changing rung never adds a
  // re-assert on top, while a parked one gets exactly one every 5 s. A failed
  // verb short-circuits to the next tick.
  //
  // RENDEZVOUS is excluded: it is the pre-link state where no GS has been
  // heard from, MAX_RANGE was applied once on the BOOT tick, and there is
  // nothing to defend the value against.
  //
  // ran_this_tick keeps it to at most one policy run per distinct now_ms: a
  // tick that has ALREADY run the policy (the failsafe entry a few lines up,
  // or an RCF the agent loop drained at the same millisecond) must not
  // immediately run it again — most visibly when that run FAILED, where the
  // retry belongs on the next tick, not back-to-back on this one.
  bool ran_this_tick = have_last_policy_ && last_policy_ms_ == now_ms;
  if (!ran_this_tick && (state_ == State::LINKED || state_ == State::FAILSAFE)) {
    bool reassert_due = verb_apply_failed_ || !have_last_bitrate_eval_ ||
                        now_ms - last_bitrate_eval_ms_ >= kReassertMs;
    if (reassert_due) run_bitrate_policy(now_ms, /*force=*/true);
  }

  run_congestion_guard(now_ms, health);
}

rc::DiscAck RcAgent::make_disc_ack(uint32_t nonce, uint16_t seq) const {
  DiscAck ack;
  ack.vtx_id = cfg_.link.vtx_id;
  ack.vrx_nonce = nonce;
  // Frame wire is the only video format maburd speaks; the bit stays on the
  // wire (one legal value) so a GS can still refuse a peer that lacks it.
  // CAP_TELEMETRY: this drone also sends T_TELEM frames on its uplink
  // (spec 2026-07-26 drone-telemetry) — display-grade only, not a gate.
  // CAP_ENH_PROBE: this drone accepts RCF_F_PROBE_ENH (spec 2026-08-05
  // s3-probe-promote).
  ack.chip_caps = rc::CAP_FRAME_WIRE | rc::CAP_TELEMETRY | rc::CAP_ENH_PROBE;
  ack.agreed_channel = cfg_.radio.channel;
  ack.agreed_width = cfg_.radio.width;
  ack.seq = seq;
  return ack;
}

}  // namespace mabur
