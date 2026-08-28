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

RcAgent::RcAgent(const Config& cfg, Actuator& act) : cfg_(cfg), act_(act) {}

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
  // later congestion reapply's recompute of shed[2]/[3] (which is
  // otherwise driven by shed_level_ alone) can't silently clobber it.
  failsafe_shed_ = true;

  applied_.ladder = ladder;
  applied_.fec_overhead = 1.0;
  applied_.shed[0] = false;
  applied_.shed[1] = false;
  applied_.shed[2] = true;  // reserved layer shed in MAX_RANGE, per spec (already covers shed_level_>=2)
  applied_.shed[3] = true;  // enhance layer shed in MAX_RANGE, per spec (already covers shed_level_>=1)
  ++applied_.generation;
  act_.apply_op(applied_);
  run_bitrate_policy(now_ms, /*force=*/true);
}

// Applies a resolved (DISC row or RCF-decoded) ladder/FEC operating point.
// Does NOT run the bitrate policy itself — callers on the RCF path invoke
// run_bitrate_policy() explicitly afterwards, per the spec.
void RcAgent::apply_ladder_op(const std::array<LayerTxSpec, 4>& ladder,
                              double fec_overhead) {
  // A resolved DISC/RCF op is only ever applied on a path that (re)enters
  // LINKED (see on_rc_frame), so the sticky MAX_RANGE forced-shed from a
  // prior RENDEZVOUS/FAILSAFE no longer applies — clear it here rather than
  // in on_rc_frame so it's cleared atomically with the op that ends it.
  failsafe_shed_ = false;

  applied_.ladder = ladder;
  applied_.fec_overhead = fec_overhead;
  applied_.shed[0] = false;
  applied_.shed[1] = false;
  applied_.shed[2] = shed_level_ >= 2;
  applied_.shed[3] = shed_level_ >= 1;
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
// shed[2]/[3] OR together every independent reason a layer must be shed:
// failsafe_shed_ (sticky for as long as MAX_RANGE is the operating point —
// see apply_max_range/apply_ladder_op) and the local congestion-shed level.
// Without the OR, this recompute-from-shed_level_-alone would clobber a
// forced failsafe shed the moment a congestion tick runs while in FAILSAFE.
void RcAgent::reapply_with_shed() {
  applied_.shed[2] = failsafe_shed_ || (shed_level_ >= 2);
  applied_.shed[3] = failsafe_shed_ || (shed_level_ >= 1);
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
void RcAgent::run_bitrate_policy(uint64_t now_ms, bool force) {
  double mbps = rc::phy_rate_mbps(applied_.ladder[1]);  // T0
  double overhead = uep_layer_overhead(1, applied_.fec_overhead);
  double kbps = mbps * 1000.0 * cfg_.waybeam.airtime_budget / (1.0 + overhead);
  kbps = std::clamp(kbps, static_cast<double>(cfg_.waybeam.bitrate_min_kbps),
                     static_cast<double>(cfg_.waybeam.bitrate_max_kbps));
  int kbps_i = round_to_100(kbps);

  bool decrease = have_last_bitrate_ && kbps_i < last_bitrate_kbps_;
  bool changed = !have_last_bitrate_ || kbps_i != last_bitrate_kbps_;
  bool throttled =
      have_last_bitrate_eval_ && now_ms - last_bitrate_eval_ms_ < 1000;
  if (force || decrease || (changed && !throttled)) {
    act_.set_bitrate_kbps(kbps_i);
    last_bitrate_kbps_ = kbps_i;
    have_last_bitrate_ = true;
    last_bitrate_eval_ms_ = now_ms;
    have_last_bitrate_eval_ = true;
  }

  bool now_low = kbps_i < cfg_.waybeam.roi_threshold_kbps;
  if (now_low != roi_low_) {
    roi_low_ = now_low;
    act_.set_roi_qp(now_low ? cfg_.waybeam.roi_qp_low : cfg_.waybeam.roi_qp_normal);
  }
}

// Escalates shed_level_ (0..3) by one step the instant tx_drops rises since
// the last tick (congestion is assumed the moment drops increase — no
// debounce on the way up), and decays it by exactly one step down per 2s
// clean window (2000ms with no further drop-rise); each step-down restarts
// the 2s window, so recovering from level 3 to level 0 takes three separate
// 2s clean windows in a row, not one. shed_level_ >= 1 sheds sid 3 (SVC-T
// enhance, droppable) and >= 2 also sheds sid 2 (reserved). Any level change
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
    apply_ladder_op(ladder, row.fec_overhead);

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

    // s3 probe (spec 2026-08-05): when the RCF carries probe3, layer 3 (s3)
    // transmits at probe_profile's MCS while mode/bw stay the base
    // profile's — MCS-only probe, everything else (CRIT/T0/T1) rides the
    // normal ladder. Recomputed on every accepted RCF, so a follow-up frame
    // without the flag reverts s3 to the base profile's mcs.
    probe3_active_ = false;
    if (r->probe3) {
      PhyMode pmode;
      uint8_t pmcs, pbw;
      rc::decode_profile(r->probe_profile, pmode, pmcs, pbw);
      ladder[3].mcs = pmcs;
      probe3_active_ = true;
    }

    State prev_state = state_;
    apply_ladder_op(ladder, r->fec_overhead());

    if (prev_state == State::BOOT || prev_state == State::RENDEZVOUS)
      link_established_ = true;
    state_ = State::LINKED;
    last_fb_ms_ = now_ms;
    have_last_fb_ = true;

    bool entering_linked = prev_state == State::FAILSAFE || prev_state == State::RENDEZVOUS;
    if (entering_linked) {
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
  // CAP_S3_PROBE: this drone accepts RCF_F_PROBE3 (spec 2026-08-05
  // s3-probe-promote).
  ack.chip_caps = rc::CAP_FRAME_WIRE | rc::CAP_TELEMETRY | rc::CAP_S3_PROBE;
  ack.agreed_channel = cfg_.radio.channel;
  ack.agreed_width = cfg_.radio.width;
  ack.seq = seq;
  return ack;
}

}  // namespace mabur
