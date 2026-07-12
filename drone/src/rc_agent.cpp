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

// Merges cfg.power_offset_db (per-rung, CRIT/T0/T1/T2) into a ladder's
// carried-but-unused-in-v1 power_offset_db field.
void merge_power_offsets(std::array<LayerTxSpec, 4>& ladder,
                         const std::array<int8_t, 4>& offsets) {
  for (size_t i = 0; i < 4; ++i) ladder[i].power_offset_db = offsets[i];
}

int round_to_100(double v) { return static_cast<int>(std::lround(v / 100.0) * 100); }

}  // namespace

RcAgent::RcAgent(const Config& cfg, Actuator& act) : cfg_(cfg), act_(act) {
  commanded_pwr_idx_ = std::min(63, cfg_.radio.max_txagc);
}

// Applies the MAX_RANGE operating point (profile_table()[MAX_RANGE_PROFILE]
// via ladder_from). This is a state-transition apply (BOOT's initial op, or
// a LINKED->FAILSAFE entry), so it forces the bitrate policy to the robust
// MCS0 floor bitrate immediately, bypassing the steady-state throttle and
// hysteresis — the spec mandates failsafe = robust MCS + floor bitrate, and
// a degraded radio link must never keep flooding at the last LINKED rate.
void RcAgent::apply_max_range(uint64_t now_ms) {
  auto ladder = rc::ladder_from(PhyMode::HT, 0, 20, cfg_.flags);
  merge_power_offsets(ladder, cfg_.power_offset_db);
  commanded_pwr_idx_ = std::min(63, cfg_.radio.max_txagc);
  thermal_derate_ = 0;

  // Forced shed while MAX_RANGE is the operating point (BOOT/RENDEZVOUS or a
  // LINKED->FAILSAFE entry) — held sticky in failsafe_shed_ until an
  // RCF/DISC takes the agent back to LINKED (see apply_ladder_op), so a
  // later thermal/congestion reapply's recompute of shed[2]/[3] (which is
  // otherwise driven by shed_level_ alone) can't silently clobber it.
  failsafe_shed_ = true;

  applied_.ladder = ladder;
  applied_.fec_overhead = 1.0;
  applied_.pwr_idx = commanded_pwr_idx_;
  applied_.shed[0] = false;
  applied_.shed[1] = false;
  applied_.shed[2] = true;  // T1 shed in MAX_RANGE, per spec (already covers shed_level_>=2)
  applied_.shed[3] = true;  // T2 shed in MAX_RANGE, per spec (already covers shed_level_>=1)
  ++applied_.generation;
  act_.apply_op(applied_);
  run_bitrate_policy(now_ms, /*force=*/true);
}

// Applies a resolved (DISC row or RCF-decoded) ladder/power/FEC operating
// point. Does NOT run the bitrate policy itself — callers on the RCF path
// invoke run_bitrate_policy() explicitly afterwards, per the spec.
void RcAgent::apply_ladder_op(const std::array<LayerTxSpec, 4>& ladder, int pwr_idx,
                              double fec_overhead) {
  commanded_pwr_idx_ = pwr_idx;
  thermal_derate_ = 0;

  // A resolved DISC/RCF op is only ever applied on a path that (re)enters
  // LINKED (see on_rc_frame), so the sticky MAX_RANGE forced-shed from a
  // prior RENDEZVOUS/FAILSAFE no longer applies — clear it here rather than
  // in on_rc_frame so it's cleared atomically with the op that ends it.
  failsafe_shed_ = false;

  applied_.ladder = ladder;
  applied_.fec_overhead = fec_overhead;
  applied_.pwr_idx = commanded_pwr_idx_;
  applied_.shed[0] = false;
  applied_.shed[1] = false;
  applied_.shed[2] = shed_level_ >= 2;
  applied_.shed[3] = shed_level_ >= 1;
  ++applied_.generation;
  act_.apply_op(applied_);
}

// Reapplies the current commanded op (ladder/fec unchanged) with the
// current thermal derate and congestion-shed levels folded in. Used by the
// thermal guard and congestion guard, neither of which changes the ladder
// or bumps generation on their own — generation tracks *new operating
// points* (BOOT/DISC/RCF/failsafe entry), not power/shed adjustments to the
// current one, so this function publishes via act_.apply_op() WITHOUT
// touching applied_.generation. Consumers (the hot-thread loops in
// main.cpp) must therefore detect "a new AppliedOp was published" by
// identity-comparing the shared_ptr they last applied against the one the
// atomic handoff currently holds, NOT by checking whether generation
// changed — otherwise congestion shed / thermal derate would be computed
// here but silently never reach the encoder.
//
// shed[2]/[3] OR together every independent reason a layer must be shed:
// failsafe_shed_ (sticky for as long as MAX_RANGE is the operating point —
// see apply_max_range/apply_ladder_op) and the local congestion-shed level.
// Without the OR, this recompute-from-shed_level_-alone would clobber a
// forced failsafe shed the moment a congestion tick runs while in FAILSAFE.
void RcAgent::reapply_with_derate_and_shed() {
  applied_.pwr_idx = std::max(0, commanded_pwr_idx_ - thermal_derate_);
  applied_.shed[2] = failsafe_shed_ || (shed_level_ >= 2);
  applied_.shed[3] = failsafe_shed_ || (shed_level_ >= 1);
  act_.apply_op(applied_);
}

// Runs whenever a new op is applied. The formula is always recomputed, but
// whether an actual set_bitrate_kbps() call happens depends on `force`:
//
//  - force=false (steady-state LINKED RCFs): the call is throttled to at
//    most once per 1000ms *and* gated by +-10% hysteresis vs the last value
//    actually sent, so repeated identical (or near-identical) RCFs within
//    one second collapse to a single call (scenario: bitrate hysteresis).
//  - force=true (BOOT's initial MAX_RANGE apply, every LINKED->FAILSAFE
//    entry, and RCFs that transition into LINKED from RENDEZVOUS/FAILSAFE):
//    the throttle and hysteresis gates are bypassed entirely — the new
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

  bool changed_enough =
      !have_last_bitrate_ || std::abs(kbps_i - last_bitrate_kbps_) > last_bitrate_kbps_ / 10;
  bool throttled =
      have_last_bitrate_eval_ && now_ms - last_bitrate_eval_ms_ < 1000;
  if (force || (changed_enough && !throttled)) {
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

void RcAgent::run_thermal_guard(const RadioHealth& health) {
  if (health.thermal_delta > cfg_.radio.thermal_max_delta) {
    thermal_derate_ = std::min(commanded_pwr_idx_, thermal_derate_ + 4);
    reapply_with_derate_and_shed();
  } else if (health.thermal_delta <= cfg_.radio.thermal_max_delta - 2) {
    if (thermal_derate_ != 0) {
      thermal_derate_ = 0;
      reapply_with_derate_and_shed();
    }
  }
}

// Escalates shed_level_ (0..3) by one step the instant tx_drops rises since
// the last tick (congestion is assumed the moment drops increase — no
// debounce on the way up), and decays it by exactly one step down per 2s
// clean window (2000ms with no further drop-rise); each step-down restarts
// the 2s window, so recovering from level 3 to level 0 takes three separate
// 2s clean windows in a row, not one. shed_level_ >= 1 sheds T2, >= 2 also
// sheds T1, and reaching level 3 additionally cuts the encoder bitrate by
// 30% once. Any level change reapplies the current op (derate+shed folded
// in) so the change reaches the actuator immediately.
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
      if (shed_level_ == 3) {
        int reduced = static_cast<int>(std::lround(last_bitrate_kbps_ * 0.7));
        act_.set_bitrate_kbps(reduced);
        last_bitrate_kbps_ = reduced;
      }
    }
  } else if (have_last_drop_rise_ && now_ms - last_drop_rise_ms_ >= 2000) {
    if (shed_level_ > 0) {
      --shed_level_;
      changed = true;
      last_drop_rise_ms_ = now_ms;  // restart the 2s window for the next step-down
    }
  }

  if (changed) reapply_with_derate_and_shed();
}

void RcAgent::on_rc_frame(const uint8_t* body, size_t len, uint64_t now_ms) {
  int type = rc::frame_type(body, len);
  if (type == rc::T_DISC) {
    auto d = rc::parse_disc(body, len);
    if (!d.has_value() || d->vtx_id != cfg_.link.vtx_id) return;

    // Python parity (rendezvous.feed_disc: `if self.state not in (RC_LOST,
    // DISCOVERY): return None`): a DISC only re-establishes a lost link. The
    // GS's SESSION keep-alive DISC (~1 Hz) exists for a drone that silently
    // fell back to rendezvous; a LINKED drone must ignore it outright — no
    // DISC_ACK, no init-profile apply, no watchdog refresh. Without this
    // guard every heard keep-alive yanked the op to the MAX_RANGE row and
    // floor bitrate once a second (bench-observed op thrash, 2026-07-12).
    if (state_ == State::LINKED) return;

    // Echo the drone's ACTUAL operating channel/width, not the GS-requested
    // d->op_channel/op_width — the drone doesn't retune in v1 (its channel
    // is fixed from its own config at startup), so acking back the GS's
    // request would claim an agreement that never happened whenever the
    // two disagree. Reporting the true op point lets the GS detect and
    // handle a mismatch instead of being told (incorrectly) that its
    // request was honored.
    DiscAck ack;
    ack.vtx_id = cfg_.link.vtx_id;
    ack.vrx_nonce = d->vrx_nonce;
    ack.chip_caps = 0;
    ack.agreed_channel = cfg_.radio.channel;
    ack.agreed_width = cfg_.radio.width;
    ack.seq = d->seq;
    act_.send_control(rc::pack_disc_ack(ack));

    int row_idx = std::clamp<int>(d->init_profile, 0,
                                   static_cast<int>(rc::profile_table().size()) - 1);
    auto ladder = rc::ladder_for_row(row_idx, cfg_.flags);
    merge_power_offsets(ladder, cfg_.power_offset_db);
    const auto& row = rc::profile_table()[static_cast<size_t>(row_idx)];
    apply_ladder_op(ladder, row.pwr_idx, row.fec_overhead);

    state_ = State::LINKED;
    last_fb_ms_ = now_ms;
    have_last_fb_ = true;

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

    PhyMode mode;
    uint8_t mcs, bw;
    rc::decode_profile(r->profile, mode, mcs, bw);
    auto ladder = rc::ladder_from(mode, mcs, bw, cfg_.flags);
    merge_power_offsets(ladder, cfg_.power_offset_db);

    int pwr_idx = commanded_pwr_idx_;
    if (r->pwr_idx != rc::PWR_NO_CHANGE) {
      pwr_idx = std::clamp<int>(r->pwr_idx, 0, cfg_.radio.max_txagc);
    }

    State prev_state = state_;
    apply_ladder_op(ladder, pwr_idx, r->fec_overhead());

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

  run_thermal_guard(health);
  run_congestion_guard(now_ms, health);
}

}  // namespace mabur
