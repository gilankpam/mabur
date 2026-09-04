#include "ladder_controller.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace maburgs {

const char* to_string(CtlReason r) {
  // Lowercase: this is the operator-visible contract (the ctl: stderr line
  // and the sideport's link.ctl.last_event.reason), per the design spec's
  // reason enum (residual | util | probation | starved | timeout | promote).
  switch (r) {
    case CtlReason::None: return "none";
    case CtlReason::Residual: return "residual";
    case CtlReason::Util: return "util";
    case CtlReason::Probation: return "probation";
    case CtlReason::Starved: return "starved";
    case CtlReason::Timeout: return "timeout";
    case CtlReason::Promote: return "promote";
    case CtlReason::S3Residual: return "s3_residual";
    case CtlReason::S3Util: return "s3_util";
    case CtlReason::Fade: return "fade";
    case CtlReason::PromoteProbed: return "promote_probed";
  }
  return "unknown";
}

const char* to_string(ProbeGateState s) {
  switch (s) {
    case ProbeGateState::Off: return "off";
    case ProbeGateState::NoInfo: return "noinfo";
    case ProbeGateState::Clean: return "clean";
    case ProbeGateState::Lossy: return "lossy";
  }
  return "?";
}

LadderController::LadderController(LadderCfg cfg)
    : cfg_(std::move(cfg)), store_(cfg_.ladder.size(), cfg_.rung_stats) {
  assert(!cfg_.ladder.empty());
  fail_count_.assign(cfg_.ladder.size(), 0);
  penalty_until_.assign(cfg_.ladder.size(), -1e18);
}

double LadderController::budget_base() const {
  const double ov = op().overhead_base;
  return ov / (1.0 + ov);
}

double LadderController::budget_base_for(int rung) const {
  const double ov = cfg_.ladder[static_cast<std::size_t>(rung)].overhead_base;
  return ov / (1.0 + ov);
}

double LadderController::budget_enh_for(int rung) const {
  const double ov = cfg_.ladder[static_cast<std::size_t>(rung)].overhead_enh;
  return ov / (1.0 + ov);
}

double LadderController::probe_util_threshold() const {
  return cfg_.probe.max_util < 0 ? cfg_.down_util : cfg_.probe.max_util;
}

double LadderController::s3_util_threshold() const {
  return cfg_.s3_down_util < 0 ? cfg_.down_util : cfg_.s3_down_util;
}

// An s3 window only carries information when it actually saw traffic: the
// enhancement layer is shed under load, so "0 missing of 3 expected" says
// nothing about the candidate MCS.
bool LadderController::s3_usable(const LinkHealth& h) const {
  return h.s3_valid &&
         h.s3_expected_syms >= static_cast<uint64_t>(cfg_.s3_min_syms);
}

void LadderController::mark_transition(double now_ms) {
  s3_blank_until_ms_ = now_ms + cfg_.s3_settle_ms;
  s3_util_start_ms_ = -1.0;
  // The new rung's probe has measured nothing yet: drop the streak and the
  // last-sample stamp, and log the NoInfo edge so the ctl log shows the gap
  // rather than an inherited verdict from the rung the link just left. The
  // next update() with a usable sample flips it to Clean or Lossy and logs
  // that edge too.
  probe_clean_since_ms_ = -1.0;
  probe_last_sample_ms_ = -1e18;
  probe_hold_active_ = false;
  if (probe_state_ != ProbeGateState::Off)
    set_probe_state(ProbeGateState::NoInfo, probe_rung(), probe_u_, now_ms);
}

int LadderController::probe_rung() const {
  if (!cfg_.probe.enable) return -1;
  const int top = static_cast<int>(cfg_.ladder.size()) - 1;
  if (idx_ >= top) return -1;
  return std::min(idx_ + cfg_.probe.rung_offset, top);
}

ProbeGate LadderController::probe_gate(double now_ms) const {
  ProbeGate g;
  g.state = probe_state_;
  g.rung = probe_rung();
  g.u = probe_u_;
  g.streak_ms = (probe_state_ == ProbeGateState::Clean && probe_clean_since_ms_ >= 0.0)
                    ? now_ms - probe_clean_since_ms_ : 0.0;
  return g;
}

void LadderController::set_probe_state(ProbeGateState s, int rung, double u,
                                        double now_ms) {
  if (s == probe_state_) return;
  last_probe_edge_ = ProbeEdge{now_ms, rung, s, u, snr_now_, evm_now_,
                               now_ms - probe_state_since_ms_};
  probe_state_ = s;
  probe_state_since_ms_ = now_ms;
}

// Continuous probe verdict (spec 2026-09-04 §4.3). Runs on every valid
// sample before the decision blocks so the RungStore sees the sample that
// triggers a decision, and so the gate is current when block 6 consults it.
void LadderController::update_probe_gate(const LinkHealth& h, double now_ms) {
  const int pr = probe_rung();
  if (pr < 0) {
    probe_clean_since_ms_ = -1.0;
    set_probe_state(ProbeGateState::Off, -1, 0.0, now_ms);
    return;
  }
  const bool usable = h.probe_valid && h.probe_rung == pr &&
                      h.probe_expected_syms >= static_cast<uint64_t>(cfg_.probe.min_syms);
  if (usable) {
    const double b = budget_enh_for(idx_ + 1);  // the CANDIDATE's enh budget
    probe_u_ = b > 0.0 ? h.probe_loss / b : (h.probe_loss > 0.0 ? 1e9 : 0.0);
    probe_last_sample_ms_ = now_ms;
    store_.observe_probe(pr, probe_u_, now_ms);
    if (probe_u_ <= probe_util_threshold()) {
      if (probe_clean_since_ms_ < 0.0) probe_clean_since_ms_ = now_ms;
      set_probe_state(ProbeGateState::Clean, pr, probe_u_, now_ms);
    } else {
      probe_clean_since_ms_ = -1.0;
      set_probe_state(ProbeGateState::Lossy, pr, probe_u_, now_ms);
    }
  } else if (now_ms - probe_last_sample_ms_ > cfg_.probe.silence_ms) {
    // A commanded probe that has gone quiet says nothing, and saying nothing
    // holds the promote (block 6) exactly as a Lossy verdict does.
    probe_clean_since_ms_ = -1.0;
    set_probe_state(ProbeGateState::NoInfo, pr, probe_u_, now_ms);
  }
}

void LadderController::reset_windows() {
  confirm_start_ms_ = -1.0;
  clean_start_ms_ = -1.0;
  probe_hold_active_ = false;
}

void LadderController::set_event(double now_ms, int from, int to,
                                  CtlReason reason, double u, double snr) {
  store_.on_transition(from, to, reason, now_ms);
  last_event_.t_ms = now_ms;
  last_event_.from = from;
  last_event_.to = to;
  last_event_.reason = reason;
  last_event_.u = u;
  last_event_.snr_db = snr;
  last_event_.evm_db = evm_now_;
}

// A rung's probation is "survived" once the clock crosses probation_until_ms_
// while the controller is still parked on the rung that was promoted into
// (no demote consumed the probation window first). Surviving resets that
// rung's consecutive-failure count, so a later failure again costs only the
// base penalty duration instead of compounding.
void LadderController::check_probation_survival(double now_ms) {
  if (probation_active_ && idx_ == probation_rung_ &&
      now_ms >= probation_until_ms_) {
    fail_count_[static_cast<std::size_t>(idx_)] = 0;
    probation_active_ = false;
  }
}

void LadderController::penalize_rung(int rung, double now_ms) {
  const auto r = static_cast<std::size_t>(rung);
  // Cap the consecutive-failure count itself, not just the resulting
  // duration: shifting a (long long) by an exponent >= its bit width is UB
  // regardless of the eventual std::min saturation to penalty_max_ms below,
  // and a persistently marginal link can accumulate failures indefinitely
  // (fail_count_ only resets on surviving probation). kMaxShiftExp is a
  // generous constant bound -- for any realistic penalty_base_ms/
  // penalty_max_ms the duration already saturates well before k reaches it
  // (typically k~4-7), so clamping here never changes observable behavior,
  // it only stops the counter (and the shift amount derived from it) from
  // growing without limit.
  constexpr int kMaxShiftExp = 32;
  if (fail_count_[r] < kMaxShiftExp) ++fail_count_[r];
  const int k = fail_count_[r];
  const long long shifted = static_cast<long long>(cfg_.penalty_base_ms) << (k - 1);
  const double dur = std::min(static_cast<double>(cfg_.penalty_max_ms),
                               static_cast<double>(shifted));
  penalty_until_[r] = now_ms + dur;
  last_penalty_ = PenaltyEvent{now_ms, rung, k, penalty_until_[r]};
}

bool LadderController::is_penalized(int rung, double now_ms) const {
  return now_ms < penalty_until_[static_cast<std::size_t>(rung)];
}

std::vector<std::pair<int, int>> LadderController::penalized(double now_ms) const {
  std::vector<std::pair<int, int>> out;
  for (std::size_t r = 0; r < penalty_until_.size(); ++r) {
    if (now_ms < penalty_until_[r]) {
      out.emplace_back(static_cast<int>(r),
                        static_cast<int>(penalty_until_[r] - now_ms));
    }
  }
  return out;
}

int LadderController::probation_ms_left(double now_ms) const {
  if (probation_active_ && idx_ == probation_rung_ && now_ms < probation_until_ms_) {
    return static_cast<int>(probation_until_ms_ - now_ms);
  }
  return 0;
}

// Check order (a spec contract; the block numbers below follow it):
//   starved -> s1 residual -> fade -> s3 residual -> s1 util -> s3 util ->
//   clean/probe-gated promote.
// The probe verdict is not a check: it is scored once, up front, and only
// the promote block consults it.
bool LadderController::update(const LinkHealth& h, double now_ms) {
  check_probation_survival(now_ms);

  // Label only, stashed for every set_event()/probe edge below. NaN is legal
  // (no SNR known this window) and never influences a decision.
  snr_now_ = h.rf_snr_db;
  evm_now_ = h.rf_evm_db;

  // No sample has measured s3 yet this tick, so nothing may be reported for
  // it. Cleared here rather than in block 5a so the promise util3() makes —
  // never a persisted stale value — also holds on the paths that early-return
  // before 5a ever runs (starved, and !sample_valid).
  u3_ = 0.0;

  // 1. Starvation forces the failsafe rung regardless of anything else.
  // Deliberately does NOT stamp last_feedback_ms_ (see the sample_valid gate
  // below): starvation already forces rung 0 directly, so it doesn't need
  // the blind-side timeout's help, and a real "no measurement" run should
  // still be free to trip the timeout independently.
  if (h.video_starved) {
    if (starved_since_ms_ < 0.0) starved_since_ms_ = now_ms;
    // Debounce: a rung transition re-keys the drone's FEC stream and
    // reliably yields 1-2 zero-completion decode windows on a healthy link
    // (hw 2026-07-27). Withhold decisions but do not demote until the
    // starved run has persisted starved_confirm_ms.
    if (now_ms - starved_since_ms_ < cfg_.starved_confirm_ms) return false;
    if (idx_ == 0) return false;
    const int from = idx_;
    idx_ = 0;
    last_down_ms_ = now_ms;
    last_change_ms_ = now_ms;
    probation_active_ = false;
    reset_windows();
    mark_transition(now_ms);
    ++counters_.starved_drops;
    set_event(now_ms, from, 0, CtlReason::Starved, 0.0, snr_now_);
    return true;
  }

  starved_since_ms_ = -1.0;  // any non-starved sample ends the starved run

  // 2. No data this window -> no decision, and — critically — no feedback
  // stamp. update() is called on every RCF slot regardless of whether the
  // S1 window produced a real sample; stamping last_feedback_ms_ here would
  // let a stream of sample_valid=false health (with video still arriving)
  // suppress on_tick()'s blind-side timeout forever, holding an aggressive
  // rung on zero real measurements (reviewer finding 2026-07-27). Only a
  // genuine sample counts as "feedback received."
  if (!h.sample_valid) return false;

  last_feedback_ms_ = now_ms;
  // Before ANY decision block: this sample's loss numbers belong to the rung
  // the link is on right now, not to whatever a demote below steps us to.
  measured_rung_ = idx_;

  // Part B EWMA feed. Kept HERE, above every decision block, and not next to
  // the trigger in 4b: the residual demote in block 4 returns early, which is
  // exactly the loss phase of a fade, and a feed that only happens on ticks
  // that reach 4b freezes fade_drssi/fade_dsnr (sideport link.ctl.fade, the
  // ctl-log S line) through the episodes this feature will be tuned from —
  // then applies the whole multi-second gap as one dt step (review finding
  // 2026-08-14). Nothing between here and 4b reads the EWMAs, so the decision
  // ordering contract in 4b is untouched.
  //
  // The starved and !sample_valid paths above still do not feed. Both withhold
  // every decision, both usually carry NaN labels anyway (the per-window RF
  // staleness gate NaNs them when no card measured s1), and feed()'s alphas
  // are dt-derived, so skipping a run of ticks and applying the next sample
  // with the larger dt is the same continuous-time EWMA answer.
  //
  if (!std::isnan(h.rf_rssi_dbm)) fade_rssi_.feed(h.rf_rssi_dbm, now_ms);
  if (!std::isnan(h.rf_snr_db)) fade_snr_.feed(h.rf_snr_db, now_ms);

  pre_fec_loss_ = h.pre_fec_loss;
  u_ = h.pre_fec_loss / budget_base();

  // 3. Probe verdict. Scored before every decision block: a demote below
  // returns early, and the gate (plus its RungStore column and its edge log)
  // must still reflect the sample that caused it. Nothing here decides
  // anything — only block 6 reads the verdict.
  update_probe_gate(h, now_ms);

  // Observe-only rung statistics (spec 2026-08-13): gated on the same
  // post-transition blank as s3 decisions — FEC re-key artifacts must not
  // be attributed to the new rung (up to ~200 ms of pre-transition symbols
  // can outlive the blanking; see CLAUDE.md tuning invariant). Fed BEFORE
  // the decision blocks so the sample that triggers a demote still lands
  // on the rung it actually measured.
  if (now_ms >= s3_blank_until_ms_) {
    store_.observe_s1(idx_, u_, h.residual_loss > 0.0, now_ms);
    if (!std::isnan(h.rf_evm_db))
      store_.observe_evm(idx_, h.rf_evm_db, now_ms);
  }

  // 4. Residual (post-FEC) loss demotes immediately, exempt from
  // min_between_changes_ms. If the current rung was on probation, this also
  // books a probation failure and penalizes the rung.
  if (h.residual_loss > 0.0 && idx_ > 0) {
    const int from = idx_;
    const bool was_probation = probation_active_ && idx_ == probation_rung_;
    if (was_probation) {
      ++counters_.probation_fails;
      penalize_rung(from, now_ms);
      probation_active_ = false;
    }
    idx_ = from - 1;
    last_down_ms_ = now_ms;
    last_change_ms_ = now_ms;
    reset_windows();
    mark_transition(now_ms);
    ++counters_.demotes_residual;
    fade_until_ms_ = now_ms + cfg_.fade.hold_ms;
    set_event(now_ms, from, idx_, CtlReason::Residual, u_, snr_now_);
    return true;
  }

  // 4b. Part B: predictive fade demote. Both signals must drop together,
  // sustained continuously — a window where either condition fails OR either
  // signal is NaN breaks the run (errs toward not demoting). Latched to
  // exactly one rung per fade EVENT; the regime (Part A) carries the cascade
  // from there on measured pressure, at the shortened in-regime confirms.
  // Never books probation-fail or penalty: this is rung-independent RF
  // evidence, not proof the rung was marginal.
  // Ordering is a spec contract: after block 4 (residual wins the tick) and
  // before 5a (fade beats the confirm-window tiers). The EWMA feed itself
  // lives above block 4 — see the comment there.
  if (cfg_.fade.predict) {
    const bool measurable =
        !std::isnan(h.rf_rssi_dbm) && !std::isnan(h.rf_snr_db) &&
        fade_rssi_.has && fade_snr_.has;
    const bool over = measurable && fade_rssi_.delta() >= cfg_.fade.rssi_db &&
                      fade_snr_.delta() >= cfg_.fade.snr_db;
    // OBSERVED recovery — both deltas measurably back under threshold — is the
    // only thing that re-arms the latch, and it is deliberately NOT the !over
    // branch below. !over is also true on a NaN tick, and absence of evidence
    // is not evidence of recovery: one telemetry gap mid-fade (Task 4's
    // staleness gate NaNs these labels on purpose) would release the brake and
    // let the same ongoing fade fire a second demote trigger_ms later.
    // Breaking the pre-fire sustain run on !over errs toward NOT demoting;
    // clearing the post-fire latch there would invert that conservatism.
    if (measurable && fade_rssi_.delta() < cfg_.fade.rssi_db &&
        fade_snr_.delta() < cfg_.fade.snr_db) {
      fade_latched_ = false;
    }
    if (!over) {
      fade_trig_start_ms_ = -1.0;
    } else {
      if (fade_trig_start_ms_ < 0.0) fade_trig_start_ms_ = now_ms;
      if (!fade_latched_ && idx_ > 0 && idx_ >= cfg_.fade.min_rung &&
          now_ms - fade_trig_start_ms_ >= cfg_.fade.trigger_ms &&
          now_ms - last_change_ms_ >= cfg_.min_between_changes_ms) {
        const int from = idx_;
        probation_active_ = false;  // cleared, but NO probation-fail booked
        idx_ = from - 1;
        last_down_ms_ = now_ms;
        last_change_ms_ = now_ms;
        reset_windows();
        mark_transition(now_ms);
        fade_until_ms_ = now_ms + cfg_.fade.hold_ms;  // enter the regime
        ++counters_.demotes_fade;
        set_event(now_ms, from, idx_, CtlReason::Fade, u_, snr_now_);
        fade_trig_start_ms_ = -1.0;
        fade_latched_ = true;  // one fire per fade event
        return true;
      }
    }
  }

  // 5a. s3 steady-state measurement and early warning: s3's smaller FEC
  // budget exhausts before s1's, so confirmed s3 residual/util pressure
  // demotes without waiting for the base layer to degrade (spec 2026-08-05
  // section 1b). Suspended inside the post-transition blanking window (FEC
  // re-key artifacts read as loss). The probe stream is its own sid and does
  // not disturb this reading any more, so there is nothing else to suspend
  // it for.
  //
  // The measurement is computed once here so BOTH s3 checks see the same
  // number, and it is left at the 0 stamped at update() entry whenever s3 is
  // not measurable this window: a persisted last-good value would make util3()
  // (sideport link.ctl.u3) report a frozen stale reading after s3 goes quiet.
  const bool s3_live = s3_usable(h) && now_ms >= s3_blank_until_ms_;

  // Continuity gate. The s3 util confirm window below is an elapsed-time test
  // against a start stamp, which only means "sustained" while the
  // measurement is unbroken. A stamp that survives a gap measures wall clock
  // instead: one bad window, 800 ms of nothing, one more bad window would
  // read as 800 ms of confirmed pressure and demote on two samples — the
  // very instant demote the confirm window exists to prevent (review
  // finding). Breaking the run on any discontinuity errs toward NOT
  // demoting, which is the right way to be wrong for an early-warning signal
  // on the expendable layer.
  //
  // This check is what covers the gaps the !s3_live branch below cannot see:
  // update() early-returns above on a starved or invalid sample and never
  // reaches this block at all.
  if (now_ms - s3_last_live_ms_ > cfg_.s3_settle_ms) {
    s3_util_start_ms_ = -1.0;
  }

  if (!s3_live) {
    // u3_ stays at the 0 stamped at entry, and an unmeasurable window is a
    // discontinuity in its own right: break the run.
    s3_util_start_ms_ = -1.0;
  } else {
    s3_last_live_ms_ = now_ms;
    // A ladder entry whose enh overhead is zero has no s3 budget at all; any
    // loss there is infinite utilization rather than a division by zero.
    // Since same-rate-fixed-pairs (Task 4), s3 rides sid 1's own literal
    // overhead_enh, not sid 0's — budget3_for() is gone, this is
    // budget_enh_for() directly.
    const double b3 = budget_enh_for(idx_);
    u3_ = b3 > 0.0 ? h.s3_pre_fec_loss / b3
                   : (h.s3_pre_fec_loss > 0.0 ? 1e9 : 0.0);
    store_.observe_s3(idx_, u3_, h.s3_residual_loss > 0.0, now_ms);
  }

  // Same bookkeeping as the s1 util/probation demotes above, deliberately —
  // only the reason and the counter differ. set_event()'s u argument carries
  // u3 for s3 events.
  auto s3_demote_now = [&](CtlReason reason, uint64_t& counter) {
    const int from = idx_;
    if (probation_active_ && idx_ == probation_rung_) {
      ++counters_.probation_fails;
      penalize_rung(from, now_ms);
      probation_active_ = false;
    }
    idx_ = from - 1;
    last_down_ms_ = now_ms;
    last_change_ms_ = now_ms;
    reset_windows();
    mark_transition(now_ms);
    ++counter;
    fade_until_ms_ = now_ms + cfg_.fade.hold_ms;
    set_event(now_ms, from, idx_, reason, u3_, snr_now_);
  };

  // s3 residual (post-FEC abandonment) demotes IMMEDIATELY, like s1's, and
  // like it is exempt from min_between_changes_ms. It was a confirmed
  // window until 2026-08-15 because a single abandoned window reads as a
  // normal shed/blip -- but a shed window carries no s3 traffic at all, so
  // s3_live is false and no s3 decision runs. What actually needed the
  // window was transition debris, and attribution removes that from the
  // input outright: the watermark is in symbol-sequence space
  // (sw_decoder.h), exact and permanent, not a settling heuristic. Checked
  // BEFORE the s1 util block so that when both ripen on the same tick the
  // event reason attributes to s3.
  //
  // Deliberately still lives in block 5a rather than beside the s1 residual
  // block (block 4) above: this check needs s3_live, computed in the 5a
  // preamble, so hoisting it means hoisting that whole preamble too, and
  // staying here keeps this check's position relative to the 4b fade
  // trigger unchanged from before the branch. Do not "fix" this into 4.
  if (cfg_.s3_demote && s3_live && h.s3_residual_loss > 0.0 && idx_ > 0) {
    s3_demote_now(CtlReason::S3Residual, counters_.demotes_s3_residual);
    return true;
  }

  // 5. Utilization pressure: immediate demote during probation, otherwise a
  // confirm window that must stay above down_util continuously.
  if (u_ > cfg_.down_util) {
    if (confirm_start_ms_ < 0.0) confirm_start_ms_ = now_ms;

    const bool in_probation = probation_active_ && idx_ == probation_rung_;
    if (in_probation) {
      const int from = idx_;
      ++counters_.probation_fails;
      penalize_rung(from, now_ms);
      probation_active_ = false;
      idx_ = from - 1;
      last_down_ms_ = now_ms;
      last_change_ms_ = now_ms;
      reset_windows();
      mark_transition(now_ms);
      set_event(now_ms, from, idx_, CtlReason::Probation, u_, snr_now_);
      return true;
    }

    if (idx_ > 0 && now_ms - confirm_start_ms_ >= eff_confirm_ms(now_ms) &&
        now_ms - last_change_ms_ >= cfg_.min_between_changes_ms) {
      const int from = idx_;
      idx_ = from - 1;
      last_down_ms_ = now_ms;
      last_change_ms_ = now_ms;
      reset_windows();
      mark_transition(now_ms);
      ++counters_.demotes_util;
      fade_until_ms_ = now_ms + cfg_.fade.hold_ms;
      set_event(now_ms, from, idx_, CtlReason::Util, u_, snr_now_);
      return true;
    }
  } else {
    confirm_start_ms_ = -1.0;
  }

  // 5b. s3 utilization pressure, on the same confirm_ms window as s1's. Ranked
  // after the s1 util block (and, like it, uses u3_ computed in 5a above) so a
  // tick where both layers are over threshold still attributes to s1's own
  // reason; s3 leads only on the residual signal in 5a.
  if (cfg_.s3_demote && s3_live) {
    if (u3_ > s3_util_threshold()) {
      if (s3_util_start_ms_ < 0.0) s3_util_start_ms_ = now_ms;
      if (idx_ > 0 &&
          now_ms - s3_util_start_ms_ >= eff_s3_util_confirm_ms(now_ms) &&
          now_ms - last_change_ms_ >= cfg_.min_between_changes_ms) {
        s3_demote_now(CtlReason::S3Util, counters_.demotes_s3_util);
        return true;
      }
    } else {
      s3_util_start_ms_ = -1.0;
    }
  }

  // 6. Clean margin: accrue a clean window and promote once it has been
  // sustained long enough, clear of a recent downgrade and the min-between
  // gate, with a next rung that exists and isn't penalized.
  if (u_ < cfg_.up_util) {
    if (clean_start_ms_ < 0.0) clean_start_ms_ = now_ms;

    const std::size_t next = static_cast<std::size_t>(idx_) + 1;
    if (next < cfg_.ladder.size() &&
        now_ms - clean_start_ms_ >= cfg_.clean_ms &&
        now_ms - last_down_ms_ >= cfg_.hold_after_down_ms &&
        now_ms - last_change_ms_ >= cfg_.min_between_changes_ms &&
        !is_penalized(static_cast<int>(next), now_ms)) {
      // Probe gate (spec 2026-09-04 §4.4): with a probe commanded, only a
      // clean streak of probe.clean_ms may commit; Lossy AND NoInfo hold —
      // a commanded-but-absent probe is exactly the blind promote the
      // stream exists to prevent. No penalty: nothing was tried.
      const ProbeGate g = probe_gate(now_ms);
      CtlReason reason = CtlReason::Promote;
      if (g.state != ProbeGateState::Off) {
        if (g.state == ProbeGateState::Clean && g.streak_ms >= cfg_.probe.clean_ms) {
          reason = CtlReason::PromoteProbed;
        } else {
          if (!probe_hold_active_) { probe_hold_active_ = true; ++counters_.probe_holds; }
          return false;
        }
      }
      const int from = idx_;
      idx_ = static_cast<int>(next);
      last_change_ms_ = now_ms;
      reset_windows();
      probation_active_ = true;
      probation_rung_ = idx_;
      probation_until_ms_ = now_ms + cfg_.probation_ms;
      mark_transition(now_ms);
      ++counters_.promotes;
      if (reason == CtlReason::PromoteProbed) ++counters_.promotes_probed;
      set_event(now_ms, from, idx_, reason, u_, snr_now_);
      return true;
    }
  } else {
    clean_start_ms_ = -1.0;
    probe_hold_active_ = false;
  }

  return false;
}

bool LadderController::on_tick(double now_ms) {
  check_probation_survival(now_ms);

  if (now_ms - last_feedback_ms_ > cfg_.feedback_timeout_ms && idx_ > 0) {
    const int from = idx_;
    idx_ = 0;
    last_down_ms_ = now_ms;
    last_change_ms_ = now_ms;
    probation_active_ = false;
    reset_windows();
    mark_transition(now_ms);
    ++counters_.timeout_drops;
    set_event(now_ms, from, 0, CtlReason::Timeout, 0.0, snr_now_);
    return true;
  }
  return false;
}

}  // namespace maburgs
