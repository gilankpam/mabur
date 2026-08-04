#include "ladder_controller.h"

#include <algorithm>
#include <cassert>

#include "mabur/uep_encoder.h"

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
  }
  return "unknown";
}

const char* to_string(ProbeOutcome o) {
  switch (o) {
    case ProbeOutcome::None: return "none";
    case ProbeOutcome::Pass: return "pass";
    case ProbeOutcome::Fail: return "fail";
    case ProbeOutcome::Abort: return "abort";
  }
  return "unknown";
}

LadderController::LadderController(LadderCfg cfg) : cfg_(std::move(cfg)) {
  assert(!cfg_.ladder.empty());
  fail_count_.assign(cfg_.ladder.size(), 0);
  penalty_until_.assign(cfg_.ladder.size(), -1e18);
}

double LadderController::budget() const {
  const double eff1 = mabur::uep_layer_overhead(1, op().overhead);
  return eff1 / (1.0 + eff1);
}

double LadderController::budget_for(int rung) const {
  const double eff1 = mabur::uep_layer_overhead(
      1, cfg_.ladder[static_cast<std::size_t>(rung)].overhead);
  return eff1 / (1.0 + eff1);
}

double LadderController::budget3_for(int rung) const {
  const double eff3 = mabur::uep_layer_overhead(
      3, cfg_.ladder[static_cast<std::size_t>(rung)].overhead);
  return eff3 / (1.0 + eff3);
}

double LadderController::probe_util_threshold() const {
  return cfg_.probe_max_util < 0 ? cfg_.down_util : cfg_.probe_max_util;
}

double LadderController::s3_util_threshold() const {
  return cfg_.s3_down_util < 0 ? cfg_.down_util : cfg_.s3_down_util;
}

// An s3 window only carries information when it actually saw traffic: the
// enhancement layer is shed under load, so "0 missing of 3 expected" says
// nothing about the candidate MCS.
bool LadderController::s3_usable(const LinkHealth& h) const {
  return h.s3_valid &&
         h.s3_expected_syms >= static_cast<uint64_t>(cfg_.probe_s3_min_syms);
}

void LadderController::mark_transition(double now_ms) {
  s3_blank_until_ms_ = now_ms + cfg_.s3_settle_ms;
  s3_resid_start_ms_ = -1.0;
  s3_util_start_ms_ = -1.0;
}

void LadderController::start_probe(int rung, double now_ms) {
  probe_active_ = true;
  probe_rung_ = rung;
  probe_start_ms_ = now_ms;
  probe_last_s3_ms_ = now_ms;
  probe_u_pred_last_ = 0.0;
  u3_ = 0.0;  // steady-state s3 is meaningless while s3 runs the candidate MCS
  ++counters_.probes_started;
  mark_transition(now_ms);
}

void LadderController::end_probe(ProbeOutcome oc, double u_pred,
                                 double now_ms) {
  last_probe_.t_ms = now_ms;
  last_probe_.rung = probe_rung_;
  last_probe_.outcome = oc;
  last_probe_.snr_db = snr_now_;
  last_probe_.u_pred = u_pred;
  last_probe_.dur_ms = static_cast<int>(now_ms - probe_start_ms_);
  probe_active_ = false;
  probe_rung_ = -1;
  mark_transition(now_ms);
}

void LadderController::reset_windows() {
  confirm_start_ms_ = -1.0;
  clean_start_ms_ = -1.0;
}

void LadderController::set_event(double now_ms, int from, int to,
                                  CtlReason reason, double u, double snr) {
  last_event_.t_ms = now_ms;
  last_event_.from = from;
  last_event_.to = to;
  last_event_.reason = reason;
  last_event_.u = u;
  last_event_.snr_db = snr;
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

bool LadderController::update(const LinkHealth& h, double now_ms) {
  check_probation_survival(now_ms);

  // Label only, stashed for every set_event()/end_probe() below. NaN is legal
  // (no SNR known this window) and never influences a decision.
  snr_now_ = h.s1_snr_db;

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
    // (Ordered before the idx_ == 0 early-out only so a probe running from
    // the failsafe rung is still abortable; both paths return false at rung 0
    // exactly as before.)
    if (now_ms - starved_since_ms_ < cfg_.starved_confirm_ms) return false;
    // Demotes always win over a probe: a confirmed starved run kills it
    // before the rung force below, so the drone stops running the candidate
    // MCS on s3 while the link is collapsing.
    if (probe_active_) {
      ++counters_.probe_aborts;
      end_probe(ProbeOutcome::Abort, 0.0, now_ms);
    }
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

  pre_fec_loss_ = h.pre_fec_loss;
  u_ = h.pre_fec_loss / budget();

  // 4. Residual (post-FEC) loss demotes immediately, exempt from
  // min_between_changes_ms. If the current rung was on probation, this also
  // books a probation failure and penalizes the rung.
  if (h.residual_loss > 0.0 && idx_ > 0) {
    // Demotes always win over a probe.
    if (probe_active_) {
      ++counters_.probe_aborts;
      end_probe(ProbeOutcome::Abort, 0.0, now_ms);
    }
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
    set_event(now_ms, from, idx_, CtlReason::Residual, u_, snr_now_);
    return true;
  }

  // 5. Utilization pressure: immediate demote during probation, otherwise a
  // confirm window that must stay above down_util continuously.
  if (u_ > cfg_.down_util) {
    if (confirm_start_ms_ < 0.0) confirm_start_ms_ = now_ms;

    const bool in_probation = probation_active_ && idx_ == probation_rung_;
    if (in_probation) {
      // Demotes always win over a probe.
      if (probe_active_) {
        ++counters_.probe_aborts;
        end_probe(ProbeOutcome::Abort, 0.0, now_ms);
      }
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

    if (idx_ > 0 && now_ms - confirm_start_ms_ >= cfg_.confirm_ms &&
        now_ms - last_change_ms_ >= cfg_.min_between_changes_ms) {
      // Demotes always win over a probe.
      if (probe_active_) {
        ++counters_.probe_aborts;
        end_probe(ProbeOutcome::Abort, 0.0, now_ms);
      }
      const int from = idx_;
      idx_ = from - 1;
      last_down_ms_ = now_ms;
      last_change_ms_ = now_ms;
      reset_windows();
      mark_transition(now_ms);
      ++counters_.demotes_util;
      set_event(now_ms, from, idx_, CtlReason::Util, u_, snr_now_);
      return true;
    }
  } else {
    confirm_start_ms_ = -1.0;
  }

  // 5b. s3 steady-state utilization. Only the measurement is maintained here
  // (so util3() and a Pass ProbeEvent report something real); the demote
  // decisions that consume it, plus s3_resid_start_ms_/s3_util_start_ms_ and
  // the s3_blank_until_ms_ gate, are the s3-steady-state-demote task's.
  //
  // ---- INSERTION POINT: s3 steady-state demotes (Task 5) ----
  if (probe_active_) {
    u3_ = 0.0;  // s3 is running the candidate MCS, not this rung's
  } else if (s3_usable(h) && now_ms >= s3_blank_until_ms_) {
    u3_ = h.s3_pre_fec_loss / budget3_for(idx_);
  }

  // 6. Probe evaluation. While a probe is up the clean/promote logic is fully
  // suspended: the candidate is being measured on s3, and the only outcomes
  // are fail (penalize the candidate, stay), pass (commit), or an abort from
  // a demote above / s3 silence in on_tick().
  if (probe_active_) {
    if (s3_usable(h)) {
      probe_last_s3_ms_ = now_ms;
      // Strictly after the settle window: the drone needs a moment to switch
      // s3 onto the candidate MCS, and the transition's own re-key glitch
      // must not be scored against the candidate.
      if (now_ms - probe_start_ms_ > cfg_.probe_settle_ms) {
        const double u_pred = h.s3_pre_fec_loss / budget_for(probe_rung_);
        // Stash every scored sample, pass or fail: a Pass that squeaked in at
        // u_pred 0.42 must not be logged as a flawless 0.0, or the labeled
        // dataset (sideport last_probe, ctl-log P lines) cannot tell a
        // marginal candidate from a clean one.
        probe_u_pred_last_ = u_pred;
        if (u_pred > probe_util_threshold()) {
          // The CANDIDATE rung earns the penalty (escalating ledger), and the
          // link never moved.
          penalize_rung(probe_rung_, now_ms);
          ++counters_.probe_fails;
          end_probe(ProbeOutcome::Fail, u_pred, now_ms);
          reset_windows();
          return false;  // rung did not change
        }
      }
    }
    if (probe_active_ && now_ms - probe_start_ms_ >= cfg_.probe_ms) {
      const int from = idx_;
      idx_ = probe_rung_;
      last_change_ms_ = now_ms;
      reset_windows();
      probation_active_ = true;
      probation_rung_ = idx_;
      probation_until_ms_ = now_ms + cfg_.probation_ms;
      ++counters_.promotes;
      ++counters_.probes_ok;
      set_event(now_ms, from, idx_, CtlReason::Promote, u_, snr_now_);
      end_probe(ProbeOutcome::Pass, probe_u_pred_last_, now_ms);
      return true;
    }
    return false;  // probing: clean/promote logic suspended
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
      // The probe replaces the promote MOMENT only — every trigger condition
      // above is untouched. When the peer can run the probe, s3 currently
      // carries enough traffic to measure, and the candidate actually changes
      // the PHY rate, spend probe_ms measuring the candidate MCS on the
      // expendable stream instead of betting the whole link on it. Otherwise
      // fall through to the legacy direct promote, byte-for-byte as before.
      const bool phy_differs =
          cfg_.ladder[next].mcs != cfg_.ladder[static_cast<std::size_t>(idx_)].mcs;
      if (h.probe_allowed && s3_usable(h) && phy_differs) {
        start_probe(static_cast<int>(next), now_ms);
        reset_windows();
        return false;
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
      set_event(now_ms, from, idx_, CtlReason::Promote, u_, snr_now_);
      return true;
    }
  } else {
    clean_start_ms_ = -1.0;
  }

  return false;
}

bool LadderController::on_tick(double now_ms) {
  check_probation_survival(now_ms);

  // A probe that stops hearing from s3 is inconclusive, not a failure: no
  // penalty, just drop it. This lives on the blind side so it works even when
  // the s1 samples stop arriving altogether (update() would never run).
  if (probe_active_ &&
      now_ms - probe_last_s3_ms_ > cfg_.probe_s3_silence_ms) {
    ++counters_.probe_aborts;
    end_probe(ProbeOutcome::Abort, 0.0, now_ms);
  }

  if (now_ms - last_feedback_ms_ > cfg_.feedback_timeout_ms && idx_ > 0) {
    // Demotes always win over a probe (normally the silence abort above has
    // already fired, since probe_s3_silence_ms < feedback_timeout_ms is the
    // usual configuration -- but that ordering is not guaranteed).
    if (probe_active_) {
      ++counters_.probe_aborts;
      end_probe(ProbeOutcome::Abort, 0.0, now_ms);
    }
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
