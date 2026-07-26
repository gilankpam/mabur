#include "ladder_controller.h"

#include <algorithm>
#include <cassert>

#include "mabur/uep_encoder.h"

namespace maburgs {

const char* to_string(CtlReason r) {
  switch (r) {
    case CtlReason::None: return "None";
    case CtlReason::Residual: return "Residual";
    case CtlReason::Util: return "Util";
    case CtlReason::Probation: return "Probation";
    case CtlReason::Starved: return "Starved";
    case CtlReason::Timeout: return "Timeout";
    case CtlReason::Promote: return "Promote";
  }
  return "Unknown";
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

void LadderController::reset_windows() {
  confirm_start_ms_ = -1.0;
  clean_start_ms_ = -1.0;
}

void LadderController::set_event(double now_ms, int from, int to,
                                  CtlReason reason, double u) {
  last_event_.t_ms = now_ms;
  last_event_.from = from;
  last_event_.to = to;
  last_event_.reason = reason;
  last_event_.u = u;
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
  last_feedback_ms_ = now_ms;
  check_probation_survival(now_ms);

  // 1. Starvation forces the failsafe rung regardless of anything else.
  if (h.video_starved) {
    if (idx_ == 0) return false;
    const int from = idx_;
    idx_ = 0;
    last_down_ms_ = now_ms;
    last_change_ms_ = now_ms;
    probation_active_ = false;
    reset_windows();
    ++counters_.starved_drops;
    set_event(now_ms, from, 0, CtlReason::Starved, 0.0);
    return true;
  }

  // 2. No data this window -> no decision.
  if (!h.sample_valid) return false;

  pre_fec_loss_ = h.pre_fec_loss;
  u_ = h.pre_fec_loss / budget();

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
    ++counters_.demotes_residual;
    set_event(now_ms, from, idx_, CtlReason::Residual, u_);
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
      set_event(now_ms, from, idx_, CtlReason::Probation, u_);
      return true;
    }

    if (idx_ > 0 && now_ms - confirm_start_ms_ >= cfg_.confirm_ms &&
        now_ms - last_change_ms_ >= cfg_.min_between_changes_ms) {
      const int from = idx_;
      idx_ = from - 1;
      last_down_ms_ = now_ms;
      last_change_ms_ = now_ms;
      reset_windows();
      ++counters_.demotes_util;
      set_event(now_ms, from, idx_, CtlReason::Util, u_);
      return true;
    }
  } else {
    confirm_start_ms_ = -1.0;
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
      const int from = idx_;
      idx_ = static_cast<int>(next);
      last_change_ms_ = now_ms;
      reset_windows();
      probation_active_ = true;
      probation_rung_ = idx_;
      probation_until_ms_ = now_ms + cfg_.probation_ms;
      ++counters_.promotes;
      set_event(now_ms, from, idx_, CtlReason::Promote, u_);
      return true;
    }
  } else {
    clean_start_ms_ = -1.0;
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
    ++counters_.timeout_drops;
    set_event(now_ms, from, 0, CtlReason::Timeout, 0.0);
    return true;
  }
  return false;
}

}  // namespace maburgs
