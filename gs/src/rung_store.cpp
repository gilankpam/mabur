#include "rung_store.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "ladder_controller.h"

namespace maburgs {

namespace {
// Mirrors CtlLog/StatsExporter clamp_util(): utilization ratios carry a 1e9
// zero-guard sentinel when a degenerate config zeroes the divisor budget.
// An EWMA poisoned by that magnitude stays wrong for the process lifetime,
// so the clamp lives at the ingest, not just at the write sites.
double clamp_util(double u) { return std::min(u, 1e3); }
}  // namespace

RungStore::RungStore(std::size_t n_rungs, RungStoreCfg cfg) : stats_(n_rungs) {
  assert(n_rungs >= 1);
  assert(cfg.half_life_samples >= 1);
  alpha_ =
      1.0 - std::pow(0.5, 1.0 / static_cast<double>(cfg.half_life_samples));
}

void RungStore::observe_s1(int rung, double u, bool residual, double now_ms) {
  auto& s = stats_[static_cast<std::size_t>(rung)];
  s.u.add(clamp_util(u), alpha_);
  s.resid.add(residual ? 1.0 : 0.0, alpha_);
  s.last_sample_ms = now_ms;
  if (parked_since_ms_ < 0.0) parked_since_ms_ = now_ms;
}

void RungStore::observe_evm(int rung, double evm_db, double now_ms) {
  (void)now_ms;  // age rides last_sample_ms, stamped by the paired observe_s1
  auto& s = stats_[static_cast<std::size_t>(rung)];
  if (s.evm_n == 0) {
    s.evm_db = evm_db;
    s.evm_var_db2 = 0.0;
  } else {
    const double d = evm_db - s.evm_db;
    const double incr = alpha_ * d;
    s.evm_db += incr;
    s.evm_var_db2 = (1.0 - alpha_) * (s.evm_var_db2 + d * incr);
  }
  ++s.evm_n;
}

void RungStore::observe_s3(int rung, double u3, bool s3_residual,
                            double now_ms) {
  auto& s = stats_[static_cast<std::size_t>(rung)];
  s.u3.add(clamp_util(u3), alpha_);
  s.s3_resid.add(s3_residual ? 1.0 : 0.0, alpha_);
  s.last_sample_ms = now_ms;
}

void RungStore::observe_probe(int rung, double u_pred, double now_ms) {
  auto& s = stats_[static_cast<std::size_t>(rung)];
  s.probe_u.add(clamp_util(u_pred), alpha_);
  s.last_probe_ms = now_ms;
}

void RungStore::on_transition(int from, int to, CtlReason reason,
                               double now_ms) {
  if (from == to) return;
  auto& f = stats_[static_cast<std::size_t>(from)];
  if (parked_since_ms_ >= 0.0) f.dwell_ms += now_ms - parked_since_ms_;
  parked_since_ms_ = now_ms;
  current_ = to;
  ++stats_[static_cast<std::size_t>(to)].visits;
  if (reason == CtlReason::Residual || reason == CtlReason::S3Residual ||
      reason == CtlReason::Probation || reason == CtlReason::Starved) {
    ++f.exits_bad;
  }
}

double RungStore::dwell_ms(int rung, double now_ms) const {
  const auto& s = stats_[static_cast<std::size_t>(rung)];
  double d = s.dwell_ms;
  if (rung == current_ && parked_since_ms_ >= 0.0)
    d += now_ms - parked_since_ms_;
  return d;
}

}  // namespace maburgs
