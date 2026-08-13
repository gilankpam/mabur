#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace maburgs {

// Defined in ladder_controller.h; forward-declared to keep the include
// direction ladder_controller.h -> rung_store.h acyclic.
enum class CtlReason;

// Per-rung EWMA store config (spec 2026-08-13-rung-store-design.md):
// alpha = 1 - 0.5^(1/half_life_samples). Default ~30 s of parked time at
// the 50 ms RCF cadence.
struct RungStoreCfg {
  int half_life_samples = 600;
};

// One exponentially weighted mean, seeded by its first sample (no
// decay-from-zero warm-up bias). n counts samples ever fed. Values are
// sample-driven and NEVER decay with wall time (spec decision): consumers
// discount staleness via the exported ages/counts instead.
struct Ewma {
  double v = 0.0;
  uint64_t n = 0;
  void add(double x, double alpha) {
    v = (n == 0) ? x : v + alpha * (x - v);
    ++n;
  }
};

struct RungStat {
  Ewma u;         // s1 utilization while parked on this rung
  Ewma resid;     // (residual_loss > 0) indicator, 0..1
  Ewma u3;        // s3 utilization while parked on this rung
  Ewma s3_resid;  // (s3_residual > 0) indicator, 0..1
  Ewma probe_u;   // u_pred from probes OF this rung; never blended with u
  // Per-rung EVM baseline: EW mean (dB, NaN until seeded) + EW variance
  // (one-pass update alongside the mean).
  double evm_db = std::numeric_limits<double>::quiet_NaN();
  double evm_var_db2 = 0.0;
  uint64_t evm_n = 0;
  double last_sample_ms = -1.0;  // parked (s1/s3) age; <0 = never sampled
  double last_probe_ms = -1.0;   // probe age; <0 = never probed
  double dwell_ms = 0.0;         // CLOSED dwell; live via RungStore::dwell_ms()
  uint32_t visits = 0;           // transitions INTO this rung (initial rung 0
                                 // occupancy is not a visit)
  uint32_t exits_bad = 0;        // departures for residual/s3_residual/
                                 // probation/starved (not util/timeout)
};

// Observe-only per-rung statistics store. Pure: no clock, no I/O; the
// owner (LadderController) supplies now_ms and attribution. Process-scoped
// like the penalty ledger — no persistence.
class RungStore {
 public:
  RungStore(std::size_t n_rungs, RungStoreCfg cfg);

  void observe_s1(int rung, double u, bool residual, double now_ms);
  void observe_evm(int rung, double evm_db, double now_ms);
  void observe_s3(int rung, double u3, bool s3_residual, double now_ms);
  void observe_probe(int rung, double u_pred, double now_ms);
  void on_transition(int from, int to, CtlReason reason, double now_ms);

  const RungStat& stat(int rung) const {
    return stats_[static_cast<std::size_t>(rung)];
  }
  std::size_t size() const { return stats_.size(); }
  // Cumulative parked time; includes the live interval when `rung` is the
  // current rung. Dwell accrues from the first observe_s1() after start; after any transition it accrues from the transition itself (even before the next sample).
  double dwell_ms(int rung, double now_ms) const;
  double alpha() const { return alpha_; }

 private:
  std::vector<RungStat> stats_;
  double alpha_ = 0.0;
  int current_ = 0;
  double parked_since_ms_ = -1.0;  // <0 until the first s1 observation
};

}  // namespace maburgs
