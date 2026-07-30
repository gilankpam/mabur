#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace maburgs {

// One rung of the measured-loss ladder: a radio MCS plus the FEC command
// overhead (0.25 = baseline) that stream_id 1's uep_layer_overhead scaling
// uses to derive budget().
struct Rung {
  int mcs = 0;
  double overhead = 1.0;
};

struct LadderCfg {
  std::vector<Rung> ladder;  // effective (post-filter), size >= 1; [0] = failsafe
  double down_util = 0.6, up_util = 0.15;
  int confirm_ms = 250, clean_ms = 5000, probation_ms = 3000;
  int penalty_base_ms = 10000, penalty_max_ms = 60000;
  int hold_after_down_ms = 4000, min_between_changes_ms = 150;
  int feedback_timeout_ms = 1000;
  // A starved sample (zero completed base packets in a decode window while
  // video frames still arrive) must PERSIST this long before it forces rung
  // 0. The decode window is one RCF period (50 ms on the bench GS), and a
  // rung transition re-keys the drone's FEC stream, which reliably produces
  // 1-2 zero-completion windows on a perfectly healthy link — hw finding
  // 2026-07-27: without the debounce every promote starved itself back to
  // the floor. Transient starved samples still withhold all decisions and
  // never stamp feedback, so the blind-side timeout stays the backstop.
  int starved_confirm_ms = 300;
};

// One feedback sample: measured pre-FEC and residual (post-FEC) loss for the
// current rung's s1 stream, over the loss window the caller maintains.
struct LinkHealth {
  bool sample_valid = false;  // false when the s1 window saw 0 expected symbols
  double pre_fec_loss = 0.0;  // s1 missing/expected over the loss window
  double residual_loss = 0.0;
  bool video_starved = false;
};

enum class CtlReason { None, Residual, Util, Probation, Starved, Timeout, Promote };
const char* to_string(CtlReason r);

struct CtlEvent {
  double t_ms = 0;
  int from = 0, to = 0;
  CtlReason reason = CtlReason::None;
  double u = 0;
};

struct CtlCounters {
  uint64_t demotes_residual = 0, demotes_util = 0, promotes = 0,
           probation_fails = 0, starved_drops = 0, timeout_drops = 0;
};

// Measured-loss ladder controller: walks a fixed, pre-filtered list of rungs
// up on sustained clean margin and down on measured loss pressure or an
// explicit residual-loss/starvation/timeout signal. Pure decision logic —
// no clock, no I/O, no radio types; the caller supplies now_ms and drives
// update()/on_tick() every feedback tick / every tick respectively.
class LadderController {
 public:
  explicit LadderController(LadderCfg cfg);

  // Feedback tick: h is this window's measured health. Returns true when the
  // rung changed. Order of checks documented in ladder_controller.cpp.
  bool update(const LinkHealth& h, double now_ms);

  // Blind-side tick, called every tick regardless of feedback arrival: forces
  // the failsafe rung on feedback timeout and expires survived probation.
  // Returns true when the rung changed.
  bool on_tick(double now_ms);

  int rung() const { return idx_; }
  const Rung& op() const { return cfg_.ladder[static_cast<std::size_t>(idx_)]; }

  double util() const { return u_; }              // last computed u (0 before first valid sample)
  double pre_fec_loss() const { return pre_fec_loss_; }

  // s1 budget of the CURRENT rung: eff1 / (1 + eff1), eff1 =
  // mabur::uep_layer_overhead(1, op().overhead).
  double budget() const;

  int probation_ms_left(double now_ms) const;  // 0 when not probing
  std::vector<std::pair<int, int>> penalized(double now_ms) const;  // {rung, ms_left}

  const CtlCounters& counters() const { return counters_; }
  const CtlEvent& last_event() const { return last_event_; }

 private:
  void reset_windows();
  void check_probation_survival(double now_ms);
  void penalize_rung(int rung, double now_ms);
  bool is_penalized(int rung, double now_ms) const;
  void set_event(double now_ms, int from, int to, CtlReason reason, double u);

  LadderCfg cfg_;
  int idx_ = 0;

  double u_ = 0.0;
  double pre_fec_loss_ = 0.0;

  double last_feedback_ms_ = -1e18;
  double starved_since_ms_ = -1.0;  // <0 = not currently in a starved run
  double last_change_ms_ = -1e18;
  double last_down_ms_ = -1e18;

  double confirm_start_ms_ = -1.0;  // -1 = no active over-down_util window
  double clean_start_ms_ = -1.0;    // -1 = no active under-up_util window

  bool probation_active_ = false;
  double probation_until_ms_ = -1e18;
  int probation_rung_ = -1;

  std::vector<int> fail_count_;        // per-rung consecutive probation-fail count k
  std::vector<double> penalty_until_;  // per-rung penalty expiry (ms); -1e18 = none

  CtlCounters counters_;
  CtlEvent last_event_;
};

}  // namespace maburgs
