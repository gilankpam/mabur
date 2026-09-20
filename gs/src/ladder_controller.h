#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "rung_store.h"

namespace maburgs {

// One rung of the measured-loss ladder: a radio MCS plus the literal FEC
// command overhead per sid that budget_base()/budget_base_for()/
// budget_enh_for() derive from directly (budget = overhead / (1 +
// overhead)). Same-rate-fixed-pairs (Task 4): overhead_base/overhead_enh
// are scored independently now -- sid 0 (base) against overhead_base, sid 1
// (enh/probe/s3) against overhead_enh -- carried alongside the RCF wire and
// ProfileRow pairs from Tasks 1-2.
struct Rung {
  int mcs = 0;
  double overhead_base = 1.0;
  double overhead_enh = 1.0;
};

// --- fade-aware demotes (spec 2026-08-14 fade-demote) ---
struct FadeCfg {
  bool cascade = true;   // regime cascade (Part A)
  bool predict = true;   // predictive RSSI+SNR trigger (Part B)
  int hold_ms = 2500;    // regime duration after a loss-driven demote
  int confirm_ms = 100;  // in-regime replacement for confirm_ms (s1 util)
                          // and the s3 util confirm
  // Trigger thresholds on delta = slow - fast (see FadeEwma below). These are
  // thresholds on a HIGH-PASS RESPONSE, not on the fade depth, so they are
  // NOT the trip points — read them with the transfer function:
  //
  //   step of depth D dB  ->  delta(t) = D * (e^(-t/20000) - e^(-t/300))
  //
  // which peaks at 0.924*D at t = 1.28 s and has decayed to 0.74*D by 6 s.
  // With the trigger_ms sustain on top, the smallest step that fires is
  // ~1.08x the configured threshold: `rssi_db: 8.0` trips on a ~8.7 dB step,
  // `snr_db: 4.0` on a ~4.3 dB one, and a fade of exactly the configured
  // depth never fires at all. A fade that stops going down goes back under
  // threshold as the 20 s baseline catches up, so this detects fading, not
  // faded. On a steady ramp the response is slope-driven — delta settles at
  // ~19.7 dB per dB/s — so ramps slower than ~0.45 dB/s never reach
  // rssi_db 8 however far they eventually fall. (Harness/model figures,
  // matching the whole-branch review's measured 6 s deltas exactly; NOT
  // flight results.) Note also that trigger_ms is not what sets the reaction
  // time: the delta needs ~0.8-1.3 s to climb to its peak, so the 300 ms fast
  // tau dominates. Do not tighten these without bench data (spec tuning
  // invariant) — the whole-branch review deliberately left them alone.
  double rssi_db = 8.0;  // baseline-minus-fast RSSI drop to trigger
  double snr_db = 4.0;   // baseline-minus-fast SNR drop to trigger
  int trigger_ms = 300;  // sustain requirement
  int min_rung = 2;      // no predictive fires below this rung
};

// --- always-on probe stream (spec 2026-09-04) ---
// The drone pushes a small canary body at the rung `current + rung_offset`
// after every video AU (base and enh since 2026-09-16; enh-only before);
// the GS scores its loss continuously and the promote trigger consults the
// verdict. No state machine: nothing starts, nothing aborts, and a demote
// never has to cancel anything.
//
// clean_bodies: the Clean streak the promote needs, in expected probe
// BODIES, not ms. The gate is a zero-loss run test whose confidence is
// (1-p)^n over the n bodies in the streak, so a duration would mean a
// different confidence at every frame rate and layer split (60 fps SVC-T
// = 60 probes/s, 30 fps = 30/s, enh shed = base only). 90 bodies rejects
// ~3.3 % PPDU loss at 95 % (rule of three) and takes 1.5 s at 60/s.
struct ProbeCfg {
  bool enable = true;
  int rung_offset = 1;
  int clean_bodies = 90;
  double max_util = -1.0;   // <0 => down_util
  // Availability floor: expected probe symbols the 500 ms window must hold
  // before its verdict counts (below it the gate reads NoInfo, which HOLDS
  // the promote). 16 = four AUs' worth at bpb 4 -- the true "no traffic"
  // floor (shed, a stalled encoder). It has to sit under the low-power
  // stream's rate: 15 fps x 4 x 0.5 s = 30 per window (bench 2026-09-20 read
  // 28-32), and the original 40 -- sized against 60 fps = 120 -- held every
  // pre-arm promote for the whole disarmed period. Keep it under
  // low_power.fps x 2 (docs/link-adaptation.md, "Low-power (disarmed)").
  int min_syms = 16;
  int silence_ms = 500;
  int pin_mcs = -1;         // consumed by VrxController only
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

  // --- always-on probe stream (see ProbeCfg above) ---
  ProbeCfg probe;

  // --- s3 steady-state demotes (consumed by the s3-demote logic) ---
  bool s3_demote = true;
  double s3_down_util = -1.0;   // <0 => down_util
  // An s3 window below this many expected symbols is "no traffic" and carries
  // no information. The flat key that used to carry this floor went with the
  // s3 probe (spec 2026-09-04); link.s3_min_syms replaces it.
  int s3_min_syms = 50;
  // After any rung transition the drone re-keys its FEC streams; blank
  // s3-derived decisions for this long so the re-key glitch does not read as
  // loss.
  int s3_settle_ms = 300;

  // --- per-rung EWMA store (spec 2026-08-13, observe-only) ---
  RungStoreCfg rung_stats;

  // --- fade-aware demotes (spec 2026-08-14): cascade/hold_ms/confirm_ms
  // (Part A) and the predictive trigger (Part B) are all live. ---
  FadeCfg fade;
};

// One feedback sample: measured pre-FEC and residual (post-FEC) loss for the
// current rung's s1 stream, over the loss window the caller maintains.
// Fields are APPEND-ONLY with defaults: existing positional brace-inits
// (LinkHealth{true, 0.0, 0.0, false} in main.cpp / test_vrx_controller.cpp)
// must keep compiling untouched. A caller that fills only the s1 fields no
// longer gets the legacy direct promote, though: with ProbeCfg::enable on
// (the default) the absent probe window reads as NoInfo and HOLDS every
// promote — that is the point of the gate. Disable the probe to get the
// legacy behaviour back.
struct LinkHealth {
  bool sample_valid = false;  // false when the s1 window saw 0 expected symbols
  double pre_fec_loss = 0.0;  // s1 missing/expected over the loss window
  double residual_loss = 0.0;
  bool video_starved = false;
  bool s3_valid = false;          // s3 loss window had traffic
  double s3_pre_fec_loss = 0.0;   // s3 missing/expected over the window
  double s3_residual_loss = 0.0;  // abandoned/expected over the window
  uint64_t s3_expected_syms = 0;  // expected s3 symbols in the window
  // Carried onto CtlEvent/ProbeEdge so the ctl log can say what the RF
  // looked like when a decision fired, AND — since the Part B predictive fade
  // trigger (spec 2026-08-14) — a decision input in its own right. NaN is a
  // legal value (no SNR known this window) and leaves the trigger inert.
  double rf_snr_db = std::numeric_limits<double>::quiet_NaN();
  // Label only: the RF EVM (dB, s1+s3 pooled) of the card that supplied
  // rf_snr_db. NaN when unsampled. Deliberately NOT a decision input — raw
  // EVM is op-point-dependent (docs/evm-sweep-findings-2026-08-10.md).
  double rf_evm_db = std::numeric_limits<double>::quiet_NaN();
  // RF RSSI (dBm, s1+s3 pooled) of the same card, the second half of the
  // Part B fade trigger's joint condition. NaN = unsampled, which leaves it
  // inert.
  double rf_rssi_dbm = std::numeric_limits<double>::quiet_NaN();
  // --- probe stream window (spec 2026-09-04), from ProbeTrack ---
  bool probe_valid = false;      // probe window had a sample
  double probe_loss = 0.0;       // union block loss over the window
  uint64_t probe_expected_syms = 0;
  // Cumulative expected probe bodies (ProbeTrack union expected_blocks /
  // bpb), monotonic: the Clean streak is counted as bodies since the
  // streak's first Clean sample (ProbeCfg::clean_bodies).
  uint64_t probe_bodies_total = 0;
  int probe_rung = -1;           // rung the sample was commanded at
};

enum class CtlReason {
  None, Residual, Util, Probation, Starved, Timeout, Promote,
  S3Residual, S3Util, Fade, PromoteProbed, HopRestore
};
const char* to_string(CtlReason r);

// Continuous verdict of the probe stream. Off = no probe is commanded (the
// feature is disabled, or the link is already on the top rung); NoInfo = one
// IS commanded but nothing usable has been measured recently, which holds a
// promote exactly as a Lossy verdict does — a commanded-but-absent probe is
// the blind promote the stream exists to prevent.
enum class ProbeGateState { Off, NoInfo, Clean, Lossy };
const char* to_string(ProbeGateState s);  // "off" "noinfo" "clean" "lossy"

struct ProbeGate {
  ProbeGateState state = ProbeGateState::Off;
  int rung = -1;         // the rung being probed; -1 when none is
  double u = 0.0;        // last scored probe utilization
  // Expected probe bodies booked since the state turned Clean; 0 otherwise.
  // The promote needs streak_bodies >= ProbeCfg::clean_bodies.
  uint64_t streak_bodies = 0;
};

// One state change of the gate, for the ctl log / sideport. prev_dur_ms is
// how long the state it replaced had stood.
struct ProbeEdge {
  double t_ms = 0;
  int rung = -1;
  ProbeGateState state = ProbeGateState::Off;
  double u = 0.0;
  double snr_db = std::numeric_limits<double>::quiet_NaN();
  double evm_db = std::numeric_limits<double>::quiet_NaN();
  double prev_dur_ms = 0;
};

// Stamped by penalize_rung() so the ctl log can report the escalating ledger
// without reaching into penalized().
struct PenaltyEvent {
  double t_ms = 0;
  int rung = 0;
  int k = 0;  // consecutive-failure count that set this penalty
  double until_ms = 0;
};

struct CtlEvent {
  double t_ms = 0;
  int from = 0, to = 0;
  CtlReason reason = CtlReason::None;
  double u = 0;
  double snr_db = std::numeric_limits<double>::quiet_NaN();
  double evm_db = std::numeric_limits<double>::quiet_NaN();
};

struct CtlCounters {
  uint64_t demotes_residual = 0, demotes_util = 0, promotes = 0,
           probation_fails = 0, starved_drops = 0, timeout_drops = 0;
  // Promotes that the probe gate cleared, and hold EPISODES (one count per
  // continuous run of held promotes, not one per sample).
  uint64_t promotes_probed = 0, probe_holds = 0;
  uint64_t demotes_s3_residual = 0, demotes_s3_util = 0;
  uint64_t demotes_fade = 0;
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

  // In-flight channel hop (spec 2026-09-14 §4): direct re-entry into `rung`
  // with no probe-before-promote and no probation, because the evidence that
  // justified that rung is still valid -- only the channel changed. `rung`
  // is clamped into the valid ladder range: the caller (VrxController, via
  // Task 11's HopController) may hand back a rung snapshotted before the
  // hop, and a stale or out-of-range index must not index out of bounds.
  // Logs a HopRestore ctl event.
  void restore(int rung, double now_ms);

  // Stop feeding the per-rung EWMA store (observe_s1/observe_evm/observe_s3/
  // observe_probe) until `until_ms`: an interferer's demoted operating point
  // is real RF evidence for the CHANNEL that just got abandoned, not for
  // what the rung can do in general, and must not poison the learned
  // per-rung statistics. Takes the MAXIMUM of the existing and new deadline
  // so overlapping blanks never shorten an existing one.
  void blank_store(double until_ms);

  int rung() const { return idx_; }
  // The rung the last VALID feedback sample was measured on, stamped before
  // any decision block runs. rung() is live and has already stepped down by
  // the time a demote returns, so anything that pairs a rung with the loss
  // numbers from that window (the ctl log's S line) must use this instead —
  // otherwise the loss is filed against the rung the link demoted TO, and the
  // rung that actually caused it never appears. Measured on flights
  // 2026-08-15: 15/16 and 13/13 post-FEC loss samples landed within 200 ms of
  // a demote.
  int measured_rung() const { return measured_rung_; }
  const Rung& op() const { return cfg_.ladder[static_cast<std::size_t>(idx_)]; }

  double util() const { return u_; }              // last computed u (0 before first valid sample)
  double pre_fec_loss() const { return pre_fec_loss_; }

  // Budget of the CURRENT rung's BASE (sid 0) overhead: overhead_base /
  // (1 + overhead_base), the literal FEC command overhead — no per-layer
  // scaling since the flatten. Used to score sid 0's util (update()'s
  // u_ = pre_fec_loss / budget_base()).
  double budget_base() const;
  // Budget of an ARBITRARY rung's BASE overhead (same formula as
  // budget_base(), any index) — public so promote candidate math and tests
  // can score a rung the controller isn't currently on.
  double budget_base_for(int rung) const;
  // Budget of an ARBITRARY rung's ENH overhead (same formula, the sid 1 /
  // probe / s3 side of the pair) — used to score enh/probe/s3 quantities,
  // which since same-rate-fixed-pairs run on their own literal overhead
  // rather than mirroring sid 0's.
  double budget_enh_for(int rung) const;

  int probation_ms_left(double now_ms) const;  // 0 when not probing
  std::vector<std::pair<int, int>> penalized(double now_ms) const;  // {rung, ms_left}

  // --- probe stream view ---
  // The rung the drone must run the probe stream at: min(rung + offset, top),
  // or -1 when nothing is to be probed (feature disabled, or already on the
  // top rung). Pure function of the current rung — there is no probe state.
  int probe_rung() const;
  // The gate's current verdict; consulted by the promote trigger and
  // exported. streak_bodies counts up to the last update()'s sample.
  ProbeGate probe_gate(double now_ms) const;
  // The last gate state CHANGE (ctl log / sideport). Off with rung -1 before
  // any change has happened.
  const ProbeEdge& last_probe_edge() const { return last_probe_edge_; }

  // Steady-state s3 utilization against the CURRENT rung's s3 budget. 0
  // whenever the last sample could not measure it: inside the post-transition
  // blanking window, or when s3 carried no usable traffic. Never a persisted
  // stale value.
  double util3() const { return u3_; }

  // Observe-only per-rung statistics (spec 2026-08-13). NEVER read by any
  // decision path in this class — exporter/ctl-log surface only.
  const RungStore& rungs() const { return store_; }

  // Raw fade-regime state (spec 2026-08-14): true while the post-demote
  // regime window is open. Deliberately NOT gated on cfg_.fade.cascade — the
  // regime is armed either way so the exported label stays truthful about
  // what the link is doing even when the cascade effect is killed.
  bool fade_active(double now_ms) const { return now_ms < fade_until_ms_; }

  // Fade-trigger deltas (baseline - fast; the threshold-tuning surface for
  // the sideport). NaN until the corresponding signal has ever been sampled.
  double fade_drssi() const { return fade_rssi_.delta(); }
  double fade_dsnr() const { return fade_snr_.delta(); }

  const CtlCounters& counters() const { return counters_; }
  const CtlEvent& last_event() const { return last_event_; }
  const PenaltyEvent& last_penalty() const { return last_penalty_; }

 private:
  void reset_windows();
  void check_probation_survival(double now_ms);
  void penalize_rung(int rung, double now_ms);
  bool is_penalized(int rung, double now_ms) const;
  void set_event(double now_ms, int from, int to, CtlReason reason, double u,
                 double snr);

  double probe_util_threshold() const;
  double s3_util_threshold() const;
  bool s3_usable(const LinkHealth& h) const;

  // --- fade regime (spec 2026-08-14 Part A) ---
  // Whether the cascade EFFECT applies right now: the regime is open and the
  // kill switch is on.
  bool in_fade_regime(double now_ms) const {
    return cfg_.fade.cascade && now_ms < fade_until_ms_;
  }
  // In-regime replacement for the confirmed-demote window on the s1 util
  // path. The instant s1-residual path, the instant s3-residual path, the
  // s3_settle_ms blanking and min_between_changes_ms are deliberately
  // untouched.
  //
  // The s1 util path demotes on an AMPLITUDE threshold (u > down_util, i.e.
  // >10% of raw symbols missing over the loss window) and has no blanking at
  // all, so debris either clears that bar — in which case the legacy 250 ms
  // confirm, which sits entirely inside the 500 ms window the debris
  // occupies, fires too — or it does not, and neither window fires.
  // Shortening opens no new duration band there.
  double eff_confirm_ms(double now_ms) const {
    return in_fade_regime(now_ms) ? cfg_.fade.confirm_ms : cfg_.confirm_ms;
  }
  // The s3 util confirm shortens inside the fade regime. The link.attrib
  // guard this used to carry went away with the switch on 2026-08-15:
  // attribution is unconditional, so post-transition debris is never in the
  // input this reads.
  double eff_s3_util_confirm_ms(double now_ms) const {
    return in_fade_regime(now_ms) ? cfg_.fade.confirm_ms : cfg_.confirm_ms;
  }

  // --- Part B predictive fade trigger (spec 2026-08-14 §3) ---
  // Dual-timescale EWMA per RF signal. Time-constant form (per-sample alpha
  // from dt) so 50 ms and 100 ms feedback configs behave identically. The
  // baseline is asymmetric: rises fast (tau 2 s), falls slowly (tau 20 s) —
  // a 3 s fade must not drag its own baseline down and erase the delta.
  // Taus are structural constants, not config.
  struct FadeEwma {
    double fast = 0.0, slow = 0.0, last_ms = 0.0;
    bool has = false;
    void feed(double v, double now_ms) {
      if (!has) { fast = slow = v; last_ms = now_ms; has = true; return; }
      const double dt = std::max(1.0, now_ms - last_ms);
      last_ms = now_ms;
      fast += (1.0 - std::exp(-dt / 300.0)) * (v - fast);
      const double tau = v >= slow ? 2000.0 : 20000.0;
      slow += (1.0 - std::exp(-dt / tau)) * (v - slow);
    }
    double delta() const {  // baseline - current level; NaN before first sample
      return has ? slow - fast : std::numeric_limits<double>::quiet_NaN();
    }
  };

  // Score this sample's probe window and move the gate (spec 2026-09-04
  // §4.3). Runs on every valid sample, before any decision block.
  void update_probe_gate(const LinkHealth& h, double now_ms);
  // Move the gate to `s`, logging an edge when it actually changes.
  void set_probe_state(ProbeGateState s, int rung, double u, double now_ms);
  // Every rung change: blank s3-derived decisions over the drone's FEC
  // re-key, drop any half-accumulated s3 window, and reset the probe gate —
  // the new rung's probe has measured nothing yet.
  void mark_transition(double now_ms);

  LadderCfg cfg_;
  RungStore store_{1, RungStoreCfg{}};  // re-initialized in the constructor
  int idx_ = 0;

  double u_ = 0.0;
  double pre_fec_loss_ = 0.0;

  double last_feedback_ms_ = -1e18;
  // See measured_rung(). Stamped on every valid sample, before any decision.
  int measured_rung_ = 0;
  double starved_since_ms_ = -1.0;  // <0 = not currently in a starved run
  double last_change_ms_ = -1e18;
  double last_down_ms_ = -1e18;

  double confirm_start_ms_ = -1.0;  // -1 = no active over-down_util window
  double clean_start_ms_ = -1.0;    // -1 = no active under-up_util window

  // Fade regime expiry: armed to now + fade.hold_ms by every loss-driven
  // demote (residual / util / s3_residual / s3_util) and by the predictive
  // fade demote itself. -1e18 = never armed.
  double fade_until_ms_ = -1e18;

  FadeEwma fade_rssi_, fade_snr_;
  double fade_trig_start_ms_ = -1.0;  // -1 = no sustained run
  // One predictive fire per fade EVENT. The slow baseline falls at tau 20 s,
  // so delta() stays over threshold for many seconds after a fade demote and
  // the sustain run would otherwise re-accrue every trigger_ms (300) —
  // min_between_changes_ms (150) is no spacing gate. Cleared ONLY by an
  // observed recovery — a measurable tick with both deltas back under
  // threshold — never by a NaN tick, which says nothing either way.
  bool fade_latched_ = false;

  bool probation_active_ = false;
  double probation_until_ms_ = -1e18;
  int probation_rung_ = -1;

  std::vector<int> fail_count_;        // per-rung consecutive probation-fail count k
  std::vector<double> penalty_until_;  // per-rung penalty expiry (ms); -1e18 = none

  // --- probe gate state (spec 2026-09-04) ---
  ProbeGateState probe_state_ = ProbeGateState::Off;
  double probe_state_since_ms_ = 0.0;
  // Clean streak in bodies: probe_bodies_total at the streak's first Clean
  // sample (probe_streak_open_ false = no streak) and the latest total.
  bool probe_streak_open_ = false;
  uint64_t probe_streak_start_bodies_ = 0;
  uint64_t probe_bodies_total_ = 0;
  double probe_last_sample_ms_ = -1e18;
  double probe_u_ = 0.0;
  bool probe_hold_active_ = false;       // one probe_holds count per hold episode
  ProbeEdge last_probe_edge_;

  double u3_ = 0.0;
  double s3_util_start_ms_ = -1.0;
  // Last sample that could actually measure s3. The confirm window above is
  // an elapsed-time test against a stamp, so it only means "sustained" while
  // measurement is CONTINUOUS: a gap invalidates the run (see update()).
  double s3_last_live_ms_ = -1e18;
  double s3_blank_until_ms_ = -1e18;
  // See blank_store(): -1e18 = nothing blanked.
  double blank_store_until_ms_ = -1e18;
  double snr_now_ = std::numeric_limits<double>::quiet_NaN();
  double evm_now_ = std::numeric_limits<double>::quiet_NaN();

  CtlCounters counters_;
  CtlEvent last_event_;
  PenaltyEvent last_penalty_;
};

}  // namespace maburgs
