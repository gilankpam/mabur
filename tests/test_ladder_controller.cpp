#include <cmath>

#include "ladder_controller.h"
#include "mtest.h"

using namespace maburgs;

namespace {

// Ladder from the task brief: {mcs, overhead} per rung, [0] = failsafe.
// Overhead values are 2x the pre-2026-08-29-airtime-balance-uep fixture:
// budget() used to scale via uep_layer_overhead(1, ov) = 2*ov (post-flatten
// kUepRefOverhead=0.5, cmd_overhead/0.25 baseline), and now reads the
// literal overhead directly -- doubling every value here keeps every
// numeric expectation in this file unchanged.
// same-rate-fixed-pairs (Task 4): overhead_enh is set explicitly here (equal
// to overhead_base) rather than left at the struct default, since
// budget_enh_for()/s3-probe scoring now reads it directly.
const std::vector<Rung> kLadder = {{0, 2.0, 2.0}, {2, 1.0, 1.0}, {4, 0.5, 0.5},
                                    {5, 0.5, 0.5}, {6, 0.3, 0.3}, {7, 0.2, 0.2}};

LadderCfg make_cfg() {
  LadderCfg cfg;
  cfg.ladder = kLadder;
  return cfg;
}

// Same ladder with the probe stream OFF. Every legacy-behaviour test below
// (probation, penalties, fade, s3 demotes, timeouts) promotes on ok(0.0)
// samples, which carry no probe window: with the probe ENABLED the gate
// reads NoInfo and HOLDS those promotes forever, so the legacy semantics are
// pinned with the gate disabled and the gate itself is tested on its own.
LadderCfg make_cfg_noprobe() {
  LadderCfg cfg = make_cfg();
  cfg.probe.enable = false;
  return cfg;
}

LinkHealth ok(double pre) {
  LinkHealth h;
  h.sample_valid = true;
  h.pre_fec_loss = pre;
  h.residual_loss = 0.0;
  h.video_starved = false;
  return h;
}

// Drives ok(0.0) samples at a 50 ms cadence until the controller reaches
// `target` (or higher, which should not happen for well-formed tests).
void promote_to(LadderController& ctl, double& t, int target) {
  while (ctl.rung() < target) {
    ctl.update(ok(0.0), t);
    t += 50;
    REQUIRE(t < 1e6);
  }
}

// Feeds neutral-utilization samples (u == util_fraction, comfortably between
// up_util and down_util) at a 50 ms cadence for at least dt_total ms. Used to
// run the clock past a rung's probation window without the sample itself
// causing any promote/demote.
void feed_for(LadderController& ctl, double& t, double dt_total,
              double util_fraction) {
  const double end = t + dt_total;
  for (; t <= end; t += 50) {
    ctl.update(ok(util_fraction * ctl.budget_base()), t);
  }
}

// Healthy base sample carrying a usable probe window measured at `rung`
// with union block loss `loss` (and usable s3 for the steady-state paths).
LinkHealth okp(double pre, int probe_rung, double loss) {
  LinkHealth h = ok(pre);
  h.s3_valid = true; h.s3_expected_syms = 500;
  h.probe_valid = true; h.probe_loss = loss;
  h.probe_expected_syms = 60; h.probe_rung = probe_rung;
  h.rf_snr_db = 30.0;
  return h;
}

// Healthy sample WITH usable s3 but NO probe window: the gate reads NoInfo
// while a probe is commanded, so this is the steady-state s3 fixture only.
LinkHealth ok3(double pre, double s3_pre = 0.0, double s3_resid = 0.0) {
  LinkHealth h = ok(pre);
  h.s3_valid = true;
  h.s3_pre_fec_loss = s3_pre;
  h.s3_residual_loss = s3_resid;
  h.s3_expected_syms = 500;
  h.rf_snr_db = 30.0;
  return h;
}

// Healthy sample with RF labels for the fade trigger.
LinkHealth rf(double pre, double snr_db, double rssi_dbm) {
  LinkHealth h = ok(pre);
  h.rf_snr_db = snr_db;
  h.rf_rssi_dbm = rssi_dbm;
  return h;
}

// Feeds dt_total ms of steady RF at a 50 ms cadence so the fade EWMAs settle
// on a baseline. Utilization is neutral (0.3 — above up_util, below down_util)
// so the window itself provokes neither a promote nor a demote: a baseline fed
// at u == 0 accrues a clean window and promotes out from under the test
// clean_ms after it starts. Feed at least probation_ms to also retire the
// preceding promote's probation.
void fade_baseline(LadderController& ctl, double& t, double dt_total,
                   double snr_db, double rssi_dbm) {
  const double end = t + dt_total;
  for (; t < end; t += 50) {
    ctl.update(rf(0.3 * ctl.budget_base(), snr_db, rssi_dbm), t);
  }
}

int penalty_ms_for(const LadderController& ctl, double now_ms, int rung) {
  for (const auto& pr : ctl.penalized(now_ms)) {
    if (pr.first == rung) return pr.second;
  }
  return -1;
}

}  // namespace

TEST(budget_is_literal_overhead_fraction) {
  // rung overhead_base 0.3 -> budget 0.3/1.3: budget_base_for() is the
  // rung's literal FEC command overhead directly since
  // task-10-airtime-balance-uep deleted the uep_layer_overhead scaling call
  // (a no-op since the 2026-08-29 flatten).
  LadderCfg cfg;
  cfg.ladder = {{7, 0.3, 0.3}};
  LadderController c(cfg);
  CHECK(std::abs(c.budget_base_for(0) - 0.3 / 1.3) < 1e-9);
}

// same-rate-fixed-pairs (Task 4): budget_base()/budget_enh_for() score each
// sid off its OWN overhead now, not a shared value.
TEST(per_sid_budgets) {
  LadderCfg cfg;
  cfg.ladder = {{1, 1.0, 0.5}};
  LadderController ctl(cfg);
  CHECK(std::abs(ctl.budget_base() - 1.0 / 2.0) < 1e-9);
  CHECK(std::abs(ctl.budget_enh_for(0) - 0.5 / 1.5) < 1e-9);
}

TEST(budget_uses_literal_overhead) {
  LadderController ctl(make_cfg_noprobe());
  // rung 0: overhead 2.0 (doubled fixture, see kLadder) -> budget = 2.0/3.0.
  CHECK(std::abs(ctl.budget_base() - (2.0 / 3.0)) < 1e-9);

  double t = 0;
  promote_to(ctl, t, 5);  // walk all the way to the top rung {7, 0.2}
  CHECK(ctl.rung() == 5);
  // top rung: overhead 0.2 -> budget = 0.2/1.2.
  CHECK(std::abs(ctl.budget_base() - (0.2 / 1.2)) < 1e-9);
}

TEST(starts_at_failsafe) {
  LadderController ctl(make_cfg());
  CHECK(ctl.rung() == 0);
}

TEST(promote_needs_clean_window) {
  LadderController ctl(make_cfg_noprobe());
  bool promoted = false;
  double promote_t = -1;
  for (double t = 0; t <= 5000; t += 50) {
    const bool changed = ctl.update(ok(0.0), t);
    if (changed) {
      promoted = true;
      promote_t = t;
    } else {
      CHECK(ctl.rung() == 0);  // no promote before clean_ms
    }
  }
  CHECK(promoted);
  CHECK(promote_t >= 5000 - 1e-9);  // promote at/after 5000 ms
  CHECK(ctl.rung() == 1);
  CHECK(ctl.probation_ms_left(promote_t) > 0);
}

TEST(residual_demotes_immediately) {
  auto cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 2);
  REQUIRE(ctl.rung() == 2);

  // Drive time forward past this rung's probation with neutral health so the
  // upcoming residual demote below is NOT scored as a probation failure.
  feed_for(ctl, t, cfg.probation_ms + 200, 0.3);
  REQUIRE(ctl.probation_ms_left(t) == 0);

  LinkHealth bad = ok(0.0);
  bad.residual_loss = 0.01;
  CHECK(ctl.update(bad, t));
  CHECK(ctl.rung() == 1);
  CHECK(ctl.last_event().reason == CtlReason::Residual);
  CHECK(ctl.counters().demotes_residual == 1);

  // A second residual sample only 50 ms later demotes again -- residual
  // demotes are exempt from min_between_changes_ms (150 ms default).
  t += 50;
  CHECK(ctl.update(bad, t));
  CHECK(ctl.rung() == 0);
  CHECK(ctl.counters().demotes_residual == 2);
}

TEST(util_demote_needs_confirm) {
  auto cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 2);
  feed_for(ctl, t, cfg.probation_ms + 200, 0.3);  // clear rung 2's probation
  REQUIRE(ctl.probation_ms_left(t) == 0);
  REQUIRE(ctl.rung() == 2);

  const double b = ctl.budget_base();
  const LinkHealth high = ok(0.7 * b);
  const LinkHealth low = ok(0.0);

  // One interleaved low sample resets the confirm window: 150 ms high, one
  // low sample, another 150 ms high -- 300 ms of "high" samples in total,
  // well over confirm_ms(250), but none of it should demote because the low
  // sample in the middle restarts the window.
  double window_start = t;
  for (; t < window_start + 150; t += 50) CHECK(!ctl.update(high, t));
  CHECK(!ctl.update(low, t));
  t += 50;
  double window2_start = t;
  for (; t < window2_start + 150; t += 50) CHECK(!ctl.update(high, t));
  CHECK(ctl.rung() == 2);

  // Reset once more to a clean slate, then check the precise confirm_ms
  // boundary: no change while sustained above down_util for 200 ms, demote
  // once it has been continuously above for 250 ms.
  CHECK(!ctl.update(low, t));
  t += 50;
  const double confirm_start = t;
  for (; t <= confirm_start + 200; t += 50) CHECK(!ctl.update(high, t));
  CHECK(ctl.rung() == 2);

  bool changed = false;
  for (; !changed; t += 50) {
    changed = ctl.update(high, t);
    REQUIRE(t < confirm_start + 1000);  // safety cap
  }
  CHECK(t - confirm_start >= cfg.confirm_ms - 1e-9);
  CHECK(ctl.rung() == 1);
  CHECK(ctl.last_event().reason == CtlReason::Util);
  CHECK(ctl.counters().demotes_util == 1);
}

TEST(probation_fail_penalizes_and_doubles) {
  auto cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;

  // --- 1st promotion into rung 1: fail probation immediately (u > down_util
  // while on probation demotes right away, reason Probation). ---
  promote_to(ctl, t, 1);
  REQUIRE(ctl.rung() == 1);
  REQUIRE(ctl.probation_ms_left(t) > 0);
  double fail1_t = t;
  CHECK(ctl.update(ok(0.7 * ctl.budget_base()), fail1_t));
  CHECK(ctl.rung() == 0);
  CHECK(ctl.last_event().reason == CtlReason::Probation);
  CHECK(ctl.counters().probation_fails == 1);
  CHECK(penalty_ms_for(ctl, fail1_t, 1) == cfg.penalty_base_ms);
  t = fail1_t + 50;

  // --- Clean-climb again: promotion into rung 1 is blocked until the
  // penalty above expires, even though the clean window itself is satisfied
  // long before that. ---
  promote_to(ctl, t, 1);
  REQUIRE(ctl.rung() == 1);
  CHECK(t >= fail1_t + cfg.penalty_base_ms);

  // --- Fail again during the 2nd probation: the penalty doubles. ---
  double fail2_t = t;
  CHECK(ctl.update(ok(0.7 * ctl.budget_base()), fail2_t));
  CHECK(ctl.rung() == 0);
  CHECK(ctl.counters().probation_fails == 2);
  CHECK(penalty_ms_for(ctl, fail2_t, 1) == 2 * cfg.penalty_base_ms);
  t = fail2_t + 50;

  // --- Clean-climb a third time (blocked until the doubled penalty
  // expires), then SURVIVE this probation: the failure count resets. ---
  promote_to(ctl, t, 1);
  REQUIRE(ctl.rung() == 1);
  CHECK(t >= fail2_t + 2 * cfg.penalty_base_ms);
  feed_for(ctl, t, cfg.probation_ms + 200, 0.0);
  REQUIRE(ctl.rung() == 1);  // survived: no demote, no further climb
  REQUIRE(ctl.probation_ms_left(t) == 0);

  // Leave rung 1 without touching probation bookkeeping (already expired),
  // climb back a 4th time, and fail immediately: if the survival above truly
  // reset the failure count, this costs the BASE penalty again rather than
  // doubling further.
  LinkHealth residual = ok(0.0);
  residual.residual_loss = 0.01;
  ctl.update(residual, t);
  REQUIRE(ctl.rung() == 0);
  t += 50;
  promote_to(ctl, t, 1);
  REQUIRE(ctl.rung() == 1);
  double fail3_t = t;
  CHECK(ctl.update(ok(0.7 * ctl.budget_base()), fail3_t));
  CHECK(ctl.rung() == 0);
  CHECK(penalty_ms_for(ctl, fail3_t, 1) == cfg.penalty_base_ms);
}

// Regression: fail_count_[rung] is only reset by surviving probation, so a
// persistently marginal link that keeps failing probation on the same rung
// accumulates k without bound. penalize_rung() computes
// penalty_base_ms << (k-1) before clamping to penalty_max_ms, so an
// uncapped k eventually shifts a (long long) by >= its bit width, which is
// UB regardless of the later std::min saturation. Drive many consecutive
// probation failures on rung 1 (well past 64) and confirm the penalty
// duration stays exactly capped at penalty_max_ms with no crash.
TEST(penalty_duration_caps_after_many_consecutive_failures) {
  auto cfg = make_cfg_noprobe();
  // Tiny timings so a fail-reclimb-fail cycle costs only a couple of 50ms
  // ticks; probation_ms must stay comfortably above one tick (50ms) so the
  // failing sample below still lands inside the just-opened probation
  // window instead of finding it already survived.
  cfg.clean_ms = 10;
  cfg.probation_ms = 100;
  cfg.hold_after_down_ms = 0;
  cfg.min_between_changes_ms = 0;
  cfg.penalty_base_ms = 1;
  cfg.penalty_max_ms = 4;
  LadderController ctl(cfg);
  double t = 0;
  constexpr int kFailures = 80;  // > 64: would UB-shift pre-fix
  double last_fail_t = 0;
  for (int i = 0; i < kFailures; ++i) {
    promote_to(ctl, t, 1);
    REQUIRE(ctl.rung() == 1);
    REQUIRE(ctl.probation_ms_left(t) > 0);
    last_fail_t = t;
    CHECK(ctl.update(ok(0.7 * ctl.budget_base()), last_fail_t));
    CHECK(ctl.rung() == 0);
    t = last_fail_t + 50;
  }
  CHECK(ctl.counters().probation_fails == static_cast<uint64_t>(kFailures));
  CHECK(penalty_ms_for(ctl, last_fail_t, 1) == cfg.penalty_max_ms);
}

TEST(starved_forces_bottom) {
  LadderController ctl(make_cfg_noprobe());
  double t = 0;
  promote_to(ctl, t, 3);
  REQUIRE(ctl.rung() == 3);

  LinkHealth starved;
  starved.sample_valid = true;
  starved.video_starved = true;
  // Debounce: samples inside starved_confirm_ms (300) withhold but do not
  // demote; the drop fires once the run has persisted past the window.
  const double t0 = t;
  bool dropped = false;
  while (t - t0 <= 400) {
    dropped = ctl.update(starved, t);
    if (dropped) break;
    CHECK(ctl.rung() == 3);  // still holding during the confirm window
    t += 50;
  }
  CHECK(dropped);
  CHECK(t - t0 >= 300);  // did not fire early
  CHECK(ctl.rung() == 0);
  CHECK(ctl.last_event().reason == CtlReason::Starved);
  CHECK(ctl.counters().starved_drops == 1);

  // Invalid samples afterward change nothing.
  LinkHealth invalid;
  invalid.sample_valid = false;
  for (int i = 0; i < 5; ++i) {
    t += 50;
    CHECK(!ctl.update(invalid, t));
  }
  CHECK(ctl.rung() == 0);
}

TEST(transient_starved_is_ignored) {
  // hw 2026-07-27: a rung transition re-keys the drone FEC stream and yields
  // 1-2 zero-completion decode windows on a healthy link. Those must not
  // demote — only a starved run persisting past starved_confirm_ms may.
  LadderController ctl(make_cfg_noprobe());
  double t = 0;
  promote_to(ctl, t, 2);
  REQUIRE(ctl.rung() == 2);

  LinkHealth starved;
  starved.sample_valid = true;
  starved.video_starved = true;
  LinkHealth healthy;
  healthy.sample_valid = true;
  healthy.pre_fec_loss = 0.01;

  for (int burst = 0; burst < 3; ++burst) {
    // Two starved windows (100 ms) — the op-switch glitch shape.
    for (int i = 0; i < 2; ++i) {
      CHECK(!ctl.update(starved, t));
      t += 50;
    }
    // Healthy samples resume; the starved run must reset.
    for (int i = 0; i < 10; ++i) {
      ctl.update(healthy, t);
      t += 50;
    }
  }
  CHECK(ctl.rung() == 2);
  CHECK(ctl.counters().starved_drops == 0);
}

TEST(timeout_forces_bottom) {
  auto cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 2);
  REQUIRE(ctl.rung() == 2);

  CHECK(ctl.on_tick(t + cfg.feedback_timeout_ms + 1));
  CHECK(ctl.rung() == 0);
  CHECK(ctl.last_event().reason == CtlReason::Timeout);
  CHECK(ctl.counters().timeout_drops == 1);

  CHECK(!ctl.on_tick(t + cfg.feedback_timeout_ms + 5000));
  CHECK(ctl.counters().timeout_drops == 1);
}

// Regression (reviewer finding 2026-07-27): update() used to stamp
// last_feedback_ms_ unconditionally, even on an invalid (sample_valid=false)
// sample. VrxController calls update() on every RCF slot while in SESSION,
// so a run where the S1 window never produces a valid sample (but video is
// still nominally arriving) stamped the feedback clock forever and
// permanently suppressed on_tick()'s blind-side timeout, holding an
// aggressive rung on zero real measurements. Confirm the timeout still
// fires off the last REAL sample despite a continuous stream of invalid
// updates in between.
TEST(invalid_samples_do_not_suppress_feedback_timeout) {
  auto cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 2);
  REQUIRE(ctl.rung() == 2);

  LinkHealth invalid;
  invalid.sample_valid = false;
  bool timed_out = false;
  for (int i = 0; i < 200 && !timed_out; ++i) {
    CHECK(!ctl.update(invalid, t));  // invalid sample: never a decision...
    timed_out = ctl.on_tick(t);      // ...and never resets the timeout clock.
    t += 50;
    REQUIRE(t < 1e6);
  }
  REQUIRE(timed_out);
  CHECK(ctl.rung() == 0);
  CHECK(ctl.last_event().reason == CtlReason::Timeout);
  CHECK(ctl.counters().timeout_drops == 1);
}

TEST(hold_after_downgrade) {
  auto cfg = make_cfg_noprobe();
  cfg.clean_ms = 100;  // isolate hold_after_down_ms as the binding gate
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 1);
  REQUIRE(ctl.rung() == 1);

  // Clear rung 1's probation with neutral health, so the demote below is a
  // genuine confirm-window Util demote rather than an immediate Probation
  // one.
  feed_for(ctl, t, cfg.probation_ms + 200, 0.3);
  REQUIRE(ctl.probation_ms_left(t) == 0);

  const double b = ctl.budget_base();
  bool demoted = false;
  for (int i = 0; i < 20 && !demoted; ++i) {
    demoted = ctl.update(ok(0.7 * b), t);
    t += 50;
  }
  REQUIRE(demoted);
  CHECK(ctl.last_event().reason == CtlReason::Util);
  CHECK(ctl.rung() == 0);
  const double down_t = ctl.last_event().t_ms;

  // Perfect health from here on: clean_ms(100) is satisfied almost
  // immediately, but promotion must not happen before down_t + 4000.
  for (; t < down_t + cfg.hold_after_down_ms; t += 50) {
    CHECK(!ctl.update(ok(0.0), t));
  }
  CHECK(ctl.rung() == 0);

  bool promoted = false;
  for (int i = 0; i < 20 && !promoted; ++i) {
    promoted = ctl.update(ok(0.0), t);
    t += 50;
  }
  CHECK(promoted);
  CHECK(ctl.rung() == 1);
  CHECK(ctl.last_event().t_ms >= down_t + cfg.hold_after_down_ms - 1e-9);
}

// --- continuous probe gate (spec 2026-09-04) -----------------------------

TEST(probe_rung_is_offset_clamped_to_top) {
  LadderCfg cfg = make_cfg();
  cfg.probe.rung_offset = 2;
  LadderController c(cfg);
  CHECK(c.probe_rung() == 2);
  double t = 0;
  cfg.probe.enable = false;  // climb without the gate, then re-check
  LadderController d(cfg);
  promote_to(d, t, 5);
  CHECK(d.probe_rung() == -1);          // disabled
  cfg.probe.enable = true;
  LadderController e(cfg);
  t = 0;
  // drive e to the top with clean probes
  while (e.rung() < 5) { e.update(okp(0.0, e.probe_rung(), 0.0), t); t += 50; REQUIRE(t < 1e6); }
  CHECK(e.probe_rung() == -1);          // top rung: nothing to probe
}

TEST(clean_probe_streak_gates_the_promote) {
  LadderCfg cfg = make_cfg();
  LadderController c(cfg);
  double t = 0;
  // Lossy probe: the base trigger is satisfied after clean_ms but the
  // promote holds, without a penalty.
  for (; t < cfg.clean_ms + 2000; t += 50) c.update(okp(0.0, c.probe_rung(), 0.9), t);
  CHECK(c.rung() == 0);
  CHECK(c.counters().probe_holds == 1);
  CHECK(c.penalized(t).empty());
  CHECK(c.probe_gate(t).state == ProbeGateState::Lossy);
  // Probe turns clean: promote lands once the streak reaches probe.clean_ms.
  const double t_clean = t;
  while (c.rung() == 0) { c.update(okp(0.0, c.probe_rung(), 0.0), t); t += 50; REQUIRE(t < t_clean + 5000); }
  CHECK(c.rung() == 1);
  CHECK(t - t_clean >= cfg.probe.clean_ms);
  CHECK(t - t_clean < cfg.probe.clean_ms + 200);
  CHECK(c.counters().promotes_probed == 1);
  CHECK(c.last_event().reason == CtlReason::PromoteProbed);
  CHECK(c.probation_ms_left(t) > 0);
}

TEST(probe_scores_against_the_candidates_enh_budget) {
  LadderCfg cfg = make_cfg();  // rung1 ov_enh 1.0 -> budget 0.5; rung2 0.5 -> 0.333
  LadderController c(cfg);
  double t = 0;
  c.update(okp(0.0, c.probe_rung(), 0.20), t);  // 0.20/0.5 = 0.4 < down_util 0.6
  CHECK(c.probe_gate(t).state == ProbeGateState::Clean);
  CHECK(std::abs(c.probe_gate(t).u - 0.4) < 1e-9);
  c.update(okp(0.0, c.probe_rung(), 0.35), t += 50);  // 0.7 > 0.6
  CHECK(c.probe_gate(t).state == ProbeGateState::Lossy);
}

TEST(noinfo_holds_while_a_probe_is_commanded) {
  LadderCfg cfg = make_cfg();
  LadderController c(cfg);
  double t = 0;
  for (; t < cfg.clean_ms + 3000; t += 50) c.update(ok3(0.0), t);  // no probe window
  CHECK(c.rung() == 0);
  CHECK(c.probe_gate(t).state == ProbeGateState::NoInfo);
  CHECK(c.counters().probe_holds == 1);
}

TEST(probe_disabled_falls_back_to_legacy_direct_promote) {
  LadderCfg cfg = make_cfg();
  cfg.probe.enable = false;
  LadderController c(cfg);
  double t = 0;
  promote_to(c, t, 1);
  CHECK(c.rung() == 1);
  CHECK(c.last_event().reason == CtlReason::Promote);
  CHECK(c.probe_gate(t).state == ProbeGateState::Off);
}

TEST(probe_silence_decays_to_noinfo_and_resets_the_streak) {
  LadderCfg cfg = make_cfg();
  LadderController c(cfg);
  double t = 0;
  for (; t < 1000; t += 50) c.update(okp(0.0, c.probe_rung(), 0.0), t);
  CHECK(c.probe_gate(t).state == ProbeGateState::Clean);
  for (; t < 1000 + cfg.probe.silence_ms + 100; t += 50) c.update(ok3(0.0), t);
  CHECK(c.probe_gate(t).state == ProbeGateState::NoInfo);
  CHECK(c.probe_gate(t).streak_ms == 0.0);
}

TEST(rung_change_resets_the_probe_streak) {
  LadderCfg cfg = make_cfg();
  LadderController c(cfg);
  double t = 0;
  while (c.rung() == 0) { c.update(okp(0.0, c.probe_rung(), 0.0), t); t += 50; REQUIRE(t < 1e6); }
  // Right after the promote the new rung's probe has no streak yet.
  CHECK(c.probe_gate(t).streak_ms < 100.0);
}

TEST(probe_edges_are_recorded_with_labels) {
  LadderCfg cfg = make_cfg();
  LadderController c(cfg);
  double t = 0;
  c.update(okp(0.0, c.probe_rung(), 0.0), t);
  CHECK(c.last_probe_edge().state == ProbeGateState::Clean);
  CHECK(c.last_probe_edge().rung == 1);
  for (int i = 0; i < 10; ++i) c.update(okp(0.0, c.probe_rung(), 0.0), t += 50);
  c.update(okp(0.0, c.probe_rung(), 0.9), t += 50);
  const ProbeEdge& e = c.last_probe_edge();
  CHECK(e.state == ProbeGateState::Lossy);
  CHECK(std::abs(e.prev_dur_ms - 550.0) < 1e-9);
  CHECK(std::abs(e.snr_db - 30.0) < 1e-9);
  CHECK(e.u > 1.0);
}

TEST(demotes_still_win_and_the_gate_follows_the_new_rung) {
  LadderCfg cfg = make_cfg();
  LadderController c(cfg);
  double t = 0;
  while (c.rung() < 2) { c.update(okp(0.0, c.probe_rung(), 0.0), t); t += 50; REQUIRE(t < 1e6); }
  feed_for(c, t, cfg.probation_ms + 100, 0.3);
  LinkHealth bad = okp(0.0, c.probe_rung(), 0.0);
  bad.residual_loss = 0.01;
  CHECK(c.update(bad, t));
  CHECK(c.rung() == 1);
  CHECK(c.probe_rung() == 2);
}

TEST(event_carries_snr) {
  LadderController ctl(make_cfg_noprobe());
  double t = 0;
  LinkHealth bad = ok3(0.0); bad.residual_loss = 0.05;
  promote_to(ctl, t, 1);
  ctl.update(bad, t);
  CHECK(ctl.last_event().reason == CtlReason::Residual);
  CHECK(ctl.last_event().snr_db == 30.0);
}

// --- s3 steady-state demotes ---------------------------------------------

// s3 residual demotes on the FIRST window now (spec 2026-08-15), matching
// the s1 residual path. What makes a 0 ms confirm safe is that attribution
// is EXACT, not fast: the watermark lives in symbol-sequence space
// (sw_decoder.h), so pre-transition debris is absent from the input rather
// than outrun by a shorter window.
TEST(s3_residual_demotes_on_the_first_window) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 4);
  feed_for(ctl, t, cfg.probation_ms + 200.0, 0.3);  // retire probation
  const int before = ctl.rung();
  const bool changed = ctl.update(ok3(0.3 * ctl.budget_base(), 0.0, 0.02), t);
  CHECK(changed);
  CHECK(ctl.rung() == before - 1);
  CHECK(ctl.counters().demotes_s3_residual == 1);
  CHECK(ctl.last_event().reason == CtlReason::S3Residual);
}

// The ctl log's S line pairs a rung with window-derived loss numbers, so the
// rung must be the one the loss was MEASURED on. rung() is the live value and
// has already stepped down by the time a demote returns, so a row built from
// it files the loss against the rung the link demoted TO. Measured on real
// flights 2026-08-15: 15/16 and 13/13 smear samples landed within 200 ms of a
// demote, which made the true culprit rung invisible in every per-rung table.
// measured_rung() is stamped before any decision block and is what the log
// must record.
TEST(measured_rung_is_the_rung_the_loss_was_measured_on) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 4);
  feed_for(ctl, t, cfg.probation_ms + 200.0, 0.3);  // retire probation
  const int before = ctl.rung();
  // A residual demote: the loss happened while the link was on `before`.
  LinkHealth h = ok(0.3 * ctl.budget_base());
  h.residual_loss = 0.02;
  REQUIRE(ctl.update(h, t));
  CHECK(ctl.rung() == before - 1);        // live rung has stepped down
  CHECK(ctl.measured_rung() == before);   // ...but the loss belongs to `before`
}

// On a tick with no transition the two must agree, or every quiet row in the
// log would disagree with itself.
TEST(measured_rung_equals_rung_when_no_demote) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 3);
  feed_for(ctl, t, cfg.probation_ms + 200.0, 0.3);
  REQUIRE(!ctl.update(ok(0.3 * ctl.budget_base()), t));
  CHECK(ctl.measured_rung() == ctl.rung());
}

TEST(s3_residual_demote_is_exempt_from_min_between_changes) {
  // s1's instant residual path is exempt from min_between_changes_ms; s3's
  // now is too. s3_settle_ms (300) is longer than min_between_changes_ms
  // (150), so the blank would mask the spacing gate entirely — zero it to
  // isolate what this test is actually about.
  LadderCfg cfg = make_cfg_noprobe();
  cfg.s3_settle_ms = 0;
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 4);
  feed_for(ctl, t, cfg.probation_ms + 200.0, 0.3);
  REQUIRE(ctl.probation_ms_left(t) == 0);
  const int before = ctl.rung();
  REQUIRE(ctl.update(ok3(0.3 * ctl.budget_base(), 0.0, 0.02), t));
  t += 50;  // well inside min_between_changes_ms (150)
  CHECK(ctl.update(ok3(0.3 * ctl.budget_base(), 0.0, 0.02), t));
  CHECK(ctl.rung() == before - 2);
}

// THE safety test (spec §8). With the confirm window gone, s3_settle_ms
// blanking is the only guard the CONTROLLER still owns between a rung
// transition and a cascade on that transition's own FEC re-key debris.
// Attribution removes the debris upstream in main.cpp and the controller
// cannot tell the difference, so this pins the half the controller can
// defend on its own.
TEST(instant_s3_residual_still_respects_the_settle_blank) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 4);
  feed_for(ctl, t, cfg.probation_ms + 200.0, 0.3);
  REQUIRE(ctl.probation_ms_left(t) == 0);
  // First demote opens the blank via mark_transition().
  REQUIRE(ctl.update(ok3(0.3 * ctl.budget_base(), 0.0, 0.02), t));
  const int after_first = ctl.rung();
  const double transition_t = t;
  // Debris keeps arriving for the whole blank. Nothing may fire.
  for (t += 50; t < transition_t + cfg.s3_settle_ms; t += 50)
    CHECK(!ctl.update(ok3(0.3 * ctl.budget_base(), 0.0, 0.02), t));
  CHECK(ctl.rung() == after_first);
}

TEST(s3_util_confirmed_demotes) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 2);
  t += cfg.probation_ms + cfg.s3_settle_ms + 200;
  const double start = t;
  // s3 pre-FEC loss high enough that u3 > down_util but no residual, s1 clean:
  for (; ctl.rung() == 2 && t - start < 2000; t += 50) ctl.update(ok3(0.0, 0.2), t);
  CHECK(ctl.rung() == 1);
  CHECK(ctl.counters().demotes_s3_util == 1);
  CHECK(ctl.last_event().reason == CtlReason::S3Util);
}

// Coverage regression (whole-branch review, I-3): the s3 residual path went
// instant on 2026-08-15 and took its two confirm-window gap tests with it
// (`s3_residual_confirm_window_resets_across_an_s3_gap` /
// `..._an_invalid_sample_gap`), but the continuity gate they drove
// (ladder_controller.cpp, just above the s3_live/!s3_live split) also guards
// the s3 UTIL confirm window, which is still elapsed-time and still live.
// Nothing left in the suite drove a real s3_valid=false or sample_valid=false
// gap through it. One bad window, ~800 ms of no measurement, one more bad
// window would satisfy "800 - 0 >= confirm_ms" and demote on two samples
// 800 ms apart -- exactly the two-sample demote the confirm window exists to
// prevent. Breaking the run on any discontinuity errs toward NOT demoting.
// (Verified: this variant, with sample_valid staying true through the gap,
// still passes if the continuity gate is deleted -- the redundant `!s3_live`
// reset a few lines below covers it. It is the invalid-sample sibling right
// below that actually pins the gate; kept here anyway as the s3-side
// documentation half of the same finding, mirroring the pair that was
// deleted.)
TEST(s3_util_confirm_window_resets_across_an_s3_gap) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 2);
  t += cfg.probation_ms + cfg.s3_settle_ms + 200;
  ctl.update(ok3(0.0, 0.2), t);   // one bad window stamps s3_util_start_ms_
  t += 50;

  LinkHealth gap = ok3(0.0, 0.2);  // s3 util pressure present but UNMEASURABLE
  gap.s3_valid = false;
  gap.s3_expected_syms = 0;
  const double gap_end = t + 800;
  for (; t < gap_end; t += 50) CHECK(!ctl.update(gap, t));

  // First usable sample after the gap: 800 ms of wall clock since the stamp,
  // but only ever one bad window of actual evidence.
  CHECK(!ctl.update(ok3(0.0, 0.2), t));
  CHECK(ctl.rung() == 2);
  CHECK(ctl.counters().demotes_s3_util == 0);

  // A genuinely sustained run still demotes, timed from the post-gap sample.
  const double fresh = t;
  t += 50;
  for (; ctl.rung() == 2 && t - fresh < 2000; t += 50)
    ctl.update(ok3(0.0, 0.2), t);
  CHECK(ctl.rung() == 1);
  CHECK(ctl.counters().demotes_s3_util == 1);
  CHECK(ctl.last_event().t_ms - fresh >= cfg.confirm_ms);
}

// The sibling half, and the only thing that exercises the continuity gate
// itself: update() early-returns on !sample_valid long before block 5a, so
// the "unmeasurable window breaks the run" branch inside 5a (the one the
// test above relies on) never runs at all during an s1 fade -- only the
// continuity gate, comparing against the stale s3_last_live_ms_, catches it
// on the far side of the gap. The run must still break.
TEST(s3_util_confirm_window_resets_across_an_invalid_sample_gap) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 2);
  t += cfg.probation_ms + cfg.s3_settle_ms + 200;
  ctl.update(ok3(0.0, 0.2), t);   // one bad window stamps s3_util_start_ms_
  t += 50;

  LinkHealth invalid;              // s1 window saw no symbols at all
  invalid.sample_valid = false;
  const double gap_end = t + 800;
  for (; t < gap_end; t += 50) CHECK(!ctl.update(invalid, t));

  CHECK(!ctl.update(ok3(0.0, 0.2), t));
  CHECK(ctl.rung() == 2);
  CHECK(ctl.counters().demotes_s3_util == 0);
}

TEST(s3_demote_kill_switch) {
  LadderCfg cfg = make_cfg_noprobe();
  cfg.s3_demote = false;
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 2);
  t += cfg.probation_ms + cfg.s3_settle_ms + 200;
  for (double e = t; e < t + 3000; e += 50) ctl.update(ok3(0.0, 0.2, 0.05), e);
  CHECK(ctl.rung() == 2);                    // inert
}

TEST(s3_demote_during_probation_books_fail) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 2);                     // rung 2 now on probation
  t += cfg.s3_settle_ms + 100;
  const double start = t;
  for (; ctl.rung() == 2 && t - start < 3000; t += 50) ctl.update(ok3(0.0, 0.02, 0.01), t);
  CHECK(ctl.rung() == 1);
  CHECK(ctl.counters().probation_fails == 1);
  CHECK(penalty_ms_for(ctl, t, 2) > 0);
}

TEST(rung_store_feeds_parked_samples) {
  LadderController ctl(make_cfg());
  double t = 0;
  ctl.update(ok(0.0), t);
  CHECK(ctl.rungs().stat(0).u.n == 1);
  t += 50;
  ctl.update(ok(0.0), t);
  CHECK(ctl.rungs().stat(0).u.n == 2);
  CHECK(ctl.rungs().stat(1).u.n == 0);
}

TEST(rung_store_blanks_post_transition_samples) {
  LadderController ctl(make_cfg_noprobe());
  double t = 0;
  promote_to(ctl, t, 1);
  const uint64_t n_before = ctl.rungs().stat(1).u.n;
  ctl.update(ok(0.0), t + 10);   // inside s3_settle_ms (300) blank: no feed
  CHECK(ctl.rungs().stat(1).u.n == n_before);
  ctl.update(ok(0.0), t + 400);  // past the blank: feeding resumes
  CHECK(ctl.rungs().stat(1).u.n == n_before + 1);
}

TEST(rung_store_residual_demote_sample_feeds_measured_rung) {
  LadderController ctl(make_cfg_noprobe());
  double t = 0;
  promote_to(ctl, t, 2);
  feed_for(ctl, t, 400, 0.5);    // clears the post-promote blank
  const uint64_t n_before = ctl.rungs().stat(2).u.n;
  LinkHealth bad = ok(0.0);
  bad.residual_loss = 0.01;
  ctl.update(bad, t);
  CHECK(ctl.rung() == 1);
  CHECK(ctl.rungs().stat(2).u.n == n_before + 1);  // fed BEFORE the demote
  CHECK(ctl.rungs().stat(2).resid.v > 0.0);
  CHECK(ctl.rungs().stat(2).exits_bad == 1);
  CHECK(ctl.rungs().stat(1).visits >= 1);
}

TEST(rung_store_util_demote_not_a_bad_exit) {
  LadderController ctl(make_cfg_noprobe());
  double t = 0;
  promote_to(ctl, t, 1);
  feed_for(ctl, t, 4000, 0.5);   // survive probation (3000 ms)
  const uint32_t bad_before = ctl.rungs().stat(1).exits_bad;
  for (int i = 0; i < 20 && ctl.rung() == 1; ++i) {
    ctl.update(ok(0.9 * ctl.budget_base()), t);
    t += 50;
  }
  CHECK(ctl.rung() == 0);
  CHECK(ctl.rungs().stat(1).exits_bad == bad_before);
}

// The probe column is labeled by the rung that was PROBED, not by the
// candidate whose budget scored it: with rung_offset 2 those differ.
TEST(rung_store_probe_labels_the_probed_rung) {
  LadderCfg cfg = make_cfg();
  cfg.probe.rung_offset = 2;
  LadderController c(cfg);
  c.update(okp(0.0, 2, 0.1), 0);
  CHECK(c.rungs().stat(2).probe_u.n == 1);
  CHECK(c.rungs().stat(1).probe_u.n == 0);
}

TEST(rung_store_evm_feeds_only_when_sampled) {
  LadderController ctl(make_cfg());
  double t = 0;
  LinkHealth h = ok(0.0);
  h.rf_evm_db = -20.0;
  ctl.update(h, t);
  CHECK(ctl.rungs().stat(0).evm_n == 1);
  CHECK(std::abs(ctl.rungs().stat(0).evm_db - -20.0) < 1e-9);
  t += 50;
  ctl.update(ok(0.0), t);         // NaN EVM (ok() default): no feed
  CHECK(ctl.rungs().stat(0).evm_n == 1);
}

TEST(rung_store_s3_steady_state_feeds_current_rung) {
  LadderController ctl(make_cfg());
  double t = 0;
  ctl.update(ok3(0.0), t);        // first sample; s3_live at rung 0
  CHECK(ctl.rungs().stat(0).u3.n >= 1);
}

MTEST_MAIN

// --- fade-aware demotes: regime cascade (Part A) --------------------------

TEST(fade_regime_cascade_shortens_util_confirm) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 3);
  // Clear rung 3's probation with neutral health, so the first demote below
  // is a genuine confirm-window Util demote (which arms the regime) rather
  // than an immediate Probation one (which deliberately does not).
  feed_for(ctl, t, cfg.probation_ms + 200, 0.3);
  REQUIRE(ctl.probation_ms_left(t) == 0);
  // First demote: sustained over-down_util pressure pays the FULL confirm.
  const double bad = 0.9 * ctl.budget_base();  // u = 0.9 > down_util 0.6
  double first_demote_t = -1;
  for (int i = 0; i < 40 && first_demote_t < 0; ++i, t += 50)
    if (ctl.update(ok(bad), t)) first_demote_t = t;
  REQUIRE(first_demote_t > 0);
  CHECK(ctl.rung() == 2);
  // Regime armed: the NEXT demote confirms in fade.confirm_ms (100) +
  // min_between_changes (150) instead of confirm_ms (250).
  double second_demote_t = -1;
  for (int i = 0; i < 40 && second_demote_t < 0; ++i, t += 50)
    if (ctl.update(ok(bad), t)) second_demote_t = t;
  REQUIRE(second_demote_t > 0);
  CHECK(ctl.rung() == 1);
  // In-regime step latency: >= 150 (min_between floor), < 250 (old confirm).
  const double step = second_demote_t - first_demote_t;
  CHECK(step >= 150.0);
  CHECK(step < 250.0);
}

TEST(fade_regime_expires_back_to_full_confirm) {
  LadderCfg cfg = make_cfg_noprobe();
  cfg.fade.hold_ms = 500;  // short regime for the test
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 3);
  // Clear rung 3's probation so the demote below is a confirmed Util one,
  // which ARMS the regime — a Probation demote does not, and would leave
  // this test measuring legacy timing and asserting nothing about expiry.
  feed_for(ctl, t, cfg.probation_ms + 200, 0.3);
  REQUIRE(ctl.probation_ms_left(t) == 0);
  const double bad = 0.9 * ctl.budget_base();
  double d1 = -1;
  for (int i = 0; i < 40 && d1 < 0; ++i, t += 50)
    if (ctl.update(ok(bad), t)) d1 = t;
  REQUIRE(d1 > 0);
  REQUIRE(ctl.fade_active(d1));  // the regime really did arm
  // Go clean past the regime expiry, then re-apply pressure: the demote
  // must pay the FULL confirm_ms again (>= 250 from pressure onset).
  for (double end = t + 800; t < end; t += 50) ctl.update(ok(0.0), t);
  const double pressure_start = t;
  REQUIRE(!ctl.fade_active(pressure_start));  // regime expired before re-load
  double d2 = -1;
  for (int i = 0; i < 40 && d2 < 0; ++i, t += 50)
    if (ctl.update(ok(bad), t)) d2 = t;
  REQUIRE(d2 > 0);
  CHECK(d2 - pressure_start >= 250.0);
}

TEST(fade_regime_s3_blanking_invariant_holds) {
  // Spec §6 test 7 (updated 2026-08-15): in-regime s3-residual steps still
  // respect s3_settle_ms — the re-key blank is NOT shortened by the regime.
  // There is no separate confirm to shorten any more: the demote fires on
  // the first live sample after the blank, in or out of the regime.
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 3);
  // Clear rung 3's probation so the demote below is a confirmed Util one.
  feed_for(ctl, t, cfg.probation_ms + 200, 0.3);
  REQUIRE(ctl.probation_ms_left(t) == 0);
  // Enter the regime via a confirmed s1 util demote.
  const double bad = 0.9 * ctl.budget_base();
  double demote_t = -1;
  for (int i = 0; i < 40 && demote_t < 0; ++i, t += 50)
    if (ctl.update(ok(bad), t)) demote_t = t;
  REQUIRE(demote_t > 0);
  // Immediately feed continuous s3 residual pressure (clean s1). The s3
  // demote may not fire before s3_settle_ms (300, blanks s3_live) has
  // elapsed, but fires instantly on the first live sample after that.
  double s3_demote_t = -1;
  for (int i = 0; i < 40 && s3_demote_t < 0; ++i, t += 50)
    if (ctl.update(ok3(0.0, 0.0, /*s3_resid=*/0.02), t)) s3_demote_t = t;
  REQUIRE(s3_demote_t > 0);
  CHECK(ctl.last_event().reason == CtlReason::S3Residual);
  CHECK(s3_demote_t - demote_t >= cfg.s3_settle_ms);        // blank fully honored
  CHECK(s3_demote_t - demote_t < cfg.s3_settle_ms + 50.0);  // fires on the very next sample
}

TEST(fade_cascade_kill_switch_is_legacy) {
  LadderCfg cfg = make_cfg_noprobe();
  cfg.fade.cascade = false;
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 3);
  // Clear rung 3's probation so BOTH demotes below run the confirm-window
  // Util path. Without this the first is an immediate Probation demote that
  // never arms the regime, and the assertion below would hold identically
  // with cascade=true — i.e. it would not test the kill switch at all.
  feed_for(ctl, t, cfg.probation_ms + 200, 0.3);
  REQUIRE(ctl.probation_ms_left(t) == 0);
  const double bad = 0.9 * ctl.budget_base();
  double d1 = -1, d2 = -1;
  for (int i = 0; i < 40 && d1 < 0; ++i, t += 50)
    if (ctl.update(ok(bad), t)) d1 = t;
  for (int i = 0; i < 40 && d2 < 0; ++i, t += 50)
    if (ctl.update(ok(bad), t)) d2 = t;
  REQUIRE(d1 > 0); REQUIRE(d2 > 0);
  CHECK(d2 - d1 >= 250.0);  // full confirm both times = today's behavior
  // The regime is still ARMED (and still observable) with the cascade killed
  // — only its effect on the confirm windows is suppressed, so the exported
  // label stays truthful about what the link is doing.
  CHECK(ctl.fade_active(d2));
  CHECK(!ctl.fade_active(d2 + cfg.fade.hold_ms));
}

// --- Part B: predictive fade trigger (spec 2026-08-14 section 3) ---

TEST(fade_predict_fires_on_joint_ramp) {
  LadderCfg cfg = make_cfg_noprobe();
  // Keep rung 3's probation open across the whole test so the "no penalty
  // path" assertions below are load-bearing: a fade demote booked like the
  // residual/util ones would charge a probation failure and penalize the rung.
  cfg.probation_ms = 8000;
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 3);
  // Establish baselines: 3 s of steady RF.
  fade_baseline(ctl, t, 3200, 33.0, -55.0);
  REQUIRE(ctl.probation_ms_left(t) > 0);
  const auto before = ctl.counters();
  // Joint fade: RSSI -12 dB, SNR -6 dB — over both thresholds (8/4) once the
  // fast EWMA (tau 300 ms) catches up; must fire with reason fade.
  bool fired = false;
  double fire_t = -1;
  const double fade_start = t;
  for (double end = t + 2000; t < end && !fired; t += 50)
    if (ctl.update(rf(0.0, 27.0, -67.0), t)) { fired = true; fire_t = t; }
  REQUIRE(fired);
  CHECK(ctl.rung() == 2);
  CHECK(ctl.last_event().reason == CtlReason::Fade);
  CHECK(ctl.counters().demotes_fade == before.demotes_fade + 1);
  CHECK(ctl.counters().probation_fails == before.probation_fails);  // no penalty path
  CHECK(ctl.penalized(t).empty());
  // Sustain requirement: cannot fire before trigger_ms (300) of sustained
  // over-threshold, and the fast EWMA needs ~2-3 taus to cross — so the
  // fire lands well after fade_start + trigger_ms.
  CHECK(fire_t - fade_start >= cfg.fade.trigger_ms);
  // Regime armed by the fade demote:
  CHECK(ctl.fade_active(t));
}

TEST(fade_predict_requires_both_signals) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 3);
  fade_baseline(ctl, t, 3200, 33.0, -55.0);
  REQUIRE(ctl.probation_ms_left(t) == 0);
  // RSSI-only drop: never fires.
  for (double end = t + 2000; t < end; t += 50)
    CHECK(!ctl.update(rf(0.0, 33.0, -70.0), t));
  CHECK(ctl.rung() == 3);
  // SNR-only drop: never fires.
  for (double end = t + 2000; t < end; t += 50)
    CHECK(!ctl.update(rf(0.0, 25.0, -55.0), t));
  CHECK(ctl.rung() == 3);
}

TEST(fade_predict_nan_breaks_sustain_run) {
  LadderCfg cfg = make_cfg_noprobe();
  cfg.fade.trigger_ms = 300;
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 3);
  fade_baseline(ctl, t, 3200, 33.0, -55.0);
  REQUIRE(ctl.probation_ms_left(t) == 0);
  // Joint fade but a NaN window every 200 ms: the 300 ms sustain can never
  // complete, so it never fires.
  int i = 0;
  for (double end = t + 3000; t < end; t += 50, ++i) {
    LinkHealth h = (i % 4 == 3) ? ok(0.0)  // NaN RF fields (ok() leaves them NaN)
                                 : rf(0.0, 27.0, -67.0);
    CHECK(!ctl.update(h, t));
  }
  CHECK(ctl.rung() == 3);
}

TEST(fade_predict_gates_min_rung_and_kill_switch) {
  LadderCfg cfg = make_cfg_noprobe();
  cfg.fade.min_rung = 3;
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 2);  // below min_rung
  fade_baseline(ctl, t, 3200, 33.0, -55.0);
  REQUIRE(ctl.probation_ms_left(t) == 0);
  for (double end = t + 2000; t < end; t += 50)
    CHECK(!ctl.update(rf(0.0, 27.0, -67.0), t));
  CHECK(ctl.rung() == 2);

  LadderCfg cfg2 = make_cfg_noprobe();
  cfg2.fade.predict = false;
  LadderController ctl2(cfg2);
  double t2 = 0;
  promote_to(ctl2, t2, 3);
  fade_baseline(ctl2, t2, 3200, 33.0, -55.0);
  REQUIRE(ctl2.probation_ms_left(t2) == 0);
  for (double end = t2 + 2000; t2 < end; t2 += 50)
    CHECK(!ctl2.update(rf(0.0, 27.0, -67.0), t2));
  CHECK(ctl2.rung() == 3);
}

TEST(fade_baseline_asymmetry_survives_3s_fade) {
  LadderCfg cfg = make_cfg_noprobe();
  cfg.fade.min_rung = 99;  // block firing; test the EWMAs only
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 3);
  fade_baseline(ctl, t, 5000, 33.0, -55.0);
  // 3 s deep fade: baseline (fall tau 20 s) must NOT erode below threshold —
  // delta stays >= rssi_db the whole time after the fast EWMA settles.
  t += 50;
  for (double end = t + 3000; t < end; t += 50) ctl.update(rf(0.0, 27.0, -67.0), t);
  CHECK(ctl.fade_drssi() >= cfg.fade.rssi_db);
  CHECK(ctl.fade_dsnr() >= cfg.fade.snr_db);
}

TEST(fade_predict_fires_once_per_fade_event) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 5);
  fade_baseline(ctl, t, 3200, 33.0, -55.0);
  REQUIRE(ctl.probation_ms_left(t) == 0);
  // 6 s of ONE sustained joint fade. The slow baseline falls at tau 20 s, so
  // delta() stays over threshold for the whole run; without a latch the
  // sustain run simply re-accrues and steps down again every trigger_ms
  // (300) — min_between_changes_ms (150) is no spacing gate at all. One fade
  // event must cost exactly one rung; anything further is Part A's job, on
  // MEASURED loss.
  // Utilization is neutral (0.3) rather than 0 so no clean window accrues:
  // a promote partway through would confound the rung count.
  int fires = 0;
  for (double end = t + 6000; t < end; t += 50)
    if (ctl.update(rf(0.3 * ctl.budget_base(), 27.0, -67.0), t)) ++fires;
  CHECK(fires == 1);
  CHECK(ctl.counters().demotes_fade == 1);
  CHECK(ctl.rung() == 4);
  CHECK(ctl.last_event().reason == CtlReason::Fade);
}

TEST(fade_predict_rearms_after_recovery) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 5);
  fade_baseline(ctl, t, 3200, 33.0, -55.0);
  REQUIRE(ctl.probation_ms_left(t) == 0);
  // Fade event 1.
  for (double end = t + 2000; t < end; t += 50)
    ctl.update(rf(0.3 * ctl.budget_base(), 27.0, -67.0), t);
  CHECK(ctl.counters().demotes_fade == 1);
  const int after_first = ctl.rung();
  // Recovery: RF back to baseline. delta() dropping back under threshold is
  // what re-arms the latch — a latch that never re-armed would be a worse bug
  // than the repeated firing it fixes.
  fade_baseline(ctl, t, 3200, 33.0, -55.0);
  REQUIRE(ctl.fade_drssi() < cfg.fade.rssi_db);
  REQUIRE(ctl.fade_dsnr() < cfg.fade.snr_db);
  // Fade event 2: a genuinely new event fires again, once.
  int fires = 0;
  for (double end = t + 2000; t < end; t += 50)
    if (ctl.update(rf(0.3 * ctl.budget_base(), 27.0, -67.0), t)) ++fires;
  CHECK(fires == 1);
  CHECK(ctl.counters().demotes_fade == 2);
  CHECK(ctl.rung() == after_first - 1);
}

TEST(fade_predict_latch_survives_nan_blip) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 5);
  fade_baseline(ctl, t, 3200, 33.0, -55.0);
  REQUIRE(ctl.probation_ms_left(t) == 0);
  // Fade event fires once.
  for (double end = t + 2000; t < end; t += 50)
    ctl.update(rf(0.3 * ctl.budget_base(), 27.0, -67.0), t);
  REQUIRE(ctl.counters().demotes_fade == 1);
  const int after_first = ctl.rung();
  // A NaN telemetry blip mid-fade is absence of evidence, not evidence of
  // recovery — the RF condition is still genuinely over threshold either side
  // of it (Task 4's staleness gate NaNs these labels deliberately). Breaking
  // the sustain run on it is conservative; releasing the latch would not be.
  CHECK(!ctl.update(ok(0.3 * ctl.budget_base()), t));  // ok() leaves the RF labels NaN
  t += 50;
  REQUIRE(ctl.fade_drssi() >= cfg.fade.rssi_db);
  REQUIRE(ctl.fade_dsnr() >= cfg.fade.snr_db);
  // The same fade continues: the latch must still hold.
  int fires = 0;
  for (double end = t + 2000; t < end; t += 50)
    if (ctl.update(rf(0.3 * ctl.budget_base(), 27.0, -67.0), t)) ++fires;
  CHECK(fires == 0);
  CHECK(ctl.counters().demotes_fade == 1);
  CHECK(ctl.rung() == after_first);
}

// --- final whole-branch review, 2026-08-14 ---

// Finding 5. The EWMA feed used to sit below block 4's residual-demote
// return, so it was skipped during exactly the loss phase of a fade and
// fade_drssi/fade_dsnr (sideport link.ctl.fade, ctl-log S line) froze at
// stale values through the episodes the feature will be tuned from.
TEST(fade_ewmas_feed_on_a_residual_demote_tick) {
  LadderCfg cfg = make_cfg_noprobe();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 3);
  for (double end = t + 3200; t < end; t += 50)
    ctl.update(rf(0.3 * ctl.budget_base(), 33.0, -55.0), t);
  const double before = ctl.fade_drssi();
  REQUIRE(before < 1.0);
  LinkHealth h = rf(0.0, 25.0, -70.0);
  h.residual_loss = 0.01;  // instant residual demote: returns before block 4b
  REQUIRE(ctl.update(h, t));
  CHECK(ctl.last_event().reason == CtlReason::Residual);
  CHECK(ctl.fade_drssi() > before + 1.0);
  CHECK(ctl.fade_dsnr() > 0.0);
}

// Until 2026-08-15 the fade regime's shortened s3-util confirm was gated on
// link.attrib: with attribution off, the ~200 ms of debris that outlives
// s3_settle_ms lands between the in-regime 100 ms confirm and the legacy
// window, and u3 is scored against s3's much smaller budget so debris
// clears s3_down_util easily. Attribution is unconditional now, so the
// guard went with the switch and only the shortening remains.
TEST(fade_regime_shortens_the_s3_util_confirm) {
  auto follow_on_demotes = []() {
    LadderCfg cfg = make_cfg_noprobe();
    cfg.s3_demote = true;
    LadderController ctl(cfg);
    double t = 0;
    promote_to(ctl, t, 4);
    feed_for(ctl, t, cfg.probation_ms + 200, 0.3);
    REQUIRE(ctl.probation_ms_left(t) == 0);
    const double bad = 0.9 * ctl.budget_base();
    double last_change = -1;
    for (int i = 0; i < 40 && last_change < 0; ++i, t += 50)
      if (ctl.update(ok(bad), t)) last_change = t;
    REQUIRE(last_change > 0);
    REQUIRE(ctl.fade_active(last_change));
    int follow_on = 0;
    for (double end = t + 4000; t < end; t += 50) {
      // Pre-FEC s3 debris only — no residual, so this exercises the util
      // path in 5b rather than the residual path in 5a. s1 stays clean.
      const double debris = (t - last_change < 500.0) ? 0.2 : 0.0;
      if (ctl.update(ok3(0.0, debris), t)) {
        ++follow_on;
        last_change = t;
      }
    }
    return follow_on;
  };
  CHECK(follow_on_demotes() > 0);
}
