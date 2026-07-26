#include <cmath>

#include "ladder_controller.h"
#include "mtest.h"

using namespace maburgs;

namespace {

// Ladder from the task brief: {mcs, overhead} per rung, [0] = failsafe.
const std::vector<Rung> kLadder = {{0, 1.0}, {2, 0.5},  {4, 0.25},
                                    {5, 0.25}, {6, 0.15}, {7, 0.1}};

LadderCfg make_cfg() {
  LadderCfg cfg;
  cfg.ladder = kLadder;
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
    ctl.update(ok(util_fraction * ctl.budget()), t);
  }
}

int penalty_ms_for(const LadderController& ctl, double now_ms, int rung) {
  for (const auto& pr : ctl.penalized(now_ms)) {
    if (pr.first == rung) return pr.second;
  }
  return -1;
}

}  // namespace

TEST(budget_uses_s1_effective) {
  LadderController ctl(make_cfg());
  // rung 0: overhead 1.0 -> eff1 = 0.75 * (1.0/0.25) = 3.0, clamped to 2.0.
  CHECK(std::abs(ctl.budget() - (2.0 / 3.0)) < 1e-9);

  double t = 0;
  promote_to(ctl, t, 5);  // walk all the way to the top rung {7, 0.1}
  CHECK(ctl.rung() == 5);
  // top rung: overhead 0.1 -> eff1 = 0.75 * (0.1/0.25) = 0.3.
  CHECK(std::abs(ctl.budget() - (0.3 / 1.3)) < 1e-9);
}

TEST(starts_at_failsafe) {
  LadderController ctl(make_cfg());
  CHECK(ctl.rung() == 0);
}

TEST(promote_needs_clean_window) {
  LadderController ctl(make_cfg());
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
  auto cfg = make_cfg();
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
  auto cfg = make_cfg();
  LadderController ctl(cfg);
  double t = 0;
  promote_to(ctl, t, 2);
  feed_for(ctl, t, cfg.probation_ms + 200, 0.3);  // clear rung 2's probation
  REQUIRE(ctl.probation_ms_left(t) == 0);
  REQUIRE(ctl.rung() == 2);

  const double b = ctl.budget();
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
  auto cfg = make_cfg();
  LadderController ctl(cfg);
  double t = 0;

  // --- 1st promotion into rung 1: fail probation immediately (u > down_util
  // while on probation demotes right away, reason Probation). ---
  promote_to(ctl, t, 1);
  REQUIRE(ctl.rung() == 1);
  REQUIRE(ctl.probation_ms_left(t) > 0);
  double fail1_t = t;
  CHECK(ctl.update(ok(0.7 * ctl.budget()), fail1_t));
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
  CHECK(ctl.update(ok(0.7 * ctl.budget()), fail2_t));
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
  CHECK(ctl.update(ok(0.7 * ctl.budget()), fail3_t));
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
  auto cfg = make_cfg();
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
    CHECK(ctl.update(ok(0.7 * ctl.budget()), last_fail_t));
    CHECK(ctl.rung() == 0);
    t = last_fail_t + 50;
  }
  CHECK(ctl.counters().probation_fails == static_cast<uint64_t>(kFailures));
  CHECK(penalty_ms_for(ctl, last_fail_t, 1) == cfg.penalty_max_ms);
}

TEST(starved_forces_bottom) {
  LadderController ctl(make_cfg());
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
  LadderController ctl(make_cfg());
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
  auto cfg = make_cfg();
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
  auto cfg = make_cfg();
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
  auto cfg = make_cfg();
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

  const double b = ctl.budget();
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

MTEST_MAIN
