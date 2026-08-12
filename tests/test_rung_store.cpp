#include <cmath>
#include <limits>

#include "ladder_controller.h"  // CtlReason
#include "mtest.h"
#include "rung_store.h"

using namespace maburgs;

namespace {
RungStoreCfg hl(int half_life_samples) {
  RungStoreCfg c;
  c.half_life_samples = half_life_samples;
  return c;
}
}  // namespace

TEST(alpha_from_half_life) {
  // half_life 1 -> alpha 0.5 exactly.
  RungStore st(2, hl(1));
  CHECK(std::abs(st.alpha() - 0.5) < 1e-12);
}

TEST(ewma_seeds_on_first_sample) {
  RungStore st(2, hl(1));
  st.observe_s1(0, 0.25, false, 1000);
  CHECK(std::abs(st.stat(0).u.v - 0.25) < 1e-12);  // seeded, not decayed-from-0
  CHECK(st.stat(0).u.n == 1);
  CHECK(st.stat(0).resid.n == 1);
  CHECK(st.stat(0).resid.v == 0.0);
  CHECK(st.stat(0).last_sample_ms == 1000);
  CHECK(st.stat(1).u.n == 0);
}

TEST(ewma_updates_with_alpha) {
  RungStore st(1, hl(1));  // alpha 0.5
  st.observe_s1(0, 0.25, true, 0);   // seed: u=0.25, resid=1.0
  st.observe_s1(0, 0.75, false, 50);
  CHECK(std::abs(st.stat(0).u.v - 0.5) < 1e-12);      // 0.25 + 0.5*(0.5)
  CHECK(std::abs(st.stat(0).resid.v - 0.5) < 1e-12);  // 1.0 -> 0.5
}

TEST(half_life_property) {
  // Seed 1.0 then half_life zeros -> exactly half the seed remains.
  RungStore st(1, hl(10));
  st.observe_s1(0, 1.0, false, 0);
  for (int i = 1; i <= 10; ++i) st.observe_s1(0, 0.0, false, i * 50);
  CHECK(std::abs(st.stat(0).u.v - 0.5) < 1e-9);
}

TEST(util_inputs_clamped_at_sentinel) {
  // 1e9 zero-budget sentinel must not poison the EWMA (same rationale as
  // CtlLog/StatsExporter clamp_util).
  RungStore st(1, hl(1));
  st.observe_s1(0, 1e9, false, 0);
  CHECK(st.stat(0).u.v == 1e3);
  st.observe_s3(0, 1e9, false, 50);
  CHECK(st.stat(0).u3.v == 1e3);
  st.observe_probe(0, 1e9, 100);
  CHECK(st.stat(0).probe_u.v == 1e3);
}

TEST(s3_and_probe_columns_are_separate) {
  RungStore st(2, hl(1));
  st.observe_s3(0, 0.1, true, 0);
  CHECK(st.stat(0).u3.n == 1);
  CHECK(st.stat(0).s3_resid.v == 1.0);
  CHECK(st.stat(0).u.n == 0);          // parked column untouched
  st.observe_probe(1, 0.3, 100);
  CHECK(st.stat(1).probe_u.n == 1);
  CHECK(std::abs(st.stat(1).probe_u.v - 0.3) < 1e-12);
  CHECK(st.stat(1).u.n == 0);          // probe never blends into parked u
  CHECK(st.stat(1).last_probe_ms == 100);
  CHECK(st.stat(1).last_sample_ms < 0); // probe does not stamp parked age
}

TEST(evm_mean_and_variance) {
  RungStore st(1, hl(1));  // alpha 0.5
  CHECK(std::isnan(st.stat(0).evm_db));  // NaN until seeded
  st.observe_evm(0, -20.0, 0);
  CHECK(std::abs(st.stat(0).evm_db - -20.0) < 1e-12);
  CHECK(st.stat(0).evm_var_db2 == 0.0);
  st.observe_evm(0, -22.0, 50);
  // d=-2, incr=-1 -> mean -21, var = (1-0.5)*(0 + (-2)*(-1)) = 1.0
  CHECK(std::abs(st.stat(0).evm_db - -21.0) < 1e-12);
  CHECK(std::abs(st.stat(0).evm_var_db2 - 1.0) < 1e-12);
  CHECK(st.stat(0).evm_n == 2);
}

TEST(transition_bookkeeping) {
  RungStore st(3, hl(1));
  st.observe_s1(0, 0.0, false, 1000);            // parked_since = 1000
  st.on_transition(0, 1, CtlReason::Promote, 3000);
  CHECK(st.stat(0).dwell_ms == 2000.0);          // closed dwell
  CHECK(st.stat(1).visits == 1);
  CHECK(st.stat(0).exits_bad == 0);              // promote is not a bad exit
  st.on_transition(1, 0, CtlReason::Residual, 5000);
  CHECK(st.stat(1).dwell_ms == 2000.0);
  CHECK(st.stat(1).exits_bad == 1);
  CHECK(st.stat(0).visits == 1);
  // Live dwell for the CURRENT rung only.
  CHECK(st.dwell_ms(0, 6000) == 3000.0);         // 2000 closed + 1000 live
  CHECK(st.dwell_ms(1, 6000) == 2000.0);         // not current: closed only
}

TEST(exits_bad_reason_classification) {
  RungStore st(2, hl(1));
  st.observe_s1(1, 0.0, false, 0);
  st.on_transition(1, 0, CtlReason::Util, 100);      // util: NOT bad
  st.on_transition(0, 1, CtlReason::Promote, 200);
  st.on_transition(1, 0, CtlReason::Timeout, 300);   // timeout: NOT bad
  CHECK(st.stat(1).exits_bad == 0);
  st.on_transition(0, 1, CtlReason::Promote, 400);
  st.on_transition(1, 0, CtlReason::S3Residual, 500);
  st.on_transition(0, 1, CtlReason::Promote, 600);
  st.on_transition(1, 0, CtlReason::Probation, 700);
  st.on_transition(0, 1, CtlReason::Promote, 800);
  st.on_transition(1, 0, CtlReason::Starved, 900);
  CHECK(st.stat(1).exits_bad == 3);
}

TEST(same_rung_transition_is_noop) {
  RungStore st(2, hl(1));
  st.observe_s1(0, 0.0, false, 0);
  st.on_transition(0, 0, CtlReason::Promote, 100);
  CHECK(st.stat(0).visits == 0);
  CHECK(st.stat(0).dwell_ms == 0.0);
}

MTEST_MAIN
