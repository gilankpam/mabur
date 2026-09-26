#include "hop_blank.h"
#include "hop_verdict.h"
#include "mtest.h"
using namespace maburgs;

// A window straight out of HopVerdict, so these tests are pinned to what
// the classifier can actually produce rather than to a hand-set bool.
static HopCfg cfg() { HopCfg c; c.enable = true; return c; }
static VerdictCardIn card(double rssi, double snr, double foreign_ps, double fa_ps) {
  VerdictCardIn c; c.valid = true; c.rssi_dbm = rssi; c.snr_db = snr;
  c.foreign = (uint32_t)(foreign_ps * 0.15 + 0.5); c.fa = (uint32_t)(fa_ps * 0.15 + 0.5);
  c.cca = c.fa; return c;
}
static VerdictCardIn clean() { return card(-61, 30, 0, 4); }
// 5 s of clean windows at rung 5 to build the references
static double warm(HopVerdict& v, double t = 0) {
  for (int i = 0; i < 40; ++i, t += 150) v.window(t, {clean(), clean()}, {0.0, 20}, 5);
  return t;
}
static VerdictOut jam(HopVerdict& v, double t) {          // interfered: impaired + contended
  return v.window(t, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
}
static VerdictOut fade(HopVerdict& v, double t) {         // impaired + weak
  return v.window(t, {card(-90, 8, 0, 0), card(-86, 10, 0, 10)}, {0.05, 90}, 5);
}
static VerdictOut unknown(HopVerdict& v, double t) {      // impaired, no domain evidence
  return v.window(t, {clean(), clean()}, {0.08, 100}, 5);
}

TEST(no_blank_while_the_link_is_clean) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  auto o = v.window(t, {clean(), clean()}, {0.0, 20}, 5);
  CHECK(o.v == Verdict::Healthy);
  CHECK(!hop_store_blank_until(o, true, 500).has_value());
}

TEST(blank_opens_at_the_first_interfered_window) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  auto o = jam(v, t);
  CHECK(o.v == Verdict::Interfered);
  const auto until = hop_store_blank_until(o, true, 500);
  REQUIRE(until.has_value());
  CHECK(*until == t + 500 + kHopSettleBlankMs);   // confirm_ms + the settle, from onset
}

// Spec section 4's last bullet is "fade/unknown: unchanged ladder
// behaviour". ref_frozen is keyed on `impaired`, and BOTH of these are
// impaired -- keying the blank on it suspended the rung store's EWMA
// writes through every fade and every unknown window, at range for
// seconds at a time, on flights whose whole purpose is recording what the
// ladder does.
TEST(a_fade_never_blanks_the_store_even_though_it_freezes_the_references) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  auto o = fade(v, t);
  CHECK(o.v == Verdict::Fade);
  CHECK(o.evidence & kEvImpaired);
  CHECK(o.ref_frozen);                                   // references ARE frozen...
  CHECK(!hop_store_blank_until(o, true, 500).has_value());   // ...and the store is NOT blanked
  // Still nothing as the fade runs on.
  for (int i = 1; i < 20; ++i)
    CHECK(!hop_store_blank_until(fade(v, t + 150 * i), true, 500).has_value());
}

TEST(an_unknown_window_never_blanks_the_store) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  auto o = unknown(v, t);
  CHECK(o.v == Verdict::Unknown);
  CHECK(o.evidence & kEvImpaired);
  CHECK(o.ref_frozen);
  CHECK(!hop_store_blank_until(o, true, 500).has_value());
}

// hop.enable = false is the bundle default and the first flights' config.
// Nothing is ever ordered and the rung is never restored, so there is no
// hop to protect the store from -- and blanking anyway would silently
// change what those recordings contain versus every pre-branch one.
TEST(disabled_never_blanks) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  auto o = jam(v, t);
  CHECK(o.v == Verdict::Interfered);
  CHECK(!hop_store_blank_until(o, /*enable=*/false, 500).has_value());
}

// The bound: one deadline per frozen episode. A jam that runs for seconds,
// or one that alternates interfered and healthy windows without ever
// reaching the 3 consecutive healthy windows a thaw needs, stays inside
// ONE episode and cannot roll the deadline forward.
TEST(a_sustained_jam_arms_the_blank_exactly_once) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  int armed = 0;
  for (int i = 0; i < 40; ++i)
    if (hop_store_blank_until(jam(v, t + 150 * i), true, 500)) ++armed;
  CHECK(armed == 1);
}
TEST(interfered_healthy_alternation_inside_one_episode_does_not_rearm) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  int armed = 0;
  for (int i = 0; i < 20; ++i) {
    if (hop_store_blank_until(jam(v, t), true, 500)) ++armed;
    t += 150;
    v.window(t, {clean(), clean()}, {0.0, 20}, 5);   // one healthy window: not a thaw
    t += 150;
  }
  CHECK(armed == 1);
}

// ...and it re-arms after a genuine thaw, by either rule.
TEST(rearms_after_three_healthy_windows) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  CHECK(hop_store_blank_until(jam(v, t), true, 500).has_value());
  for (int i = 1; i <= 3; ++i) v.window(t + 150 * i, {clean(), clean()}, {0.0, 20}, 5);
  CHECK(hop_store_blank_until(jam(v, t + 600), true, 500).has_value());
}
TEST(rearms_after_reset_ends_a_verify_window) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  CHECK(hop_store_blank_until(jam(v, t), true, 500).has_value());
  CHECK(!hop_store_blank_until(jam(v, t + 150), true, 500).has_value());
  v.reset();
  CHECK(hop_store_blank_until(jam(v, t + 300), true, 500).has_value());
}

TEST(span_follows_the_configured_confirm_ms) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  CHECK(*hop_store_blank_until(jam(v, t), true, 2000) == t + 2000 + kHopSettleBlankMs);
}
MTEST_MAIN

// ---- the verify's loss input must describe the channel it verifies -----
// The verdict's link-loss input is a 500 ms trailing S1LossWindow. Nothing
// cleared it when a hop landed, so the first verify window on the new
// channel still carried the jam on the old one plus the hop's own retune
// gap: 128 read 50.9 % loss with 0 foreign frames and failed its verify
// (bench 2026-09-24, GS session 0207).
#include "hop_controller.h"
#include "s1_loss.h"
TEST(verify_loss_window_blanks_when_the_hop_lands) {
  HopAction confirm; confirm.kind = HopAction::Confirm;
  auto b = hop_verdict_loss_blank_until(confirm, 1000);
  REQUIRE(b.has_value());
  CHECK(*b == 1000 + kHopSettleBlankMs);
}
TEST(verify_loss_window_untouched_by_every_other_action) {
  for (auto k : {HopAction::None, HopAction::Order, HopAction::OneCardRetune,
                 HopAction::Withdraw, HopAction::Hold, HopAction::VerifyPass}) {
    HopAction a; a.kind = k;
    CHECK(!hop_verdict_loss_blank_until(a, 1000).has_value());
  }
}
TEST(carried_over_loss_does_not_reach_the_new_channels_verify) {
  S1LossWindow w;   // the verdict's 500 ms window
  uint64_t exp = 0, arr = 0; double t = 0;
  for (; t < 1000; t += 10) { exp += 10; arr += 5; w.add(exp, arr, t); }   // 50 % on the jammed op
  REQUIRE(w.sample(t).valid && w.sample(t).loss > 0.4);
  HopAction confirm; confirm.kind = HopAction::Confirm;
  if (auto b = hop_verdict_loss_blank_until(confirm, t)) w.blank_until(*b);
  for (; t < 1400; t += 10) { exp += 10; arr += 10; w.add(exp, arr, t); }  // clean on the new channel
  auto s = w.sample(t);
  CHECK(!s.valid || s.loss < 0.001);
}
