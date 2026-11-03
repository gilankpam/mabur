#include "hop_blank.h"
#include "hop_verdict.h"
#include "mtest.h"
using namespace maburgs;

// A window straight out of HopVerdict, so these tests are pinned to what
// the classifier can actually produce rather than to a hand-set bool.
static HopCfg cfg() { return HopCfg{}; }
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
static int occ_n() { return cfg().verdict.occupied_windows; }
static int det_n() { return cfg().verdict.detect_windows; }
// One blocked window: NHM foreign busy 92 % on both cards, lossy. Interfered
// once the detect ring holds a majority of such readings (a lossy jam is the
// detect route too) or the blocked streak reaches occupied_windows,
// whichever comes first; unknown before that.
static VerdictOut jam(HopVerdict& v, double t) {
  auto c = card(-55, 33, 237, 2); c.busy_valid = true; c.nhm_busy_pct = 95; c.own_air_pct = 3;
  return v.window(t, {c, c}, {0.06, 80}, 5);
}
// occupied_windows blocked windows from t (t advances past them): the last
// one is the episode's first interfered window. Returns how many of them
// armed the blank.
static int occupy(HopVerdict& v, double& t, VerdictOut* last = nullptr) {
  int armed = 0;
  for (int i = 0; i < occ_n(); ++i, t += 150) {
    const auto o = jam(v, t);
    if (hop_store_blank_until(o, true, 500)) ++armed;
    if (last) *last = o;
  }
  return armed;
}
// A healthy window with a clean NHM reading (resets the blocked streak).
static VerdictOut quiet(HopVerdict& v, double t) {
  auto c = clean(); c.busy_valid = true; c.nhm_busy_pct = 10; c.own_air_pct = 3;
  return v.window(t, {c, c}, {0.0, 20}, 5);
}
static VerdictOut fade(HopVerdict& v, double t) {         // impaired + weak
  return v.window(t, {card(-90, 8, 0, 0), card(-86, 10, 0, 10)}, {0.05, 90}, 5);
}
static VerdictOut unknown(HopVerdict& v, double t) {      // impaired, no domain evidence
  return v.window(t, {clean(), clean()}, {0.08, 100}, 5);
}

TEST(no_blank_while_the_link_is_clean) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  auto o = v.window(t, {clean(), clean()}, {0.0, 20}, 5);
  CHECK(o.v == Verdict::Healthy);
  CHECK(!hop_store_blank_until(o, true, 500).has_value());
}

// The blank opens on the jam's FIRST blocked window, windows before it
// classifies `interfered`: the lossy lead-in is where the ladder demotes,
// and those demotes must not reach the per-rung store.
// Revert (arm on the first `interfered` window): the first window has no
// deadline and the occupied_windows-th one does.
TEST(blank_opens_at_the_first_blocked_window) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  const double onset = t;
  auto first = jam(v, t); t += 150;
  CHECK(first.v == Verdict::Unknown);
  const auto until = hop_store_blank_until(first, true, 500);
  REQUIRE(until.has_value());
  CHECK(*until == onset + 500 + kHopSettleBlankMs);   // confirm_ms + the settle, from the first blocked window
  // Nothing later in the episode re-arms, the first interfered window included.
  VerdictOut o;
  for (int i = 1; i < occ_n(); ++i, t += 150) {
    o = jam(v, t);
    CHECK(!hop_store_blank_until(o, true, 500).has_value());
  }
  CHECK(o.v == Verdict::Interfered);
}
// Strong signal, NHM busy `nhm` over our own 35 % airtime.
static VerdictCardIn low_busy(double nhm) {
  auto c = card(-55, 33, 0, 2); c.busy_valid = true; c.nhm_busy_pct = nhm; c.own_air_pct = 35; return c;
}
// The detect route's lead-in: a lossy window whose own foreign reading clears
// detect_pct (8) but not blocked_pct (30) is the earliest window a detect run
// can start from, so the blank arms there -- before the ring fills, or the
// loss-driven demotes meanwhile land in the per-rung store.
// Revert (arm on `blocked` alone): the first window has no deadline.
TEST(blank_opens_at_the_first_lossy_low_foreign_window) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  auto first = v.window(t, {low_busy(45), low_busy(45)}, {0.04, 80}, 5);   // lossy, foreign 10 >= detect_pct, not blocked
  CHECK(!(first.evidence & kEvBlocked));
  REQUIRE(hop_store_blank_until(first, true, 500).has_value());
  CHECK(*hop_store_blank_until(first, true, 500) == t + 500 + kHopSettleBlankMs);
  auto second = v.window(t + 150, {low_busy(45), low_busy(45)}, {0.04, 80}, 5);
  CHECK(!hop_store_blank_until(second, true, 500).has_value());          // one edge per episode
}
// Loss in clean air (foreign 5, under detect_pct) is an unexplained loss,
// not a detect run's lead-in: the ladder owns it.
// Revert (arm on any lossy window with a reading): the first window arms.
TEST(a_lossy_window_with_low_foreign_never_blanks) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  for (int i = 0; i < 3; ++i, t += 150) {
    auto o = v.window(t, {low_busy(40), low_busy(40)}, {0.04, 80}, 5);
    CHECK(o.v == Verdict::Unknown);
    CHECK(!hop_store_blank_until(o, true, 500).has_value());
  }
}
// A detect run whose loss window read under detect_pct (foreign 5) among
// foreign-10 readings has no lossy lead-in to arm on: the window that first
// reads `detected` arms the blank instead, so a detect-route hop never
// reaches Order with the store unprotected.
// Revert (drop `|| detected` from the lead-in): no window of the run arms.
TEST(blank_opens_at_the_first_detected_window_when_the_loss_window_read_low) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  auto lossy = v.window(t, {low_busy(40), low_busy(40)}, {0.04, 80}, 5); t += 150;   // foreign 5: no lead-in
  CHECK(lossy.ref_frozen);
  CHECK(!hop_store_blank_until(lossy, true, 500).has_value());
  VerdictOut o;
  for (int i = 0; i < 2 * det_n() && !(o.evidence & kEvDetected); ++i, t += 150) {
    o = v.window(t, {low_busy(45), low_busy(45)}, {0.0, 20}, 4);                     // foreign 10, clean
    if (!(o.evidence & kEvDetected)) CHECK(!hop_store_blank_until(o, true, 500).has_value());
  }
  REQUIRE(o.evidence & kEvDetected);
  CHECK(!(o.evidence & kEvWeak));
  REQUIRE(hop_store_blank_until(o, true, 500).has_value());
  CHECK(*hop_store_blank_until(o, true, 500) == o.t_ms + 500 + kHopSettleBlankMs);
}
// A weak blocked window is a fade: the ladder owns it, no blank.
TEST(a_weak_blocked_window_never_blanks) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  auto c = card(-90, 8, 237, 2); c.busy_valid = true; c.nhm_busy_pct = 95; c.own_air_pct = 3;
  auto o = v.window(t, {c, c}, {0.06, 80}, 5);
  CHECK(o.v == Verdict::Fade);
  CHECK(!hop_store_blank_until(o, true, 500).has_value());
}

// docs/inflight-channel-hop.md §4: fade and unknown leave the ladder's
// behaviour unchanged. ref_frozen is keyed on `impaired`, and BOTH of these are
// impaired -- keying the blank on it suspended the rung store's EWMA
// writes through every fade and every unknown window, at range for
// seconds at a time, on flights whose whole purpose is recording what the
// ladder does.
TEST(a_fade_never_blanks_the_store_even_though_it_freezes_the_references) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
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
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  auto o = unknown(v, t);
  CHECK(o.v == Verdict::Unknown);
  CHECK(o.evidence & kEvImpaired);
  CHECK(o.ref_frozen);
  CHECK(!hop_store_blank_until(o, true, 500).has_value());
}

// Pinned mode: the reactive layer is off, nothing is ever ordered and the
// rung is never restored, so there is no hop to protect the store from.
TEST(pinned_never_blanks) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  for (int i = 0; i < occ_n(); ++i, t += 150) {
    const auto o = jam(v, t);
    CHECK(!hop_store_blank_until(o, /*reactive=*/false, 500).has_value());
    if (i == occ_n() - 1) CHECK(o.v == Verdict::Interfered);
  }
}

// The bound: one deadline per frozen episode. A jam that runs for seconds,
// or one that alternates blocked and healthy windows without ever reaching
// the 3 consecutive healthy windows a thaw needs, stays inside ONE episode
// and cannot roll the deadline forward. A quiet window's clean NHM reading
// resets the blocked streak, so each stretch of the alternation re-earns
// `interfered`; the detect ring still reads the stretch's lossy majority
// for det_n() - ceil(N/2) quiet windows, and only the two after them are
// healthy.
TEST(a_sustained_jam_arms_the_blank_exactly_once) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  int armed = 0;
  for (int i = 0; i < 40; ++i)
    if (hop_store_blank_until(jam(v, t + 150 * i), true, 500)) ++armed;
  CHECK(armed == 1);
}
TEST(interfered_healthy_alternation_inside_one_episode_does_not_rearm) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  const int tail = det_n() - (det_n() + 1) / 2;
  int armed = 0, interfered = 0;
  for (int i = 0; i < 10; ++i) {
    for (int k = 0; k < occ_n(); ++k, t += 150) {
      const auto o = jam(v, t);
      if (k == occ_n() - 1 && o.v == Verdict::Interfered) ++interfered;
      if (hop_store_blank_until(o, true, 500)) ++armed;
    }
    for (int k = 0; k < tail; ++k, t += 150)
      CHECK(quiet(v, t).evidence & kEvDetected);   // the ring's tail
    for (int k = 0; k < 2; ++k, t += 150)
      CHECK(quiet(v, t).v == Verdict::Healthy);    // two healthy windows: not a thaw
  }
  CHECK(interfered == 10);
  CHECK(armed == 1);
}

// ...and it re-arms after a genuine thaw, by either rule.
TEST(rearms_after_three_healthy_windows) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  CHECK(occupy(v, t) == 1);
  // The ring reads the jam for its tail, then three healthy windows thaw.
  const int tail = det_n() - (det_n() + 1) / 2;
  for (int i = 0; i < tail + 3; ++i, t += 150) quiet(v, t);
  CHECK(!quiet(v, t).ref_frozen); t += 150;
  CHECK(occupy(v, t) == 1);
}
TEST(rearms_after_reset_ends_a_verify_window) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  CHECK(occupy(v, t) == 1);
  CHECK(!hop_store_blank_until(jam(v, t), true, 500).has_value());
  t += 150;
  v.reset();
  CHECK(hop_store_blank_until(jam(v, t), true, 500).has_value());
}

TEST(span_follows_the_configured_confirm_ms) {
  HopVerdict v(cfg(), BusyCfg{}); double t = warm(v);
  CHECK(*hop_store_blank_until(jam(v, t), true, 2000) == t + 2000 + kHopSettleBlankMs);
}
MTEST_MAIN

// ---- the verify's loss input must describe the channel it verifies -----
// The verdict's link-loss input is a 500 ms trailing S1LossWindow. Nothing
// cleared it when a hop landed, so the first verify window on the new
// channel still carried the jam on the old one plus the hop's own retune
// gap: 128 read 50.9 % loss with 0 foreign frames and failed its verify
// (bench, GS session 0207).
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
