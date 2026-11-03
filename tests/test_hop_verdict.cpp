#include "hop_verdict.h"
#include "hop_blank.h"
#include "mtest.h"
using namespace maburgs;
static HopCfg cfg() { HopCfg c; return c; }   // config defaults
// hop.verdict.occupied_windows at its default: the consecutive blocked
// windows it takes for `occupied`, one of the two routes to `interfered`.
static int occ_n() { return cfg().verdict.occupied_windows; }
// hop.verdict.detect_windows at its default: the detect ring's length.
static int det_n() { return cfg().verdict.detect_windows; }
static VerdictCardIn card(double rssi, double snr, double foreign_ps, double fa_ps, double crc_ps = 0) {
  VerdictCardIn c; c.valid = true; c.rssi_dbm = rssi; c.snr_db = snr;
  c.foreign = (uint32_t)(foreign_ps * 0.15 + 0.5); c.fa = (uint32_t)(fa_ps * 0.15 + 0.5);
  c.cca = c.fa; c.crc_fail = (uint32_t)(crc_ps * 0.15 + 0.5); return c;
}
// 5 s of clean windows at rung 5 to build the references
static double warm(HopVerdict& v, double t = 0) {
  for (int i = 0; i < 40; ++i, t += 150) v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5);
  return t;
}
TEST(baseline_is_healthy) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.005, 20}, 5);
  CHECK(o.v == Verdict::Healthy); CHECK(!o.trigger); CHECK(o.ref_rung == -1);
}
// Detection reads NHM foreign airtime only: foreign frames and false alarms
// next to loss do not trip the trigger. A fade can lower our signal but
// cannot add foreign energy, so only foreign airtime names interference.
TEST(co_channel_802_11_neighbour_with_loss_is_not_a_trigger) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  // spike jam250c120: loss 4-8 %, foreign 237/s, FA 2/s, RSSI rose to -55, SNR 33
  auto o1 = v.window(t, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
  CHECK(o1.v == Verdict::Unknown); CHECK(!o1.trigger); CHECK(o1.ref_rung == 5);
  CHECK(o1.evidence & kEvImpaired);
  auto o2 = v.window(t + 150, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 4);  // ladder demoted meanwhile
  CHECK(o2.v == Verdict::Unknown); CHECK(!o2.trigger);
  CHECK(o2.ref_rung == 5);   // snapshot taken at the FIRST impaired window
}
TEST(fade_is_fade_not_interfered) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  auto o = v.window(t, {card(-90, 8, 0, 0, 30), card(-86, 10, 0, 10, 5)}, {0.05, 90}, 1);
  CHECK(o.v == Verdict::Fade); CHECK(o.evidence & kEvWeak); CHECK(!o.trigger);
}
TEST(loss_with_no_domain_evidence_is_unknown) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.08, 100}, 5);
  CHECK(o.v == Verdict::Unknown); CHECK(!o.trigger);
}
TEST(recovered_rate_alone_marks_impaired_and_its_reference_freezes) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);          // recovered mean 20/window
  auto o1 = v.window(t, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.0, 100}, 5);   // 5x
  CHECK(o1.evidence & kEvImpaired);
  for (int i = 1; i < 30; ++i) v.window(t + 150 * i, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.0, 100}, 0);
  auto o2 = v.window(t + 150 * 30, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.0, 100}, 0);
  CHECK(o2.evidence & kEvImpaired);   // frozen mean: 100 is still 5x the pre-onset 20
}
TEST(skipped_card_does_not_contribute_and_one_card_fade_covered_is_healthy) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  VerdictCardIn dead; dead.valid = false;
  auto o = v.window(t, {card(-61, 30, 0, 4), dead}, {0.004, 20}, 5);
  CHECK(o.v == Verdict::Healthy);
  auto o2 = v.window(t + 150, {card(-88, 9, 0, 4), card(-61, 30, 0, 4)}, {0.004, 20}, 5);
  CHECK(o2.v == Verdict::Healthy);   // best card is fine and the link is not impaired
}
TEST(references_thaw_after_three_healthy_windows) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  v.window(t, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
  CHECK(v.ref_rung() == 5);
  for (int i = 1; i <= 3; ++i) v.window(t + 150 * i, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 3);
  CHECK(v.ref_rung() == -1);
}
// weak takes priority over unknown: a lossy window that is weak (range
// edge) is a fade, the ladder's, not an unexplained loss. Without `weak`
// the same window reads unknown, never interfered (weak over occupied is
// weak_still_wins_over_occupied).
// Revert (drop the `weak` branch): reads Unknown.
TEST(weak_takes_priority_over_unknown) {
  HopVerdict v(cfg(), BusyCfg{}, 2);
  double t = 0;
  // Warm at RSSI -80 / SNR 10, no loss: not impaired, so these windows are
  // Healthy.
  for (int i = 0; i < 40; ++i, t += 150) v.window(t, {card(-80, 10, 0, 4), card(-80, 10, 0, 4)}, {0.0, 20}, 5);
  // Now: loss 6% (> loss_pct 3) -> impaired. RSSI/SNR still -80/10 -> weak.
  auto o = v.window(t, {card(-80, 10, 237, 2), card(-80, 10, 237, 2)}, {0.06, 20}, 5);
  CHECK(o.v == Verdict::Fade);
  CHECK(!o.trigger);
  CHECK(o.evidence & kEvImpaired);
  CHECK(o.evidence & kEvWeak);
}
// C1: every VerdictOut carries the wall-clock span its counter deltas were
// gathered over, because the consumer (main.cpp -> HopController) re-feeds
// a cached one on every ~10 ms control tick between 150 ms windows and has
// to be able to tell a fresh verdict from a pre-hop one.
TEST(every_window_carries_the_span_it_was_measured_over) {
  HopVerdict v(cfg(), BusyCfg{}, 2);
  auto o1 = v.window(1000, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5);
  CHECK(o1.t_start_ms == 1000 && o1.t_ms == 1000);   // first window: no previous one
  auto o2 = v.window(1150, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5);
  CHECK(o2.t_start_ms == 1000 && o2.t_ms == 1150);
  auto o3 = v.window(1300, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5);
  CHECK(o3.t_start_ms == 1150 && o3.t_ms == 1300);
  // Even a window with no valid card at all (every card mid-dwell) gets an
  // honest span rather than the default 0/0.
  VerdictCardIn dead; dead.valid = false;
  auto o4 = v.window(1450, {dead, dead}, {0.0, 20}, 5);
  CHECK(o4.v == Verdict::Unknown && o4.t_start_ms == 1300 && o4.t_ms == 1450);
}
// The rung-store blank's arming edge (gs/src/hop_blank.h) -- the FIRST
// impaired, non-weak, blocked window of a frozen episode, once and once only
// until a thaw. That is the jam's lossy lead-in, windows before the first
// `interfered` one. Deliberately not ref_frozen, which is keyed on
// `impaired` and so is set through fades and unknown windows too.
// A blocked card (NHM foreign busy 92 %) on a lossy link: impaired, and
// interfered once the detect ring holds a majority of its readings (the
// lossy jam is the detect route too) or the blocked streak reaches
// occupied_windows, whichever comes first.
static VerdictCardIn jam_card() {
  auto c = card(-55, 33, 237, 2); c.busy_valid = true; c.nhm_busy_pct = 95; c.own_air_pct = 3; return c;
}
// Feed n blocked lossy windows from t; returns the last verdict.
static VerdictOut jam(HopVerdict& v, double& t, int n, int rung = 5) {
  VerdictOut o;
  for (int i = 0; i < n; ++i, t += 150) o = v.window(t, {jam_card(), jam_card()}, {0.06, 80}, rung);
  return o;
}
// Revert (arm on the first `interfered` window, `o.v == Interfered`): u1
// carries no edge and o1 does.
TEST(first_blocked_fires_once_per_frozen_episode) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  // A fade freezes the references but is not interference: no edge.
  auto f = v.window(t, {card(-90, 8, 0, 0, 30), card(-86, 10, 0, 10, 5)}, {0.05, 90}, 1);
  CHECK(f.v == Verdict::Fade && f.ref_frozen && !f.first_blocked);
  t += 150;
  // Loss without a blocked reading: unknown, no edge.
  auto n = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.08, 100}, 5); t += 150;
  CHECK(n.v == Verdict::Unknown && !n.first_blocked);
  // The jam's first blocked window -- still unknown, not occupied -- arms.
  auto u1 = jam(v, t, 1);
  CHECK(u1.v == Verdict::Unknown && u1.ref_frozen && u1.first_blocked);
  auto u = jam(v, t, occ_n() - 2);
  CHECK(!u.first_blocked);
  auto o1 = jam(v, t, 1);
  CHECK(o1.v == Verdict::Interfered && !o1.first_blocked);   // same episode
  // A thaw re-arms it. A thaw needs healthy windows, and a held occupied
  // streak keeps every window impaired, so the healthy windows carry a
  // clean NHM reading -- which resets the streak. The detect ring still
  // holds the jam's lossy majority for its first det_n() - ceil(N/2)
  // windows (detected, not healthy); three healthy windows follow.
  auto clean = [] { auto c = card(-61, 30, 0, 4); c.busy_valid = true; c.nhm_busy_pct = 10; c.own_air_pct = 3; return c; };
  const int tail = det_n() - (det_n() + 1) / 2;
  for (int i = 0; i < tail + 3; ++i, t += 150) v.window(t, {clean(), clean()}, {0.0, 20}, 5);
  CHECK(!v.window(t, {clean(), clean()}, {0.0, 20}, 5).ref_frozen); t += 150;
  CHECK(jam(v, t, 1).first_blocked);
  // ...and so does reset(), the other thaw rule.
  v.reset();
  CHECK(jam(v, t, 1).first_blocked);
}
// A blocked window at zero loss before occupancy is not impaired, so it is
// no episode and arms nothing: no loss, no demote to keep out of the store.
// Revert (drop `impaired &&` from the edge): w1 arms.
TEST(zero_loss_blocked_lead_in_does_not_arm) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  auto c = card(-61, 30, 0, 4); c.busy_valid = true; c.nhm_busy_pct = 95; c.own_air_pct = 3;
  auto w1 = v.window(t, {c, c}, {0.0, 20}, 5); t += 150;
  CHECK(w1.v == Verdict::Healthy && !w1.first_blocked);
  VerdictOut o;
  for (int i = 1; i < occ_n(); ++i, t += 150) o = v.window(t, {c, c}, {0.0, 20}, 5);
  CHECK(o.v == Verdict::Interfered && o.first_blocked);   // occupancy itself is the first impaired window
}
// A weak blocked window is a fade, and a fade never arms.
// Revert (drop `!weak` from the edge): reads first_blocked.
TEST(weak_blocked_window_does_not_arm) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  auto c = jam_card(); c.rssi_dbm = -90; c.snr_db = 8;
  auto o = v.window(t, {c, c}, {0.06, 80}, 5);
  CHECK(o.v == Verdict::Fade && (o.evidence & kEvBlocked) && !o.first_blocked);
}
TEST(ref_frozen_marks_the_impaired_episode) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  CHECK(!v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5).ref_frozen);
  t += 150;
  auto o = v.window(t, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
  CHECK(o.evidence & kEvImpaired);
  CHECK(o.ref_frozen);                      // the first impaired window itself
  t += 150;
  CHECK(v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5).ref_frozen);
  t += 150;
  CHECK(v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5).ref_frozen);
  t += 150;   // third consecutive healthy window: thaw
  CHECK(!v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5).ref_frozen);
}
// reset() is the other thaw rule: a hop's verify window ending. It had zero
// callers in the tree until main.cpp gained HopAction::VerifyPass; this pins
// what it has to clear, including the latched persistence window -- a trigger
// built from windows measured on the channel we have just left must not
// survive the hop.
TEST(reset_thaws_the_snapshot_and_clears_the_latched_trigger) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  auto o1 = jam(v, t, occ_n());
  auto o2 = jam(v, t, 1);
  CHECK(o1.ref_frozen && o2.trigger && v.ref_rung() == 5);
  v.reset();
  CHECK(v.ref_rung() == -1);
  // One clean window on the NEW channel: no latched trigger, no frozen
  // reference.
  auto o3 = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5);
  CHECK(!o3.trigger && !o3.ref_frozen && o3.v == Verdict::Healthy);
}
// Bench: main.cpp fed the aggregator's RAW EMAs (RSSI ~64, SNR ~66 half-dB)
// straight into rssi_dbm/snr_db, so `weak` (rssi < -78 dBm && snr < 12 dB)
// could never trip and Fade was unreachable on hardware. The conversion lives
// next to the engine so scan.log V lines and flightreport's medians read in
// dBm/dB like ctl.log does.
TEST(raw_units_convert_to_dbm_and_db) {
  CHECK(rssi_raw_to_dbm(64.0) == -46.0);
  CHECK(snr_raw_to_db(66.0) == 33.0);
  CHECK(rssi_raw_to_dbm(0.0) == 0.0);   // "no frame heard yet" keeps the no-reference sentinel
}
TEST(converted_edge_of_range_reading_is_fade) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  const double r = rssi_raw_to_dbm(28), s = snr_raw_to_db(20);   // -82 dBm, 10 dB
  auto o = v.window(t, {card(r, s, 0, 4), card(r, s, 0, 4)}, {0.06, 80}, 5);
  CHECK(o.v == Verdict::Fade); CHECK(o.evidence & kEvWeak);
  // The same numbers unconverted are what the bug fed: never weak.
  auto raw = v.window(t + 150, {card(28, 20, 0, 4), card(28, 20, 0, 4)}, {0.06, 80}, 5);
  CHECK(!(raw.evidence & kEvWeak));
}
MTEST_MAIN

static maburgs::VerdictCardIn busy_card(double busy, double own, bool valid = true) {
  maburgs::VerdictCardIn c;
  c.valid = true; c.rssi_dbm = -48; c.snr_db = 30;
  c.busy_valid = valid; c.nhm_busy_pct = busy; c.own_air_pct = own;
  return c;
}
// Bench run 1: 80 % loss, no foreign/FA, strong signal, air 95 % busy.
// Loss next to a blocked reading is still not interference until the
// blocked streak reaches occupied_windows: a lossy window is no evidence
// that the foreign energy is sustained.
// Revert (restore `blocked` + loss-type impairment -> Interfered): the
// first window reads Interfered.
TEST(long_frame_jam_is_interfered_once_occupied) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, BusyCfg{}, 1);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.80;
  maburgs::VerdictOut o;
  double t = 0;
  for (int i = 1; i < occ_n(); ++i) {
    o = v.window(t += 150, {busy_card(95, 2)}, bad, 0);
    CHECK(o.v == maburgs::Verdict::Unknown);
    CHECK(o.evidence & maburgs::kEvBlocked);
    CHECK(!(o.evidence & maburgs::kEvOccupied));
  }
  o = v.window(t += 150, {busy_card(95, 2)}, bad, 0);
  CHECK(o.v == maburgs::Verdict::Interfered);
  CHECK(o.evidence & maburgs::kEvOccupied);
  o = v.window(t += 150, {busy_card(95, 2)}, bad, 0);
  CHECK(o.trigger);
}
// Pin (a): loss + blocked without occupancy is unknown, whatever the loss.
// Revert (restore `blocked` + loss-type impairment -> Interfered): reads
// Interfered.
TEST(loss_and_blocked_without_occupancy_is_unknown) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  VerdictLinkIn bad; bad.pre_fec_loss = 0.5; bad.recovered = 200;
  auto o = v.window(t, {busy_card(98, 0), busy_card(98, 0)}, bad, 5);
  CHECK(o.evidence & kEvBlocked); CHECK(o.evidence & kEvImpaired); CHECK(!(o.evidence & kEvOccupied));
  CHECK(o.v == Verdict::Unknown);
  CHECK(o.first_blocked);   // the store blank's edge: before occupancy
}
// A 40 % busy jam: blocked under radio.scan.blocked_pct 35, not under 45 --
// the threshold is the engine's input, not a constant.
TEST(blocked_threshold_comes_from_busy_cfg) {
  maburgs::HopCfg cfg;
  BusyCfg lo; lo.blocked_pct = 35;
  BusyCfg hi; hi.blocked_pct = 45;
  maburgs::HopVerdict low(cfg, lo, 1), high(cfg, hi, 1);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.80;
  maburgs::VerdictOut ol, oh;
  double t = 0;
  for (int i = 0; i < 3; ++i) { ol = low.window(t += 150, {busy_card(40, 2)}, bad, 0); oh = high.window(t, {busy_card(40, 2)}, bad, 0); }
  CHECK(ol.evidence & maburgs::kEvBlocked);
  CHECK(!(oh.evidence & maburgs::kEvBlocked));
}
TEST(busy_but_healthy_link_stays_healthy) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, BusyCfg{}, 1);
  maburgs::VerdictLinkIn ok; ok.pre_fec_loss = 0.0;
  auto o = v.window(150, {busy_card(95, 2)}, ok, 0);
  CHECK(o.v == maburgs::Verdict::Healthy);
}
TEST(own_airtime_is_subtracted) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, BusyCfg{}, 1);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.20;
  auto o = v.window(150, {busy_card(60, 45)}, bad, 0);   // 15 % foreign, under blocked_pct
  CHECK(!(o.evidence & maburgs::kEvBlocked));
  CHECK(o.v == maburgs::Verdict::Unknown);
}
TEST(no_busy_reading_is_todays_verdict) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, BusyCfg{}, 1);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.8;
  auto o = v.window(150, {busy_card(100, 0, /*valid=*/false)}, bad, 0);
  CHECK(o.v == maburgs::Verdict::Unknown);
  CHECK(!(o.evidence & maburgs::kEvBlocked));
}
// Blocked is the MINIMUM foreign-busy reading across cards that have one, not
// any-card's. Reverting to any-card makes
// weak_diversity_card_alone_is_not_blocked fail (it would see card A's foreign
// 65 alone and call Interfered).
TEST(weak_diversity_card_alone_is_not_blocked) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, BusyCfg{}, 2);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.2;
  auto a = busy_card(70, 5);    // foreign 65 -- would clear blocked_pct alone
  auto b = busy_card(70, 68);   // foreign 2 -- the same interferer, seen weakly
  auto o = v.window(150, {a, b}, bad, 0);
  CHECK(!(o.evidence & maburgs::kEvBlocked));
  CHECK(o.v != maburgs::Verdict::Interfered);
}
TEST(interferer_seen_by_both_cards_is_blocked) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, BusyCfg{}, 2);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.2;
  auto a = busy_card(95, 2);   // foreign 93
  auto b = busy_card(95, 2);   // foreign 93
  maburgs::VerdictOut o;
  double t = 0;
  for (int i = 0; i < occ_n(); ++i) o = v.window(t += 150, {a, b}, bad, 0);
  CHECK(o.evidence & maburgs::kEvBlocked);
  CHECK(o.v == maburgs::Verdict::Interfered);
}
TEST(card_without_reading_does_not_veto) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, BusyCfg{}, 2);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.2;
  auto a = busy_card(95, 2);                     // foreign 93, only reading card
  auto b = busy_card(100, 0, /*valid=*/false);   // no busy reading this window
  maburgs::VerdictOut o;
  double t = 0;
  for (int i = 0; i < occ_n(); ++i) o = v.window(t += 150, {a, b}, bad, 0);
  CHECK(o.evidence & maburgs::kEvBlocked);
  CHECK(o.v == maburgs::Verdict::Interfered);
}

// ---- a floor on the recovered-symbols impaired term --------------------
// Bench (session 0232): on a clean channel whose trailing recovered mean is
// ~0.2/window, 1-5 recovered symbols read "3x the reference" and marked the
// window impaired with 0 % loss -- which, next to 112's ambient false-alarm
// background, hopped a perfectly good link into a blocked channel.
// recovered_min (default 8) removes ~97 % of those. Revert (drop
// `&& recovered >= recovered_min` in hop_verdict.cpp):
// recovered_below_floor_is_not_impaired fails (impaired, not healthy).
static double warm_quiet(HopVerdict& v, double t = 0) {
  // trailing recovered mean ~0.2/window: one recovered symbol every 5th window
  for (int i = 0; i < 40; ++i, t += 150)
    v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, (uint32_t)(i % 5 == 0 ? 1 : 0)}, 5);
  return t;
}
TEST(recovered_below_floor_is_not_impaired) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm_quiet(v);
  auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 2}, 5);
  CHECK(!(o.evidence & kEvImpaired));
  CHECK(o.v == Verdict::Healthy);
}
// Revert (floor compared with > 12 instead of >=, or floor applied to the
// loss term too): the at/above-floor case must still be impaired.
TEST(recovered_at_floor_is_impaired) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm_quiet(v);
  auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 12}, 5);
  CHECK(o.evidence & kEvImpaired);
  HopVerdict v8(cfg(), BusyCfg{}, 2); t = warm_quiet(v8);
  auto o8 = v8.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 8}, 5);
  CHECK(o8.evidence & kEvImpaired);   // exactly the floor counts
}
// Revert (treat 0 as "use the default"): the zero-floor window stays healthy.
TEST(recovered_min_zero_disables_floor) {
  HopCfg c = cfg(); c.verdict.recovered_min = 0;
  HopVerdict v(c, BusyCfg{}, 2); double t = warm_quiet(v);
  auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 2}, 5);
  CHECK(o.evidence & kEvImpaired);
}

// new_session() clears only the blocked streak
// (reset_and_new_session_clear_the_occupied_count); the recovered trailing
// reference is untouched -- a session that resumes mid-stream has no reason
// to forget what "normal" recovered looked like.
TEST(new_session_leaves_the_recovered_reference_intact) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);   // recovered mean 20
  v.new_session();
  auto r = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 100}, 5);
  CHECK(r.evidence & kEvImpaired);   // 100 is 5x the pre-session mean of 20
}
TEST(weak_without_a_real_snr_fires_on_rssi_alone) {
  // Relay-only evidence this window: RSSI -90 dBm, "SNR" reads RSSI+95 = 5
  // -- but even if it read 40 the card must still be weak, because its SNR
  // is not a measurement.
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  auto relay = card(-90, 40, 0, 0, 30); relay.snr_valid = false;
  auto o = v.window(t, {relay, VerdictCardIn{}}, {0.05, 90}, 1);
  CHECK(o.v == Verdict::Fade); CHECK(o.evidence & kEvWeak);
  // A Realtek card with the same numbers and a real SNR of 40 is NOT weak.
  HopVerdict u(cfg(), BusyCfg{}, 2); t = warm(u);
  auto o2 = u.window(t, {card(-90, 40, 0, 0, 30), VerdictCardIn{}}, {0.05, 90}, 1);
  CHECK(!(o2.evidence & kEvWeak));
}

// ---- sustained occupancy is impaired on its own -------------------------
// A long-frame or sub-correlator jam fills airtime (`blocked`) without
// moving loss or tripping foreign/fa: at zero loss a clean signal reads
// `healthy` forever unless `blocked` holding for occupied_windows
// consecutive windows becomes `impaired` on its own (hop.verdict.
// occupied_windows). Width-free: HopVerdict never sees radio.width.
static VerdictCardIn busy(double nhm, double own) { auto c = card(-61, 30, 0, 4); c.busy_valid = true; c.nhm_busy_pct = nhm; c.own_air_pct = own; return c; }
TEST(occupied_fires_at_default_width_after_occupied_windows_blocked_windows_with_zero_loss) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  VerdictOut o;
  for (int i = 1; i < occ_n(); ++i, t += 150) { o = v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5); CHECK(o.v == Verdict::Healthy); CHECK(!(o.evidence & kEvOccupied)); }
  o = v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);          // the occupied_windows-th
  CHECK(o.v == Verdict::Interfered); CHECK(o.evidence & kEvOccupied); CHECK(o.evidence & kEvImpaired); CHECK(o.ref_rung == 5);
  CHECK(!(o.evidence & kEvDetected));   // the occupied route: no loss in the ring, so not the detect route
  o = v.window(t + 150, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 4);   // next: persist 2 of 3
  CHECK(o.trigger); CHECK(o.ref_rung == 5);
}
TEST(a_clean_window_resets_the_occupied_count) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  for (int i = 1; i < occ_n(); ++i, t += 150) v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);
  v.window(t, {busy(10, 3), busy(10, 3)}, {0.0, 20}, 5); t += 150;
  auto o = v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);
  CHECK(o.v == Verdict::Healthy); CHECK(!(o.evidence & kEvOccupied));
}
TEST(no_busy_reading_holds_the_occupied_count) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  for (int i = 1; i < occ_n(); ++i, t += 150) v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);
  v.window(t, {card(-61, 30, 0, 4), card(-61, 30, 0, 4)}, {0.0, 20}, 5); t += 150;   // valid cards, no NHM
  VerdictCardIn off; off.valid = false;
  v.window(t, {off, off}, {0.0, 20}, 5); t += 150;                                    // no usable card
  auto o = v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);                    // the occupied_windows-th blocked
  CHECK(o.v == Verdict::Interfered); CHECK(o.evidence & kEvOccupied);
}
// Pin (d): at the range edge our own frames stop decoding and the own-airtime
// subtraction can read our video as foreign, so `weak` gates occupancy.
// Revert (test `occupied && blocked` before `weak`): reads Interfered.
TEST(weak_still_wins_over_occupied) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  auto weak_busy = [] { auto c = busy(95, 3); c.rssi_dbm = -90; c.snr_db = 8; return c; };
  VerdictOut o;
  for (int i = 0; i < occ_n() + 1; ++i, t += 150) o = v.window(t, {weak_busy(), weak_busy()}, {0.0, 20}, 1);
  CHECK(o.v == Verdict::Fade); CHECK(o.evidence & kEvOccupied); CHECK(!o.trigger);
}
TEST(reset_and_new_session_clear_the_occupied_count) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  for (int i = 1; i < occ_n(); ++i, t += 150) v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);
  v.reset();
  auto o = v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);
  CHECK(!(o.evidence & kEvOccupied));
  for (int i = 1; i < occ_n(); ++i, t += 150) v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);
  v.new_session();
  o = v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);
  CHECK(!(o.evidence & kEvOccupied));
}
TEST(own_airtime_is_subtracted_before_the_occupied_count) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  VerdictOut o;
  for (int i = 0; i < occ_n() + 2; ++i, t += 150) o = v.window(t, {busy(60, 45), busy(60, 45)}, {0.0, 20}, 5);  // 15 % foreign under blocked_pct
  CHECK(o.v == Verdict::Healthy); CHECK(!(o.evidence & kEvOccupied));
}
// Controller ruling: once occupied, a window with NO busy reading (valid
// cards, no NHM -- the common shape right after a hop lands, since the
// retune invalidates the NHM arm) must classify `unknown`, never
// `interfered` on its own -- `HopController` fails verify only on an
// `interfered` window measured after landing, and failing it on a reading
// that never happened would kill a clean target. The streak still holds
// (no_busy_reading_holds_the_occupied_count), so the very next window with
// an actual blocked reading reads `interfered` again.
// Pin (e). Revert (classify on `occupied` alone, without `&& blocked`): the
// reading-less window reads Interfered.
TEST(occupied_persists_but_a_reading_less_window_classifies_unknown) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  for (int i = 0; i < occ_n(); ++i, t += 150) v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);   // reach occupied
  t += 150;
  auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 30, 0, 4)}, {0.0, 20}, 5);   // valid cards, no NHM
  CHECK(o.v == Verdict::Unknown);
  CHECK(o.evidence & kEvOccupied);
  CHECK(o.evidence & kEvImpaired);
  t += 150;
  auto o2 = v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);
  CHECK(o2.v == Verdict::Interfered);
}
// `occupied` reaches `interfered` only together with `blocked` in the same
// window. A no-reading window on a busy-but-unblocked target that hears
// foreign frames or false alarms is `unknown`, not `interfered`, so it
// cannot fail a hop's verify on traffic NHM never measured.
TEST(occupied_reading_less_window_with_foreign_or_fa_stays_unknown) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  for (int i = 0; i < occ_n(); ++i, t += 150) v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);   // reach occupied
  auto o = v.window(t, {card(-61, 30, 60, 4), card(-61, 30, 60, 4)}, {0.0, 20}, 5);   // foreign frames, no NHM
  CHECK(o.evidence & kEvOccupied); CHECK(o.evidence & kEvImpaired);
  CHECK(o.v == Verdict::Unknown);
  t += 150;
  o = v.window(t, {card(-61, 30, 0, 150), card(-61, 30, 0, 150)}, {0.0, 20}, 5);      // false alarms, no NHM
  CHECK(o.evidence & kEvOccupied); CHECK(o.evidence & kEvImpaired);
  CHECK(o.v == Verdict::Unknown);
}
// Pin (b) while occupied: loss + foreign frames on a window with no NHM
// reading is still not interference -- only a blocked reading is.
// Revert (classify occupied + a reading-less loss window as Interfered):
// reads Interfered.
TEST(lossy_foreign_reading_less_window_is_unknown_while_occupied) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  for (int i = 0; i < occ_n(); ++i, t += 150) v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 5);   // reach occupied
  auto o = v.window(t, {card(-61, 30, 60, 4), card(-61, 30, 60, 4)}, {0.06, 20}, 5);  // lossy, foreign frames, no NHM
  CHECK(o.evidence & kEvOccupied);
  CHECK(o.v == Verdict::Unknown);
}

// ---- the detect route: low-floor NHM occupancy confirmed by loss ---------
// `detected` = the last detect_windows windows (the ring) hold at least
// ceil(N/2) busy readings whose median foreign % is >= detect_pct, and at
// least detect_impaired of them were loss-impaired, with the ring full.
// A "low jam" window is busy(45, 35): foreign 10, over detect_pct (8) and
// well under blocked_pct (30), so neither `blocked` nor `occupied` moves.
// Every window pushes a slot, so warm()'s reading-less windows fill the
// ring; the tests that pin the full-ring rule start from an empty ring --
// the state a clear (landing, new session) leaves it in -- via
// new_session(), whose clear reset_and_new_session_clear_the_detect_ring
// pins on its own.
static VerdictCardIn low_jam() { return busy(45, 35); }
// Fill an empty ring: one loss window, then det_n() - 1 clean ones. The
// last is the first detected window.
static VerdictOut detect_run(HopVerdict& v, double& t, int first_rung = 5, int rung = 4) {
  auto o = v.window(t, {low_jam(), low_jam()}, {0.04, 20}, first_rung); t += 150;
  for (int i = 1; i < det_n(); ++i, t += 150) o = v.window(t, {low_jam(), low_jam()}, {0.0, 20}, rung);
  return o;
}
// The final window carries no loss of its own and still reads impaired (the
// ring holds the loss) -- so the freeze and the persist see the run.
// Revert (drop the ring-full condition): the third window already reads
// detected. Revert (no thaw hold): ref_rung re-freezes at the demoted 4.
TEST(detected_fires_with_one_loss_window_inside_a_low_foreign_run) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  v.new_session();                                                      // empty ring
  auto o = v.window(t, {low_jam(), low_jam()}, {0.04, 20}, 5); t += 150;   // loss 4 % > loss_pct
  CHECK(o.v == Verdict::Unknown); CHECK(o.ref_rung == 5);
  for (int i = 1; i < det_n() - 1; ++i, t += 150) { o = v.window(t, {low_jam(), low_jam()}, {0.0, 20}, 4); CHECK(!(o.evidence & kEvDetected)); }
  o = v.window(t, {low_jam(), low_jam()}, {0.0, 20}, 4);               // ring full
  CHECK(o.v == Verdict::Interfered); CHECK(o.evidence & kEvDetected); CHECK(o.evidence & kEvImpaired);
  CHECK(!(o.evidence & kEvBlocked)); CHECK(!(o.evidence & kEvOccupied)); CHECK(o.ref_rung == 5);
}
// Revert (drop the loss condition): reads Interfered.
TEST(detected_needs_a_loss_window) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v); VerdictOut o;
  for (int i = 0; i < det_n() + 2; ++i, t += 150) o = v.window(t, {low_jam(), low_jam()}, {0.0, 20}, 5);
  CHECK(o.v == Verdict::Healthy); CHECK(!(o.evidence & kEvDetected));
}
// Readings 2,2,2,2,40: mean 9.6 clears detect_pct, median 2 does not.
// Revert (mean, or max, instead of the median): reads Interfered.
TEST(detected_needs_the_median_not_a_single_window) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v); VerdictOut o;
  for (int i = 0; i < det_n() - 1; ++i, t += 150) o = v.window(t, {busy(37, 35), busy(37, 35)}, {0.0, 20}, 5);   // foreign 2
  o = v.window(t, {busy(75, 35), busy(75, 35)}, {0.04, 20}, 5);   // one 40 % window with loss
  CHECK(o.v == Verdict::Unknown); CHECK(!(o.evidence & kEvDetected));
}
// Foreign 5 with the loss windows last, so the final window is impaired and
// Unknown (not Interfered) is the meaningful reading.
// Revert (drop the detect_pct test, or compare against a lower floor): reads
// Interfered.
TEST(detected_does_not_fire_below_detect_pct) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v); VerdictOut o;
  for (int i = 0; i < det_n(); ++i, t += 150) o = v.window(t, {busy(40, 35), busy(40, 35)}, {i >= det_n() - 2 ? 0.04 : 0.0, 20}, 5);   // foreign 5
  CHECK(o.evidence & kEvImpaired);
  CHECK(o.v == Verdict::Unknown); CHECK(!(o.evidence & kEvDetected));
}
// Reading-less windows take a slot (warm() filled the ring with them): two
// readings of five is no majority, three is.
// Revert (drop the ceil(N/2) readings condition): the second window reads
// detected.
TEST(detected_needs_a_majority_of_reading_windows) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  REQUIRE(det_n() == 5);   // two readings is under ceil(5/2), three is not
  v.window(t, {low_jam(), low_jam()}, {0.04, 20}, 5); t += 150;
  auto o = v.window(t, {low_jam(), low_jam()}, {0.0, 20}, 5); t += 150;
  CHECK(o.v == Verdict::Healthy); CHECK(!(o.evidence & kEvDetected));
  o = v.window(t, {low_jam(), low_jam()}, {0.0, 20}, 5);
  CHECK(o.v == Verdict::Interfered); CHECK(o.evidence & kEvDetected);
}
// The current window must carry a busy reading of its own for `detected` to
// classify -- a post-landing window with valid cards and no NHM stays
// unknown, like `occupied && blocked`. The window carries loss itself so a
// loss window is still inside the ring once it evicts the run's first one.
// Revert (classify `detected` without `have_busy_reading`): reads
// Interfered.
TEST(detected_reading_less_current_window_is_unknown) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  v.new_session();
  CHECK(detect_run(v, t).evidence & kEvDetected);
  auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 30, 0, 4)}, {0.04, 20}, 4);   // valid cards, no NHM
  CHECK(o.v == Verdict::Unknown); CHECK(o.evidence & kEvDetected);
}
// At the range edge the own-airtime subtraction can read our video as
// foreign, so `weak` gates the detect route as it gates occupancy.
// Revert (test `detected` before `weak`): reads Interfered.
TEST(weak_still_wins_over_detected) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  v.new_session();
  auto weak_low = [] { auto c = low_jam(); c.rssi_dbm = -90; c.snr_db = 8; return c; };
  auto o = v.window(t, {weak_low(), weak_low()}, {0.04, 20}, 5); t += 150;
  for (int i = 1; i < det_n(); ++i, t += 150) o = v.window(t, {weak_low(), weak_low()}, {0.0, 20}, 4);
  CHECK(o.v == Verdict::Fade); CHECK(o.evidence & kEvDetected);
  CHECK(!o.first_blocked); CHECK(!o.trigger);
}
// landed() is the hop's landing: the ring holds the old channel's readings
// and must not fail verify on a clean target, while the blocked streak
// carries so an occupied target still fails on its first blocked reading.
// Without landed() the ring would be full (the four lossy 92 % windows and
// this one, median 92) and read detected. Written for the shipped defaults,
// where occupied_windows == detect_windows.
// Revert (landed() leaves the ring): reads detected. Revert (landed() also
// clears blocked_streak_): not occupied. Revert (landed() also thaws):
// ref_rung reads -1, then 3.
TEST(landed_clears_the_detect_ring_and_keeps_the_blocked_streak) {
  REQUIRE(occ_n() == det_n());
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  for (int i = 1; i < occ_n(); ++i, t += 150) v.window(t, {busy(95, 3), busy(95, 3)}, {0.04, 20}, 5);   // blocked AND detecting
  REQUIRE(v.ref_rung() == 5);
  v.landed(t);
  CHECK(v.ref_rung() == 5);                                           // the frozen references survive
  auto o = v.window(t, {busy(95, 3), busy(95, 3)}, {0.0, 20}, 3);     // the occupied_windows-th blocked; ladder sank
  CHECK(o.evidence & kEvOccupied);
  CHECK(!(o.evidence & kEvDetected));
  CHECK(o.ref_frozen); CHECK(o.ref_rung == 5);                        // a thawing landed() re-freezes at 3
}
// A window whose measurement span starts inside the landing settle
// (kHopSettleBlankMs after landed()) carries the hop's own retune gap as
// recovered symbols: its slot keeps its foreign reading but never counts as
// the ring's loss window, so a 10 % target with no loss of its own never
// reads detected once the ring fills. HopController ignores the same
// windows for its verify decision; without this the ring would hold the
// debris and fail verify a ring later.
// Revert (the slot keeps loss_impaired inside the settle): the fifth window
// reads detected.
TEST(settle_debris_never_confirms_the_detect_ring) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  v.landed(t - 150);                                                  // between windows
  auto o = v.window(t, {low_jam(), low_jam()}, {0.04, 20}, 5); t += 150;   // starts inside the settle, lossy
  CHECK(!(o.evidence & kEvDetected));
  for (int i = 1; i < det_n() + 2; ++i, t += 150) {
    o = v.window(t, {low_jam(), low_jam()}, {0.0, 20}, 4);
    CHECK(!(o.evidence & kEvDetected));
  }
}
// The twin: a loss window whose span starts exactly at the settle's end is
// the target's own loss and confirms the ring like any other.
// Revert (the settle test widened to `<=`, or every slot after a landing
// non-lossy): never reads detected.
TEST(a_loss_window_after_the_settle_confirms_the_detect_ring) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  const double landed_ms = t - 150;
  v.landed(landed_ms);
  v.window(t, {low_jam(), low_jam()}, {0.0, 20}, 5); t += 150;        // inside the settle, clean
  REQUIRE(t - 150 == landed_ms + kHopSettleBlankMs);
  auto o = v.window(t, {low_jam(), low_jam()}, {0.04, 20}, 5); t += 150;   // starts at the settle's end
  for (int i = 2; i < det_n(); ++i, t += 150) o = v.window(t, {low_jam(), low_jam()}, {0.0, 20}, 4);
  CHECK(o.v == Verdict::Interfered); CHECK(o.evidence & kEvDetected);
}
// Each leg leaves a full ring that already reads detected before the clear
// (a 10 % reading majority with a loss window among them), so without the
// clear the next window would read detected too.
// Revert (reset() keeps the ring) / (new_session() keeps the ring): the
// matching leg reads detected.
TEST(reset_and_new_session_clear_the_detect_ring) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  for (int i = 0; i < det_n() - 1; ++i, t += 150) v.window(t, {low_jam(), low_jam()}, {i == 0 ? 0.04 : 0.0, 20}, 5);
  v.reset();
  auto o = v.window(t, {low_jam(), low_jam()}, {0.0, 20}, 5); t += 150;
  CHECK(!(o.evidence & kEvDetected));
  for (int i = 0; i < det_n() - 1; ++i, t += 150) v.window(t, {low_jam(), low_jam()}, {i == 0 ? 0.04 : 0.0, 20}, 5);
  v.new_session();
  o = v.window(t, {low_jam(), low_jam()}, {0.0, 20}, 5);
  CHECK(!(o.evidence & kEvDetected));
}
// The loss window freezes the references at the pre-demote rung; the ring is
// not yet full, so the next windows read healthy -- three of them would thaw
// the references and the detected window would re-freeze at the demoted
// rung, and the hop would restore that. The thaw waits while the ring holds
// a detection in the making (median >= detect_pct, a loss window inside).
// Revert (thaw on three healthy windows alone): ref_rung reads -1, then 3.
TEST(thaw_is_held_while_the_ring_is_detecting) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  v.new_session();
  v.window(t, {low_jam(), low_jam()}, {0.04, 20}, 5); t += 150;    // loss window: freeze at rung 5
  for (int i = 0; i < 3; ++i, t += 150) CHECK(v.window(t, {low_jam(), low_jam()}, {0.0, 20}, 3).v == Verdict::Healthy);   // ladder sank
  CHECK(v.ref_rung() == 5);                                        // held: foreign still >= detect_pct with a loss window in the ring
  auto o = v.window(t, {low_jam(), low_jam()}, {0.0, 20}, 3);
  CHECK(o.v == Verdict::Interfered); CHECK(o.ref_rung == 5);
}
// The hold needs the median too: in clean air (foreign 2) a loss window in
// the ring does not hold the thaw.
// Revert (hold on a loss window in the ring alone): ref_rung stays 5.
TEST(thaw_is_not_held_in_clean_air_with_a_loss_window_in_the_ring) {
  HopVerdict v(cfg(), BusyCfg{}, 2); double t = warm(v);
  v.new_session();
  v.window(t, {busy(37, 35), busy(37, 35)}, {0.04, 20}, 5); t += 150;
  CHECK(v.ref_rung() == 5);
  for (int i = 0; i < 3; ++i, t += 150) v.window(t, {busy(37, 35), busy(37, 35)}, {0.0, 20}, 3);
  CHECK(v.ref_rung() == -1);
}
TEST(trigger_evidence_names_both_routes) {
  CHECK(trigger_evidence(kEvBlocked)); CHECK(trigger_evidence(kEvDetected));
  CHECK(trigger_evidence(kEvBlocked | kEvDetected | kEvImpaired));
  CHECK(!trigger_evidence(kEvImpaired | kEvWeak | kEvOccupied));
}
