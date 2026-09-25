#include "hop_verdict.h"
#include "mtest.h"
using namespace maburgs;
static HopCfg cfg() { HopCfg c; return c; }   // spec defaults
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
  HopVerdict v(cfg(), 2); double t = warm(v);
  auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.005, 20}, 5);
  CHECK(o.v == Verdict::Healthy); CHECK(!o.trigger); CHECK(o.ref_rung == -1);
}
TEST(co_channel_802_11_neighbour_is_interfered_after_two_windows) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  // spike jam250c120: loss 4-8 %, foreign 237/s, FA 2/s, RSSI rose to -55, SNR 33
  auto o1 = v.window(t, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
  CHECK(o1.v == Verdict::Interfered); CHECK(!o1.trigger); CHECK(o1.ref_rung == 5);
  CHECK(o1.evidence & kEvImpaired); CHECK(o1.evidence & kEvContended); CHECK(!(o1.evidence & kEvRaised));
  auto o2 = v.window(t + 150, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 4);  // ladder demoted meanwhile
  CHECK(o2.trigger); CHECK(o2.ref_rung == 5);   // snapshot taken at the FIRST impaired window
}
TEST(o4_raised_floor_is_interfered_even_with_rssi_up) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  auto o = v.window(t, {card(-48, 30, 0, 744, 130), card(-50, 27, 0, 388, 130)}, {0.85, 5}, 0);
  CHECK(o.v == Verdict::Interfered); CHECK(o.evidence & kEvRaised); CHECK(!(o.evidence & kEvFading));
}
TEST(fade_is_fade_not_interfered) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  auto o = v.window(t, {card(-90, 8, 0, 0, 30), card(-86, 10, 0, 10, 5)}, {0.05, 90}, 1);
  CHECK(o.v == Verdict::Fade); CHECK(o.evidence & kEvWeak); CHECK(!o.trigger);
}
TEST(loss_with_no_domain_evidence_is_unknown) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.08, 100}, 5);
  CHECK(o.v == Verdict::Unknown); CHECK(!o.trigger);
}
TEST(off_channel_blocking_reads_as_raised) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  auto o = v.window(t, {card(-59, 28, 0, 209, 94), card(-59, 28, 0, 299, 130)}, {0.04, 70}, 4);
  CHECK(o.v == Verdict::Interfered); CHECK(o.evidence & kEvRaised);
}
TEST(recovered_rate_alone_marks_impaired_and_its_reference_freezes) {
  HopVerdict v(cfg(), 2); double t = warm(v);          // recovered mean 20/window
  auto o1 = v.window(t, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.0, 100}, 5);   // 5x
  CHECK(o1.evidence & kEvImpaired);
  for (int i = 1; i < 30; ++i) v.window(t + 150 * i, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.0, 100}, 0);
  auto o2 = v.window(t + 150 * 30, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.0, 100}, 0);
  CHECK(o2.evidence & kEvImpaired);   // frozen mean: 100 is still 5x the pre-onset 20
}
TEST(skipped_card_does_not_contribute_and_one_card_fade_covered_is_healthy) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  VerdictCardIn dead; dead.valid = false;
  auto o = v.window(t, {card(-61, 30, 0, 4), dead}, {0.004, 20}, 5);
  CHECK(o.v == Verdict::Healthy);
  auto o2 = v.window(t + 150, {card(-88, 9, 0, 4), card(-61, 30, 0, 4)}, {0.004, 20}, 5);
  CHECK(o2.v == Verdict::Healthy);   // best card is fine and the link is not impaired
}
TEST(references_thaw_after_three_healthy_windows) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  v.window(t, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
  CHECK(v.ref_rung() == 5);
  for (int i = 1; i <= 3; ++i) v.window(t + 150 * i, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 3);
  CHECK(v.ref_rung() == -1);
}
// Guard fix: a mid-dwell (invalid) card with no trailing RSSI history yet
// must not have its frozen reference fabricated from a stale rssi_dbm=0
// reading. Freeze while card 1 has never once been valid (so its history is
// empty), then let card 1 come back as the best card and check its fading
// evidence is not fabricated from a bogus 0 dBm reference.
TEST(frozen_reference_for_a_mid_dwell_card_is_not_fabricated_from_zero) {
  HopVerdict v(cfg(), 2);
  VerdictCardIn dead; dead.valid = false;
  double t = 0;
  // 5 s of clean windows -- card 1 hasn't dwelt on home once yet, so its
  // trailing RSSI history stays empty the whole time.
  for (int i = 0; i < 40; ++i, t += 150) v.window(t, {card(-61, 30, 0, 4), dead}, {0.0, 20}, 5);
  // Card 0 goes interfered: freezes references, including card 1's (still
  // invalid this window, still no history) -- it must not get ref_rssi = 0.
  auto o1 = v.window(t, {card(-55, 33, 237, 2), dead}, {0.06, 80}, 5);
  CHECK(o1.v == Verdict::Interfered);
  CHECK(v.ref_rung() == 5);
  // Card 1 now comes back as the best card (much stronger than card 0) with
  // a perfectly normal RSSI. If its frozen reference had been fabricated as
  // 0 dBm, this reading (well above 0) would spuriously read as "fading"
  // relative to that bogus reference -- it must not.
  auto o2 = v.window(t + 150, {card(-55, 33, 237, 2), card(-40, 30, 0, 4)}, {0.06, 80}, 5);
  CHECK(!(o2.evidence & kEvFading));
}
// weak takes priority over interfered evidence: a link that is weak but
// NOT currently fading (RSSI stable, just low) and also shows genuine
// jammer symptoms (contended) must still classify as Fade and must never
// trigger a hop -- the ladder owns weak links, not the hop.
TEST(weak_takes_priority_over_contended_when_not_fading) {
  HopVerdict v(cfg(), 2);
  double t = 0;
  // Warm at RSSI -80 / SNR 10, no loss: not impaired, so these windows are
  // Healthy and the per-card RSSI references build at -80 (not frozen).
  for (int i = 0; i < 40; ++i, t += 150) v.window(t, {card(-80, 10, 0, 4), card(-80, 10, 0, 4)}, {0.0, 20}, 5);
  // Now: loss 6% (> loss_pct 3) -> impaired. RSSI/SNR still -80/10 -> weak.
  // RSSI unchanged from the just-built -80 reference -> NOT fading.
  // foreign 237/s (> foreign_pps 50) -> contended.
  auto o = v.window(t, {card(-80, 10, 237, 2), card(-80, 10, 237, 2)}, {0.06, 20}, 5);
  CHECK(o.v == Verdict::Fade);
  CHECK(!o.trigger);
  CHECK(o.evidence & kEvImpaired);
  CHECK(o.evidence & kEvWeak);
  CHECK(o.evidence & kEvContended);
  CHECK(!(o.evidence & kEvFading));
}
// C1: every VerdictOut carries the wall-clock span its counter deltas were
// gathered over, because the consumer (main.cpp -> HopController) re-feeds
// a cached one on every ~10 ms control tick between 150 ms windows and has
// to be able to tell a fresh verdict from a pre-hop one.
TEST(every_window_carries_the_span_it_was_measured_over) {
  HopVerdict v(cfg(), 2);
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
// I4: the rung-store blank's arming edge (gs/src/hop_blank.h) -- the
// FIRST interfered window of a frozen episode, once and once only until a
// thaw. Deliberately not ref_frozen, which is keyed on `impaired` and so
// is set through fades and unknown windows too.
TEST(first_interfered_fires_once_per_frozen_episode) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  // A fade freezes the references but is not interference: no edge.
  auto f = v.window(t, {card(-90, 8, 0, 0, 30), card(-86, 10, 0, 10, 5)}, {0.05, 90}, 1);
  CHECK(f.v == Verdict::Fade && f.ref_frozen && !f.first_interfered);
  t += 150;
  auto o1 = v.window(t, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
  CHECK(o1.v == Verdict::Interfered && o1.first_interfered);
  t += 150;
  auto o2 = v.window(t, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
  CHECK(o2.v == Verdict::Interfered && !o2.first_interfered);   // same episode
  // A thaw re-arms it.
  for (int i = 1; i <= 3; ++i) v.window(t + 150 * i, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5);
  auto o3 = v.window(t + 600, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
  CHECK(o3.first_interfered);
  // ...and so does reset(), the other thaw rule.
  v.reset();
  auto o4 = v.window(t + 750, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
  CHECK(o4.first_interfered);
}
TEST(ref_frozen_marks_the_impaired_episode) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  CHECK(!v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5).ref_frozen);
  t += 150;
  auto o = v.window(t, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
  CHECK(o.v == Verdict::Interfered);
  CHECK(o.ref_frozen);                      // the first impaired window itself
  t += 150;
  CHECK(v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5).ref_frozen);
  t += 150;
  CHECK(v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5).ref_frozen);
  t += 150;   // third consecutive healthy window: thaw
  CHECK(!v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5).ref_frozen);
}
// reset() is spec section 2's other thaw rule ("or after a hop's verify
// window ends"). It had zero callers in the tree until main.cpp gained
// HopAction::VerifyPass; this pins what it has to clear, including the
// latched persistence window -- a trigger built from windows measured on
// the channel we have just left must not survive the hop.
TEST(reset_thaws_the_snapshot_and_clears_the_latched_trigger) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  auto o1 = v.window(t, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
  auto o2 = v.window(t + 150, {card(-55, 33, 237, 2), card(-55, 33, 237, 2)}, {0.06, 80}, 5);
  CHECK(o1.ref_frozen && o2.trigger && v.ref_rung() == 5);
  v.reset();
  CHECK(v.ref_rung() == -1);
  // One clean window on the NEW channel: no latched trigger, no frozen
  // reference, and `fading` measured against the trailing history rather
  // than the old channel's frozen median.
  auto o3 = v.window(t + 300, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5);
  CHECK(!o3.trigger && !o3.ref_frozen && o3.v == Verdict::Healthy);
}
// Bench 2026-09-15: main.cpp fed the aggregator's RAW EMAs (RSSI ~64,
// SNR ~66 half-dB) straight into rssi_dbm/snr_db, so `weak` (rssi < -78
// dBm && snr < 12 dB) could never trip and Fade was unreachable on
// hardware. The conversion lives next to the engine so scan.log V lines
// and flightreport's medians read in dBm/dB like ctl.log does.
TEST(raw_units_convert_to_dbm_and_db) {
  CHECK(rssi_raw_to_dbm(64.0) == -46.0);
  CHECK(snr_raw_to_db(66.0) == 33.0);
  CHECK(rssi_raw_to_dbm(0.0) == 0.0);   // "no frame heard yet" keeps the no-reference sentinel
}
TEST(converted_edge_of_range_reading_is_fade) {
  HopVerdict v(cfg(), 2); double t = warm(v);
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
// Run 1 of 2026-09-25: 80 % loss, no foreign/FA, strong signal, air 95 % busy.
// blocked_pct default is 50 (hw spike findings, docs/nhm-airtime-spike-
// findings-2026-09-25.md), not the spec's 30 -- 95-2=93 still clears it.
TEST(long_frame_jam_is_interfered_via_blocked) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, 1);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.80;
  maburgs::VerdictOut o;
  double t = 0;
  for (int i = 0; i < 3; ++i) o = v.window(t += 150, {busy_card(95, 2)}, bad, 0);
  CHECK(o.v == maburgs::Verdict::Interfered);
  CHECK(o.evidence & maburgs::kEvBlocked);
  CHECK(o.trigger);
}
TEST(busy_but_healthy_link_stays_healthy) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, 1);
  maburgs::VerdictLinkIn ok; ok.pre_fec_loss = 0.0;
  auto o = v.window(150, {busy_card(95, 2)}, ok, 0);
  CHECK(o.v == maburgs::Verdict::Healthy);
}
TEST(own_airtime_is_subtracted) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, 1);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.20;
  auto o = v.window(150, {busy_card(60, 45)}, bad, 0);   // 15 % foreign < 50
  CHECK(!(o.evidence & maburgs::kEvBlocked));
  CHECK(o.v == maburgs::Verdict::Unknown);
}
TEST(blocked_beats_fading_but_not_weak) {
  maburgs::HopCfg cfg;
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.5;
  {  // fading: RSSI 15 dB under its trailing reference (analog desense)
    maburgs::HopVerdict v(cfg, 1);
    maburgs::VerdictLinkIn ok;
    double t = 0;
    for (int i = 0; i < 10; ++i) v.window(t += 150, {busy_card(0, 2)}, ok, 0);
    auto c = busy_card(100, 2); c.rssi_dbm = -63;
    auto o = v.window(t += 150, {c}, bad, 0);
    CHECK(o.evidence & maburgs::kEvFading);
    CHECK(o.v == maburgs::Verdict::Interfered);
  }
  {  // weak: range edge
    maburgs::HopVerdict v(cfg, 1);
    auto c = busy_card(100, 2); c.rssi_dbm = -85; c.snr_db = 5;
    auto o = v.window(150, {c}, bad, 0);
    CHECK(o.v == maburgs::Verdict::Fade);
  }
}
TEST(no_busy_reading_is_todays_verdict) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, 1);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.8;
  auto o = v.window(150, {busy_card(100, 0, /*valid=*/false)}, bad, 0);
  CHECK(o.v == maburgs::Verdict::Unknown);
  CHECK(!(o.evidence & maburgs::kEvBlocked));
}
// Final-review fix wave 2026-09-25: blocked is the MINIMUM foreign-busy
// reading across cards that have one, not any-card's. Reverting to
// any-card makes weak_diversity_card_alone_is_not_blocked fail (it would
// see card A's foreign 65 alone and call Interfered).
TEST(weak_diversity_card_alone_is_not_blocked) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, 2);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.2;
  auto a = busy_card(70, 5);    // foreign 65 -- would clear blocked_pct(50) alone
  auto b = busy_card(70, 68);   // foreign 2 -- the same interferer, seen weakly
  auto o = v.window(150, {a, b}, bad, 0);
  CHECK(!(o.evidence & maburgs::kEvBlocked));
  CHECK(o.v != maburgs::Verdict::Interfered);
}
TEST(interferer_seen_by_both_cards_is_blocked) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, 2);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.2;
  auto a = busy_card(95, 2);   // foreign 93
  auto b = busy_card(95, 2);   // foreign 93
  auto o = v.window(150, {a, b}, bad, 0);
  CHECK(o.evidence & maburgs::kEvBlocked);
  CHECK(o.v == maburgs::Verdict::Interfered);
}
TEST(card_without_reading_does_not_veto) {
  maburgs::HopCfg cfg;
  maburgs::HopVerdict v(cfg, 2);
  maburgs::VerdictLinkIn bad; bad.pre_fec_loss = 0.2;
  auto a = busy_card(95, 2);                     // foreign 93, only reading card
  auto b = busy_card(100, 0, /*valid=*/false);   // no busy reading this window
  auto o = v.window(150, {a, b}, bad, 0);
  CHECK(o.evidence & maburgs::kEvBlocked);
  CHECK(o.v == maburgs::Verdict::Interfered);
}

// ---- Task 11 (b): a floor on the recovered-symbols impaired term --------
// Bench 2026-09-26 (session 0232): on a clean channel whose trailing
// recovered mean is ~0.2/window, 1-5 recovered symbols read "3x the
// reference" and marked the window impaired with 0 % loss -- which, next
// to 112's ambient FA background (`raised`), hopped a perfectly good link
// into a blocked channel. recovered_min (default 8) removes ~97 % of those.
// Revert (drop `&& recovered >= recovered_min` in hop_verdict.cpp):
// recovered_below_floor_is_not_impaired fails (impaired, not healthy).
static double warm_quiet(HopVerdict& v, double t = 0) {
  // trailing recovered mean ~0.2/window: one recovered symbol every 5th window
  for (int i = 0; i < 40; ++i, t += 150)
    v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, (uint32_t)(i % 5 == 0 ? 1 : 0)}, 5);
  return t;
}
TEST(recovered_below_floor_is_not_impaired) {
  HopVerdict v(cfg(), 2); double t = warm_quiet(v);
  auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 2}, 5);
  CHECK(!(o.evidence & kEvImpaired));
  CHECK(o.v == Verdict::Healthy);
}
// Revert (floor compared with > 12 instead of >=, or floor applied to the
// loss term too): the at/above-floor case must still be impaired.
TEST(recovered_at_floor_is_impaired) {
  HopVerdict v(cfg(), 2); double t = warm_quiet(v);
  auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 12}, 5);
  CHECK(o.evidence & kEvImpaired);
  HopVerdict v8(cfg(), 2); t = warm_quiet(v8);
  auto o8 = v8.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 8}, 5);
  CHECK(o8.evidence & kEvImpaired);   // exactly the floor counts
}
// Revert (treat 0 as "use the default"): the zero-floor window stays healthy.
TEST(recovered_min_zero_disables_floor) {
  HopCfg c = cfg(); c.verdict.recovered_min = 0;
  HopVerdict v(c, 2); double t = warm_quiet(v);
  auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 2}, 5);
  CHECK(o.evidence & kEvImpaired);
}

// ---- Task 11 (c): a starved window is impaired --------------------------
// Earlier bench run: the drone was completely starved by a jam (zero own
// frames on every card) and the verdict read `healthy` -- no frames means
// no loss. main.cpp now sets VerdictLinkIn::starved.
// Revert (drop `|| link.starved`): starved_and_blocked_is_interfered reads
// Healthy with no kEvStarved.
TEST(starved_and_blocked_is_interfered) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  VerdictLinkIn s; s.pre_fec_loss = 0.0; s.recovered = 0; s.starved = true;
  auto o = v.window(t, {busy_card(98, 0), busy_card(98, 0)}, s, 5);
  CHECK(o.v == Verdict::Interfered);
  CHECK(o.evidence & kEvStarved);
  CHECK(o.evidence & kEvBlocked);
  CHECK(o.evidence & kEvImpaired);
}
// Starved with nothing else to explain it falls through to Unknown, like
// any other unexplained impairment -- never a trigger on its own.
// Revert (classify starved as Interfered directly): this reads Interfered.
TEST(starved_alone_is_unknown) {
  HopVerdict v(cfg(), 2); double t = warm(v);
  VerdictLinkIn s; s.starved = true;
  VerdictOut o;
  for (int i = 0; i < 3; ++i) o = v.window(t + 150 * i, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, s, 5);
  CHECK(o.v == Verdict::Unknown);
  CHECK(o.evidence & kEvStarved);
  CHECK(!o.trigger);
}
// The reference-poisoning check the brief asks for: a starved window is
// impaired, so it freezes the references and pushes nothing into the
// trailing histories; after the starve ends, the recovered reference is
// still the pre-starve one.
TEST(starved_windows_do_not_feed_the_trailing_references) {
  HopVerdict v(cfg(), 2); double t = warm(v);   // recovered mean 20
  VerdictLinkIn s; s.starved = true;             // recovered 0 while starved
  for (int i = 0; i < 30; ++i, t += 150) {
    auto o = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, s, 5);
    CHECK(o.ref_frozen);
  }
  // 3 healthy windows thaw. 12 recovered is well under 3x the pre-starve
  // mean of 20 -- had the 30 zero-recovered starved windows been pushed,
  // the mean would sit near 2 and 12 would trip (it clears the floor of 8).
  // Revert (push histories regardless of frozen_): this window is impaired.
  for (int i = 0; i < 3; ++i, t += 150) v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 20}, 5);
  auto ok = v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, {0.0, 12}, 5);
  CHECK(!(ok.evidence & kEvImpaired));
}

// ---- Task 12 (e): an AU-rate collapse counts as starved -----------------
// Bench 2026-09-26 (GS session 0232): a long-frame jam let a trickle of own
// frames through (5-30/s vs ~3200/s), so the zero-own-frames rule missed
// it and the loss tracker had nothing to measure yet: the second window
// read `healthy` and broke the 2-of-3 persistence. The AU count against its
// trailing mean catches it.
static VerdictLinkIn au_link(uint32_t au) { VerdictLinkIn l; l.recovered = 20; l.au_count = au; return l; }
static double warm_au(HopVerdict& v, uint32_t au, double t = 0, int n = 10) {
  for (int i = 0; i < n; ++i, t += 150) v.window(t, {card(-61, 30, 0, 4), card(-61, 29, 0, 4)}, au_link(au), 5);
  return t;
}
// Revert (drop the au_count term from `starved`): Healthy, no kEvStarved.
TEST(au_collapse_is_starved) {
  HopVerdict v(cfg(), 2); double t = warm_au(v, 9);
  auto o = v.window(t, {busy_card(98, 0), busy_card(98, 0)}, au_link(1), 5);
  CHECK(o.evidence & kEvStarved);
  CHECK(o.evidence & kEvImpaired);
  CHECK(o.v == Verdict::Interfered);
}
// Revert (threshold 0.5 instead of starved_frac): 4 < 4.5 reads starved.
TEST(half_rate_is_not_starved) {
  HopVerdict v(cfg(), 2); double t = warm_au(v, 9);
  auto o = v.window(t, {busy_card(98, 0), busy_card(98, 0)}, au_link(4), 5);
  CHECK(!(o.evidence & kEvStarved));
  CHECK(o.v == Verdict::Healthy);
}
// No history (first window, or a trailing mean under 2 AUs): the AU rule
// has no baseline and stays silent; own frames are present so the
// zero-own-frames rule is not set either.
// Revert (drop `au_ref >= 2.0`): 0 < 0.25 * 0 is false anyway, so the
// second half pins it -- a mean of 1 with au_count 0 would read starved.
TEST(no_au_baseline_is_not_starved) {
  HopVerdict v(cfg(), 2);
  auto o = v.window(0, {busy_card(98, 0), busy_card(98, 0)}, au_link(0), 5);
  CHECK(!(o.evidence & kEvStarved));
  CHECK(o.v == Verdict::Healthy);
  HopVerdict w(cfg(), 2); double t = warm_au(w, 1);
  auto p = w.window(t, {busy_card(98, 0), busy_card(98, 0)}, au_link(0), 5);
  CHECK(!(p.evidence & kEvStarved));
}
// Revert (ignore starved_frac == 0): au 1 against 9 reads starved.
TEST(starved_frac_zero_disables_au_rule) {
  HopCfg c = cfg(); c.verdict.starved_frac = 0.0;
  HopVerdict v(c, 2); double t = warm_au(v, 9);
  auto o = v.window(t, {busy_card(98, 0), busy_card(98, 0)}, au_link(1), 5);
  CHECK(!(o.evidence & kEvStarved));
  CHECK(o.v == Verdict::Healthy);
}
// A starved window freezes the references; every later window of the
// episode compares against the pre-freeze AU rate, not a mean poisoned by
// the collapse itself. 30 windows at 1 AU would drag a live mean under the
// 2-AU baseline floor and silence the rule mid-jam.
// Revert (push au_hist_ regardless of frozen_ and read the live mean):
// the later windows lose kEvStarved.
TEST(frozen_au_reference_is_used_while_frozen) {
  HopVerdict v(cfg(), 2); double t = warm_au(v, 9, 0, 33);
  for (int i = 0; i < 30; ++i, t += 150) {
    auto o = v.window(t, {busy_card(98, 0), busy_card(98, 0)}, au_link(1), 5);
    CHECK(o.evidence & kEvStarved);
    CHECK(o.ref_frozen);
  }
  // reset() drops the frozen snapshot; the trailing history (still 9s)
  // survives, so the rule keeps its baseline.
  v.reset();
  auto r = v.window(t, {busy_card(98, 0), busy_card(98, 0)}, au_link(1), 5);
  CHECK(r.evidence & kEvStarved);
}
