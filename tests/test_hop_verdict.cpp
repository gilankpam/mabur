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
