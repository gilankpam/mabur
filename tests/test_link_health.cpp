// LinkHealthAssembler (gs/src/link_health.h): the ladder's per-tick input,
// extracted from maburgs main.cpp 2026-09-27 so the web GS feeds the SAME
// windows. These pin the wiring the main loop is otherwise unreachable for.
#include <cmath>

#include "aggregator.h"
#include "body_gen.h"
#include "link_health.h"
#include "mabur/probe_wire.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"
#include "mtest.h"
using namespace maburgs;

namespace {
std::array<mabur::UepLayerCfg, 2> layers() { return gen_layers(); }

// A probe body for the given profile/enh_fid/seq, first seen at t_ms. Parses
// with parse_probe_body(..., 14 + 332, ...) -- the ENH layer's geometry.
mabur::node::RxBody make_probe_body(uint8_t profile, uint16_t enh_fid,
                                    uint32_t seq, double t_ms) {
  mabur::probe::ProbeHdr h;
  h.seq = seq;
  h.profile = profile;
  h.enh_fid = enh_fid;
  mabur::node::RxBody m;
  m.body = mabur::probe::build_probe_body(h, 4, 14 + 332);
  m.mono_us = static_cast<uint64_t>(t_ms * 1000.0);
  m.crc_ok = true;
  m.phy_valid = true;
  m.snr[0] = m.snr[1] = 40;
  return m;
}
}  // namespace

TEST(clean_video_is_valid_zero_loss) {
  Aggregator agg(layers(), 512, 1, 192);
  LinkHealthAssembler a({1, 4, 14 + 332});
  double t = feed_video(agg, 60, 0.0, 16.0);
  LinkHealthInputs in;  // default op, no probe
  auto k = a.tick(t, agg, in);
  CHECK(k.health.sample_valid);
  CHECK(k.health.pre_fec_loss == 0.0);
  CHECK(!k.health.video_starved);
  CHECK(!k.probe_tail_ms);
}

TEST(lossy_video_reads_pre_fec_loss) {
  Aggregator agg(layers(), 512, 1, 192);
  LinkHealthAssembler a({1, 4, 14 + 332});
  LinkHealthInputs in;
  a.tick(0.0, agg, in);                 // first tick marks the edge (settle)
  double t = feed_video(agg, 120, 200.0, 16.0, /*drop_every=*/10);
  auto k = a.tick(t, agg, in);
  CHECK(k.health.sample_valid);
  CHECK(k.health.pre_fec_loss > 0.05 && k.health.pre_fec_loss < 0.2);
}

TEST(starved_when_no_packets_since_last_step_but_video_seen) {
  Aggregator agg(layers(), 512, 1, 192);
  LinkHealthAssembler a({1, 4, 14 + 332});
  LinkHealthInputs in;
  double t = feed_video(agg, 30, 0.0, 16.0);
  auto k1 = a.tick(t, agg, in);
  CHECK(!k1.health.video_starved);
  a.on_step_sent(agg);                  // window boundary at the RCF
  auto k2 = a.tick(t + 100.0, agg, in); // no new packets out
  CHECK(k2.health.video_starved);
}

TEST(no_video_ever_is_not_starved) {
  Aggregator agg(layers(), 512, 1, 192);
  LinkHealthAssembler a({1, 4, 14 + 332});
  auto k = a.tick(1000.0, agg, LinkHealthInputs{});
  CHECK(!k.health.video_starved);
  CHECK(!k.health.sample_valid);
}

TEST(probe_profile_edge_returns_tail_and_blanks) {
  Aggregator agg(layers(), 512, 1, 192);
  LinkHealthAssembler a({1, 4, 14 + 332});
  LinkHealthInputs in;
  in.probe_profile = mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 4, 20);
  in.probe_rung = 2;
  auto k = a.tick(0.0, agg, in);
  REQUIRE(k.probe_tail_ms.has_value());
  CHECK(*k.probe_tail_ms >= 1);
  CHECK(a.probe_commanded() == in.probe_profile);
  auto k2 = a.tick(10.0, agg, in);      // same profile: no edge
  CHECK(!k2.probe_tail_ms);
  in.probe_profile = mabur::rc::kNoProbeProfile;
  auto k3 = a.tick(20.0, agg, in);      // edge to none: tail 0
  REQUIRE(k3.probe_tail_ms.has_value());
  CHECK(*k3.probe_tail_ms == 0);
  CHECK(k3.health.probe_rung == 2);     // passed through from inputs

  // The blank: fill the probe window at profile P until it is valid, then
  // switch to P2 -- the window must read invalid for the 150 ms settle
  // blank, not carry P's sample across the edge.
  in.probe_profile = mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 4, 20);
  double t = 100.0;
  a.tick(t, agg, in);                   // edge back to P
  bool valid_before = false;
  for (uint16_t fid = 1; fid <= 120; ++fid) {
    t += 16.0;
    a.on_au_begin(1, fid, t);
    a.on_probe_body(0, make_probe_body(in.probe_profile, fid, fid, t));
    valid_before = a.tick(t, agg, in).health.probe_valid;
  }
  REQUIRE(valid_before);
  in.probe_profile = mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 5, 20);
  auto e1 = a.tick(t + 1.0, agg, in);   // the edge
  REQUIRE(e1.probe_tail_ms.has_value());
  CHECK(!e1.health.probe_valid);
  CHECK(!a.tick(t + 100.0, agg, in).health.probe_valid);   // still inside 150 ms
}

TEST(probe_finalized_is_per_tick) {
  // Bodies finalize ~100 ms after first sight; the assembler drains
  // ProbeTrack every tick so an absent log never grows it without bound.
  Aggregator agg(layers(), 512, 1, 192);
  LinkHealthAssembler a({1, 4, 14 + 332});
  LinkHealthInputs in;
  in.probe_profile = mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 4, 20);
  a.tick(0.0, agg, in);
  a.on_au_begin(1, 7, 10.0);
  a.on_probe_body(0, make_probe_body(in.probe_profile, /*enh_fid=*/7, /*seq=*/1, 10.0));
  a.tick(50.0, agg, in);
  CHECK(a.probe_finalized().empty());   // not finalized yet
  a.tick(200.0, agg, in);
  CHECK(a.probe_finalized().size() == 1);
  a.tick(210.0, agg, in);
  CHECK(a.probe_finalized().empty());   // drained: rows are per tick
}

TEST(rf_labels_nan_without_fresh_frames) {
  Aggregator agg(layers(), 512, 1, 192);
  LinkHealthAssembler a({1, 4, 14 + 332});
  LinkHealthInputs in;
  double t = feed_video(agg, 30, 0.0, 16.0);   // bodies with phy_valid + snr
  auto k1 = a.tick(t, agg, in);
  CHECK(!std::isnan(k1.health.rf_snr_db));
  a.on_step_sent(agg);                          // snapshot pool frames
  auto k2 = a.tick(t + 100.0, agg, in);         // no new frames
  CHECK(std::isnan(k2.health.rf_snr_db));
  CHECK(std::isnan(k2.health.rf_rssi_dbm));
}

TEST(residual_settle_blank_on_op_change) {
  // TransitionEdge is owned by the assembler: an op change must blank
  // s1_resid_cur for kResidSettleMs (test_transition_edge pins the edge
  // itself; this pins that the assembler routes its own windows into it).
  Aggregator agg(layers(), 512, 1, 192);
  LinkHealthAssembler a({1, 4, 14 + 332});
  LinkHealthInputs in;
  in.op.mcs = 4;
  a.tick(0.0, agg, in);                 // first tick: the boot edge's settle
  // Clean first half (units complete, so the session-start stale booking
  // is behind us), then drop every other body: heavy enough to abandon.
  // 400 AUs so the lossy half runs well past the 512-symbol horizon --
  // abandonment only books when a seq falls off it. Built on gen_bodies
  // directly: feed_video's drop_every would lose from the first body.
  const auto bodies = gen_bodies(400, 16.0, 0);
  for (size_t i = 0; i < bodies.size(); ++i) {
    if (i >= bodies.size() / 2 && i % 2 == 0) continue;
    auto m = bodies[i];
    m.mono_us += 200000;
    agg.on_rx_body(m);
  }
  const double t = 200.0 + 399 * 16.0;
  auto k0 = a.tick(t, agg, in);
  REQUIRE(k0.health.residual_loss > 0.0);       // not vacuous: loss is booked
  in.op.mcs = 3;                                // demote edge
  auto k = a.tick(t + 1.0, agg, in);
  CHECK(k.health.residual_loss == 0.0);         // blanked, not old-rung debris
}

TEST(probe_row_snr_is_nan_for_a_card_without_real_snr) {
  Aggregator agg(layers(), 512, 2, 192);
  LinkHealthAssembler a({2, 4, 14 + 332, {true, false}});   // card 1 = relay
  LinkHealthInputs in;
  in.probe_profile = mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 4, 20);
  a.tick(0.0, agg, in);
  a.on_au_begin(1, 7, 10.0);
  a.on_probe_body(0, make_probe_body(in.probe_profile, 7, 1, 10.0));
  a.on_probe_body(1, make_probe_body(in.probe_profile, 7, 1, 10.0));
  a.tick(200.0, agg, in);
  REQUIRE(a.probe_finalized().size() == 1);
  const auto& row = a.probe_finalized()[0];
  CHECK(row.snr_db[0] > 19.0 && row.snr_db[0] < 21.0);   // 40 raw half-dB = 20 dB
  CHECK(std::isnan(row.snr_db[1]));
}

MTEST_MAIN
