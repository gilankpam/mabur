// WebGs (web/src/web_gs.h): the web GS core. Pins spotter silence, the Gs
// rendezvous/RCF cadence, and the wiring into the shared units.
#include <algorithm>
#include <stdexcept>

#include "body_gen.h"
#include "mtest.h"
#include "web_gs.h"
#include "mabur/msp_dp.h"
#include "mabur/msp_source.h"
#include "osd_screen.h"
using namespace webgs;

namespace {
maburgs::Config cfg() {
  return maburgs::load_config(MABUR_SOURCE_DIR "/gs/bundle/maburgs.default.toml");
}
// Feed a fresh gen_bodies run (new drone encoder: new seqs, frame_ids from 0)
// shifted to start at t0_us, ticking after every body. Returns the last stamp.
uint64_t feed(WebGs& g, int n_aus, uint64_t t0_us, int drop_every = 0) {
  uint64_t t = t0_us;
  for (auto& b : gen_bodies(n_aus, 16.0, drop_every)) {
    b.mono_us += t0_us;
    t = b.mono_us;
    g.on_rx(b);
    g.tick(t);
  }
  return t;
}
mabur::node::RxBody rc_body(std::vector<uint8_t> wire, uint64_t mono_us) {
  mabur::node::RxBody m;
  m.card_id = 0;
  m.mono_us = mono_us;
  m.crc_ok = true;
  m.phy_valid = true;
  m.body = std::move(wire);
  return m;
}
mabur::node::RxBody telem_body(uint16_t tlm_seq, uint64_t mono_us) {
  mabur::rc::Telem t;
  t.tlm_seq = tlm_seq;
  return rc_body(mabur::rc::pack_telem(t), mono_us);
}
// One full DisplayPort screen: CLEAR, optional SET_OPTIONS (hd_option),
// `text` at row 0 col 0, then DRAW_SCREEN unless `finish` is false.
std::vector<uint8_t> osd_blob(const std::string& text, int hd_option = -1, bool finish = true) {
  std::vector<uint8_t> s;
  const uint8_t clr = mabur::MSP_DP_CLEAR;
  mabur::msp_append_message(s, mabur::MSP_CMD_DISPLAYPORT, &clr, 1);
  if (hd_option >= 0) {
    const uint8_t opt[3] = {mabur::MSP_DP_SET_OPTIONS, 0, static_cast<uint8_t>(hd_option)};
    mabur::msp_append_message(s, mabur::MSP_CMD_DISPLAYPORT, opt, 3);
  }
  std::vector<uint8_t> ds = {mabur::MSP_DP_DRAW_STRING, 0, 0, 0};
  for (char ch : text) ds.push_back(static_cast<uint8_t>(ch));
  mabur::msp_append_message(s, mabur::MSP_CMD_DISPLAYPORT, ds.data(), ds.size());
  if (finish) {
    const uint8_t scr = mabur::MSP_DP_DRAW_SCREEN;
    mabur::msp_append_message(s, mabur::MSP_CMD_DISPLAYPORT, &scr, 1);
  }
  return s;
}
struct Pub {
  int rows = 0, cols = 0;
  std::vector<uint16_t> cells;
};
OsdScreen::PublishFn collect(std::vector<Pub>& out) {
  return [&out](int r, int c, const uint16_t* p) {
    out.push_back({r, c, std::vector<uint16_t>(p, p + static_cast<size_t>(r) * c)});
  };
}
// The drone's MSP path for one screen: MspSource (defaults 1312/16 == the
// bundle's [msp]) -> SBI/FEC bodies stamped at mono_us.
std::vector<mabur::node::RxBody> msp_bodies(const std::string& text, uint64_t mono_us) {
  std::vector<mabur::node::RxBody> out;
  mabur::MspSource src(mabur::MspSourceCfg{}, [&](const uint8_t* b, size_t n) {
    out.push_back(rc_body(std::vector<uint8_t>(b, b + n), mono_us));
  });
  const auto blob = osd_blob(text);
  src.on_serial_bytes(blob.data(), blob.size(), mono_us / 1000);
  return out;
}
}  // namespace

TEST(spotter_never_sends_over_lossy_replay) {
  int aus = 0;
  Io io;
  io.on_au = [&](Au&&) { ++aus; };
  bool called = false;
  io.send = [&](const std::vector<uint8_t>&) { called = true; };
  WebGs g(cfg(), Mode::Spotter, 136, 40, io);
  CHECK(g.vrx() == nullptr);
  uint64_t t = 0;
  for (auto& b : gen_bodies(/*aus=*/600, /*dt_ms=*/16.0, /*drop_every=*/7)) {
    t = b.mono_us;
    g.on_rx(b);
    g.tick(t);
  }
  for (int i = 0; i < 400; ++i) g.tick(t += 10000);   // 4 s idle: no keep-alive
  CHECK(!called);
  CHECK(g.sends() == 0);
  CHECK(aus > 500);
}

TEST(gs_beacons_then_rcf_after_ack) {
  std::vector<std::vector<uint8_t>> sent;
  Io io;
  io.on_au = [](Au&&) {};
  io.send = [&](const std::vector<uint8_t>& b) { sent.push_back(b); };
  auto c = cfg();
  WebGs g(c, Mode::Gs, 136, 40, io);
  uint64_t t = 1'000'000;
  for (int i = 0; i < 50; ++i) g.tick(t += 10000);    // 500 ms, no drone
  REQUIRE(!sent.empty());
  for (auto& s : sent) CHECK(mabur::rc::frame_type(s.data(), s.size()) == mabur::rc::T_DISC);
  // Beacons are sends but not RCFs: "RCF heard %" divides the drone's
  // RCF-only rcf_rx by rcf_sent, never by sends.
  CHECK(g.stats().sends == sent.size());
  CHECK(g.stats().rcf_sent == 0);
  const uint64_t sends_before = g.stats().sends;
  g.inject_disc_ack_for_replay(t);
  sent.clear();
  auto bodies = gen_bodies(120, 16.0, 0);
  for (auto& b : bodies) { b.mono_us += t; g.on_rx(b); g.tick(b.mono_us); }
  int rcf = 0;
  for (auto& s : sent) rcf += mabur::rc::frame_type(s.data(), s.size()) == mabur::rc::T_RCF;
  // ~1.9 s at link.feedback_ms: at least half the nominal count gets out
  const int nominal = static_cast<int>(1900 / c.link.feedback_ms);
  CHECK(rcf >= nominal / 2);
  CHECK(g.stats().session);
  CHECK(g.stats().rcf_sent == static_cast<uint64_t>(rcf));
  CHECK(g.stats().sends == sends_before + sent.size());
  CHECK(g.stats().rcf_sent < g.stats().sends);
  CHECK(stats_json(g.stats()).find("\"rcf_sent\":" + std::to_string(rcf)) != std::string::npos);
}

TEST(probe_expectation_wired_from_frame_stream) {
  // AU begin must reach LinkHealthAssembler::on_au_begin: with a probe
  // commanded, every video AU books bpb expected blocks after finalize.
  // Gs mode after ack, ladder below top -> probe commanded.
  Io io;
  io.on_au = [](Au&&) {};
  io.send = [](const std::vector<uint8_t>&) {};
  WebGs g(cfg(), Mode::Gs, 136, 40, io);
  uint64_t t = 1'000'000;
  g.inject_disc_ack_for_replay(t);
  for (auto& b : gen_bodies(300, 16.0, 0)) { b.mono_us += t; g.on_rx(b); g.tick(b.mono_us); }
  // The ladder starts at rung 0, so a probe is always commanded here.
  REQUIRE(g.health().probe_commanded() != mabur::rc::kNoProbeProfile);
  CHECK(g.health().probe_track().union_counts().expected_blocks > 0);
}

TEST(cap_to_complete_basic_and_wrap) {
  // pts clock = GS-mono + off. Captured at mono 1000 us -> pts = 1000 + off.
  const int64_t off = 5'000'000;
  CHECK(cap_to_complete_us(static_cast<uint32_t>(1000 + off), 41'000, off) == 40'000);
  // pts wrapped: capture just before 2^32 in pts space, complete after.
  const int64_t off2 = (int64_t{1} << 32) - 20'000;   // pts = mono + off2
  const uint64_t cap_mono = 10'000;                   // pts = 2^32 - 10'000 (pre-wrap)
  const uint32_t pts = static_cast<uint32_t>(cap_mono + off2);
  CHECK(cap_to_complete_us(pts, cap_mono + 35'000, off2) == 35'000);
}

TEST(no_cap_without_offset) {
  Au last;
  Io io;
  io.send = [](const std::vector<uint8_t>&) {};
  int n = 0;
  io.on_au = [&](Au&& a) { ++n; last = std::move(a); };
  WebGs g(cfg(), Mode::Gs, 136, 40, io);
  uint64_t t = 1'000'000;
  g.inject_disc_ack_for_replay(t);
  g.tick(t);
  feed(g, 30, t);
  CHECK(n > 0);
  CHECK(!last.cap_to_complete_us.has_value());   // no Telem -> no RTT offset
}

TEST(stats_json_has_mode_and_nulls) {
  Stats s;
  s.mode = Mode::Spotter;
  const std::string j = stats_json(s);
  CHECK(j.find("\"mode\":\"spotter\"") != std::string::npos);
  CHECK(j.find("\"rtt_ms\":null") != std::string::npos);
  CHECK(j.find('\n') == std::string::npos);
}

TEST(cap_to_complete_from_telem_offset) {
  // A Telem echoing a sent RCF's seq gives the RttEstimator an RTT and a pts
  // offset; AUs completed afterwards carry cap_to_complete_us.
  std::vector<uint8_t> last_sent;
  uint64_t last_sent_us = 0;
  Au last;
  int n = 0;
  Io io;
  io.on_au = [&](Au&& a) { ++n; last = std::move(a); };
  io.send = [](const std::vector<uint8_t>&) {};
  io.on_control_tick = [&](double now_ms, const maburgs::LinkHealth&, int,
                           const std::vector<uint8_t>* sent) {
    if (sent && mabur::rc::frame_type(sent->data(), sent->size()) == mabur::rc::T_RCF) {
      last_sent = *sent;
      last_sent_us = static_cast<uint64_t>(now_ms) * 1000;
    }
  };
  WebGs g(cfg(), Mode::Gs, 136, 40, io);
  // One drone run: 90 AUs, the Telem lands after AU 59. The drone's pts is
  // gen's 0-based t_ms, so pts = mono - t0: offset = -t0 (mod 2^32).
  const uint64_t t0 = 1'000'000;
  g.inject_disc_ack_for_replay(t0);
  g.tick(t0);
  auto bodies = gen_bodies(90, 16.0, 0);
  const uint64_t split_us = 60 * 16'000;
  size_t i = 0;
  uint64_t t = t0;
  for (; i < bodies.size() && bodies[i].mono_us < split_us; ++i) {
    bodies[i].mono_us += t0;
    t = bodies[i].mono_us;
    g.on_rx(bodies[i]);
    g.tick(t);
  }
  CHECK(n > 50);
  REQUIRE(!last_sent.empty());
  const auto rcf = mabur::rc::parse_rcf(last_sent.data(), last_sent.size());
  REQUIRE(rcf.has_value());
  // Telem 4 ms after the send, aged 0: rtt ~4 ms.
  const uint64_t rx_us = last_sent_us + 4000;
  const int64_t off = (int64_t{1} << 32) - static_cast<int64_t>(t0);
  const int64_t rtt_us = static_cast<int64_t>(rx_us - last_sent_us);
  mabur::rc::Telem tm;
  tm.tlm_seq = 1;
  tm.flags = 0x08;
  tm.rcf_seq_echo = rcf->seq;
  tm.rcf_age_ms = 0;
  tm.pts_at_build = static_cast<uint64_t>(static_cast<int64_t>(rx_us) + off - rtt_us / 2);
  g.on_rx(rc_body(mabur::rc::pack_telem(tm), rx_us));
  g.tick(std::max(t, rx_us));
  REQUIRE(g.stats().pts_off_us.has_value());
  CHECK(g.stats().rtt_ms.has_value());
  n = 0;
  last = Au{};
  for (; i < bodies.size(); ++i) {
    bodies[i].mono_us += t0;
    g.on_rx(bodies[i]);
    g.tick(bodies[i].mono_us);
  }
  CHECK(n > 25);
  REQUIRE(last.cap_to_complete_us.has_value());
  // Bodies land at their capture stamp, so capture->complete is ~0 (+/- the
  // ms quantization of the send stamp and the RTT/2 split).
  CHECK(*last.cap_to_complete_us > -3000);
  CHECK(*last.cap_to_complete_us < 50'000);
}

TEST(gs_no_video_before_ack) {
  // maburgs parity: FrameStream is fed only while in SESSION with a peer
  // that advertised CAP_FRAME_WIRE.
  int n = 0;
  Io io;
  io.on_au = [&](Au&&) { ++n; };
  io.send = [](const std::vector<uint8_t>&) {};
  WebGs g(cfg(), Mode::Gs, 136, 40, io);
  uint64_t t = feed(g, 60, 1'000'000);
  CHECK(n == 0);
  g.inject_disc_ack_for_replay(t);
  g.tick(t);
  feed(g, 60, t + 16'000);
  CHECK(n > 50);
}

TEST(gs_session_loss_then_reack_resets_and_flows) {
  int n = 0;
  Io io;
  io.on_au = [&](Au&&) { ++n; };
  io.send = [](const std::vector<uint8_t>&) {};
  WebGs g(cfg(), Mode::Gs, 136, 40, io);
  uint64_t t = 1'000'000;
  g.inject_disc_ack_for_replay(t);
  g.tick(t);
  t = feed(g, 60, t);
  CHECK(n > 50);
  const uint64_t r0 = g.resets();
  for (int i = 0; i < 150; ++i) g.tick(t += 10000);   // 1.5 s silence: session lost
  CHECK(g.vrx()->link_state() == maburgs::VrxState::BEACONING);
  CHECK(g.resets() == r0 + 1);                        // leaving session resets
  g.inject_disc_ack_for_replay(t);
  g.tick(t);
  CHECK(g.resets() == r0 + 2);                        // new session resets
  n = 0;
  feed(g, 60, t + 16'000);                            // new encoder: seqs/ids restart
  CHECK(n > 50);
}

namespace {
// Last RCF the core sent after feeding n AUs from t0.
std::vector<uint8_t> last_rcf_after(WebGs& g, std::vector<uint8_t>& last, int n, uint64_t t0) {
  last.clear();
  feed(g, n, t0);
  return last;
}
}  // namespace

TEST(vtx_rec_wish_reaches_rcf_byte) {
  std::vector<uint8_t> last;
  Io io;
  io.on_au = [](Au&&) {};
  io.send = [](const std::vector<uint8_t>&) {};
  io.on_control_tick = [&](double, const maburgs::LinkHealth&, int,
                           const std::vector<uint8_t>* sent) {
    if (sent && mabur::rc::frame_type(sent->data(), sent->size()) == mabur::rc::T_RCF)
      last = *sent;
  };
  WebGs g(cfg(), Mode::Gs, 136, 40, io);
  const uint64_t t0 = 1'000'000;
  g.inject_disc_ack_for_replay(t0);
  g.tick(t0);

  auto r0 = last_rcf_after(g, last, 30, t0);
  REQUIRE(!r0.empty());
  CHECK(mabur::rc::parse_rcf(r0.data(), r0.size())->rec == 0);   // never pressed: unknown

  g.set_vtx_rec(true);
  auto r1 = last_rcf_after(g, last, 30, t0 + 1'000'000);
  REQUIRE(!r1.empty());
  CHECK(mabur::rc::parse_rcf(r1.data(), r1.size())->rec ==
        (mabur::rc::kRecKnown | mabur::rc::kRecOn));

  g.set_vtx_rec(false);
  auto r2 = last_rcf_after(g, last, 30, t0 + 2'000'000);
  REQUIRE(!r2.empty());
  CHECK(mabur::rc::parse_rcf(r2.data(), r2.size())->rec == mabur::rc::kRecKnown);
}

TEST(vtx_rec_wish_is_noop_in_spotter) {
  Io io;
  io.on_au = [](Au&&) {};
  WebGs g(cfg(), Mode::Spotter, 136, 40, io);
  g.set_vtx_rec(true);   // must not crash; there is no send path
  CHECK(g.vrx() == nullptr);
  CHECK(g.sends() == 0);
}

TEST(idr_requests_reach_rcf_epoch_byte) {
  std::vector<uint8_t> last;
  Io io;
  io.on_au = [](Au&&) {};
  io.send = [](const std::vector<uint8_t>&) {};
  io.on_control_tick = [&](double, const maburgs::LinkHealth&, int,
                           const std::vector<uint8_t>* sent) {
    if (sent && mabur::rc::frame_type(sent->data(), sent->size()) == mabur::rc::T_RCF)
      last = *sent;
  };
  WebGs g(cfg(), Mode::Gs, 136, 40, io);
  const uint64_t t0 = 1'000'000;
  g.inject_disc_ack_for_replay(t0);
  g.tick(t0);
  auto r0 = last_rcf_after(g, last, 30, t0);
  REQUIRE(!r0.empty());
  CHECK(mabur::rc::parse_rcf(r0.data(), r0.size())->idr_epoch == 0);

  g.set_idr_requests(3);
  auto r1 = last_rcf_after(g, last, 30, t0 + 1'000'000);
  REQUIRE(!r1.empty());
  CHECK(mabur::rc::parse_rcf(r1.data(), r1.size())->idr_epoch == 3);

  g.set_idr_requests(256 + 5);                 // count wraps into the byte
  auto r2 = last_rcf_after(g, last, 30, t0 + 2'000'000);
  REQUIRE(!r2.empty());
  CHECK(mabur::rc::parse_rcf(r2.data(), r2.size())->idr_epoch == 5);
  CHECK(g.stats().idr_req == 261u);
  CHECK(stats_json(g.stats()).find("\"idr_req\":261") != std::string::npos);
}

TEST(idr_requests_are_noop_in_spotter) {
  Io io;
  io.on_au = [](Au&&) {};
  WebGs g(cfg(), Mode::Spotter, 136, 40, io);
  g.set_idr_requests(4);   // must not crash; there is no send path
  CHECK(g.vrx() == nullptr);
  CHECK(g.sends() == 0);
}

// A spotter's link setting is just the configured width (2026-09-30: the
// drone's applied-op echo left Telem). mcs is not tracked, and a Telem
// arriving must not change either.
TEST(spotter_op_is_configured_width_no_mcs) {
  Io io;
  io.on_au = [](Au&&) {};
  WebGs g(cfg(), Mode::Spotter, 136, 40, io);
  g.tick(1'000'000);
  CHECK(g.stats().bw == 40);
  CHECK(g.stats().mcs == -1);
  mabur::rc::Telem t;
  t.tlm_seq = 1;
  g.on_rx(rc_body(mabur::rc::pack_telem(t), 1'000'000));
  g.tick(1'000'000);
  CHECK(g.stats().bw == 40);
  CHECK(g.stats().mcs == -1);
  CHECK(stats_json(g.stats()).find("drone_idr_gs") == std::string::npos);
  WebGs g20(cfg(), Mode::Spotter, 136, 20, io);
  CHECK(g20.stats().bw == 20);
}

TEST(drone_temp_from_telem_in_stats_json) {
  Io io;
  io.on_au = [](Au&&) {};
  WebGs g(cfg(), Mode::Spotter, 136, 40, io);
  CHECK(stats_json(g.stats()).find("\"drone_temp_c\":null") != std::string::npos);
  mabur::rc::Telem t;
  t.tlm_seq = 1;   // soc_temp_c defaults to -128 = unavailable
  g.on_rx(rc_body(mabur::rc::pack_telem(t), 1'000'000));
  g.tick(1'000'000);
  CHECK(stats_json(g.stats()).find("\"drone_temp_c\":null") != std::string::npos);
  t.tlm_seq = 2;
  t.soc_temp_c = 67;
  g.on_rx(rc_body(mabur::rc::pack_telem(t), 2'000'000));
  g.tick(2'000'000);
  CHECK(stats_json(g.stats()).find("\"drone_temp_c\":67") != std::string::npos);
}

TEST(rec_status_from_telem_in_stats_json) {
  Io io;
  io.on_au = [](Au&&) {};
  WebGs g(cfg(), Mode::Spotter, 136, 40, io);
  CHECK(!g.stats().rec_status.has_value());
  CHECK(stats_json(g.stats()).find("\"rec_state\":null") != std::string::npos);
  mabur::rc::Telem t;
  t.tlm_seq = 1;
  t.rec_status = static_cast<uint8_t>(2 | (5 << 2));   // Error, LowSpace
  g.on_rx(rc_body(mabur::rc::pack_telem(t), 1'000'000));
  g.tick(1'000'000);
  REQUIRE(g.stats().rec_status.has_value());
  const std::string j = stats_json(g.stats());
  CHECK(j.find("\"rec_state\":2") != std::string::npos);
  CHECK(j.find("\"rec_err\":5") != std::string::npos);
}

TEST(spotter_drone_restart_resets_and_flows) {
  int n = 0;
  Io io;
  io.on_au = [&](Au&&) { ++n; };
  WebGs g(cfg(), Mode::Spotter, 136, 40, io);
  uint64_t t = 1'000'000;
  g.on_rx(telem_body(5000, t));
  t = feed(g, 60, t);
  g.on_rx(telem_body(5001, t));
  CHECK(n > 50);
  CHECK(g.resets() == 0);
  t += 100'000;
  g.on_rx(telem_body(0, t));                           // maburd restarted
  g.tick(t);
  CHECK(g.resets() == 1);
  n = 0;
  feed(g, 60, t + 16'000);
  CHECK(n > 50);
  CHECK(g.sends() == 0);
}

TEST(gs_without_send_throws) {
  Io io;
  bool threw = false;
  try {
    WebGs g(cfg(), Mode::Gs, 136, 40, io);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
  WebGs s(cfg(), Mode::Spotter, 136, 40, io);   // spotter needs no send
  CHECK(s.vrx() == nullptr);
}

TEST(channel_width_override_validation) {
  const auto c = cfg();   // bundle: ch 136 / 40, ladder has 40 MHz rungs
  CHECK(!channel_width_error(c, Mode::Gs, 136, 40));
  CHECK(!channel_width_error(c, Mode::Spotter, 136, 40));
  CHECK(channel_width_error(c, Mode::Gs, 0, 20).has_value());
  CHECK(channel_width_error(c, Mode::Gs, 201, 20).has_value());
  CHECK(channel_width_error(c, Mode::Spotter, 136, 80).has_value());
  auto e = channel_width_error(c, Mode::Spotter, 165, 40);   // no HT40 pair
  REQUIRE(e.has_value());
  CHECK(e->find("radio.width") != std::string::npos && e->find("165") != std::string::npos);
  // GS commands the ladder: a 40 MHz rung while tuned 20 is refused...
  bool has40 = false;
  for (const auto& r : c.link.ladder_cfg.ladder) has40 |= r.bw == 40;
  REQUIRE(has40);
  auto g = channel_width_error(c, Mode::Gs, 165, 20);
  REQUIRE(g.has_value());
  CHECK(g->find("link.ladder[") != std::string::npos);
  // ...a spotter only listens, so 20 is fine.
  CHECK(!channel_width_error(c, Mode::Spotter, 165, 20));
}

MTEST_MAIN

TEST(osd_screen_publishes_one_snapshot) {
  std::vector<Pub> pubs;
  OsdScreen o(collect(pubs));
  const auto b = osd_blob("HELLO");
  o.feed(b.data(), b.size(), 1000);
  REQUIRE(pubs.size() == 1);
  CHECK(pubs[0].rows == 18);
  CHECK(pubs[0].cols == 50);
  CHECK(pubs[0].cells[0] == 'H');
  CHECK(pubs[0].cells[4] == 'O');
  CHECK(pubs[0].cells[5] == 0);
  CHECK(o.screens() == 1);
}

TEST(osd_screen_holds_inside_interval_latest_wins) {
  std::vector<Pub> pubs;
  OsdScreen o(collect(pubs));
  const auto a = osd_blob("AAA"), b = osd_blob("BBB"), c = osd_blob("CCC");
  o.feed(a.data(), a.size(), 1000);
  o.feed(b.data(), b.size(), 1010);
  o.feed(c.data(), c.size(), 1020);
  CHECK(pubs.size() == 1);
  o.tick(1029);
  CHECK(pubs.size() == 1);
  o.tick(1030);
  REQUIRE(pubs.size() == 2);
  CHECK(pubs[1].cells[0] == 'C');   // latest wins, B never shown
  o.tick(2000);
  CHECK(pubs.size() == 2);          // nothing pending: no re-publish
  CHECK(o.screens() == 2);
}

TEST(osd_screen_ignores_garbage_and_unfinished) {
  std::vector<Pub> pubs;
  OsdScreen o(collect(pubs));
  const std::string junk = "not msp at all $M< garbage";
  o.feed(reinterpret_cast<const uint8_t*>(junk.data()), junk.size(), 1000);
  const auto part = osd_blob("XYZ", -1, /*finish=*/false);
  o.feed(part.data(), part.size(), 1100);
  o.tick(5000);
  CHECK(pubs.empty());
  CHECK(o.screens() == 0);
}

TEST(osd_screen_held_copy_survives_later_partial) {
  // A held screen is the copy taken at its DRAW_SCREEN: a later unfinished
  // snapshot (CLEAR + DRAW_STRING, no DRAW_SCREEN) must not leak into it.
  std::vector<Pub> pubs;
  OsdScreen o(collect(pubs));
  const auto a = osd_blob("AAA"), b = osd_blob("BBB"), z = osd_blob("ZZZ", -1, false);
  o.feed(a.data(), a.size(), 1000);
  o.feed(b.data(), b.size(), 1010);   // held
  o.feed(z.data(), z.size(), 1015);   // unfinished
  o.tick(1040);
  REQUIRE(pubs.size() == 2);
  CHECK(pubs[1].cells[0] == 'B');
}

TEST(osd_screen_sd_canvas_from_set_options) {
  std::vector<Pub> pubs;
  OsdScreen o(collect(pubs));
  const auto sd = osd_blob("SD", /*hd_option=*/0);   // msp_hd_options_e 0 = SD 30x16
  o.feed(sd.data(), sd.size(), 1000);
  REQUIRE(pubs.size() == 1);
  CHECK(pubs[0].rows == 16);
  CHECK(pubs[0].cols == 30);
  CHECK(pubs[0].cells.size() == 480);
  CHECK(pubs[0].cells[1] == 'D');
  const auto hd = osd_blob("HD", /*hd_option=*/1);   // back to HD 50x18
  o.feed(hd.data(), hd.size(), 1100);
  REQUIRE(pubs.size() == 2);
  CHECK(pubs[1].rows == 18);
  CHECK(pubs[1].cols == 50);
}

TEST(spotter_msp_bodies_reach_on_osd) {
  std::vector<Pub> pubs;
  Io io;
  io.on_au = [](Au&&) {};
  io.on_osd = collect(pubs);
  WebGs g(cfg(), Mode::Spotter, 136, 40, io);
  const uint64_t t = 1'000'000;
  const auto bodies = msp_bodies("HELLO", t);
  REQUIRE(!bodies.empty());
  for (const auto& b : bodies) { g.on_rx(b); g.tick(t); }
  REQUIRE(pubs.size() == 1);
  CHECK(pubs[0].rows == 18);
  CHECK(pubs[0].cells[0] == 'H');
  CHECK(g.stats().osd_snaps == 1);
  CHECK(g.stats().osd_screens == 1);
  CHECK(g.sends() == 0);   // OSD never opens a send path in Spotter
  const auto j = stats_json(g.stats());
  CHECK(j.find("\"osd_snaps\":1") != std::string::npos);
  CHECK(j.find("\"osd_screens\":1") != std::string::npos);
}

TEST(gs_mode_shows_osd_before_session) {
  // MSP is independent of the video gate: the OSD shows while rendezvous
  // is still beaconing.
  std::vector<Pub> pubs;
  Io io;
  io.on_au = [](Au&&) {};
  io.send = [](const std::vector<uint8_t>&) {};
  io.on_osd = collect(pubs);
  WebGs g(cfg(), Mode::Gs, 136, 40, io);
  const uint64_t t = 1'000'000;
  for (const auto& b : msp_bodies("GS", t)) { g.on_rx(b); g.tick(t); }
  CHECK(!g.stats().peer_acked);   // no DISC_ACK yet: rendezvous starts in SESSION, so peer_acked is the gate
  REQUIRE(pubs.size() == 1);
  CHECK(pubs[0].cells[1] == 'S');
}

TEST(msp_disabled_publishes_nothing) {
  std::vector<Pub> pubs;
  Io io;
  io.on_au = [](Au&&) {};
  io.on_osd = collect(pubs);
  auto c = cfg();
  c.msp.enable = false;
  WebGs g(c, Mode::Spotter, 136, 40, io);
  const uint64_t t = 1'000'000;
  for (const auto& b : msp_bodies("OFF", t)) { g.on_rx(b); g.tick(t); }
  CHECK(pubs.empty());
  CHECK(g.stats().osd_snaps == 0);
  CHECK(g.stats().osd_screens == 0);
}

TEST(osd_without_on_osd_is_counted_not_crashing) {
  // Replay/Node builds leave Io::on_osd unset.
  Io io;
  io.on_au = [](Au&&) {};
  WebGs g(cfg(), Mode::Spotter, 136, 40, io);
  const uint64_t t = 1'000'000;
  for (const auto& b : msp_bodies("X", t)) { g.on_rx(b); g.tick(t); }
  CHECK(g.stats().osd_screens == 1);
}
