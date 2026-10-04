// WebGs (web/src/web_gs.h): the web GS core. Pins spotter silence, the Gs
// rendezvous/RCF cadence, and the wiring into the shared units.
#include <algorithm>
#include <cstdio>
#include <memory>
#include <stdexcept>

#include "body_gen.h"
#include "channel_core.h"
#include "fake_link_card.h"
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
mabur::node::RxBody rc_body(std::vector<uint8_t> wire, uint64_t mono_us, uint8_t rx_ch = 0) {
  mabur::node::RxBody m;
  m.card_id = 0;
  m.mono_us = mono_us;
  m.crc_ok = true;
  m.phy_valid = true;
  m.rx_channel = rx_ch;
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
  WebGs g(cfg(), Mode::Spotter, 136, 40, {}, 0, io);
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

// The spotter's LOSS row with the width-only op (2026-09-30): the op no
// longer tracks the flying rung, so TransitionEdge fires once at start and
// never again. Pre-FEC must still read real erasures, post-FEC must stay a
// valid number, and a clean link must read ~0 -- bodies tagged at a real
// rate (mcs 4) the fixed op never names.
TEST(spotter_loss_row_reads_real_loss) {
  auto run = [](int drop_every) {
    Io io;
    io.on_au = [](Au&&) {};
    WebGs g(cfg(), Mode::Spotter, 136, 40, {}, 0, io);
    uint64_t t = 0;
    for (auto& b : gen_bodies(/*aus=*/600, /*dt_ms=*/16.0, drop_every)) {
      b.mcs = 4;
      t = b.mono_us;
      g.on_rx(b);
      g.tick(t);
    }
    return g.stats();
  };
  const auto lossy = run(7);
  REQUIRE(lossy.pre_fec_loss.has_value());
  REQUIRE(lossy.residual.has_value());
  CHECK(*lossy.pre_fec_loss > 0.05);                   // ~1 in 7 bodies gone
  const auto clean = run(0);
  REQUIRE(clean.pre_fec_loss.has_value());
  REQUIRE(clean.residual.has_value());
  CHECK(*clean.pre_fec_loss < 0.001);
  CHECK(*clean.residual < 0.001);
  const auto heavy = run(2);                           // half gone: past what FEC repairs
  REQUIRE(heavy.residual.has_value());
  CHECK(*heavy.residual > 0.0);
}

TEST(gs_beacons_then_rcf_after_ack) {
  std::vector<std::vector<uint8_t>> sent;
  Io io;
  io.on_au = [](Au&&) {};
  io.send = [&](const std::vector<uint8_t>& b) { sent.push_back(b); };
  auto c = cfg();
  WebGs g(c, Mode::Gs, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Gs, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Gs, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Gs, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Gs, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Gs, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Gs, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Spotter, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Gs, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Spotter, 136, 40, {}, 0, io);
  g.set_idr_requests(4);   // must not crash; there is no send path
  CHECK(g.vrx() == nullptr);
  CHECK(g.sends() == 0);
}

// A spotter's link setting is just the configured width (2026-09-30: the
// drone's applied-op echo left Telem). Its mcs readout comes off the air
// (spotter_mcs_is_base_stream_rx_mcs), never from a Telem.
TEST(spotter_op_is_configured_width_no_mcs) {
  Io io;
  io.on_au = [](Au&&) {};
  WebGs g(cfg(), Mode::Spotter, 136, 40, {}, 0, io);
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
  WebGs g20(cfg(), Mode::Spotter, 136, 20, {}, 0, io);
  CHECK(g20.stats().bw == 20);
}

// The spotter's MCS readout is read off the air: the RX-descriptor MCS of
// CRC-clean BASE-stream (sid 0) bodies only -- enh, probe (next rung's
// candidate MCS) and corrupt bodies never move it, and an unknown rate
// (255: legacy/VHT, or a relay frame without one) keeps the last value.
// Display only: the spotter's link-health op stays width-only.
TEST(spotter_mcs_is_base_stream_rx_mcs) {
  Io io;
  io.on_au = [](Au&&) {};
  WebGs g(cfg(), Mode::Spotter, 136, 40, {}, 0, io);
  CHECK(g.stats().mcs == -1);                          // nothing heard yet
  uint64_t t = 1'000'000;
  for (auto b : gen_bodies(20, 16.0, 0)) {
    const int sid = mabur::sbi_peek_stream_id(b.body.data(), b.body.size());
    b.mcs = sid == 0 ? 4 : 7;                          // enh at a different rate
    b.mono_us += t;
    g.on_rx(b);
  }
  CHECK(g.stats().mcs == 4);
  auto bodies = gen_bodies(2, 16.0, 0);
  mabur::node::RxBody base;
  for (auto& b : bodies)
    if (mabur::sbi_peek_stream_id(b.body.data(), b.body.size()) == 0) base = b;
  base.mcs = 2; base.crc_ok = false;                   // corrupt: ignored
  g.on_rx(base);
  CHECK(g.stats().mcs == 4);
  base.mcs = 255; base.crc_ok = true;                  // unknown rate: ignored
  g.on_rx(base);
  CHECK(g.stats().mcs == 4);
  base.mcs = 3;                                        // a real change lands
  g.on_rx(base);
  CHECK(g.stats().mcs == 3);
}

TEST(drone_temp_from_telem_in_stats_json) {
  Io io;
  io.on_au = [](Au&&) {};
  WebGs g(cfg(), Mode::Spotter, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Spotter, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Spotter, 136, 40, {}, 0, io);
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
    WebGs g(cfg(), Mode::Gs, 136, 40, {}, 0, io);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
  WebGs s(cfg(), Mode::Spotter, 136, 40, {}, 0, io);   // spotter needs no send
  CHECK(s.vrx() == nullptr);
}

TEST(channel_width_override_validation) {
  auto c = cfg();   // bundle ladder has 40 MHz rungs
  c.radio.channels = {136};   // the start channel must be a member (set checks: the test below)
  CHECK(!channel_width_error(c, Mode::Gs, 136, 40));
  CHECK(!channel_width_error(c, Mode::Spotter, 136, 40));
  CHECK(channel_width_error(c, Mode::Gs, 0, 20).has_value());
  CHECK(channel_width_error(c, Mode::Gs, 201, 20).has_value());
  CHECK(channel_width_error(c, Mode::Spotter, 136, 80).has_value());
  c.radio.channels = {165};
  auto e = channel_width_error(c, Mode::Spotter, 165, 40);   // no HT40 pair: the set check says so
  REQUIRE(e.has_value());
  CHECK(e->find("radio.channels") != std::string::npos && e->find("165") != std::string::npos);
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

TEST(relay_stats_fields_keeps_the_page_keys) {
  maburgs::RelayStatsIn r;
  r.state = 0; r.ch = 136; r.sec = 2; r.owned = true; r.frames = 1000; r.gaps = 3;
  r.your_drops = 4; r.tx = 20; r.tx_fail = 1; r.tx_refused = 2; r.reconnects = 0;
  r.you_own = true; r.rx_drops = 5; r.tx_drops = 6;
  const std::string s = relay_stats_fields(r);
  CHECK(s == ",\"radio\":\"relay\",\"relay_state\":0,\"relay_ch\":136,\"relay_sec\":2,"
             "\"relay_owned\":1,\"relay_you_own\":1,\"relay_frames\":1000,\"relay_gaps\":3,"
             "\"relay_rx_drops\":5,\"relay_tx_ring_drops\":6,\"relay_tx\":20,\"relay_tx_fail\":1,"
             "\"relay_tx_refused\":2,\"relay_your_drops\":4");
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
  WebGs g(cfg(), Mode::Spotter, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Gs, 136, 40, {}, 0, io);
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
  WebGs g(c, Mode::Spotter, 136, 40, {}, 0, io);
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
  WebGs g(cfg(), Mode::Spotter, 136, 40, {}, 0, io);
  const uint64_t t = 1'000'000;
  for (const auto& b : msp_bodies("X", t)) { g.on_rx(b); g.tick(t); }
  CHECK(g.stats().osd_screens == 1);
}

namespace {
// GS mode on fake USB cards, bundle config with the channel set, no core
// threads: each tick() runs one scout step; the scout's sleeps advance the
// WebGs clock (and, through Opts::sleep_hook, the rig's clock and the fake
// cards' energy clock with it). One card: `card` is the scout and opens at
// 20 MHz like maburgs's boot scout card. Two cards: `card` is the link card
// at radio.width, `card2` the scout at 20.
//
// No threads means the scout's beacon windows open and close inside one
// tick()'s scout step, so the send path (after the core tick) never sees
// the scout beaconing: a one-card rig never offers a DISC, and the scout
// card's own sends are always gated. Tests that need frames on the air use
// two cards (the link card's DISCs pass) or a session (RCFs to card 0).
struct GsRig {
  maburgs::Config c = cfg();
  FakeClock clk;
  FakeCard card, card2;
  std::vector<std::string> logs;
  std::vector<uint8_t> stored;
  std::vector<std::vector<uint8_t>> io_sends;
  std::unique_ptr<WebGs> g;
  uint64_t t = 1'000'000;
  uint64_t ticks_with_sent = 0;
  // Called from inside the scout's sleeps (mid-dwell / mid-beacon): where the
  // production core thread would be running the send path.
  std::function<void()> on_sleep;
  explicit GsRig(uint8_t start = 40, bool pinned = false, int n_usb = 1) {
    c.radio.channels = {40, 64, 112, 144};
    c.radio.width = 40;
    if (pinned) c.radio.pin = start;
    card.ch = start; card.width_mhz = n_usb == 1 ? 20 : 40;
    card2.ch = start; card2.width_mhz = 20;
    card.clk = card2.clk = &clk;
    clk.ms = t / 1000;
    Io io;
    io.on_au = [](Au&&) {};
    io.send = [this](const std::vector<uint8_t>& b) { io_sends.push_back(b); };
    io.on_log = [this](const std::string& l) { logs.push_back(l); };
    io.on_channel_store = [this](uint8_t ch) { stored.push_back(ch); };
    io.on_control_tick = [this](double, const maburgs::LinkHealth&, int,
                                const std::vector<uint8_t>* sent) { if (sent) ++ticks_with_sent; };
    Opts o; o.core_threads = false;
    o.sleep_hook = [this](int ms) {
      clk.ms += static_cast<uint64_t>(ms);
      t += static_cast<uint64_t>(ms) * 1000;
      if (on_sleep && g) on_sleep();
    };
    std::vector<maburgs::LinkCard*> roster{&card};
    if (n_usb == 2) roster.push_back(&card2);
    g = std::make_unique<WebGs>(c, Mode::Gs, start, 40, roster, n_usb, io, o);
  }
  GsRig(const GsRig&) = delete;
  GsRig& operator=(const GsRig&) = delete;
  bool has_log(const std::string& needle) const {
    for (auto& l : logs) if (l.find(needle) != std::string::npos) return true;
    return false;
  }
  void ticks(int n, int step_ms = 10) {
    for (int i = 0; i < n; ++i) {
      t += static_cast<uint64_t>(step_ms) * 1000;
      clk.ms = t / 1000;
      g->tick(t);
    }
  }
};
}  // namespace

TEST(gs_roster_builds_core_and_one_card_prelude_commits_and_stores) {
  GsRig r;
  r.card.cca_per_ms_on[40] = 5;              // the start pair is the busy one
  REQUIRE(r.g->channel_core() != nullptr);
  CHECK(r.g->stats().chan.has_value());
  CHECK(r.g->stats().chan->scan_state == "scouting");
  for (int i = 0; i < 800 && !r.has_log("one-card prelude ranking picks"); ++i) r.ticks(1);
  REQUIRE(r.has_log("one-card prelude ranking picks"));
  // the prelude committed away from 40: the card was retuned and CHANNEL fired
  bool retuned = false;
  for (auto& call : r.card.calls) retuned = retuned || call.rfind("retune", 0) == 0;
  CHECK(retuned);
  REQUIRE(!r.stored.empty());
  CHECK(r.stored.back() != 40);
  CHECK(r.stored.back() == r.g->channel_core()->op());
  // "channel" is the card's LIVE channel: with one card that can be a scout
  // dwell channel rather than op -- spec'd, and what the page shows.
  CHECK(r.g->stats().channel == r.card.ch);
  CHECK(stats_json(r.g->stats()).find("\"channel\":" + std::to_string(r.card.ch)) != std::string::npos);
}

TEST(gs_roster_sends_through_the_card_not_io_send) {
  GsRig r(40, /*pinned=*/true, /*n_usb=*/2);   // card 0 = the link card on op
  r.ticks(60);                                // beaconing
  CHECK(r.io_sends.empty());
  CHECK(!r.card.sent.empty());                // DISCs went to the card
  CHECK(r.g->sends() == r.card.sent.size() + r.card2.sent.size());
  for (auto& s : r.card.sent) CHECK(mabur::rc::frame_type(s.data(), s.size()) == mabur::rc::T_DISC);
}

// Review Focus 3: an RCF due while the scout owns the card off a beacon is
// dropped by may_send(); rcf_sent and the RTT estimator never see it.
TEST(send_gate_drops_rcf_and_skips_rtt_stamp) {
  GsRig r;                                    // auto, one card: the scout owns it (pick open)
  r.ticks(1);
  r.g->inject_disc_ack_for_replay(r.t);       // SESSION: RCFs are due, all to card 0
  REQUIRE(r.g->stats().session);
  const uint64_t gated_before = r.g->channel_core()->scout_gated_sends();
  const auto seq_before = r.g->vrx()->rcf_seq();
  r.ticks(3);                                 // RCFs offered while the scout is not beaconing
  REQUIRE(r.g->stats().session);
  CHECK(r.g->vrx()->rcf_seq() != seq_before);  // RCFs were built...
  CHECK(r.g->channel_core()->scout_gated_sends() > gated_before);   // ...and gated
  CHECK(r.card.sent.empty());
  CHECK(r.ticks_with_sent == 0);   // plan 2 review: a gated frame is not reported as `sent`
  CHECK(r.g->stats().rcf_sent == 0);
  CHECK(r.g->stats().sends == r.card.sent.size());   // sends counts frames handed to the card only
}

TEST(stats_json_carries_channel_fields_gs_and_spotter) {
  // Two cards so card 0 (the stats card) stays on the pin and "channel" is
  // deterministic; the one-card live-channel case is in the prelude test.
  GsRig r(40, true, 2);
  r.ticks(2);
  const std::string js = stats_json(r.g->stats());
  CHECK(js.find("\"channel\":40") != std::string::npos);
  CHECK(js.find("\"scan_state\":\"off\"") != std::string::npos);     // pinned
  CHECK(js.find("\"scan_pick\":40") != std::string::npos);
  CHECK(js.find("\"hop\":{") != std::string::npos);
  CHECK(js.find("\"state\":\"idle\"") != std::string::npos);
  CHECK(js.find("\"target\":null") != std::string::npos);
  // spotter with a card: channel from the card, no core fields
  FakeCard sc; sc.ch = 64; sc.width_mhz = 40;
  Io io; io.on_au = [](Au&&) {};
  WebGs s(cfg(), Mode::Spotter, 64, 40, {&sc}, 1, io);
  const std::string sj = stats_json(s.stats());
  CHECK(s.channel_core() == nullptr);
  CHECK(sj.find("\"channel\":64") != std::string::npos);
  CHECK(sj.find("\"scan_state\":null") != std::string::npos);
  CHECK(sj.find("\"hop\":null") != std::string::npos);
}

TEST(channel_width_error_checks_the_set_membership_and_pin) {
  auto c = cfg();
  c.radio.channels = {40, 64, 112, 144};
  CHECK(!channel_width_error(c, Mode::Gs, 40, 40).has_value());
  CHECK(!channel_width_error(c, Mode::Spotter, 144, 20).has_value());
  // Review Focus 2: a non-member start
  auto e = channel_width_error(c, Mode::Gs, 136, 40);
  REQUIRE(e.has_value());
  CHECK(e->find("136") != std::string::npos && e->find("member") != std::string::npos);
  // Review Focus 1: pinned, --ch is a stale remembered member
  c.radio.pin = 40;
  e = channel_width_error(c, Mode::Gs, 64, 40);
  REQUIRE(e.has_value());
  CHECK(e->find("pinned to 40") != std::string::npos);
  CHECK(!channel_width_error(c, Mode::Gs, 40, 40).has_value());
  // the set itself vs the PAGE width: 40 needs pairs on one offset
  c.radio.pin.reset();
  c.radio.channels = {40, 44};                // 44 is HT40-, 40 is HT40+
  e = channel_width_error(c, Mode::Gs, 40, 40);
  REQUIRE(e.has_value());
  CHECK(e->find("radio.channels") != std::string::npos);
  CHECK(!channel_width_error(c, Mode::Spotter, 40, 20).has_value());   // fine at 20 (Spotter: the bundle ladder's 40 MHz rungs refuse a 20 MHz GS)
}

// The page's own shape: ONE card, owned by the scout. Its frames leave only
// while the scout beacons (disc_targets(0) == {0} and may_send(0) passes);
// without threads that window lives inside the scout step, so the gate is
// sampled from the scout's sleeps (test_channel_core.cpp's gate_obs idiom),
// where the production core thread would be sending. Each sample builds the
// DISC copy the send path would hand the card and checks it proposes the
// channel the card is on.
TEST(one_card_frames_pass_to_the_card_while_the_scout_beacons) {
  GsRig r;
  r.card.cca_per_ms_on[40] = 5;
  struct Obs { std::vector<int> targets; bool pass; uint8_t ch; bool proposes_ch; };
  std::vector<Obs> obs;
  r.on_sleep = [&] {
    const auto* core = r.g->channel_core();
    Obs o;
    o.targets = core->disc_targets(0);
    if (o.targets.empty()) return;            // not beaconing: the send path offers nothing
    o.pass = core->may_send(0);
    o.ch = r.card.ch;
    mabur::rc::Disc d;
    d.op_channel = 0;
    const auto wire = core->disc_for_card(mabur::rc::pack_disc(d, r.c.link.key), 0);
    const auto back = mabur::rc::parse_disc(wire.data(), wire.size());
    o.proposes_ch = back && back->op_channel == o.ch;
    obs.push_back(o);
  };
  for (int i = 0; i < 800 && !r.has_log("one-card prelude ranking picks"); ++i) r.ticks(1);
  REQUIRE(r.has_log("one-card prelude ranking picks"));
  r.ticks(20);
  REQUIRE(!obs.empty());
  bool on_op = false, on_other = false;
  for (const auto& o : obs) {
    CHECK(o.targets == std::vector<int>{0});  // the one card, never Io::send
    CHECK(o.pass);                            // the gate lets a beaconing card send
    CHECK(o.proposes_ch);                     // a DISC proposes the channel it is sent on
    (o.ch == r.g->channel_core()->op() ? on_op : on_other) = true;
  }
  CHECK(on_op);                               // the one-card op window
  CHECK(on_other);                            // a search burst on another member
  CHECK(r.io_sends.empty());
  CHECK(r.g->sends() == r.card.sent.size());
}

namespace {
// Spotter mode on one fake card at the link width, the bundle config with
// the channel set. The follower runs on tick(); the card is the fake, so a
// retune is synchronous unless retune_deferred (relay-style).
struct SpotRig {
  maburgs::Config c = cfg();
  FakeCard card;
  std::unique_ptr<WebGs> g;
  uint64_t t = 1'000'000;
  int aus = 0;
  explicit SpotRig(uint8_t start = 40) {
    c.radio.channels = {40, 64, 112, 144};
    c.radio.width = 40;
    card.ch = start; card.width_mhz = 40;
    Io io;
    io.on_au = [this](Au&&) { ++aus; };
    g = std::make_unique<WebGs>(c, Mode::Spotter, start, 40, std::vector<maburgs::LinkCard*>{&card}, 1, io);
  }
  SpotRig(const SpotRig&) = delete;
  SpotRig& operator=(const SpotRig&) = delete;
  void ticks(int n, int step_ms = 10) {
    for (int i = 0; i < n; ++i) { t += static_cast<uint64_t>(step_ms) * 1000; g->tick(t); }
  }
  // One video body heard on `ch` (a CRC-good canonical body).
  void frame_on(uint8_t ch) {
    auto b = gen_bodies(1, 16.0, 0).front();
    b.mono_us = t;
    b.rx_channel = ch;
    g->on_rx(b);
  }
  // The real GS's RCF heard on rx_ch, ordering hop_ch/epoch (tagged with the
  // default key: the spotter never checks it, spec §6.2).
  void rcf_on(uint8_t rx_ch, uint8_t hop_ch, uint8_t epoch) {
    mabur::rc::Rcf r;
    r.hop_ch = hop_ch;
    r.hop_epoch = epoch;
    g->on_rx(rc_body(mabur::rc::pack_rcf(r), t, rx_ch));
  }
  int retunes() const {
    int n = 0;
    for (auto& s : card.calls) n += s.rfind("retune ", 0) == 0;
    return n;
  }
  std::string state() const { return g->stats().follow_state.value_or("none"); }
};
}  // namespace

TEST(spotter_roster_sweeps_from_start_and_retunes_the_card) {
  SpotRig r(40);
  CHECK(r.g->channel_core() == nullptr);          // no core in spotter mode
  CHECK(r.state() == "sweeping");
  r.ticks(1);                                     // on 40 already: no retune, the dwell starts
  CHECK(r.retunes() == 0);
  r.ticks(15);                                    // 150 ms, nothing heard
  CHECK(r.card.ch == 64);
  CHECK(r.card.calls.back() == "retune 64");
  r.ticks(15);
  CHECK(r.card.ch == 112);
  CHECK(r.g->stats().channel == 112);             // the card's live channel
  CHECK(r.g->stats().follows == 0);
  CHECK(r.g->sends() == 0);
}

TEST(spotter_locks_on_a_frame_follows_an_rcf_and_confirms) {
  SpotRig r(40);
  r.ticks(1);
  r.frame_on(40);
  CHECK(r.state() == "locked");
  r.rcf_on(40, 64, 1);
  CHECK(r.state() == "following");
  CHECK(r.retunes() == 0);                        // the card moves on the next tick
  r.ticks(1);
  CHECK(r.card.ch == 64);
  CHECK(r.card.calls.back() == "retune 64");
  r.frame_on(64);
  CHECK(r.state() == "locked");
  CHECK(r.g->stats().channel == 64);
  CHECK(r.g->stats().follows == 1);
  // DISC (any GS frame) refreshes the lock, no transmit in response
  mabur::rc::Disc d;
  r.ticks(90);                                    // 900 ms quiet
  r.g->on_rx(rc_body(mabur::rc::pack_disc(d), r.t, 64));
  r.ticks(50);                                    // 500 ms more: < silence_ms since the DISC
  CHECK(r.state() == "locked");
  CHECK(r.g->sends() == 0);
  // the move left the video path alone: AUs keep coming on 64
  const int before = r.aus;
  for (auto& b : gen_bodies(5, 16.0, 0)) { b.mono_us = (r.t += 16000); b.rx_channel = 64; r.g->on_rx(b); r.g->tick(r.t); }
  CHECK(r.aus > before);
}

TEST(spotter_follow_timeout_returns_the_card) {
  SpotRig r(40);
  r.ticks(1);
  r.frame_on(40);
  r.rcf_on(40, 64, 1);
  r.ticks(1);
  REQUIRE(r.card.ch == 64);
  r.ticks(201);                                   // > confirm_ms with nothing on 64
  CHECK(r.state() == "locked");
  CHECK(r.card.ch == 40);
  CHECK(r.card.calls.back() == "retune 40");
}

// Review Focus 4 (glue half): the retune is re-issued every tick until the
// card takes it (a RadioFrontend refuses pre-ready).
TEST(spotter_retune_is_retried_until_the_card_takes_it) {
  SpotRig r(40);
  r.ticks(16);                                    // dwell over: wants 64
  r.card.retune_ok = false;
  r.card.ch = 40;                                 // the fake refused: still on 40
  const int n0 = r.retunes();
  r.ticks(3);
  CHECK(r.retunes() == n0 + 3);
  r.card.retune_ok = true;
  r.ticks(1);
  CHECK(r.card.ch == 64);
}

TEST(spotter_relay_style_card_dwell_counts_from_ready) {
  SpotRig r(40);
  r.card.retune_deferred = true;
  r.ticks(16);                                    // -> retune 64, card not ready yet
  REQUIRE(r.card.ch == 64);
  REQUIRE(!r.card.is_ready);
  const int n = r.retunes();
  r.ticks(30);                                    // 300 ms: no second retune, no advance (dwell not started)
  CHECK(r.retunes() == n);
  CHECK(r.card.ch == 64);
  r.card.is_ready = true;                         // STATUS: tuned
  r.ticks(1);                                     // reported
  r.ticks(14);
  CHECK(r.card.ch == 64);
  r.ticks(1);                                     // 150 ms on the member
  CHECK(r.card.ch == 112);
}

TEST(stats_json_carries_follow_fields_for_spotter_only) {
  SpotRig r(40);
  std::string js = stats_json(r.g->stats());
  CHECK(js.find("\"follow_state\":\"sweeping\"") != std::string::npos);
  CHECK(js.find("\"follows\":0") != std::string::npos);
  CHECK(js.find("\"scan_state\":null") != std::string::npos);
  // GS mode: null
  GsRig g(40, true, 2);
  g.ticks(1);
  js = stats_json(g.g->stats());
  CHECK(js.find("\"follow_state\":null") != std::string::npos);
  CHECK(js.find("\"follows\":null") != std::string::npos);
  // spotter without a roster (replay): no follower, null
  Io io; io.on_au = [](Au&&) {};
  WebGs s(cfg(), Mode::Spotter, 136, 40, {}, 0, io);
  js = stats_json(s.stats());
  CHECK(js.find("\"follow_state\":null") != std::string::npos);
  CHECK(!s.stats().follow_state.has_value());
}
