// WebGs (web/src/web_gs.h): the web GS core. Pins spotter silence, the Gs
// rendezvous/RCF cadence, and the wiring into the shared units.
#include "body_gen.h"
#include "mtest.h"
#include "web_gs.h"
using namespace webgs;

namespace {
maburgs::Config cfg() {
  return maburgs::load_config(MABUR_SOURCE_DIR "/gs/bundle/maburgs.default.toml");
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
  if (g.health().probe_commanded() != mabur::rc::kNoProbeProfile)
    CHECK(g.health().probe_track().union_counts().expected_blocks > 0);
  else
    CHECK(g.vrx()->ctl().probe_rung() < 0);   // top rung: nothing to probe
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
  io.on_au = [&](Au&& a) { last = std::move(a); };
  io.send = [](const std::vector<uint8_t>&) {};
  WebGs g(cfg(), Mode::Gs, 136, 40, io);
  for (auto& b : gen_bodies(30, 16.0, 0)) { g.on_rx(b); g.tick(b.mono_us); }
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

MTEST_MAIN
