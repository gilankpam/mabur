#include "mtest.h"
#include "vrx_controller.h"
#include "mabur/rc_proto.h"
using namespace maburgs;

static VrxController make() {
  static LinkTable lt;
  VrxCfg cfg;
  cfg.vtx_id = 1;
  return VrxController(lt, cfg);
}

// Drive video at 1 kHz and step at 10 ms; classify emissions per second.
TEST(rcf_pacing_and_keepalive_disc) {
  auto vrx = make();
  int rcf = 0, disc = 0;
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  for (int t = 0; t < 5000; t += 10) {
    const double now = t;
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(t / 10), now);
    if (auto out = vrx.step(now, ld, 0.0)) {
      const int ft = mabur::rc::frame_type(out->frame.data(), out->frame.size());
      if (ft == mabur::rc::T_RCF) { CHECK(!out->is_disc); ++rcf; }
      else if (ft == mabur::rc::T_DISC) { CHECK(out->is_disc); ++disc; }
    }
  }
  CHECK(rcf >= 40 && rcf <= 50);   // ~10 Hz for 5 s, minus keepalive slots
  CHECK(disc >= 4 && disc <= 6);   // keep-alive ~1 Hz (fix a)
}

TEST(rcf_fields_are_correct) {
  auto vrx = make();
  std::array<uint8_t, 4> ld{100, 98, 80, 10};
  vrx.on_video(-55.0, 25.0, false, 500, 0.0);
  std::optional<VrxController::Out> out;
  double now = 0;
  while (!out || out->is_disc) {         // skip a leading keepalive DISC
    now += 10;
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(500 + now / 10), now);
    out = vrx.step(now, ld, 0.05);
  }
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(r->vtx_id == 1);
  CHECK(r->ack_seq >= 500);
  REQUIRE(r->layer_delivery.size() == 4);
  CHECK(r->layer_delivery[2] == 80);
  CHECK(r->pwr_offset_biased ==
        mabur::rc::encode_pwr_offset_qdb(vrx.cur_op().pwr_offset_qdb));
  CHECK(r->fec_overhead_16ths ==
        mabur::rc::overhead_to_16ths(vrx.cur_op().overhead));
}

TEST(silence_beacons_fast_and_recovers) {
  auto vrx = make();
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  vrx.on_video(-55.0, 25.0, false, 1, 0.0);
  // 2 s of silence: BEACONING at the 20 ms cadence.
  int discs = 0;
  for (double now = 1200; now < 2200; now += 10)
    if (auto out = vrx.step(now, ld, std::nullopt)) {
      CHECK(out->is_disc);
      ++discs;
    }
  CHECK(vrx.link_state() == VrxState::BEACONING);
  CHECK(discs >= 45);                       // ~50 in 1 s at 20 ms pacing
  // Failsafe op point while blind:
  CHECK(vrx.cur_op().mcs == 0);
  CHECK(vrx.cur_op().pwr_offset_qdb == 0);
  // Video returns -> SESSION and RCFs resume.
  vrx.on_video(-55.0, 25.0, false, 900, 2500.0);
  CHECK(vrx.link_state() == VrxState::SESSION);
}

TEST(disc_ack_feeds_rendezvous) {
  auto vrx = make();
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  vrx.step(1500, ld, std::nullopt);          // silence -> BEACONING
  CHECK(vrx.link_state() == VrxState::BEACONING);
  mabur::rc::DiscAck ack;
  ack.vtx_id = 1;
  ack.vrx_nonce = static_cast<uint32_t>((1ull * 2654435761ull) & 0xFFFFFFFFull);
  auto wire = mabur::rc::pack_disc_ack(ack);
  vrx.on_rc_frame(wire.data(), wire.size(), 1600);
  CHECK(vrx.link_state() == VrxState::SESSION);
}

// peer_caps() surfaces the most recently accepted DiscAck's chip_caps (0
// before any accept), so main.cpp's core loop can gate the frame-wire tail
// on the peer's advertised CAP_FRAME_WIRE bit (Task 10).
TEST(peer_caps_captured_from_disc_ack) {
  auto vrx = make();
  CHECK(vrx.peer_caps() == 0);
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  vrx.step(1500, ld, std::nullopt);          // silence -> BEACONING
  mabur::rc::DiscAck ack;
  ack.vtx_id = 1;
  ack.vrx_nonce = static_cast<uint32_t>((1ull * 2654435761ull) & 0xFFFFFFFFull);
  ack.chip_caps = mabur::rc::CAP_FRAME_WIRE;
  auto wire = mabur::rc::pack_disc_ack(ack);
  vrx.on_rc_frame(wire.data(), wire.size(), 1600);
  CHECK(vrx.link_state() == VrxState::SESSION);
  CHECK(vrx.peer_caps() & mabur::rc::CAP_FRAME_WIRE);
}

// peer_acked() separates "no DiscAck yet" from "peer advertised caps == 0".
// Both read peer_caps() == 0, and the rendezvous starts in SESSION, so without
// this main.cpp cannot tell a fresh start from a pre-frame-wire drone — it
// logged "upgrade maburd" at every maburgs startup (caught on the rig
// 2026-07-25).
TEST(peer_acked_false_until_a_disc_ack_is_accepted) {
  auto vrx = make();
  CHECK(!vrx.peer_acked());
  CHECK(vrx.peer_caps() == 0);
  CHECK(vrx.link_state() == VrxState::SESSION);  // initial state, no peer yet

  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  vrx.step(1500, ld, std::nullopt);              // silence -> BEACONING
  CHECK(!vrx.peer_acked());

  mabur::rc::DiscAck ack;
  ack.vtx_id = 1;
  ack.vrx_nonce = static_cast<uint32_t>((1ull * 2654435761ull) & 0xFFFFFFFFull);
  ack.chip_caps = 0;                             // a peer that advertises none
  auto wire = mabur::rc::pack_disc_ack(ack);
  vrx.on_rc_frame(wire.data(), wire.size(), 1600);
  CHECK(vrx.peer_acked());                       // now caps==0 means it truly said 0
  CHECK(vrx.peer_caps() == 0);
}
// T_TELEM frames are drone->GS display-only telemetry, not rendezvous
// traffic: on_rc_frame must tolerate the unknown (to it) frame type and
// leave rendezvous/link state completely untouched. GS routes T_TELEM to a
// separate holder before it ever reaches on_rc_frame (Task 3), but the
// controller itself must not choke if it ever sees one. Spec 2026-07-26
// drone-telemetry.
TEST(on_rc_frame_tolerates_unknown_type_telem) {
  auto vrx = make();
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  vrx.step(1500, ld, std::nullopt);  // silence -> BEACONING, seq_ advances

  const auto state_before = vrx.link_state();
  const auto op_before = vrx.cur_op();
  const auto seq_before = vrx.rcf_seq();

  mabur::rc::Telem t;
  t.tlm_seq = 42;
  t.state = 3;
  auto wire = mabur::rc::pack_telem(t);
  vrx.on_rc_frame(wire.data(), wire.size(), 1600);

  CHECK(vrx.link_state() == state_before);
  CHECK(vrx.cur_op().mcs == op_before.mcs);
  CHECK(vrx.rcf_seq() == seq_before);
}

MTEST_MAIN

// Starvation guard: a decode-collapse window (zero completed base-layer
// packets) must NOT feed the controller the survivor-biased SNR of the few
// frames that still decode — bench 2026-07-12: at NLOS range the estimator
// read 38-48 dB off a 20 fps trickle of a 1,750 fps stream, delivery read
// 100 (empty window), and the controller pinned agc0 with video frozen
// indefinitely. With video_starved=true the update is skipped, so the
// controller's own blind-side on_tick (feedback_timeout_ms) restores
// MAX_RANGE and the link self-heals.
TEST(starved_windows_fall_back_to_max_range) {
  auto vrx = make();
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  // Healthy phase: strong SNR, real traffic -> controller walks off the
  // MAX_RANGE floor (same stimulus as the pacing test).
  double now = 0;
  for (; now < 8000; now += 10) {
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    vrx.step(now, ld, 0.0);
  }
  REQUIRE(!(vrx.cur_op().mcs == 0 && vrx.cur_op().pwr_offset_qdb == 0));

  // Collapse phase: survivor frames keep arriving with HIGH reported SNR,
  // but the caller signals starvation (no completed packets this window).
  for (; now < 10000; now += 10) {
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    vrx.step(now, ld, std::nullopt, /*video_starved=*/true);
  }
  CHECK(vrx.cur_op().mcs == 0);
  CHECK(vrx.cur_op().pwr_offset_qdb == 0);

  // Traffic returns -> updates resume, controller may walk up again.
  for (; now < 18000; now += 10) {
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    vrx.step(now, ld, 0.0);
  }
  CHECK(!(vrx.cur_op().mcs == 0 && vrx.cur_op().pwr_offset_qdb == 0));
}

// Static-link mode: pin_mcs >= 0 bypasses the adaptive controller — every
// RCF carries exactly the pinned op regardless of SNR/delivery input, and
// neither blind-side MAX_RANGE nor ctrl updates can move it.
TEST(static_pin_overrides_controller) {
  static LinkTable lt;
  VrxCfg cfg;
  cfg.vtx_id = 1;
  cfg.pin_mcs = 5;
  cfg.pin_overhead = 0.25;
  cfg.pin_offset_qdb = -12;
  VrxController vrx(lt, cfg);
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  std::optional<VrxController::Out> out;
  double now = 0;
  for (; now < 8000; now += 10) {
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    auto o = vrx.step(now, ld, 0.0);
    if (o && !o->is_disc) out = o;
  }
  CHECK(vrx.cur_op().mcs == 5);
  CHECK(vrx.cur_op().pwr_offset_qdb == -12);
  REQUIRE(out.has_value());
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(r->pwr_offset_biased == mabur::rc::encode_pwr_offset_qdb(-12));
  CHECK(r->fec_overhead_16ths == mabur::rc::overhead_to_16ths(0.25));
}
