#include "mtest.h"
#include "vrx_controller.h"
#include "ladder_controller.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"
using namespace maburgs;

static LadderCfg default_ladder() {
  LadderCfg lcfg;
  lcfg.ladder = {{0, 1.0}, {2, 0.5}, {4, 0.25}, {5, 0.25}, {6, 0.15}, {7, 0.1}};
  return lcfg;
}

static VrxController make(LadderCfg lcfg = default_ladder()) {
  VrxCfg cfg;
  cfg.vtx_id = 1;
  cfg.ladder = std::move(lcfg);
  return VrxController(cfg);
}

// A healthy sample: valid, no pre-FEC or residual loss, not starved.
static LinkHealth healthy() { return LinkHealth{true, 0.0, 0.0, false}; }
// No feedback data this window (e.g. pre-link / silence).
static LinkHealth no_data() { return LinkHealth{false, 0.0, 0.0, false}; }

// Drive video at 1 kHz and step at 10 ms; classify emissions per second.
TEST(rcf_pacing_and_keepalive_disc) {
  auto vrx = make();
  int rcf = 0, disc = 0;
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  for (int t = 0; t < 5000; t += 10) {
    const double now = t;
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(t / 10), now);
    if (auto out = vrx.step(now, ld, healthy())) {
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
  LinkHealth h{true, 0.0, 0.05, false};
  while (!out || out->is_disc) {         // skip a leading keepalive DISC
    now += 10;
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(500 + now / 10), now);
    out = vrx.step(now, ld, h);
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

// (a) Non-pin RCFs always carry offset 0, regardless of health — the ladder
// differentiates rungs by MCS + FEC overhead only; RCF power offset is
// fixed at the calibrated baseline (offset 0) in ladder mode.
TEST(non_pin_rcf_offset_always_zero) {
  auto vrx = make();
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  std::optional<VrxController::Out> out;
  double now = 0;
  // Deliberately hostile health: if an offset loop were still wired in this
  // path it would move the offset away from 0. It must not.
  LinkHealth hostile{true, 0.9, 0.9, false};
  for (int i = 0; i < 60 && (!out || out->is_disc); ++i) {
    now += 10;
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    out = vrx.step(now, ld, hostile);
  }
  REQUIRE(out.has_value());
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(r->pwr_offset_biased == mabur::rc::encode_pwr_offset_qdb(0));
  CHECK(vrx.cur_op().pwr_offset_qdb == 0);
}

// (b) Profile/overhead in the RCF track ctl().op() after a forced demote:
// walk the ladder up on clean health, then feed residual loss and confirm
// the very next RCF already reflects the demoted rung, not the stale one.
TEST(profile_and_overhead_track_ladder_after_forced_demote) {
  LadderCfg lcfg = default_ladder();
  lcfg.ladder = {{0, 1.0}, {4, 0.25}};
  lcfg.up_util = 0.1;
  lcfg.confirm_ms = 10;
  lcfg.clean_ms = 10;
  lcfg.probation_ms = 10;
  lcfg.hold_after_down_ms = 0;
  lcfg.min_between_changes_ms = 0;
  lcfg.feedback_timeout_ms = 100000;  // isolate from the blind-side timeout
  auto vrx = make(lcfg);
  std::array<uint8_t, 4> ld{100, 100, 100, 100};

  double now = 0;
  for (; now < 1000 && vrx.ctl().rung() == 0; now += 10) {
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    vrx.step(now, ld, healthy());
  }
  REQUIRE(vrx.ctl().rung() == 1);
  CHECK(vrx.cur_op().mcs == 4);

  std::optional<VrxController::Out> out;
  LinkHealth lossy{true, 0.0, 0.2, false};  // residual_loss > 0 -> demote
  for (int i = 0; i < 40 && vrx.ctl().rung() != 0; ++i) {
    now += 10;
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    out = vrx.step(now, ld, lossy);
  }
  REQUIRE(vrx.ctl().rung() == 0);
  CHECK(vrx.cur_op().mcs == 0);
  REQUIRE(out.has_value());
  REQUIRE(!out->is_disc);
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(r->profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 0, 20));
  CHECK(r->fec_overhead_16ths == mabur::rc::overhead_to_16ths(1.0));
  CHECK(vrx.cur_op().mcs == vrx.ctl().op().mcs);
  CHECK(vrx.cur_op().overhead == vrx.ctl().op().overhead);
}

TEST(silence_beacons_fast_and_recovers) {
  auto vrx = make();
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  vrx.on_video(-55.0, 25.0, false, 1, 0.0);
  // 2 s of silence: BEACONING at the 20 ms cadence.
  int discs = 0;
  for (double now = 1200; now < 2200; now += 10)
    if (auto out = vrx.step(now, ld, no_data())) {
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
  vrx.step(1500, ld, no_data());          // silence -> BEACONING
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
  vrx.step(1500, ld, no_data());          // silence -> BEACONING
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
  vrx.step(1500, ld, no_data());              // silence -> BEACONING
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
// s3-capable healthy sample; probe_allowed is set by the CONTROLLER from
// peer caps, not by callers, so the test must feed a DiscAck first.
static LinkHealth healthy3() {
  LinkHealth h = healthy();
  h.s3_valid = true; h.s3_expected_syms = 500; h.s1_snr_db = 30.0;
  return h;
}

// The controller must not encode a probe until the peer has advertised
// CAP_S3_PROBE via a DiscAck: mirror disc_ack_feeds_rendezvous's flow, add
// CAP_S3_PROBE to chip_caps.
TEST(probe_encoded_in_rcf_when_peer_capable) {
  auto vrx = make();
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  vrx.step(1500, ld, healthy3());  // silence -> BEACONING
  mabur::rc::DiscAck ack;
  ack.vtx_id = 1;
  ack.vrx_nonce = static_cast<uint32_t>((1ull * 2654435761ull) & 0xFFFFFFFFull);
  ack.chip_caps = mabur::rc::CAP_S3_PROBE;
  auto wire = mabur::rc::pack_disc_ack(ack);
  vrx.on_rc_frame(wire.data(), wire.size(), 1600);
  CHECK(vrx.peer_caps() & mabur::rc::CAP_S3_PROBE);

  bool saw_probe = false;
  double now = 1600;
  for (int t = 0; t < 9000; t += 10) {
    now = 1600 + t;
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(t / 10), now);
    if (auto out = vrx.step(now, ld, healthy3())) {
      if (out->is_disc) continue;
      auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
      REQUIRE(r.has_value());
      if (r->probe3) {
        saw_probe = true;
        // base profile still rung 0 (mcs0), probe targets rung 1 (mcs2):
        mabur::rc::PhyMode m; uint8_t mcs, bw;
        mabur::rc::decode_profile(r->profile, m, mcs, bw);
        CHECK(mcs == 0);
        mabur::rc::decode_profile(r->probe_profile, m, mcs, bw);
        CHECK(mcs == 2);
        break;
      }
    }
  }
  CHECK(saw_probe);
}

TEST(no_probe_without_cap) {
  auto vrx = make();      // no DiscAck: peer_caps == 0
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  for (int t = 0; t < 9000; t += 10) {
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(t / 10), t);
    if (auto out = vrx.step(t, ld, healthy3()); out && !out->is_disc) {
      auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
      if (r) CHECK(!r->probe3);
    }
  }
  CHECK(vrx.ctl().counters().probes_started == 0);
  CHECK(vrx.ctl().rung() >= 1);   // legacy promote happened instead
}

TEST(on_rc_frame_tolerates_unknown_type_telem) {
  auto vrx = make();
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  vrx.step(1500, ld, no_data());  // silence -> BEACONING, seq_ advances

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

// (e) Blind-side timeout must reach the wire: after promoting off rung 0 on
// clean feedback, feedback silently stops (sample_valid=false, e.g. the s1
// window saw 0 expected symbols) while video keeps flowing. LadderController
// ::on_tick() forces rung 0 internally once feedback_timeout_ms elapses, but
// that is only useful if VrxController actually copies the demoted rung into
// cur_op_/the next RCF -- otherwise the drone keeps flying the last
// aggressive op on stale wire content through and after the blind period.
TEST(blind_side_timeout_demotes_rcf_profile) {
  LadderCfg lcfg = default_ladder();
  lcfg.ladder = {{0, 1.0}, {4, 0.25}};
  lcfg.up_util = 0.1;
  lcfg.confirm_ms = 10;
  lcfg.clean_ms = 10;
  lcfg.probation_ms = 10;
  lcfg.hold_after_down_ms = 0;
  lcfg.min_between_changes_ms = 0;
  // feedback_timeout_ms left at its default (1000 ms) -- exactly what this
  // test is guarding.
  auto vrx = make(lcfg);
  std::array<uint8_t, 4> ld{100, 100, 100, 100};

  // Promote off rung 0 on real, healthy feedback samples.
  double now = 0;
  for (; now < 1000 && vrx.ctl().rung() == 0; now += 10) {
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    vrx.step(now, ld, healthy());
  }
  REQUIRE(vrx.ctl().rung() == 1);
  CHECK(vrx.cur_op().mcs == 4);

  // Feedback goes blind (sample_valid=false) but video keeps arriving, so
  // only the ladder's feedback timeout -- not rendezvous video silence --
  // is in play.
  std::optional<VrxController::Out> out;
  const double blind_until = now + 1500;  // > default feedback_timeout_ms
  for (; now < blind_until; now += 10) {
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    if (auto o = vrx.step(now, ld, no_data()); o && !o->is_disc) out = o;
  }
  REQUIRE(vrx.ctl().rung() == 0);
  CHECK(vrx.cur_op().mcs == 0);
  REQUIRE(out.has_value());
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(r->profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 0, 20));
  CHECK(r->fec_overhead_16ths == mabur::rc::overhead_to_16ths(1.0));
}

MTEST_MAIN

// (c) Starvation guard: a decode-collapse window (zero completed base-layer
// packets) must force the ladder to its failsafe rung (0) regardless of any
// other health field — bench 2026-07-12: at NLOS range the old SNR
// estimator read high off a trickle of survivor frames and pinned an
// aggressive op with video frozen indefinitely. LadderController::update
// forces rung 0 unconditionally on video_starved (ladder_controller.cpp
// step 1), so the link self-heals to the conservative floor, and clean
// health afterward lets it walk back up.
TEST(starved_health_forces_ladder_rung_zero_and_recovers) {
  LadderCfg lcfg = default_ladder();
  lcfg.ladder = {{0, 1.0}, {2, 0.5}, {4, 0.25}};
  lcfg.up_util = 0.1;
  lcfg.confirm_ms = 10;
  lcfg.clean_ms = 10;
  lcfg.probation_ms = 10;
  lcfg.hold_after_down_ms = 0;
  lcfg.min_between_changes_ms = 0;
  lcfg.feedback_timeout_ms = 100000;  // isolate from the blind-side timeout
  auto vrx = make(lcfg);
  std::array<uint8_t, 4> ld{100, 100, 100, 100};

  // Healthy phase: clean margin walks the ladder off rung 0.
  double now = 0;
  for (; now < 1000 && vrx.ctl().rung() == 0; now += 10) {
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    vrx.step(now, ld, healthy());
  }
  REQUIRE(vrx.ctl().rung() > 0);

  // Collapse phase: a SUSTAINED starved run forces rung 0 (single starved
  // windows are debounced by starved_confirm_ms=300 — the op-switch FEC
  // re-key glitch, hw 2026-07-27). 45 x 10 ms = 450 ms > 300 ms.
  std::optional<VrxController::Out> out;
  LinkHealth starved{true, 0.0, 0.0, /*video_starved=*/true};
  for (int i = 0; i < 45 && vrx.ctl().rung() != 0; ++i) {
    now += 10;
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    out = vrx.step(now, ld, starved);
  }
  REQUIRE(vrx.ctl().rung() == 0);
  CHECK(vrx.cur_op().mcs == 0);
  REQUIRE(out.has_value());
  REQUIRE(!out->is_disc);
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(r->profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 0, 20));
  CHECK(r->fec_overhead_16ths == mabur::rc::overhead_to_16ths(1.0));

  // Traffic returns -> clean health lets it walk back up.
  const double until = now + 1000;
  for (; now < until && vrx.ctl().rung() == 0; now += 10) {
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    vrx.step(now, ld, healthy());
  }
  CHECK(vrx.ctl().rung() > 0);
}

// (d) Static-link mode: pin_mcs >= 0 bypasses the ladder controller
// entirely — every RCF carries exactly the pinned op regardless of health
// input (including starvation / heavy loss that would force a real ladder
// to its floor), and the ladder is never even ticked.
TEST(static_pin_overrides_controller) {
  VrxCfg cfg;
  cfg.vtx_id = 1;
  cfg.pin_mcs = 5;
  cfg.pin_overhead = 0.25;
  cfg.pin_offset_qdb = -12;
  cfg.ladder.ladder = {{0, 1.0}};  // must never be consulted while pinned
  VrxController vrx(cfg);
  std::array<uint8_t, 4> ld{100, 100, 100, 100};
  std::optional<VrxController::Out> out;
  double now = 0;
  for (int i = 0; i < 800; ++i, now += 10) {
    vrx.on_video(-55.0, 25.0, false, static_cast<uint16_t>(now / 10), now);
    // Garbage/hostile health that would force a real ladder to rung 0 (or
    // demote hard) — pin mode must ignore all of it.
    LinkHealth garbage{true, 0.95, 0.95, (i % 3) == 0};
    auto o = vrx.step(now, ld, garbage);
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
