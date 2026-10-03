#include <cmath>
#include "mtest.h"
#include "vrx_controller.h"
#include "ladder_controller.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"
using namespace maburgs;

static LadderCfg default_ladder() {
  LadderCfg lcfg;
  // Same-rate-fixed-pairs (Task 4): overhead_enh set explicitly (equal to
  // overhead_base) rather than left at the struct default, since
  // op_from_rung() now feeds it straight into the RCF's enh field.
  lcfg.ladder = {{0, 1.0, 1.0}, {2, 0.5, 0.5},  {4, 0.25, 0.25},
                {5, 0.25, 0.25}, {6, 0.15, 0.15}, {7, 0.1, 0.1}};
  return lcfg;
}

static VrxController make(LadderCfg lcfg = default_ladder()) {
  VrxCfg cfg;
  cfg.ladder = std::move(lcfg);
  return VrxController(cfg);
}

// A healthy sample: valid, no pre-FEC or residual loss, not starved.
static LinkHealth healthy() { return LinkHealth{true, 0.0, 0.0, false}; }
// No feedback data this window (e.g. pre-link / silence).
static LinkHealth no_data() { return LinkHealth{false, 0.0, 0.0, false}; }

// Accept an unflagged DiscAck for this controller's vrx_nonce: the only way
// into SESSION (link-pairing spec 2026-10-01 §6) -- no RCF goes out before it.
static void link(VrxController& vrx, double now, uint32_t vtx = 0xBEEF0001) {
  mabur::rc::DiscAck ack;
  ack.vrx_nonce = vrx.rz_nonce();
  ack.vtx_nonce = vtx;
  ack.chip_caps = mabur::rc::CAP_FRAME_WIRE;
  ack.seq = 1;
  auto wire = mabur::rc::pack_disc_ack(ack);
  vrx.on_rc_frame(wire.data(), wire.size(), now);
}

// Drive video at 1 kHz and step at 10 ms; classify emissions per second.
TEST(rcf_pacing_and_keepalive_disc) {
  auto vrx = make();
  int rcf_pre = 0, rcf = 0, disc = 0;
  for (int t = 0; t < 5000; t += 10) {
    const double now = t;
    vrx.on_video(now);

    // Link early via DiscAck at t=500ms to measure steady-state cadence
    if (t == 500) link(vrx, now);

    if (auto out = vrx.step(now, healthy())) {
      const int ft = mabur::rc::frame_type(out->frame.data(), out->frame.size());
      if (t < 500) { if (ft == mabur::rc::T_RCF) ++rcf_pre; continue; }   // beaconing
      if (ft == mabur::rc::T_RCF) { CHECK(!out->is_disc); ++rcf; }
      else if (ft == mabur::rc::T_DISC) { CHECK(out->is_disc); ++disc; }
    }
  }
  CHECK(rcf_pre == 0);             // video alone never opens a session
  CHECK(rcf >= 40 && rcf <= 46);   // ~10 Hz for 4.5 s, minus keepalive slots
  CHECK(disc == 5);                // keep-alive at 1000 ms: t=500,1500,2500,3500,4500 (fix a)
}

// Bench 2026-09-26 (GS session 0232, escape 144 -> 112): the drone took the
// hop order from an RCF, then processed a keep-alive DISC it had received on
// the OLD channel just after it -- proposal 144 != its new channel 112 ->
// a "disc" retune straight back, and GS and drone sat apart ~33 s until
// rendezvous. While a hop order is in flight the keep-alive must not go out
// (its proposal is the old op by construction); it resumes, due at once,
// when the hold lifts. Revert = drop the hold check in step(): a DISC
// appears inside the held span and this fails.
TEST(keepalive_disc_held_while_a_hop_is_in_flight) {
  auto vrx = make();
  int disc_held = 0, rcf_held = 0, disc_after = 0;
  double first_after = -1;
  for (int t = 0; t < 6000; t += 10) {
    const double now = t;
    vrx.on_video(now);
    if (t == 500) link(vrx, now);
    vrx.set_keepalive_hold(t >= 1000 && t < 4000);
    if (auto out = vrx.step(now, healthy())) {
      const int ft = mabur::rc::frame_type(out->frame.data(), out->frame.size());
      const bool held = t >= 1000 && t < 4000;
      if (held) (ft == mabur::rc::T_DISC ? disc_held : rcf_held)++;
      if (!held && t >= 4000 && ft == mabur::rc::T_DISC) {
        ++disc_after;
        if (first_after < 0) first_after = now;
      }
    }
  }
  CHECK(disc_held == 0);
  CHECK(rcf_held >= 25);             // RCFs (which carry the order) keep flowing
  CHECK(disc_after >= 1);
  CHECK(first_after >= 4000 && first_after <= 4020);   // overdue keep-alive fires at once
}

// The hop hold gates only the SESSION keep-alive: before the first DiscAck
// there is no session, and BEACONING's 20 ms DISCs (the stale-caps
// re-teach) go out regardless of the hold.
TEST(beaconing_discs_ignore_the_hop_hold) {
  auto vrx = make();
  vrx.set_keepalive_hold(true);
  int disc = 0;
  for (int t = 0; t < 1000; t += 10) {
    vrx.on_video(t);
    if (auto out = vrx.step(t, healthy()))
      if (mabur::rc::frame_type(out->frame.data(), out->frame.size()) == mabur::rc::T_DISC) ++disc;
  }
  CHECK(vrx.link_state() == VrxState::BEACONING);
  CHECK(disc >= 45);   // every beacon_period_ms (20 ms) despite the hold
}

TEST(rcf_fields_are_correct) {
  auto vrx = make();
  link(vrx, 0.0);
  vrx.on_video(0.0);
  std::optional<VrxController::Out> out;
  double now = 0;
  LinkHealth h{true, 0.0, 0.05, false};
  while (!out || out->is_disc) {         // skip a leading keepalive DISC
    now += 10;
    vrx.on_video(now);
    out = vrx.step(now, h);
  }
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(r->seq > 0);
  CHECK(r->profile == mabur::rc::encode_profile(
                          mabur::rc::PhyMode::HT,
                          static_cast<uint8_t>(vrx.cur_op().mcs),
                          static_cast<uint8_t>(vrx.cur_op().bw)));
  CHECK(std::abs(r->fec_overhead_base - vrx.cur_op().overhead_base) < 1e-9);
}

// (b) Profile/overhead in the RCF track ctl().op() after a forced demote:
// walk the ladder up on clean health, then feed residual loss and confirm
// the very next RCF already reflects the demoted rung, not the stale one.
TEST(profile_and_overhead_track_ladder_after_forced_demote) {
  LadderCfg lcfg = default_ladder();
  lcfg.ladder = {{0, 1.0, 1.0}, {4, 0.25, 0.25}};
  lcfg.up_util = 0.1;
  lcfg.confirm_ms = 10;
  lcfg.clean_ms = 10;
  lcfg.probation_ms = 10;
  lcfg.hold_after_down_ms = 0;
  lcfg.min_between_changes_ms = 0;
  // Legacy promote semantics: these tests climb on healthy() samples, which
  // carry no probe window, so the always-on probe gate would read NoInfo and
  // hold every promote. The gate itself is covered in test_ladder_controller.
  lcfg.probe.enable = false;
  lcfg.feedback_timeout_ms = 100000;  // isolate from the blind-side timeout
  auto vrx = make(lcfg);
  link(vrx, 0.0);

  double now = 0;
  for (; now < 1000 && vrx.ctl().rung() == 0; now += 10) {
    vrx.on_video(now);
    vrx.step(now, healthy());
  }
  REQUIRE(vrx.ctl().rung() == 1);
  CHECK(vrx.cur_op().mcs == 4);

  std::optional<VrxController::Out> out;
  LinkHealth lossy{true, 0.0, 0.2, false};  // residual_loss > 0 -> demote
  for (int i = 0; i < 40 && vrx.ctl().rung() != 0; ++i) {
    now += 10;
    vrx.on_video(now);
    out = vrx.step(now, lossy);
  }
  REQUIRE(vrx.ctl().rung() == 0);
  CHECK(vrx.cur_op().mcs == 0);
  REQUIRE(out.has_value());
  REQUIRE(!out->is_disc);
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(r->profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 0, 20));
  CHECK(std::abs(r->fec_overhead_base - 1.0) < 1e-9);
  CHECK(vrx.cur_op().mcs == vrx.ctl().op().mcs);
  CHECK(vrx.cur_op().overhead_base == vrx.ctl().op().overhead_base);
}

TEST(silence_beacons_fast_and_recovers) {
  auto vrx = make();
  link(vrx, 0.0);
  vrx.on_video(0.0);
  // 2 s of silence: BEACONING at the 20 ms cadence.
  int discs = 0;
  for (double now = 1200; now < 2200; now += 10)
    if (auto out = vrx.step(now, no_data())) {
      CHECK(out->is_disc);
      ++discs;
    }
  CHECK(vrx.link_state() == VrxState::BEACONING);
  CHECK(discs >= 45);                       // ~50 in 1 s at 20 ms pacing
  // Failsafe op point while blind:
  CHECK(vrx.cur_op().mcs == 0);
  // Video returns -> SESSION and RCFs resume.
  vrx.on_video(2500.0);
  CHECK(vrx.link_state() == VrxState::SESSION);
}

TEST(disc_ack_feeds_rendezvous) {
  auto vrx = make();
  vrx.step(1500, no_data());          // silence -> BEACONING
  CHECK(vrx.link_state() == VrxState::BEACONING);
  mabur::rc::DiscAck ack;
  ack.vrx_nonce = vrx.rz_nonce();
  ack.vtx_nonce = 1;
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
  vrx.step(1500, no_data());          // silence -> BEACONING
  mabur::rc::DiscAck ack;
  ack.vrx_nonce = vrx.rz_nonce();
  ack.vtx_nonce = 1;
  ack.chip_caps = mabur::rc::CAP_FRAME_WIRE;
  auto wire = mabur::rc::pack_disc_ack(ack);
  vrx.on_rc_frame(wire.data(), wire.size(), 1600);
  CHECK(vrx.link_state() == VrxState::SESSION);
  CHECK(vrx.peer_caps() & mabur::rc::CAP_FRAME_WIRE);
}

// peer_acked() separates "no DiscAck yet" from "peer advertised caps == 0".
// Both read peer_caps() == 0, so without
// this main.cpp cannot tell a fresh start from a pre-frame-wire drone — it
// logged "upgrade maburd" at every maburgs startup (caught on the rig
// 2026-07-25).
TEST(peer_acked_false_until_a_disc_ack_is_accepted) {
  auto vrx = make();
  CHECK(!vrx.peer_acked());
  CHECK(vrx.peer_caps() == 0);
  CHECK(vrx.link_state() == VrxState::BEACONING);  // initial state, no peer yet

  vrx.step(1500, no_data());              // silence -> BEACONING
  CHECK(!vrx.peer_acked());

  mabur::rc::DiscAck ack;
  ack.vrx_nonce = vrx.rz_nonce();
  ack.vtx_nonce = 1;
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
  vrx.step(1500, no_data());  // silence -> BEACONING, seq_ advances

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
  lcfg.ladder = {{0, 1.0, 1.0}, {4, 0.25, 0.25}};
  lcfg.up_util = 0.1;
  lcfg.confirm_ms = 10;
  lcfg.clean_ms = 10;
  lcfg.probation_ms = 10;
  lcfg.hold_after_down_ms = 0;
  lcfg.min_between_changes_ms = 0;
  // Legacy promote semantics: these tests climb on healthy() samples, which
  // carry no probe window, so the always-on probe gate would read NoInfo and
  // hold every promote. The gate itself is covered in test_ladder_controller.
  lcfg.probe.enable = false;
  // feedback_timeout_ms left at its default (1000 ms) -- exactly what this
  // test is guarding.
  auto vrx = make(lcfg);
  link(vrx, 0.0);

  // Promote off rung 0 on real, healthy feedback samples.
  double now = 0;
  for (; now < 1000 && vrx.ctl().rung() == 0; now += 10) {
    vrx.on_video(now);
    vrx.step(now, healthy());
  }
  REQUIRE(vrx.ctl().rung() == 1);
  CHECK(vrx.cur_op().mcs == 4);

  // Feedback goes blind (sample_valid=false) but video keeps arriving, so
  // only the ladder's feedback timeout -- not rendezvous video silence --
  // is in play.
  std::optional<VrxController::Out> out;
  const double blind_until = now + 1500;  // > default feedback_timeout_ms
  for (; now < blind_until; now += 10) {
    vrx.on_video(now);
    if (auto o = vrx.step(now, no_data()); o && !o->is_disc) out = o;
  }
  REQUIRE(vrx.ctl().rung() == 0);
  CHECK(vrx.cur_op().mcs == 0);
  REQUIRE(out.has_value());
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(r->profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 0, 20));
  CHECK(std::abs(r->fec_overhead_base - 1.0) < 1e-9);
}

TEST(controller_exposes_agreed_channel) {
  auto vrx = make();
  CHECK(vrx.agreed_channel() == 0);
  vrx.set_proposal(149);
  vrx.on_video(0.0);
  double now = 0;
  std::optional<VrxController::Out> out;
  while (!out || !out->is_disc) { now += 10; out = vrx.step(now, no_data()); }   // no video -> BEACONING
  auto d = mabur::rc::parse_disc(out->frame.data(), out->frame.size());
  REQUIRE(d.has_value());
  CHECK(d->op_channel == 149);
  mabur::rc::DiscAck ack;
  ack.vrx_nonce = vrx.rz_nonce(); ack.vtx_nonce = 1; ack.chip_caps = mabur::rc::CAP_FRAME_WIRE;
  ack.agreed_channel = 149; ack.seq = 1;
  auto wire = mabur::rc::pack_disc_ack(ack);
  vrx.on_rc_frame(wire.data(), wire.size(), now);
  CHECK(vrx.agreed_channel() == 149);
  ack.agreed_channel = 136; ack.seq = 2;
  wire = mabur::rc::pack_disc_ack(ack);
  vrx.on_rc_frame(wire.data(), wire.size(), now + 1);
  CHECK(vrx.agreed_channel() == 136);
}

TEST(no_rcf_before_an_ack_even_with_video) {
  auto vrx = make();
  int rcf = 0;
  for (int t = 0; t < 2000; t += 10) {
    vrx.on_video(t);
    if (auto out = vrx.step(t, healthy()); out && !out->is_disc) ++rcf;
  }
  CHECK(rcf == 0);
  CHECK(vrx.link_state() == VrxState::BEACONING);
}

TEST(rcf_is_tagged_with_the_session_and_seq32_resets_on_new_vtx_nonce) {
  auto vrx = make();
  link(vrx, 100);
  std::vector<uint8_t> first;
  for (int t = 100; t < 400 && first.empty(); t += 10) {
    vrx.on_video(t);
    if (auto out = vrx.step(t, healthy()); out && !out->is_disc) first = out->frame;
  }
  REQUIRE(!first.empty());
  auto r = mabur::rc::parse_rcf(first.data(), first.size());
  REQUIRE(r.has_value());
  CHECK(r->seq == 1);
  CHECK(mabur::rc::verify_control(first.data(), first.size(), mabur::kDefaultLinkKey,
                                  mabur::rc::TagCtx{vrx.rz_nonce(), 0xBEEF0001, 1}));
  CHECK(!mabur::rc::verify_control(first.data(), first.size(), mabur::kDefaultLinkKey,
                                   mabur::rc::TagCtx{vrx.rz_nonce(), 0xBEEF0002, 1}));
  // Same ack again (lost-ack retry): seq keeps counting.
  link(vrx, 500);
  CHECK(vrx.rcf_seq32() >= 1);
  // New vtx_nonce (drone restarted): seq32 restarts so the drone's fresh
  // tracker and our tag ctx agree from the first RCF of the new session.
  link(vrx, 600, 0xBEEF0002);
  CHECK(vrx.rcf_seq32() == 0);
}

TEST(move_edge_fires_on_linked_telem_or_after_five_rcfs) {
  auto vrx = make();
  CHECK(!vrx.take_move_edge());
  link(vrx, 100);
  CHECK(!vrx.take_move_edge());            // ack alone no longer moves the GS
  vrx.note_drone_state(2);                  // RcAgent::State::LINKED
  CHECK(vrx.take_move_edge());
  CHECK(!vrx.take_move_edge());
  auto vrx2 = make();
  link(vrx2, 100);
  int rcfs = 0;
  for (int t = 100; t < 2000 && rcfs < 5; t += 10) {
    vrx2.on_video(t);
    if (auto out = vrx2.step(t, healthy()); out && !out->is_disc) ++rcfs;
  }
  CHECK(rcfs == 5);
  CHECK(vrx2.take_move_edge());
}

// Controller side of the same race: KEY_MISMATCH entered while holding our
// drone's session must not zero seq32 when that drone's next ack repeats the
// same vtx_nonce -- the drone would reject every RCF (seq not increasing)
// until failsafe. RCFs resume with the next seq.
// Linked, video flowing, and every SESSION keep-alive DISC (1 s) answered
// flagged only -- as if our drone's key changed. The stranger rule needs
// both >= kKeyMismatchMs AND >= kKeyMismatchBeacons DISCs since the first
// flagged ack, so KEY_MISMATCH lands on the ack of the 3rd keep-alive after
// the first flagged one (~3 s), and no RCF goes out after it. Our drone's
// next unflagged ack (same vtx_nonce) returns to SESSION and seq32 simply
// continues -- not a new session.
TEST(session_keepalives_answered_flagged_only_enter_key_mismatch_seq32_continues) {
  auto vrx = make();
  link(vrx, 0);
  mabur::rc::DiscAck flagged;
  flagged.vrx_nonce = vrx.rz_nonce();
  flagged.flags = mabur::rc::kAckKeyMismatch;
  auto fw = mabur::rc::pack_disc_ack(flagged);
  int flagged_rounds = 0, rcf_after = 0;
  double entered = -1;
  int t = 0;
  for (; t < 6000 && entered < 0; t += 10) {
    vrx.on_video(t);
    if (auto out = vrx.step(t, healthy())) {
      if (out->is_disc) {
        vrx.on_rc_frame(fw.data(), fw.size(), t + 5);
        ++flagged_rounds;
        if (vrx.key_mismatch()) entered = t;
      }
    }
  }
  REQUIRE(entered >= 0);
  CHECK(flagged_rounds == 4);          // first flagged ack + 3 more keep-alives
  CHECK(entered >= 2990 && entered <= 3010);
  const uint32_t seq_before = vrx.rcf_seq32();
  REQUIRE(seq_before > 0);
  for (const int end = t + 2000; t < end; t += 10) {
    vrx.on_video(t);
    if (auto out = vrx.step(t, healthy()); out && !out->is_disc) ++rcf_after;
  }
  CHECK(rcf_after == 0);
  CHECK(vrx.key_mismatch());
  link(vrx, t);                                   // our drone, same vtx_nonce
  CHECK(!vrx.key_mismatch());
  CHECK(vrx.link_state() == VrxState::SESSION);
  CHECK(vrx.rcf_seq32() == seq_before);
  std::vector<uint8_t> next;
  for (int end = t + 500; t < end && next.empty(); t += 10) {
    vrx.on_video(t);
    if (auto out = vrx.step(t, healthy()); out && !out->is_disc) next = out->frame;
  }
  REQUIRE(!next.empty());
  CHECK(vrx.rcf_seq32() == seq_before + 1);
  CHECK(mabur::rc::verify_control(next.data(), next.size(), mabur::kDefaultLinkKey,
                                  mabur::rc::TagCtx{vrx.rz_nonce(), 0xBEEF0001, seq_before + 1}));
}

// A foreign-key drone answers every keep-alive flagged beside ours. Our
// drone's ack is lost on one round, and on the next round the stranger's
// arrives first. One missing unflagged ack is one flagged-only round, far
// short of the rule: the GS stays in SESSION and keeps sending RCFs.
TEST(stranger_plus_one_lost_ack_of_ours_stays_in_session) {
  auto vrx = make();
  link(vrx, 0);
  mabur::rc::DiscAck flagged;
  flagged.vrx_nonce = vrx.rz_nonce();
  flagged.flags = mabur::rc::kAckKeyMismatch;
  auto fw = mabur::rc::pack_disc_ack(flagged);
  int round = 0, rcf = 0;
  for (int t = 0; t < 8000; t += 10) {
    vrx.on_video(t);
    if (auto out = vrx.step(t, healthy())) {
      if (!out->is_disc) { ++rcf; continue; }
      ++round;
      vrx.on_rc_frame(fw.data(), fw.size(), t + 1);          // stranger, first
      // Even transiently: gs main acts on the edge (video tail reset,
      // hopc.on_session_lost, cal peer unlinked).
      CHECK(!vrx.key_mismatch());
      if (round != 3) link(vrx, t + 2);                      // ours, lost on round 3
    }
    CHECK(vrx.link_state() == VrxState::SESSION);
    CHECK(!vrx.key_mismatch());
  }
  CHECK(round >= 7);
  CHECK(rcf > 50);
}

TEST(key_mismatch_sends_no_rcf_and_reports) {
  auto vrx = make();
  mabur::rc::DiscAck ack;
  ack.vrx_nonce = vrx.rz_nonce();
  ack.flags = mabur::rc::kAckKeyMismatch;
  auto wire = mabur::rc::pack_disc_ack(ack);
  int rcf = 0;
  for (int t = 0; t < 3000; t += 10) {
    vrx.on_video(t);
    vrx.on_rc_frame(wire.data(), wire.size(), t);
    if (auto out = vrx.step(t, healthy()); out && !out->is_disc) ++rcf;
  }
  CHECK(rcf == 0);
  CHECK(vrx.key_mismatch());
  CHECK(vrx.link_state() == VrxState::KEY_MISMATCH);
  CHECK(!vrx.peer_acked());
}

// Final review C1 addendum A: a DISC proposes the channel it is sent on.
// The rewritten copy must still verify under the key (the drone drops an
// untagged DISC as a wrong-key GS) and keep every other field.
// Revert (send the op copy everywhere): the burst copy proposes 136.
TEST(disc_for_channel_reproposes_and_retags) {
  mabur::LinkKey key = mabur::kDefaultLinkKey;
  key[0] ^= 0x5a;
  mabur::rc::Disc d;
  d.vrx_nonce = 0x1234;
  d.op_channel = 136;
  d.seq = 7;
  const auto op_copy = mabur::rc::pack_disc(d, key);
  const auto x_copy = disc_for_channel(op_copy, 149, key);
  auto px = mabur::rc::parse_disc(x_copy.data(), x_copy.size());
  REQUIRE(px.has_value());
  CHECK(px->op_channel == 149 && px->vrx_nonce == 0x1234 && px->seq == 7);
  CHECK(mabur::rc::verify_control(x_copy.data(), x_copy.size(), key, mabur::rc::TagCtx{}));
  CHECK(disc_for_channel(op_copy, 136, key) == op_copy);   // already proposes it
  CHECK(disc_for_channel(op_copy, 0, key) == op_copy);     // unknown channel: untouched
  mabur::rc::Rcf r;
  const auto rcf = mabur::rc::pack_rcf(r, key, mabur::rc::TagCtx{1, 2, 3});
  CHECK(disc_for_channel(rcf, 149, key) == rcf);           // not a DISC
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
  lcfg.ladder = {{0, 1.0, 1.0}, {2, 0.5, 0.5}, {4, 0.25, 0.25}};
  lcfg.up_util = 0.1;
  lcfg.confirm_ms = 10;
  lcfg.clean_ms = 10;
  lcfg.probation_ms = 10;
  lcfg.hold_after_down_ms = 0;
  lcfg.min_between_changes_ms = 0;
  // Legacy promote semantics: these tests climb on healthy() samples, which
  // carry no probe window, so the always-on probe gate would read NoInfo and
  // hold every promote. The gate itself is covered in test_ladder_controller.
  lcfg.probe.enable = false;
  lcfg.feedback_timeout_ms = 100000;  // isolate from the blind-side timeout
  auto vrx = make(lcfg);
  link(vrx, 0.0);

  // Healthy phase: clean margin walks the ladder off rung 0.
  double now = 0;
  for (; now < 1000 && vrx.ctl().rung() == 0; now += 10) {
    vrx.on_video(now);
    vrx.step(now, healthy());
  }
  REQUIRE(vrx.ctl().rung() > 0);

  // Collapse phase: a SUSTAINED starved run forces rung 0 (single starved
  // windows are debounced by starved_confirm_ms=300 — the op-switch FEC
  // re-key glitch, hw 2026-07-27). 45 x 10 ms = 450 ms > 300 ms.
  std::optional<VrxController::Out> out;
  LinkHealth starved{true, 0.0, 0.0, /*video_starved=*/true};
  for (int i = 0; i < 45 && vrx.ctl().rung() != 0; ++i) {
    now += 10;
    vrx.on_video(now);
    out = vrx.step(now, starved);
  }
  REQUIRE(vrx.ctl().rung() == 0);
  CHECK(vrx.cur_op().mcs == 0);
  REQUIRE(out.has_value());
  REQUIRE(!out->is_disc);
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(r->profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 0, 20));
  CHECK(std::abs(r->fec_overhead_base - 1.0) < 1e-9);

  // Traffic returns -> clean health lets it walk back up.
  const double until = now + 1000;
  for (; now < until && vrx.ctl().rung() == 0; now += 10) {
    vrx.on_video(now);
    vrx.step(now, healthy());
  }
  CHECK(vrx.ctl().rung() > 0);
}

// (d) Static-link mode: pin_mcs >= 0 bypasses the ladder controller
// entirely — every RCF carries exactly the pinned op regardless of health
// input (including starvation / heavy loss that would force a real ladder
// to its floor), and the ladder is never even ticked.
TEST(static_pin_overrides_controller) {
  VrxCfg cfg;
  cfg.pin_mcs = 5;
  cfg.pin_overhead_base = 0.25;
  cfg.pin_overhead_enh = 0.4;  // distinct from base: proves the pin is a real pair
  cfg.ladder.ladder = {{0, 1.0}};  // must never be consulted while pinned
  VrxController vrx(cfg);
  link(vrx, 0.0);
  std::optional<VrxController::Out> out;
  double now = 0;
  for (int i = 0; i < 800; ++i, now += 10) {
    vrx.on_video(now);
    // Garbage/hostile health that would force a real ladder to rung 0 (or
    // demote hard) — pin mode must ignore all of it.
    LinkHealth garbage{true, 0.95, 0.95, (i % 3) == 0};
    auto o = vrx.step(now, garbage);
    if (o && !o->is_disc) out = o;
  }
  CHECK(vrx.cur_op().mcs == 5);
  REQUIRE(out.has_value());
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(std::abs(r->fec_overhead_base - 0.25) < 1e-9);
  CHECK(std::abs(r->fec_overhead_enh - 0.4) < 1e-9);
}

// --- link.probe byte in the RCF head (spec 2026-09-04 sections 4.2, 5) -----

// Drives the link into SESSION with a DiscAck, then steps until an RCF is
// emitted; returns the parsed RCF.
static mabur::rc::Rcf first_rcf(VrxController& vrx, const LinkHealth& h, double& t) {
  link(vrx, t, 1);
  for (int i = 0; i < 400; ++i, t += 10) {
    vrx.on_video(t);
    if (auto out = vrx.step(t, h); out && !out->is_disc) {
      auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
      REQUIRE(r.has_value());
      return *r;
    }
  }
  REQUIRE(false);
  return {};
}

TEST(rcf_carries_the_probe_rung_profile) {
  auto vrx = make();  // rung 0 = mcs0, rung 1 = mcs2
  double t = 0;
  auto r = first_rcf(vrx, healthy(), t);
  CHECK(r.probe_profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 2, 20));
  CHECK(vrx.probe_profile() == r.probe_profile);
}

TEST(rcf_probe_byte_is_none_when_disabled_or_pinned_without_pin_mcs) {
  LadderCfg l = default_ladder(); l.probe.enable = false;
  auto vrx = make(l);
  double t = 0;
  CHECK(first_rcf(vrx, healthy(), t).probe_profile == mabur::rc::kNoProbeProfile);
  VrxCfg cfg; cfg.ladder = default_ladder(); cfg.pin_mcs = 4;
  VrxController pinned(cfg);
  t = 0;
  CHECK(first_rcf(pinned, healthy(), t).probe_profile == mabur::rc::kNoProbeProfile);
}

TEST(pinned_link_can_probe_a_fixed_mcs) {
  VrxCfg cfg; cfg.ladder = default_ladder(); cfg.pin_mcs = 4;
  cfg.probe_pin_mcs = 5;
  VrxController vrx(cfg);
  double t = 0;
  CHECK(first_rcf(vrx, healthy(), t).probe_profile ==
        mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 5, 20));
}

// ---- per-rung width (2026-09-24) ------------------------------------------

static LadderCfg bw40_ladder() {
  LadderCfg lcfg;
  lcfg.ladder = {{0, 0.5, 0.25, 20}, {4, 0.5, 0.25, 20}, {3, 0.5, 0.25, 40}, {4, 0.5, 0.25, 40}};
  return lcfg;
}

TEST(rcf_profile_carries_the_rungs_width) {
  auto vrx = make(bw40_ladder());   // starts at rung 0 = 20/0
  double t = 0;
  auto r = first_rcf(vrx, healthy(), t);
  CHECK(r.profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 0, 20));
  CHECK(vrx.cur_op().bw == 20);
}

TEST(rcf_probe_profile_carries_the_probe_rungs_width) {
  // Rung 1 is 20/4 and rung 2 is 40/3: sitting on rung 1 the probe must
  // fly 40 MHz, or its clean streak says nothing about the 40 rung.
  //
  // restore_rung() does not stamp last_feedback_ms_, so calling it before
  // any real feedback has ever landed lets on_tick()'s blind-side timeout
  // (measured off the never-stamped default) force rung 0 right back on
  // the very next tick -- same as
  // restore_rung_rcf_in_the_same_tick_carries_restored_profile works
  // around it: bring the link up for real first, THEN restore.
  LadderCfg l = bw40_ladder();
  l.feedback_timeout_ms = 100000;
  VrxCfg cfg; cfg.ladder = l;
  VrxController vrx(cfg);
  double t = 0;
  first_rcf(vrx, healthy(), t);  // stamps last_feedback_ms_ before the restore
  vrx.restore_rung(1, t);   // park the ladder on rung 1 (20/4)
  auto r = first_rcf(vrx, healthy(), t);
  CHECK(r.profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 4, 20));
  CHECK(r.probe_profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 3, 40));
}

TEST(static_pin_carries_pin_bw) {
  VrxCfg cfg; cfg.ladder = bw40_ladder(); cfg.pin_mcs = 3; cfg.pin_bw = 40;
  cfg.probe_pin_mcs = 4;
  VrxController vrx(cfg);
  double t = 0;
  auto r = first_rcf(vrx, healthy(), t);
  CHECK(r.profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 3, 40));
  CHECK(r.probe_profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 4, 40));
}

// --- Task 8: hop plumbing (spec 2026-09-14 in-flight channel hop) ---

// set_hop() is carried in EVERY RCF from then on, defaulting to 0/0 (no
// order ever issued) until the first call.
TEST(set_hop_populates_every_rcf_from_then_on) {
  auto vrx = make();
  double t = 0;
  CHECK(vrx.hop_ch() == 0);
  CHECK(vrx.hop_epoch() == 0);
  auto r0 = first_rcf(vrx, healthy(), t);
  CHECK(r0.hop_ch == 0);
  CHECK(r0.hop_epoch == 0);

  vrx.set_hop(157, 3);
  CHECK(vrx.hop_ch() == 157);
  CHECK(vrx.hop_epoch() == 3);
  auto r1 = first_rcf(vrx, healthy(), t);
  CHECK(r1.hop_ch == 157);
  CHECK(r1.hop_epoch == 3);
}

// restore_rung() must refresh cur_op_ synchronously -- NOT wait for the
// next step() to notice a rung change it did not itself make (step() only
// resyncs cur_op_ when ITS OWN ctrl_.on_tick()/ctrl_.update() call returns
// true; a rung change made directly via ctrl_.restore() from outside would
// otherwise leave cur_op_ stale forever).
TEST(restore_rung_syncs_cur_op_before_any_step) {
  LadderCfg lcfg = default_ladder();
  lcfg.feedback_timeout_ms = 100000;  // isolate from the blind-side timeout
  auto vrx = make(lcfg);
  REQUIRE(vrx.cur_op().mcs == 0);
  vrx.restore_rung(3, 0.0);           // default_ladder()[3] = mcs 5
  CHECK(vrx.ctl().rung() == 3);
  CHECK(vrx.cur_op().mcs == 5);
  CHECK(vrx.ctl().last_event().reason == CtlReason::HopRestore);
}

// And the RCF built on the SAME tick as the restore already carries the
// restored profile, not the pre-restore one.
TEST(restore_rung_rcf_in_the_same_tick_carries_restored_profile) {
  LadderCfg lcfg = default_ladder();
  lcfg.feedback_timeout_ms = 100000;
  auto vrx = make(lcfg);
  double t = 0;
  // Bring the link up for real first (accepts the DiscAck, and a genuine
  // ctrl_.update() stamps the ladder's last_feedback_ms_): restore_rung on
  // a controller that has never once been fed real feedback is not a
  // scenario Task 11's HopController produces, and on_tick()'s blind-side
  // timeout (measured off that never-stamped default) would otherwise
  // force rung 0 right back on the very first tick.
  auto r0 = first_rcf(vrx, healthy(), t);
  CHECK(r0.profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 0, 20));

  vrx.restore_rung(3, t);             // default_ladder()[3] = mcs 5
  auto r1 = first_rcf(vrx, healthy(), t);
  CHECK(r1.profile == mabur::rc::encode_profile(mabur::rc::PhyMode::HT, 5, 20));
}

TEST(rcf_carries_the_rec_wish) {
  auto vrx = make();
  vrx.set_rec_wish(mabur::rc::kRecKnown | mabur::rc::kRecOn);
  link(vrx, 0.0);
  vrx.on_video(0.0);
  std::optional<VrxController::Out> out;
  double now = 0;
  LinkHealth h{true, 0.0, 0.05, false};
  while (!out || out->is_disc) {
    now += 10;
    vrx.on_video(now);
    out = vrx.step(now, h);
  }
  auto r = mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  REQUIRE(r.has_value());
  CHECK(r->rec == (mabur::rc::kRecKnown | mabur::rc::kRecOn));
}

TEST(rcf_carries_the_idr_epoch) {
  // first_rcf() (file helper, ~line 495) acks the rendezvous and returns the
  // first non-DISC frame; t carries forward between calls.
  auto vrx = make();
  double t = 0;
  CHECK(first_rcf(vrx, healthy(), t).idr_epoch == 0);   // never set: 0 (maburgs)
  vrx.set_idr_epoch(0x2B);
  CHECK(first_rcf(vrx, healthy(), t).idr_epoch == 0x2B);
}
