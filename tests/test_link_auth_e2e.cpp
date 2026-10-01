// RcAgent <-> VrxController loopback: the only place both ends of the
// rendezvous run against each other on the host. Spec 2026-10-01 §11.
#include <vector>
#include "mtest.h"
#include "mabur/link_key.h"
#include "mabur/rc_proto.h"
#include "rc_agent.h"
#include "vrx_controller.h"
using namespace mabur;

struct LoopActuator : Actuator {
  std::vector<std::vector<uint8_t>> controls;
  std::vector<AppliedOp> applied;
  std::vector<uint8_t> retunes;
  void apply_op(const AppliedOp& op) override { applied.push_back(op); }
  void send_control(const std::vector<uint8_t>& b) override { controls.push_back(b); }
  bool set_bitrate_kbps(int) override { return true; }
  bool set_roi_qp(int) override { return true; }
  bool set_fps(int) override { return true; }
  void request_idr() override {}
  void retune(uint8_t ch, const char*) override { retunes.push_back(ch); }
  bool set_record(bool) override { return true; }
};

// Same shape as tests/test_agent.cpp make_cfg(), plus the session key.
static Config drone_cfg(const LinkKey& k) {
  Config cfg;
  cfg.link.key = k;
  cfg.link.failsafe_ms = 1000;
  cfg.link.rendezvous_ms = 30000;
  cfg.link.tick_ms = 100;
  cfg.link.move_confirm_ms = 2000;
  cfg.encoder.airtime_budget = 0.65;
  cfg.encoder.bitrate_min_kbps = 1000;
  cfg.encoder.bitrate_max_kbps = 20000;
  cfg.encoder.roi_threshold_kbps = 3000;
  cfg.encoder.roi_qp_low = 8;
  cfg.encoder.roi_qp_normal = 0;
  cfg.radio.channel = 136;
  cfg.radio.follow_gs = true;
  cfg.venc.core.fps = 60;
  cfg.msp.enable = true;
  cfg.low_power.enable = true;
  cfg.low_power.bitrate_kbps = 1000;
  cfg.low_power.fps = 15;
  cfg.low_power.stale_ms = 2000;
  return cfg;
}
static maburgs::VrxCfg gs_cfg(const LinkKey& k) {
  maburgs::VrxCfg v;
  v.key = k;
  v.op_channel = 136;
  v.ladder = maburgs::LadderCfg{{{0, 0.5, 0.25}, {1, 0.5, 0.25}, {2, 0.5, 0.25}}};
  return v;
}
static maburgs::LinkHealth healthy() { return maburgs::LinkHealth{true, 0.0, 0.0, false}; }

struct Loop {
  LoopActuator act;
  Config cfg;
  RcAgent agent;
  maburgs::VrxController vrx;
  int rcfs = 0, discs = 0;
  Loop(const LinkKey& drone_key, const LinkKey& gs_key)
      : cfg(drone_cfg(drone_key)), agent(cfg, act), vrx(gs_cfg(gs_key)) {}
  // One 10 ms step of both ends with a perfect air link.
  void step(double t) {
    vrx.on_video(t);
    if (auto out = vrx.step(t, healthy())) {
      (out->is_disc ? discs : rcfs)++;
      agent.on_rc_frame(out->frame.data(), out->frame.size(), static_cast<uint64_t>(t));
    }
    for (auto& c : act.controls) vrx.on_rc_frame(c.data(), c.size(), t);
    act.controls.clear();
    if (static_cast<int>(t) % 100 == 0) agent.tick(static_cast<uint64_t>(t), RadioHealth{});
  }
};

TEST(matching_keys_link_within_a_second) {
  Loop l(kDefaultLinkKey, kDefaultLinkKey);
  double t = 0;
  for (; t < 1000 && l.agent.state() != RcAgent::State::LINKED; t += 10) l.step(t);
  CHECK(l.agent.state() == RcAgent::State::LINKED);
  CHECK(l.vrx.link_state() == maburgs::VrxState::SESSION);
  CHECK(!l.vrx.key_mismatch());
  CHECK(l.rcfs >= 1);
  CHECK(l.agent.current_session().vrx_nonce == l.vrx.rz_nonce());
}

TEST(mismatched_keys_end_in_key_mismatch_with_no_rcf_and_no_op) {
  LinkKey other = kDefaultLinkKey; other[3] ^= 0x10;
  Loop l(other, kDefaultLinkKey);
  for (double t = 0; t < 3000; t += 10) l.step(t);
  CHECK(l.vrx.link_state() == maburgs::VrxState::KEY_MISMATCH);
  CHECK(l.rcfs == 0);
  CHECK(l.agent.state() != RcAgent::State::LINKED);
  // tick() unconditionally applies the MAX_RANGE default once on the very
  // first BOOT->RENDEZVOUS transition, independent of any RC frame — so
  // "no op" from an unverified peer means the applied list never grows past
  // that one automatic entry, not that it stays empty.
  CHECK(l.act.applied.size() == 1);
  CHECK(l.act.retunes.empty());
  CHECK(l.agent.take_auth_reject());
}

TEST(stranger_drone_beside_ours_never_trips_key_mismatch) {
  LinkKey other = kDefaultLinkKey; other[0] ^= 1;
  Loop ours(kDefaultLinkKey, kDefaultLinkKey);
  LoopActuator sact;
  Config scfg = drone_cfg(other);
  RcAgent stranger(scfg, sact);
  for (double t = 0; t < 3000; t += 10) {
    ours.vrx.on_video(t);
    if (auto out = ours.vrx.step(t, healthy())) {
      ours.agent.on_rc_frame(out->frame.data(), out->frame.size(), static_cast<uint64_t>(t));
      stranger.on_rc_frame(out->frame.data(), out->frame.size(), static_cast<uint64_t>(t));
    }
    for (auto& c : sact.controls) ours.vrx.on_rc_frame(c.data(), c.size(), t);   // stranger answers first
    for (auto& c : ours.act.controls) ours.vrx.on_rc_frame(c.data(), c.size(), t);
    sact.controls.clear(); ours.act.controls.clear();
    if (static_cast<int>(t) % 100 == 0) { ours.agent.tick(static_cast<uint64_t>(t), RadioHealth{}); stranger.tick(static_cast<uint64_t>(t), RadioHealth{}); }
  }
  CHECK(ours.vrx.link_state() == maburgs::VrxState::SESSION);
  CHECK(!ours.vrx.key_mismatch());
  CHECK(ours.agent.state() == RcAgent::State::LINKED);
  CHECK(stranger.state() != RcAgent::State::LINKED);
}

TEST(drone_restart_relinks_through_the_keepalive_disc) {
  Loop l(kDefaultLinkKey, kDefaultLinkKey);
  double t = 0;
  for (; t < 1000 && l.agent.state() != RcAgent::State::LINKED; t += 10) l.step(t);
  REQUIRE(l.agent.state() == RcAgent::State::LINKED);
  // "Battery swap": a fresh RcAgent on the same GS.
  LoopActuator act2;
  RcAgent fresh(l.cfg, act2);
  double relinked_at = -1;
  for (double u = t; u < t + 3000; u += 10) {
    l.vrx.on_video(u);
    if (auto out = l.vrx.step(u, healthy()))
      fresh.on_rc_frame(out->frame.data(), out->frame.size(), static_cast<uint64_t>(u));
    for (auto& c : act2.controls) l.vrx.on_rc_frame(c.data(), c.size(), u);
    act2.controls.clear();
    if (static_cast<int>(u) % 100 == 0) fresh.tick(static_cast<uint64_t>(u), RadioHealth{});
    if (fresh.state() == RcAgent::State::LINKED) { relinked_at = u - t; break; }
  }
  CHECK(relinked_at >= 0);
  CHECK(relinked_at <= 1200);   // one keep-alive interval + one RCF
}
MTEST_MAIN
