#include <cstdint>
#include <memory>
#include <set>
#include <vector>

#include "mtest.h"
#include "config.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"
#include "mabur/uep_encoder.h"
#include "rc_agent.h"

using namespace mabur;
using namespace mabur::rc;

namespace {

// Records every Actuator call for inspection by the tests. bitrate_ok /
// roi_ok simulate an encoder that refuses a verb (a transient MI failure on
// the real drone): the ATTEMPT is still recorded, so a test can tell "not
// called" from "called and refused".
struct MockActuator : Actuator {
  std::vector<AppliedOp> applied;
  std::vector<std::vector<uint8_t>> controls;
  std::vector<int> bitrates;
  std::vector<int> roi_qps;
  int idr_calls = 0;
  bool bitrate_ok = true;
  bool roi_ok = true;

  void apply_op(const AppliedOp& op) override { applied.push_back(op); }
  void send_control(const std::vector<uint8_t>& body) override { controls.push_back(body); }
  bool set_bitrate_kbps(int kbps) override {
    bitrates.push_back(kbps);
    return bitrate_ok;
  }
  bool set_roi_qp(int qp) override {
    roi_qps.push_back(qp);
    return roi_ok;
  }
  void request_idr() override { ++idr_calls; }
};

Config make_cfg() {
  Config cfg;
  cfg.link.vtx_id = 1;
  cfg.link.failsafe_ms = 1000;
  cfg.link.rendezvous_ms = 30000;
  cfg.link.tick_ms = 100;
  cfg.encoder.airtime_budget = 0.65;
  cfg.encoder.bitrate_min_kbps = 1000;
  cfg.encoder.bitrate_max_kbps = 20000;
  cfg.encoder.roi_threshold_kbps = 3000;
  cfg.encoder.roi_qp_low = 8;
  cfg.encoder.roi_qp_normal = 0;
  return cfg;
}

// Builds a CRC-valid RCF wire frame for vtx_id/seq/profile/fec_overhead,
// with DISTINCT base/enh overheads (ov_16ths keeps the callers' existing
// sixteenths-based literals: 8 == 0.5, 16 == 1.0, ...). This is the widened
// form (controller ruling, Task 6): the 4-arg overload below still packs one
// value into both wire fields for the ~40 call sites that don't care about
// the split, but a test that means to exercise the base/enh PAIR (the
// share-weighting tests, and the AppliedOp-pair test) must call this 5-arg
// form directly, or the two wire fields collapse to the same value and the
// weighted target can't tell base from enh apart (see the weakened-test
// comments this replaced, Task 2 finding).
std::vector<uint8_t> make_rcf_wire(uint32_t vtx_id, uint16_t seq, uint8_t profile,
                                    uint8_t ov_base_16ths, uint8_t ov_enh_16ths) {
  Rcf r;
  r.vtx_id = vtx_id;
  r.seq = seq;
  r.profile = profile;
  r.fec_overhead_base = ov_base_16ths / 16.0;
  r.fec_overhead_enh = ov_enh_16ths / 16.0;
  return pack_rcf(r);
}

// Convenience overload: the common case where a test wants the same
// overhead commanded on both wire fields (RC_VERSION 4 behavior). Kept so
// the ~40 existing call sites don't need touching.
std::vector<uint8_t> make_rcf_wire(uint32_t vtx_id, uint16_t seq, uint8_t profile,
                                    uint8_t ov_16ths) {
  return make_rcf_wire(vtx_id, seq, profile, ov_16ths, ov_16ths);
}

std::vector<uint8_t> make_disc_wire(uint32_t vtx_id, uint32_t nonce, uint8_t op_channel,
                                     uint8_t op_width, uint8_t init_profile, uint16_t seq) {
  Disc d;
  d.vtx_id = vtx_id;
  d.vrx_nonce = nonce;
  d.op_channel = op_channel;
  d.op_width = op_width;
  d.init_profile = init_profile;
  d.seq = seq;
  return pack_disc(d);
}

}  // namespace

// 1. BOOT->RENDEZVOUS on first tick; op is MAX_RANGE (both slots mcs0, enh
// shed, ov 2.0).
TEST(boot_first_tick_applies_max_range_and_moves_to_rendezvous) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  CHECK(agent.state() == RcAgent::State::BOOT);

  agent.tick(0, RadioHealth{});

  CHECK(agent.state() == RcAgent::State::RENDEZVOUS);
  REQUIRE(!act.applied.empty());
  const AppliedOp& op = act.applied.back();
  auto expect_ladder = ladder_from(PhyMode::HT, 0, 20);
  for (int i = 0; i < 2; ++i) {
    CHECK(op.ladder[static_cast<size_t>(i)].mode == expect_ladder[static_cast<size_t>(i)].mode);
    CHECK(op.ladder[static_cast<size_t>(i)].mcs == expect_ladder[static_cast<size_t>(i)].mcs);
    CHECK(op.ladder[static_cast<size_t>(i)].bw == expect_ladder[static_cast<size_t>(i)].bw);
  }
  // 2.0, not the wire-literal 1.0: apply_max_range's hardcoded constant
  // carries the old pre-Task-1 ×2 translation itself (see its comment).
  CHECK(op.fec_ov_base > 1.999 && op.fec_ov_base < 2.001);
  CHECK(op.fec_ov_enh > 1.999 && op.fec_ov_enh < 2.001);
  CHECK(op.shed[0] == false);
  CHECK(op.shed[1] == true);

  // BOOT's initial MAX_RANGE apply forces the bitrate policy to the robust
  // MCS0 floor immediately. Both slots are mcs0 (base clamps at 0), so the
  // blend collapses to the single-rate case: denom = (1+ov)/rate =
  // 3.0/6.5 = 0.46154, kbps = 1000*0.65/0.46154 = 1408.33 -> rounds to 1400.
  REQUIRE(!act.bitrates.empty());
  CHECK(act.bitrates.back() == 1400);
}

// 2. DISC (vtx_id matches cfg) -> send_control called once with a valid
// DISC_ACK (nonce echo matches) and state LINKED. The GS requests a
// DIFFERENT channel/width (36/40) than the drone's own cfg.radio (149/20,
// the RadioCfg struct defaults) specifically to catch a DISC_ACK that
// echoes the GS's request instead of reporting reality — the drone never
// retunes in v1, so echoing the request would misreport an agreement that
// never happened.
TEST(disc_replies_disc_ack_and_moves_to_linked) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  auto wire = make_disc_wire(cfg.link.vtx_id, 0xCAFEF00D, /*op_channel=*/36,
                              /*op_width=*/40, 0, 2);
  agent.on_rc_frame(wire.data(), wire.size(), 100);

  CHECK(agent.state() == RcAgent::State::LINKED);
  REQUIRE(act.controls.size() == 1);
  auto parsed = parse_disc_ack(act.controls[0].data(), act.controls[0].size());
  REQUIRE(parsed.has_value());
  CHECK(parsed->vtx_id == cfg.link.vtx_id);
  CHECK(parsed->vrx_nonce == 0xCAFEF00D);
  // Must reflect the drone's own configured channel/width (149/20), NOT the
  // GS-requested 36/40 — the drone doesn't retune in v1.
  CHECK(parsed->agreed_channel == cfg.radio.channel);
  CHECK(parsed->agreed_width == cfg.radio.width);
  CHECK(parsed->agreed_channel == 149);
  CHECK(parsed->agreed_width == 20);

  // DISC apply must force-run the bitrate policy immediately (same force
  // semantics as any other LINKED-entering transition), not leave the
  // encoder stuck at whatever bitrate was last set until the first RCF.
  REQUIRE(!act.bitrates.empty());
}

// 2c. DiscAck.chip_caps always advertises CAP_FRAME_WIRE: the frame wire is
// the only video format maburd speaks since the pre-frame-shm path was
// deleted. The bit stays on the wire so a GS can refuse a peer without it.
TEST(disc_ack_advertises_frame_wire_cap) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  auto wire = make_disc_wire(cfg.link.vtx_id, 0xCAFEF00D, /*op_channel=*/36,
                              /*op_width=*/40, 0, 2);
  agent.on_rc_frame(wire.data(), wire.size(), 100);

  REQUIRE(act.controls.size() == 1);
  auto parsed = parse_disc_ack(act.controls[0].data(), act.controls[0].size());
  REQUIRE(parsed.has_value());
  CHECK(parsed->chip_caps & mabur::rc::CAP_FRAME_WIRE);
}

// 2b. Keep-alive DISC while LINKED: ACK-ONLY. The drone must reply with a
// DiscAck (so a rebooted GS re-learns chip_caps — the stale-caps deadlock,
// 2026-08-12/2026-08-28) but must otherwise treat the DISC as noise: no
// init-profile apply (the 2026-07-12 op-thrash fix stands), no state
// change, no failsafe-watchdog refresh, no bitrate re-force.
TEST(keepalive_disc_while_linked_acks_without_op_change) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  // Link via RCF at a non-default rung (mcs2, ov 0.5).
  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto rcf = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(rcf.data(), rcf.size(), 100);
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  const uint64_t gen = agent.current().generation;
  const size_t n_controls = act.controls.size();
  const size_t n_bitrates = act.bitrates.size();

  auto disc = make_disc_wire(cfg.link.vtx_id, 0xCAFEF00D, 149, 20,
                             /*init_profile=*/0, /*seq=*/7);
  agent.on_rc_frame(disc.data(), disc.size(), 600);

  // Exactly one new control frame: a well-formed DiscAck echoing our real
  // caps and channel, and the DISC's nonce/seq.
  REQUIRE(act.controls.size() == n_controls + 1);
  auto parsed = parse_disc_ack(act.controls.back().data(),
                               act.controls.back().size());
  REQUIRE(parsed.has_value());
  CHECK(parsed->vtx_id == cfg.link.vtx_id);
  CHECK(parsed->vrx_nonce == 0xCAFEF00D);
  CHECK(parsed->seq == 7);
  CHECK(parsed->chip_caps & mabur::rc::CAP_FRAME_WIRE);
  CHECK(parsed->agreed_channel == cfg.radio.channel);
  CHECK(parsed->agreed_width == cfg.radio.width);

  // Everything else about the DISC is still ignored.
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current().generation == gen);   // op untouched (no MAX_RANGE yank)
  CHECK(agent.current().ladder[1].mcs == 2);  // enh still at the RCF's mcs
  CHECK(act.bitrates.size() == n_bitrates);   // bitrate policy not re-forced

  // Watchdog untouched: last real feedback was the RCF at t=100, so
  // failsafe_ms=1000 fires at t=1100 despite the DISC at t=600.
  agent.tick(1099, RadioHealth{});
  CHECK(agent.state() == RcAgent::State::LINKED);
  agent.tick(1100, RadioHealth{});
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
}

// 3. RCF profile HT mcs2/20, fec16=8 (ov=0.5) -> op ladder is BASE=mcs2,
// ENH=mcs2 (same-rate ruling, 2026-08-30 spec same-rate-fixed-pairs), ov
// 0.5; set_bitrate_kbps called with the weighted target: rate_b=rate_e=19.5
// (mcs2, both slots), ovb==ove so the fixed share cancels out, denom =
// 0.6*1.5/19.5 + 0.4*1.5/19.5 = 0.076923, kbps = 1000*0.65/0.076923 =
// 8450.0 -> rounds to 8500.
// link-rtt: the telem echo must name the RCF rcf_age_ms is aging against,
// and go INVALID whenever last_fb_ms_ was refreshed by something that is
// not an RCF. Failsafe entry is exactly that case: it resets the seq
// window AND rebases last_fb_ms_ to now, so a fresh-looking age paired
// with a stale echoed seq would let the GS fabricate an RTT sample from
// the wrong send time. (A keepalive DISC while LINKED changes nothing —
// feedback state included — so the echo correctly stays valid there.)
TEST(last_feedback_seq_tracks_rcf_and_invalidates_on_failsafe) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  CHECK(!agent.last_feedback_seq().has_value());  // no RCF ever

  auto rcf = make_rcf_wire(1, 4711, encode_profile(PhyMode::HT, 5, 20), 4);
  agent.on_rc_frame(rcf.data(), rcf.size(), 100);
  REQUIRE(agent.last_feedback_seq().has_value());
  CHECK(*agent.last_feedback_seq() == 4711);

  agent.tick(100 + static_cast<uint64_t>(cfg.link.failsafe_ms), RadioHealth{});
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  CHECK(agent.have_feedback());  // last_fb_ms_ rebased, still "has feedback"
  CHECK(!agent.last_feedback_seq().has_value());
}

TEST(rcf_apply_computes_ladder_fec_and_bitrate) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(wire.data(), wire.size(), 100);

  CHECK(agent.state() == RcAgent::State::LINKED);
  const AppliedOp& op = agent.current();
  CHECK(op.ladder[0].mcs == 2);  // BASE = mcs (same-rate)
  CHECK(op.ladder[1].mcs == 2);  // ENH = mcs
  CHECK(op.fec_ov_base > 0.499 && op.fec_ov_base < 0.501);
  CHECK(op.fec_ov_enh > 0.499 && op.fec_ov_enh < 0.501);

  REQUIRE(!act.bitrates.empty());
  CHECK(act.bitrates.back() == 8500);
}

// 3a. AppliedOp carries the RCF's base/enh overheads as a genuine PAIR, not
// folded into one value (Task 6, RC_VERSION 5): base=1.0, enh=0.5 must
// survive on_rc_frame -> apply_ladder_op -> AppliedOp distinctly.
TEST(applied_op_carries_distinct_base_enh_overhead_pair) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  // ov_base=1.0 (16/16), ov_enh=0.5 (8/16) -- distinct on the wire.
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 16, 8);
  agent.on_rc_frame(wire.data(), wire.size(), 100);

  CHECK(agent.state() == RcAgent::State::LINKED);
  REQUIRE(!act.applied.empty());
  const AppliedOp& op = act.applied.back();
  CHECK(op.fec_ov_base > 0.999 && op.fec_ov_base < 1.001);
  CHECK(op.fec_ov_enh > 0.499 && op.fec_ov_enh < 0.501);
}

// 3b. The clamp, isolated from the mcs2 case above: profile mcs5 ->
// BASE=mcs5, ENH=mcs5 (52 Mbps, both slots — same-rate ruling). cfg:
// airtime_budget 0.60, ov (RCF literal) 0.50 on both pair members.
// denom = 0.6*1.5/52 + 0.4*1.5/52 = 0.028846, kbps =
// 1000*0.60/0.028846 = 20800 -> clamps to bitrate_max_kbps (20000, raised
// here from the prod default 10000). Same-rate plus an equal pair makes
// the share weighting cancel, so this case only proves the clamp; 3d
// exercises the weighting via a distinct pair.
TEST(bitrate_policy_clamps_to_bitrate_max) {
  Config cfg = make_cfg();
  cfg.encoder.airtime_budget = 0.60;
  cfg.encoder.bitrate_max_kbps = 20000;
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);  // ov16=8 -> 0.5
  agent.on_rc_frame(wire.data(), wire.size(), 100);

  CHECK(agent.state() == RcAgent::State::LINKED);
  REQUIRE(!act.bitrates.empty());
  CHECK(act.bitrates.back() == 20000);
}

// 3c. MAX_RANGE degenerate case: both ladder slots are mcs0 (base clamps at
// 0) and both pair members are 2.0, so the weighting cancels and the
// failsafe floor bitrate must be unchanged by the policy rewrites.
// denom = (1+2.0)/6.5 = 0.46154, kbps = 1000*0.60/0.46154 = 1300.0.
TEST(bitrate_policy_failsafe_degenerates_to_single_rate) {
  Config cfg = make_cfg();
  cfg.encoder.airtime_budget = 0.60;
  MockActuator act;
  RcAgent agent(cfg, act);

  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS, MAX_RANGE applied

  CHECK(agent.state() == RcAgent::State::RENDEZVOUS);
  REQUIRE(!act.bitrates.empty());
  CHECK(act.bitrates.back() == 1300);
}

// 3d. The commanded overhead PAIR is weighted by the FIXED base share
// (kShareBase = 0.60), with no measured term anywhere: AirFeed is deleted
// and the target is once again a pure function of the operating point
// (2026-09-01, "be like master"). Same-rate ruling means BASE and ENH share
// one PHY rate (mcs5, 52 Mbps both slots), so with ovb==ove the ov term
// factors out of the weighted sum entirely and the share is unobservable —
// this test therefore uses a DISTINCT pair (ov_base=0.5, ov_enh=1.0), which
// is what makes both the weight and the ov->weight pairing testable.
//   denom = f0*(1+ovb)/rate + (1-f0)*(1+ove)/rate
//         = [0.60*1.50 + 0.40*2.00]/52 = 1.70/52 = 0.0326923
//   kbps  = 1000*0.60/0.0326923 = 18352.9 -> round100 = 18400.
// Contrast wrong wirings, both of which land elsewhere:
//   - swap ovb/ove: [0.60*2.00 + 0.40*1.50]/52 = 1.80/52 -> 17300.
//   - fb left at 0.50 (share constant not applied): 1.75/52 -> 17800.
TEST(bitrate_policy_weights_pair_by_fixed_share) {
  Config cfg = make_cfg();
  cfg.encoder.airtime_budget = 0.60;
  cfg.encoder.bitrate_max_kbps = 20000;
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  // ov_base=0.5 (8/16), ov_enh=1.0 (16/16) -- distinct, so ovb/ove stay
  // observable in the weighted target instead of factoring out.
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8, 16);
  agent.on_rc_frame(wire.data(), wire.size(), 100);

  CHECK(agent.state() == RcAgent::State::LINKED);
  REQUIRE(!act.bitrates.empty());
  CHECK(act.bitrates.back() == 18400);
}

// 3e. The target is a pure function of the operating point: the SAME op
// must produce the SAME bitrate no matter what the transport has been
// doing. This is the regression guard for the deleted AirFeed blend, whose
// measured share/excess terms made the target drift under a fixed op —
// 151 of 226 bitrate writes in flight-0000 had no rung change behind them,
// and on Star6E every write costs a keyframe (median 63.8 kB in flight).
// Feeding the same RCF op repeatedly across 30 s must therefore yield
// exactly ONE distinct commanded value.
TEST(bitrate_policy_is_constant_for_a_fixed_op) {
  Config cfg = make_cfg();
  cfg.encoder.airtime_budget = 0.60;
  cfg.encoder.bitrate_max_kbps = 20000;
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  uint8_t profile_byte = encode_profile(PhyMode::HT, 3, 20);
  for (uint16_t seq = 1; seq <= 300; ++seq) {
    auto wire = make_rcf_wire(cfg.link.vtx_id, seq, profile_byte, 16, 4);
    agent.on_rc_frame(wire.data(), wire.size(), 100 + seq * 100);
    agent.tick(100 + seq * 100, RadioHealth{});
  }

  CHECK(agent.state() == RcAgent::State::LINKED);
  REQUIRE(!act.bitrates.empty());
  std::set<int> distinct(act.bitrates.begin(), act.bitrates.end());
  // MAX_RANGE's boot floor, then the one steady-state value for this op.
  CHECK(distinct.size() == 2);
  //   denom = [0.60*(1+1.0) + 0.40*(1+0.25)]/26 = 1.70/26 = 0.0653846
  //   kbps  = 600/0.0653846 = 9176.5 -> round100 = 9200.
  CHECK(act.bitrates.back() == 9200);
}

// 4. Stale seq (same seq again, then seq-1) -> no new apply_op (generation
// unchanged).
TEST(stale_seq_is_ignored) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(cfg.link.vtx_id, 10, profile_byte, 8);
  agent.on_rc_frame(wire.data(), wire.size(), 100);
  uint64_t gen_after_first = agent.current().generation;

  // Same seq again.
  agent.on_rc_frame(wire.data(), wire.size(), 200);
  CHECK(agent.current().generation == gen_after_first);

  // seq - 1 (stale/old).
  auto stale_wire = make_rcf_wire(cfg.link.vtx_id, 9, profile_byte, 8);
  agent.on_rc_frame(stale_wire.data(), stale_wire.size(), 300);
  CHECK(agent.current().generation == gen_after_first);
}

// 5. Corrupt RCF (flip a byte) -> ignored.
TEST(corrupt_rcf_is_ignored) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  wire[5] ^= 0xFF;  // corrupt a payload byte -> CRC mismatch

  uint64_t gen_before = agent.current().generation;
  auto state_before = agent.state();
  agent.on_rc_frame(wire.data(), wire.size(), 100);

  CHECK(agent.current().generation == gen_before);
  CHECK(agent.state() == state_before);
}

// 6. Silence: last RCF at t=0, tick at t=999 -> LINKED; t=1000 -> FAILSAFE +
// MAX_RANGE reapplied; t=31000 -> RENDEZVOUS.
TEST(failsafe_and_rendezvous_timers_fire_exactly) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS at t=0

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(wire.data(), wire.size(), 0);  // RCF at t=0 -> LINKED
  CHECK(agent.state() == RcAgent::State::LINKED);

  agent.tick(999, RadioHealth{});
  CHECK(agent.state() == RcAgent::State::LINKED);

  agent.tick(1000, RadioHealth{});
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  const AppliedOp& op = agent.current();
  CHECK(op.ladder[0].mcs == 0);
  CHECK(op.fec_ov_base > 1.999 && op.fec_ov_base < 2.001);
  CHECK(op.fec_ov_enh > 1.999 && op.fec_ov_enh < 2.001);

  // LINKED->FAILSAFE entry forces the bitrate policy to the MCS0 floor
  // (1400 kbps) immediately, bypassing the steady-state throttle/hysteresis
  // — the encoder must not keep flooding at the last LINKED bitrate (8500)
  // once the radio has dropped to the robust MAX_RANGE profile.
  REQUIRE(!act.bitrates.empty());
  CHECK(act.bitrates.back() == 1400);

  agent.tick(30999, RadioHealth{});
  CHECK(agent.state() == RcAgent::State::FAILSAFE);

  agent.tick(31000, RadioHealth{});
  CHECK(agent.state() == RcAgent::State::RENDEZVOUS);
}

// 7. RCF after failsafe -> LINKED + request_idr called.
TEST(rcf_after_failsafe_requests_idr) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(wire.data(), wire.size(), 0);
  agent.tick(1000, RadioHealth{});  // -> FAILSAFE
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  int idr_before = act.idr_calls;

  auto wire2 = make_rcf_wire(cfg.link.vtx_id, 2, profile_byte, 8);
  agent.on_rc_frame(wire2.data(), wire2.size(), 1500);

  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(act.idr_calls == idr_before + 1);
}

// 9. tx_drops rising 2 ticks -> shed_level_ 1 then 2, both sheding the enh
// layer (shed[1] — the sole droppable layer left, the old reserved layer
// and its shed[2] slot are gone; shed_level_ still counts 0..3, so level 2
// is not visible as a SEPARATE flag anymore, only as one more step the
// 2s-per-level decay has to climb down); 2s clean -> back off. Refreshed
// with a periodic RCF (every <failsafe_ms=1000ms) throughout so the agent
// stays LINKED for the whole scenario — without it, the last two ticks
// (2300/4400ms with no RCF since t=0) land past failsafe_ms and silently
// transition LINKED->FAILSAFE, which forces shed[1] true via failsafe_shed_
// (see C2(b)) and would make this congestion-only decay test spuriously
// fail/pass for the wrong reason.
TEST(congestion_shed_escalates_and_recovers) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(wire.data(), wire.size(), 0);
  CHECK(agent.current().shed[1] == false);
  CHECK(agent.congestion_shed() == false);

  agent.tick(100, RadioHealth{0, 5});  // drops rose 0->5: level 1 (shed enh)
  CHECK(agent.current().shed[1] == true);
  // Telemetry accessor (flags bit4): any level >= 1 is "congestion shed",
  // independent of failsafe_shed() — this test never leaves LINKED.
  CHECK(agent.congestion_shed() == true);
  CHECK(agent.failsafe_shed() == false);

  agent.tick(200, RadioHealth{0, 10});  // drops rose 5->10: level 2 (still sheds enh — no 2nd layer left to differentiate)
  CHECK(agent.current().shed[1] == true);

  // Keep LINKED alive with fresh RCFs (seq increasing) before the failsafe
  // window (1000ms since last feedback) would otherwise elapse.
  auto wire2 = make_rcf_wire(cfg.link.vtx_id, 2, profile_byte, 8);
  agent.on_rc_frame(wire2.data(), wire2.size(), 900);
  auto wire3 = make_rcf_wire(cfg.link.vtx_id, 3, profile_byte, 8);
  agent.on_rc_frame(wire3.data(), wire3.size(), 1800);

  // 2000ms clean (no new drops) -> level decrements 2->1, still sheding enh.
  agent.tick(2300, RadioHealth{0, 10});
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current().shed[1] == true);

  auto wire4 = make_rcf_wire(cfg.link.vtx_id, 4, profile_byte, 8);
  agent.on_rc_frame(wire4.data(), wire4.size(), 2700);
  auto wire5 = make_rcf_wire(cfg.link.vtx_id, 5, profile_byte, 8);
  agent.on_rc_frame(wire5.data(), wire5.size(), 3600);

  // A second 2000ms clean window: level decrements 1->0, shed lifts.
  agent.tick(4400, RadioHealth{0, 10});
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current().shed[1] == false);
}

// 9a. TxQueue backpressure is congestion too. The guard's only trigger was
// TxStats::failed (USB bulk-OUT failures), so when the encoder overshot
// the pipe the queue ran to its cap and drop-oldest threw bodies away with
// shed_level_ still 0 -- and those drops are indistinguishable from RF
// loss at the GS, which booked them as residual and demoted 5->4->3->2 in
// 450 ms at 36 dB SNR (flight-0011 @88 s / @102 s, 2026-09-03). A queue at
// or past half its cap must shed the enh layer BEFORE the first drop (a
// shed is invisible to the ladder: no s3 traffic, no s3 decision); below
// that it must not. Same 2 s clean decay as the USB trigger. REVERT CHECK:
// ignore txq_depth/txq_cap in run_congestion_guard and the t=100 tick
// leaves shed[1] false.
TEST(txq_pressure_sheds_enh_before_drops) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(wire.data(), wire.size(), 0);
  CHECK(agent.current().shed[1] == false);

  RadioHealth h;
  h.txq_cap = 255;
  h.txq_depth = 64;  // a quarter full: normal burst headroom, no shed
  agent.tick(100, h);
  CHECK(agent.current().shed[1] == false);

  h.txq_depth = 128;  // half the cap: pressure, shed enh (no USB failure, no drop yet)
  agent.tick(200, h);
  CHECK(agent.current().shed[1] == true);
  CHECK(agent.current().ladder[0].mcs == 5);  // op otherwise untouched

  // Keep LINKED alive (failsafe_ms = 1000) while the queue drains.
  auto wire2 = make_rcf_wire(cfg.link.vtx_id, 2, profile_byte, 8);
  agent.on_rc_frame(wire2.data(), wire2.size(), 900);
  auto wire3 = make_rcf_wire(cfg.link.vtx_id, 3, profile_byte, 8);
  agent.on_rc_frame(wire3.data(), wire3.size(), 1800);

  h.txq_depth = 0;
  agent.tick(2300, h);  // 2000 ms since the last pressure tick: level 1 -> 0
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current().shed[1] == false);
}

// 9b. FAILSAFE entry forces shed[1]; a subsequent congestion-guard reapply
// (which recomputes shed[1] from shed_level_ alone) must not clobber the
// failsafe-forced shed — it has to OR failsafe_shed_ in. Also covers:
// reapply_with_shed() must publish (act.apply_op called again) even though
// it never bumps generation.
TEST(failsafe_shed_survives_congestion_reapply) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS, MAX_RANGE: shed[1]=true

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(wire.data(), wire.size(), 0);  // -> LINKED, shed[1]=false
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current().shed[1] == false);

  agent.tick(1000, RadioHealth{});  // silence -> FAILSAFE, MAX_RANGE reapplied
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  CHECK(agent.current().shed[1] == true);
  uint64_t gen_at_failsafe = agent.current().generation;
  size_t applies_at_failsafe = act.applied.size();

  // Congestion guard reapply while still in FAILSAFE: generation must NOT
  // bump (reapply_with_shed never touches it), but the actuator must see a
  // fresh apply_op (a new object every time, per the AppliedOp::generation
  // doc comment) and shed[1] must remain forced true — NOT recomputed down
  // to shed_level_'s (0) sheds.
  agent.tick(1100, RadioHealth{0, 5});  // tx_drops rose 0->5
  CHECK(agent.current().generation == gen_at_failsafe);
  CHECK(act.applied.size() > applies_at_failsafe);
  CHECK(agent.current().shed[1] == true);
}

// 9c. Proves the main.cpp hot-loop seam directly: an identity-compare pump
// (mirroring apply_op_to_uep's callers in main.cpp) observes every
// congestion-triggered shed reapply even though reapply_with_shed never
// bumps generation, whereas a generation-compare pump (the pre-fix
// behavior) would observe only the very first op and then silently miss
// every subsequent shed-only republish — reproducing finding C2(a).
TEST(identity_compare_seam_catches_shed_only_republish_generation_compare_misses) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(wire.data(), wire.size(), 0);  // -> LINKED, shed all false
  CHECK(agent.current().shed[1] == false);

  // Simulate the atomic shared_ptr handoff main.cpp uses: MockActuator
  // doesn't publish shared_ptrs, so build the same sequence of AppliedOp
  // snapshots RealActuator::apply_op would have published (one per
  // act.applied entry so far) and feed them through both a generation-gated
  // pump and an identity-gated pump, exactly mirroring apply_op_to_uep's
  // two call sites in main.cpp.
  std::vector<std::shared_ptr<const AppliedOp>> published;
  for (const auto& op : act.applied) published.push_back(std::make_shared<const AppliedOp>(op));

  uint64_t last_gen = 0;
  bool have_gen = false;
  int gen_compare_applies = 0;
  for (const auto& op : published) {
    if (!have_gen || op->generation != last_gen) {
      ++gen_compare_applies;
      last_gen = op->generation;
      have_gen = true;
    }
  }

  std::shared_ptr<const AppliedOp> last_applied;
  int identity_compare_applies = 0;
  for (const auto& op : published) {
    if (op != last_applied) {
      ++identity_compare_applies;
      last_applied = op;
    }
  }

  // Now drive a congestion-only shed change (no new generation) and append
  // its published op to both replay sequences.
  agent.tick(100, RadioHealth{0, 5});  // tx_drops rose 0->5 -> shed[1]=true, no gen bump
  CHECK(agent.current().shed[1] == true);
  auto congestion_op = std::make_shared<const AppliedOp>(act.applied.back());

  if (!have_gen || congestion_op->generation != last_gen) ++gen_compare_applies;
  if (congestion_op != last_applied) ++identity_compare_applies;

  // The congestion-only republish carries the SAME generation as the prior
  // RCF op, so a generation-gated pump must NOT have counted it (reproducing
  // the bug: the shed change never reaches the encoder) while the
  // identity-gated pump (the fix) must count every distinct object,
  // including this one.
  CHECK(congestion_op->generation == published.back()->generation);
  CHECK(identity_compare_applies == static_cast<int>(published.size()) + 1);
  CHECK(gen_compare_applies < identity_compare_applies);
}

// 10. Bitrate hysteresis: same RCF twice within 1s -> one set_bitrate_kbps.
TEST(bitrate_policy_hysteresis_within_one_second) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire1 = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(wire1.data(), wire1.size(), 0);
  size_t count_after_first = act.bitrates.size();
  REQUIRE(count_after_first >= 1);

  auto wire2 = make_rcf_wire(cfg.link.vtx_id, 2, profile_byte, 8);
  agent.on_rc_frame(wire2.data(), wire2.size(), 500);  // same op, within 1s
  CHECK(act.bitrates.size() == count_after_first);
}

// 10b. A demote cascade must shed the encoder on EVERY decreased target in
// the same tick that applies the MCS. The radio capacity has already
// dropped when the RCF lands; a swallowed shed means the encoder floods a
// smaller pipe and TxQueue drop-oldest kills whole FEC bodies (the
// 2026-08-09 freeze-crash — docs/shed-lag-findings-2026-08-09.md). The
// v1 throttle/hysteresis exist to dedup repeated identical RCFs, never to
// defer a decrease.
TEST(demote_cascade_sheds_every_decrease_immediately) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  // Enter LINKED at the top rung: mcs7, ov 2/16 -> clamp to max = 20000.
  auto top = make_rcf_wire(cfg.link.vtx_id, 1, encode_profile(PhyMode::HT, 7, 20), 2);
  agent.on_rc_frame(top.data(), top.size(), 0);
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  REQUIRE(!act.bitrates.empty());
  CHECK(act.bitrates.back() == 20000);

  // Steady top-rung RCFs for 3s: no further calls (dedup), stamp expires.
  uint16_t seq = 2;
  for (uint64_t t = 100; t <= 3000; t += 100) {
    auto w = make_rcf_wire(cfg.link.vtx_id, seq++,
                            encode_profile(PhyMode::HT, 7, 20), 2);
    agent.on_rc_frame(w.data(), w.size(), t);
  }
  size_t steady = act.bitrates.size();

  // ctl-0020-shaped cascade at RCF cadence: each step's lower target must
  // go out on the SAME on_rc_frame call, throttle stamp notwithstanding.
  struct Step { uint8_t mcs, ov16; int kbps; };
  // Re-derived for the same-rate ruling (both slots ride the scored mcs,
  // fb=0.5, budget 0.65, no feed). mcs4/ov0.25 (the old step 1) now
  // computes to 20280 -> clamps to 20000, identical to the top rung above
  // it, so it can't prove a swallowed decrease; ov0.5 is used instead to
  // keep this step's target strictly below the top rung's clamp:
  //  mcs4/ov0.5: rate=39 (both slots), denom=(1+0.5)/39=0.0384615,
  //    kbps=650/0.0384615=16900.0 -> 16900.
  //  mcs2/ov0.5: rate=19.5 (both slots), denom=(1+0.5)/19.5=0.0769231,
  //    kbps=650/0.0769231=8450.0 -> 8500.
  //  mcs0/ov1.0: both slots clamp to mcs0 (6.5), denom=(1+1.0)/6.5=0.30769,
  //    kbps=650/0.30769=2112.5 -> 2100. (Unaffected by the ruling: mcs0
  //    already clamped both slots to the floor under the old mcs-1 rule
  //    too. No longer collides with the MAX_RANGE floor's 1400: that is
  //    ov=2.0, not this RCF's literal 1.0 — the old uep_layer_overhead
  //    clamp-to-2.0 translation that made them coincide is gone since
  //    Task 1/RC_VERSION 4.)
  const Step down[] = {{4, 8, 16900}, {2, 8, 8500}, {0, 16, 2100}};
  uint64_t t = 3100;
  for (const Step& d : down) {
    auto w = make_rcf_wire(cfg.link.vtx_id, seq++,
                            encode_profile(PhyMode::HT, d.mcs, 20), d.ov16);
    agent.on_rc_frame(w.data(), w.size(), t);
    REQUIRE(!act.bitrates.empty());
    CHECK(act.bitrates.back() == d.kbps);
    t += 100;
  }
  CHECK(act.bitrates.size() == steady + 3);
}

// 10c. Increases stay lazy: a higher target inside the 1s throttle window
// is deferred (a late quality bump is harmless; only decreases are
// safety-critical). Guard for the 10b change.
TEST(bitrate_increase_still_gated) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  // Enter LINKED at mcs2/ov0.5 -> 8500 (forced entry call, stamp t=0; see
  // test 3/10b's blended-formula derivation for this mcs/ov combo).
  auto w0 = make_rcf_wire(cfg.link.vtx_id, 1, encode_profile(PhyMode::HT, 2, 20), 8);
  agent.on_rc_frame(w0.data(), w0.size(), 0);
  REQUIRE(act.bitrates.back() == 8500);

  // Promote to mcs4/ov0.5 (16900, see 10b's derivation) once the stamp
  // expired: call fires. (ov0.5, not the old ov0.25 -- that combo now
  // clamps to 20000 under same-rate, same as the mcs7 step below, and
  // couldn't show a genuine further increase.)
  auto w1 = make_rcf_wire(cfg.link.vtx_id, 2, encode_profile(PhyMode::HT, 4, 20), 8);
  agent.on_rc_frame(w1.data(), w1.size(), 1100);
  REQUIRE(act.bitrates.back() == 16900);
  size_t n = act.bitrates.size();

  // Further promote to mcs7 (20000) 100ms later: inside the throttle
  // window -> deferred.
  auto w2 = make_rcf_wire(cfg.link.vtx_id, 3, encode_profile(PhyMode::HT, 7, 20), 2);
  agent.on_rc_frame(w2.data(), w2.size(), 1200);
  CHECK(act.bitrates.size() == n);

  // Same op re-sent after the window: goes out.
  auto w3 = make_rcf_wire(cfg.link.vtx_id, 4, encode_profile(PhyMode::HT, 7, 20), 2);
  agent.on_rc_frame(w3.data(), w3.size(), 2200);
  CHECK(act.bitrates.back() == 20000);
}

// 10d. A promote must reach the encoder even when bitrate_max_kbps clamps
// the new target to within 10% of the last applied value. Measured on the
// bench 2026-08-28: prod runs airtime_budget 0.60 / bitrate_max 10000.
// Re-derived for the same-rate ruling (both slots ride the scored mcs,
// ov literal per rung, budget 0.60):
//   rung4 (mcs4/ov1.5 -- ov16=24, not the two-rate-era 16/1.0, which now
//     computes unclamped to 11700 and can't demonstrate the trap): rate=39
//     (both slots), denom=(1+1.5)/39=0.064103, kbps=600/0.064103=9360.0
//     -> 9400.
//   rung5 (mcs5/ov1.0 -- ov16=16, unchanged): rate=52 (both slots),
//     denom=(1+1.0)/52=0.038462, kbps=600/0.038462=15600.0, clamps to
//     10000.
// |10000-9400| = 600 is inside the v1 `changed_enough` deadband (last/10 =
// 940) that existed until 2026-08-28, so the promote was swallowed and the
// encoder stayed on the mcs4 bitrate forever while the link ran mcs5 -- a
// permanent 6% undershoot at the rung the link sits on nearly all the
// time. The deadband is a filter on an absolute target with no
// accumulator, so the error can never grow into the band: 9400 is a fixed
// point. Dedup of repeated identical RCFs is the 1s throttle's job (test
// 10 and 10b's steady window pin that); this test pins that a genuinely
// CHANGED target is never discarded for being too small a step.
TEST(promote_reaches_encoder_when_clamp_puts_target_inside_deadband) {
  Config cfg = make_cfg();
  cfg.encoder.airtime_budget = 0.60;    // prod value (/etc/mabur.json)
  cfg.encoder.bitrate_max_kbps = 10000; // prod value; this clamp is the trap
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  // Enter LINKED at rung 4: mcs4/ov1.5 -> 9360.0 -> 9400 (see the
  // derivation above).
  auto w0 = make_rcf_wire(cfg.link.vtx_id, 1, encode_profile(PhyMode::HT, 4, 20), 24);
  agent.on_rc_frame(w0.data(), w0.size(), 0);
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  REQUIRE(!act.bitrates.empty());
  REQUIRE(act.bitrates.back() == 9400);

  // Promote to rung 5 well after the 1s throttle window: mcs5/ov1.0 ->
  // 15600.0, clamped to 10000. Only 600 above the last applied value.
  auto w1 = make_rcf_wire(cfg.link.vtx_id, 2, encode_profile(PhyMode::HT, 5, 20), 16);
  agent.on_rc_frame(w1.data(), w1.size(), 1100);
  CHECK(act.bitrates.back() == 10000);
}

// 10e. The congestion guard must never command the encoder below
// waybeam.bitrate_min_kbps. run_bitrate_policy fences its own write with
// clamp(kbps, bitrate_min_kbps, bitrate_max_kbps), but the shed-level-3 cut
// wrote act_.set_bitrate_kbps(last * 0.7) straight to the actuator with no
// clamp -- the only path in maburd that could go under the configured
// floor. It also compounded, because it overwrote last_bitrate_kbps_ with
// its own output, so each re-entry into level 3 multiplied the
// already-cut value: 1300 -> 910 -> 637 -> 446. In LINKED an incoming RCF
// repairs that within ~1s, but tick() never calls run_bitrate_policy, so
// in FAILSAFE/RENDEZVOUS (where the op is MAX_RANGE = mcs0) nothing
// restored it and the ratchet ran unopposed toward ~0.4 Mbps.
TEST(congestion_shed_never_commands_below_bitrate_min) {
  Config cfg = make_cfg();
  cfg.encoder.airtime_budget = 0.60;    // prod value (/etc/mabur.json)
  cfg.encoder.bitrate_max_kbps = 10000; // prod value
  MockActuator act;
  RcAgent agent(cfg, act);

  // BOOT tick applies MAX_RANGE (both slots mcs0, ov=2.0) -> the blend
  // collapses to the single-rate case: 1000*0.60*6.5/(1+2.0) = 1300, and
  // returns before the guard runs. State stays RENDEZVOUS: no RCF ever
  // arrives, so run_bitrate_policy is never called again.
  agent.tick(0, RadioHealth{});
  REQUIRE(!act.bitrates.empty());
  REQUIRE(act.bitrates.back() == 1300);

  uint64_t drops = 0;
  auto rise = [&](uint64_t at) {
    RadioHealth h; h.tx_drops = ++drops; agent.tick(at, h);
  };
  auto quiet = [&](uint64_t at) {
    RadioHealth h; h.tx_drops = drops; agent.tick(at, h);
  };

  rise(100); rise(200); rise(300);  // shed 0->1->2->3, first cut on the edge

  // Three more decay->re-entry cycles: 2s quiet drops 3->2, the next rise
  // climbs back to 3 and cuts again from the already-reduced value.
  uint64_t t = 300;
  for (int i = 0; i < 3; ++i) {
    t += 2100; quiet(t);
    t += 100;  rise(t);
  }

  int lowest = act.bitrates[0];
  for (int b : act.bitrates)
    if (b < lowest) lowest = b;
  CHECK(lowest >= cfg.encoder.bitrate_min_kbps);
}

TEST(link_established_latches_on_rendezvous_to_linked_rcf_not_on_failsafe_flap) {
  // BOOT/RENDEZVOUS -> LINKED is the process-(re)start scenario: frames
  // encoded before the link is up never reach the air (rig 2026-07-25:
  // discont_seen=0 at every stall onset), so main re-marks the frame
  // discontinuity window when the link first comes up. FAILSAFE -> LINKED
  // must NOT latch: a routine RF flap would re-base the GS's id space and
  // evict its in-flight frames for nothing.
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  CHECK(!agent.take_link_established());  // BOOT: nothing yet
  agent.tick(0, RadioHealth{});           // BOOT -> RENDEZVOUS
  CHECK(!agent.take_link_established());

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto rcf1 = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(rcf1.data(), rcf1.size(), 10);  // RENDEZVOUS -> LINKED
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.take_link_established());
  CHECK(!agent.take_link_established());  // consumed

  agent.tick(1010, RadioHealth{});        // feedback silence -> FAILSAFE
  REQUIRE(agent.state() == RcAgent::State::FAILSAFE);
  auto rcf2 = make_rcf_wire(cfg.link.vtx_id, 2, profile_byte, 8);
  agent.on_rc_frame(rcf2.data(), rcf2.size(), 1020);  // FAILSAFE -> LINKED
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  CHECK(!agent.take_link_established());  // flap, not a (re)start
}

// 11. RCF with probe3=true overrides only the enh layer (ladder[1], the old
// s3) with probe_profile's MCS while the base layer (ladder[0]) stays on
// the base profile's scored mcs (same-rate ruling); agent.probing()
// reflects the last accepted RCF's probe3 bit and reverts (both
// op.ladder[1] and probing()) the moment a follow-up RCF arrives without
// the flag.
TEST(probe_rcf_overrides_layer3_mcs) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  uint8_t probe_byte = encode_profile(PhyMode::HT, 6, 20);
  Rcf r;
  r.vtx_id = cfg.link.vtx_id;
  r.seq = 1;
  r.profile = profile_byte;
  r.fec_overhead_base = 0.5;
  r.fec_overhead_enh = 0.5;
  r.probe3 = true;
  r.probe_profile = probe_byte;
  auto wire = pack_rcf(r);
  agent.on_rc_frame(wire.data(), wire.size(), 0);

  CHECK(agent.state() == RcAgent::State::LINKED);
  const AppliedOp& op = agent.current();
  CHECK(op.ladder[0].mcs == 5);  // BASE = scored mcs, unaffected by the probe
  CHECK(op.ladder[1].mcs == 6);  // ENH at the probe MCS
  CHECK(agent.probing());

  // A follow-up RCF without the flag reverts the enh layer to the base
  // profile's mcs and clears probing().
  r.probe3 = false;
  r.seq = 2;
  auto wire2 = pack_rcf(r);
  agent.on_rc_frame(wire2.data(), wire2.size(), 100);
  CHECK(agent.current().ladder[1].mcs == 5);
  CHECK(!agent.probing());
}

// 11b. Failsafe entry (MAX_RANGE) clears probing() even if the last RCF
// before the silence carried probe3=true — a degraded/lost link must never
// report itself as still probing.
TEST(probing_cleared_on_failsafe) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  Rcf r;
  r.vtx_id = cfg.link.vtx_id;
  r.seq = 1;
  r.profile = encode_profile(PhyMode::HT, 5, 20);
  r.fec_overhead_base = 0.5;
  r.fec_overhead_enh = 0.5;
  r.probe3 = true;
  r.probe_profile = encode_profile(PhyMode::HT, 6, 20);
  auto wire = pack_rcf(r);
  agent.on_rc_frame(wire.data(), wire.size(), 0);
  CHECK(agent.probing());

  agent.tick(1000, RadioHealth{});  // silence -> FAILSAFE (MAX_RANGE reapplied)
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  CHECK(!agent.probing());
}

// 11c. A probe must not move the encoder bitrate (spec 2026-08-05 s3-probe-
// promote: "Base link (... encoder bitrate) is untouched; the probe changes
// MCS only" / "Bitrate policy stays keyed to the base profile"). Since the
// 2-slot op (2bbaa3f) the policy's enh term read the probed slot's PHY rate,
// so every probe entry raised the command (~+11-15% at low rungs) and every
// exit lowered it back -- two SetChnAttr writes, two forced IDRs, and the
// flight-0011 air backlog at rungs 1-3 (2026-09-03 analysis). The balancer
// that was meant to compensate the probe window's air shift was deleted
// 2026-09-01. REVERT CHECK: feed rate_e = phy_rate(ladder[1]) unguarded and
// the probe RCF writes 9400 (mcs2 base 19.5 / mcs3 probe 26.0, ov 0.5,
// budget 0.65) and the exit RCF writes 8500 again -> two writes past count_linked.
TEST(probe_rcf_does_not_change_bitrate) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  Rcf r;
  r.vtx_id = cfg.link.vtx_id;
  r.seq = 1;
  r.profile = encode_profile(PhyMode::HT, 2, 20);
  r.fec_overhead_base = 0.5;
  r.fec_overhead_enh = 0.5;
  auto wire = pack_rcf(r);
  agent.on_rc_frame(wire.data(), wire.size(), 0);  // RENDEZVOUS -> LINKED
  REQUIRE(!act.bitrates.empty());
  CHECK(act.bitrates.back() == 8500);  // 1000*0.65/(1.5/19.5) = 8450 -> 8500
  const size_t count_linked = act.bitrates.size();

  // Probe entry, past the 1 s write throttle so a changed target WOULD
  // be written: the enh slot flies mcs3 but the command must hold.
  r.seq = 2;
  r.probe3 = true;
  r.probe_profile = encode_profile(PhyMode::HT, 3, 20);
  auto wire2 = pack_rcf(r);
  agent.on_rc_frame(wire2.data(), wire2.size(), 1100);
  REQUIRE(agent.probing());
  CHECK(agent.current().ladder[1].mcs == 3);
  CHECK(act.bitrates.size() == count_linked);

  // Probe exit (the decrease path is never throttled): still no write.
  r.seq = 3;
  r.probe3 = false;
  auto wire3 = pack_rcf(r);
  agent.on_rc_frame(wire3.data(), wire3.size(), 1200);
  REQUIRE(!agent.probing());
  CHECK(act.bitrates.size() == count_linked);
}

// 2d. DiscAck.chip_caps advertises CAP_ENH_PROBE alongside the existing
// caps — this drone accepts RCF_F_PROBE_ENH.
TEST(disc_ack_advertises_s3_probe) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  auto wire = make_disc_wire(cfg.link.vtx_id, 0xCAFEF00D, /*op_channel=*/36,
                              /*op_width=*/40, 0, 2);
  agent.on_rc_frame(wire.data(), wire.size(), 100);

  REQUIRE(act.controls.size() == 1);
  auto parsed = parse_disc_ack(act.controls[0].data(), act.controls[0].size());
  REQUIRE(parsed.has_value());
  CHECK(parsed->chip_caps & mabur::rc::CAP_ENH_PROBE);
}

TEST(link_established_latches_on_disc_link_up) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS
  auto disc = make_disc_wire(cfg.link.vtx_id, 0xCAFEF00D, 149, 20,
                             /*init_profile=*/0, /*seq=*/1);
  agent.on_rc_frame(disc.data(), disc.size(), 10);  // RENDEZVOUS -> LINKED
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.take_link_established());
  CHECK(!agent.take_link_established());
}

MTEST_MAIN

// A restarted GS resets its RCF seq to ~0 while the drone's tracker holds
// the old session's high seq — Python has NO stale check (applies every
// valid RCF), so the port's replay protection must forget its baseline at
// session boundaries (failsafe entry / DISC re-link) or a GS restart locks
// the drone out for up to 32k seqs (bench 2026-07-12).
TEST(gs_restart_low_seq_accepted_after_failsafe) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto old_sess = make_rcf_wire(cfg.link.vtx_id, 40000, profile_byte, 8);
  agent.on_rc_frame(old_sess.data(), old_sess.size(), 100);
  REQUIRE(agent.state() == RcAgent::State::LINKED);

  // GS dies; failsafe fires.
  agent.tick(1200, RadioHealth{});
  REQUIRE(agent.state() == RcAgent::State::FAILSAFE);

  // Restarted GS: fresh seq numbering near zero must be accepted.
  auto new_sess = make_rcf_wire(cfg.link.vtx_id, 3, profile_byte, 8);
  agent.on_rc_frame(new_sess.data(), new_sess.size(), 1300);
  CHECK(agent.state() == RcAgent::State::LINKED);
  uint64_t gen = agent.current().generation;

  // In-session replay protection still works: same seq again is ignored.
  agent.on_rc_frame(new_sess.data(), new_sess.size(), 1400);
  CHECK(agent.current().generation == gen);
}

// 14. Spec 2026-08-28 venc-foldin §4: ONE IDR pacer in RcAgent. 100 ms min
// spacing overall; a chain-break request additionally holds off 1 s from the
// previous chain-break-triggered IDR. GS-requested IDRs (the
// RCF-after-failsafe path) share the same 100 ms floor.
//
// The chain-break intake runs BEFORE the failsafe/rendezvous timers in
// tick(), which is what the t=1300 leg pins: last feedback was the RCF at
// t=100 and failsafe_ms is 1000, so that same tick also drops the agent into
// FAILSAFE. The break happened while the link was up and one IDR is what
// heals it, so it must still go out.
TEST(idr_pacer_min_spacing_and_chain_break_holdoff) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto rcf = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(rcf.data(), rcf.size(), 100);  // LINKED
  const int base = act.idr_calls;

  agent.note_chain_break();
  agent.tick(200, RadioHealth{});          // first chain-break IDR fires
  CHECK(act.idr_calls == base + 1);

  agent.note_chain_break();
  agent.tick(300, RadioHealth{});          // inside 1 s holdoff: suppressed
  CHECK(act.idr_calls == base + 1);

  agent.note_chain_break();
  agent.tick(1300, RadioHealth{});         // past holdoff: fires
  CHECK(act.idr_calls == base + 2);
}

// 15. A refused encoder verb must not be latched as applied. Before this,
// run_bitrate_policy() recorded last_bitrate_kbps_ straight after the (void)
// actuator call, so one dropped MI call left `changed` false forever and the
// encoder ran the old rate for the rest of the flight — the waybeam bitrate
// wedge, recreated in-process.
// REVERT CHECK: latch last_bitrate_kbps_ unconditionally and the third leg
// below sees no re-send (bitrates.size() stays 2).
TEST(refused_bitrate_is_retried_next_policy_tick_then_latched) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);

  act.bitrate_ok = false;
  agent.tick(0, RadioHealth{});               // BOOT -> MAX_RANGE, forced apply
  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto r1 = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(r1.data(), r1.size(), 100);   // -> LINKED, forced apply
  REQUIRE(act.bitrates.size() == 2);
  const int wanted = act.bitrates.back();

  // Steady-state RCF, same operating point: with the refusal not latched the
  // target still counts as changed, and the throttle window never opened
  // (its timestamp is latched on success too), so the retry goes out at once.
  act.bitrate_ok = true;
  auto r2 = make_rcf_wire(cfg.link.vtx_id, 2, profile_byte, 8);
  agent.on_rc_frame(r2.data(), r2.size(), 200);
  REQUIRE(act.bitrates.size() == 3);
  CHECK(act.bitrates.back() == wanted);

  // Now it IS latched: an unchanged target past the throttle window is not
  // re-sent, i.e. the success path still dedups exactly as before.
  auto r3 = make_rcf_wire(cfg.link.vtx_id, 3, profile_byte, 8);
  agent.on_rc_frame(r3.data(), r3.size(), 1400);
  CHECK(act.bitrates.size() == 3);
}

// 16. Same rule for the ROI QP: roi_low_ flips only once the encoder has
// taken the value, so a refused transition is re-attempted.
// REVERT CHECK: flip roi_low_ before the call and the second leg sees no
// retry (roi_qps.size() stays 1).
TEST(refused_roi_qp_is_retried_next_policy_tick) {
  Config cfg = make_cfg();
  cfg.encoder.airtime_budget = 0.60;
  cfg.encoder.bitrate_max_kbps = 10000;
  MockActuator act;
  RcAgent agent(cfg, act);

  // MAX_RANGE (mcs0/ov1.00) = 1300 kbps, under roi_threshold_kbps (3000), so
  // the BOOT apply is a normal->low ROI transition.
  act.roi_ok = false;
  agent.tick(0, RadioHealth{});
  REQUIRE(act.roi_qps.size() == 1);
  CHECK(act.roi_qps.back() == cfg.encoder.roi_qp_low);

  // Still "normal" as far as the agent knows, so the next policy run at the
  // same low bitrate re-attempts the same transition.
  act.roi_ok = true;
  uint8_t mcs0 = encode_profile(PhyMode::HT, 0, 20);
  auto r1 = make_rcf_wire(cfg.link.vtx_id, 1, mcs0, 16);  // ov 1.00, same op
  agent.on_rc_frame(r1.data(), r1.size(), 100);
  REQUIRE(act.roi_qps.size() == 2);
  CHECK(act.roi_qps.back() == cfg.encoder.roi_qp_low);

  // Latched now: no further transition at the same operating point.
  auto r2 = make_rcf_wire(cfg.link.vtx_id, 2, mcs0, 16);
  agent.on_rc_frame(r2.data(), r2.size(), 1400);
  CHECK(act.roi_qps.size() == 2);
}

// 17. The pacer's "have I ever fired" state is a companion flag, not a
// 0-millisecond sentinel: callers supply their own clock and maburd's starts
// wherever steady_clock does, so t=0 is a legal first IDR and must arm the
// 100 ms floor like any other.
// REVERT CHECK: replace have_last_idr_ with `last_idr_ms_ != 0` and the
// t=50 chain break is no longer suppressed.
TEST(idr_at_time_zero_arms_the_floor) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto rcf = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(rcf.data(), rcf.size(), 0);  // -> LINKED, IDR at t=0
  REQUIRE(act.idr_calls == 1);

  agent.note_chain_break();
  agent.tick(50, RadioHealth{});   // inside the 100 ms floor from t=0
  CHECK(act.idr_calls == 1);

  agent.note_chain_break();
  agent.tick(150, RadioHealth{});  // clear of it
  CHECK(act.idr_calls == 2);
}

// 18. Periodic re-assert, half one: a verb refused on a FAILSAFE ENTRY is
// retried without any inbound RC frame. Before the re-assert, tick() never
// called run_bitrate_policy(), so "retried on the next policy tick" meant
// "on the next RCF/DISC/max-range entry" — and FAILSAFE is by definition the
// state with no RCFs, so a refusal there went unrepaired for up to
// rendezvous_ms (30 s) with the encoder still flooding at the previous
// rung's rate.
// REVERT CHECK: delete the re-assert block at the bottom of RcAgent::tick()
// and the t=1200/t=1300 ticks issue no set_bitrate_kbps at all — bitrates
// stays at 3 entries to the end of the test.
TEST(refused_apply_on_failsafe_entry_is_retried_by_the_periodic_reassert) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);

  agent.tick(0, RadioHealth{});                    // BOOT -> MAX_RANGE, forced
  uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  auto r1 = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 4);
  agent.on_rc_frame(r1.data(), r1.size(), 100);    // -> LINKED, forced apply
  REQUIRE(act.bitrates.size() == 2);

  // The encoder starts refusing, then the link goes quiet past failsafe_ms.
  act.bitrate_ok = false;
  agent.tick(1100, RadioHealth{});                 // LINKED -> FAILSAFE, forced
  REQUIRE(agent.state() == RcAgent::State::FAILSAFE);
  REQUIRE(act.bitrates.size() == 3);               // the entry's own attempt
  const int floor_kbps = act.bitrates.back();

  // No RCF, no DISC — only ticks. The failed apply short-circuits the
  // re-assert interval, so the very next tick retries.
  agent.tick(1200, RadioHealth{});
  REQUIRE(act.bitrates.size() == 4);
  CHECK(act.bitrates.back() == floor_kbps);

  // Encoder recovers: the next tick's retry lands and latches.
  act.bitrate_ok = true;
  agent.tick(1300, RadioHealth{});
  REQUIRE(act.bitrates.size() == 5);
  CHECK(act.bitrates.back() == floor_kbps);

  // And having landed, the retry stops: back to the 5 s cadence.
  agent.tick(1400, RadioHealth{});
  agent.tick(1500, RadioHealth{});
  CHECK(act.bitrates.size() == 5);
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
}

// 19. Periodic re-assert, half two: with a HEALTHY actuator the re-assert is
// a 5 s cadence, not a per-tick one. Ticks inside the interval must issue no
// set_bitrate_kbps at all, and crossing the interval must issue exactly one.
// The force=true is load-bearing: the re-assert restates an UNCHANGED target,
// which is precisely what run_bitrate_policy's changed/decrease gate exists
// to suppress.
// REVERT CHECK, two ways: (a) pass force=false in the re-assert call and the
// count never leaves 2 — the gate swallows every re-apply; (b) drop the
// `now_ms - last_bitrate_eval_ms_ >= kReassertMs` term (retry on every tick)
// and the first leg's 48 ticks each add a call.
TEST(reassert_is_a_five_second_cadence_not_a_per_tick_spam) {
  Config cfg = make_cfg();
  // Long enough that LINKED never times out during the window under test —
  // a failsafe entry would force a policy run of its own and confuse the
  // call count with something that is not the re-assert.
  cfg.link.failsafe_ms = 600000;
  MockActuator act;
  RcAgent agent(cfg, act);

  agent.tick(0, RadioHealth{});                    // BOOT apply  -> call 1
  uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  auto r1 = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 4);
  agent.on_rc_frame(r1.data(), r1.size(), 100);    // LINKED apply -> call 2
  REQUIRE(act.bitrates.size() == 2);
  const int target = act.bitrates.back();

  // t=200..5000 at tick_ms=100: 4900 ms since the last accepted apply (t=100),
  // still inside the interval. Not one call.
  for (uint64_t t = 200; t <= 5000; t += 100) agent.tick(t, RadioHealth{});
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(act.bitrates.size() == 2);

  // t=5100 is 5000 ms since t=100 — the interval elapses, exactly one
  // re-apply lands, and it restates the same target.
  agent.tick(5100, RadioHealth{});
  REQUIRE(act.bitrates.size() == 3);
  CHECK(act.bitrates.back() == target);

  // The clock restarts from the accepted apply, so the next window is quiet
  // again and then fires exactly once more.
  for (uint64_t t = 5200; t <= 10000; t += 100) agent.tick(t, RadioHealth{});
  CHECK(act.bitrates.size() == 3);
  agent.tick(10100, RadioHealth{});
  CHECK(act.bitrates.size() == 4);
  CHECK(act.bitrates.back() == target);
}
