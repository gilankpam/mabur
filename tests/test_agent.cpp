#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "mtest.h"
#include "config.h"
#include "mabur/link_key.h"
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
  std::vector<int> fps;
  bool fps_ok = true;
  // Verb order log: "fps" / "bitrate" in call order (the low-power
  // transition must send fps BEFORE the bitrate whose IDR seeds the stream).
  std::vector<std::string> verbs;
  int idr_calls = 0;
  bool bitrate_ok = true;
  bool roi_ok = true;
  std::vector<uint8_t> retunes;
  // Parallel to `retunes` (same index): the spec §7 reason literal the agent
  // passed with each move. Kept separate so channel assertions stay
  // `retunes[i] == ch`.
  std::vector<std::string> retune_reasons;

  void apply_op(const AppliedOp& op) override { applied.push_back(op); }
  void send_control(const std::vector<uint8_t>& body) override { controls.push_back(body); }
  bool set_bitrate_kbps(int kbps) override {
    bitrates.push_back(kbps);
    verbs.push_back("bitrate");
    return bitrate_ok;
  }
  bool set_fps(int f) override {
    fps.push_back(f);
    verbs.push_back("fps");
    return fps_ok;
  }
  bool set_roi_qp(int qp) override {
    roi_qps.push_back(qp);
    return roi_ok;
  }
  void request_idr() override { ++idr_calls; }
  void retune(uint8_t ch, const char* reason) override {
    retunes.push_back(ch);
    retune_reasons.push_back(reason ? reason : "");
  }
  std::vector<bool> records;
  bool record_ok = true;
  bool set_record(bool on) override {
    records.push_back(on);
    return record_ok;
  }
};

Config make_cfg() {
  Config cfg;
  cfg.link.failsafe_ms = 1000;
  cfg.link.rendezvous_ms = 30000;
  cfg.link.tick_ms = 100;
  cfg.encoder.airtime_budget = 0.65;
  cfg.encoder.bitrate_min_kbps = 1000;
  cfg.encoder.bitrate_max_kbps = 20000;
  cfg.encoder.roi_threshold_kbps = 3000;
  cfg.encoder.roi_qp_low = 8;
  cfg.encoder.roi_qp_normal = 0;
  cfg.radio.channel = 136;
  cfg.radio.follow_gs = true;
  cfg.link.move_confirm_ms = 2000;
  cfg.venc.core.fps = 60;
  cfg.msp.enable = true;
  cfg.low_power.enable = true;
  cfg.low_power.bitrate_kbps = 1000;
  cfg.low_power.fps = 15;
  cfg.low_power.stale_ms = 2000;
  return cfg;
}

// Link pairing (spec 2026-10-01 §6/§7): every RCF the drone accepts must
// carry a SipHash tag for a session pair (vrx_nonce, vtx_nonce) plus the
// extended seq32. kVrx is the test GS's vrx_nonce.
static const uint32_t kVrx = 0xCAFEF00D;

// Builds a CRC-valid, tagged RCF wire frame for seq/profile/fec_overhead,
// with DISTINCT base/enh overheads (ov_16ths keeps the callers' existing
// sixteenths-based literals: 8 == 0.5, 16 == 1.0, ...). This is the widened
// form (controller ruling, Task 6): the 4-arg overload below still packs one
// value into both wire fields for the ~40 call sites that don't care about
// the split, but a test that means to exercise the base/enh PAIR (the
// share-weighting tests, and the AppliedOp-pair test) must call this form
// directly, or the two wire fields collapse to the same value and the
// weighted target can't tell base from enh apart (see the weakened-test
// comments this replaced, Task 2 finding). vtx_nonce is the session's (from
// the DISC_ACK); seq32 0 means "same as the wire seq" (no wrap yet).
std::vector<uint8_t> make_rcf_wire(uint16_t seq, uint8_t profile, uint8_t ov_base_16ths,
                                   uint8_t ov_enh_16ths, uint8_t probe_profile,
                                   uint32_t vtx_nonce, uint32_t seq32 = 0,
                                   uint32_t vrx_nonce = kVrx,
                                   const mabur::LinkKey& key = mabur::kDefaultLinkKey) {
  Rcf r;
  r.seq = seq;
  r.profile = profile;
  r.fec_overhead_base = ov_base_16ths / 16.0;
  r.fec_overhead_enh = ov_enh_16ths / 16.0;
  r.probe_profile = probe_profile;
  return pack_rcf(r, key, TagCtx{vrx_nonce, vtx_nonce, seq32 ? seq32 : seq});
}

// Convenience overload: the common case with no probe stream commanded.
std::vector<uint8_t> make_rcf_wire(uint16_t seq, uint8_t profile, uint8_t ov_base_16ths,
                                   uint8_t ov_enh_16ths, uint32_t vtx_nonce) {
  return make_rcf_wire(seq, profile, ov_base_16ths, ov_enh_16ths, kNoProbeProfile, vtx_nonce);
}

// Convenience overload: the common case where a test wants the same
// overhead commanded on both wire fields (RC_VERSION 4 behavior).
std::vector<uint8_t> make_rcf_wire(uint16_t seq, uint8_t profile, uint8_t ov_16ths,
                                   uint32_t vtx_nonce) {
  return make_rcf_wire(seq, profile, ov_16ths, ov_16ths, vtx_nonce);
}

std::vector<uint8_t> make_disc_wire(uint32_t nonce, uint8_t op_channel, uint8_t op_width,
                                    uint8_t init_profile, uint16_t seq,
                                    const mabur::LinkKey& key = mabur::kDefaultLinkKey) {
  Disc d;
  d.vrx_nonce = nonce;
  d.op_channel = op_channel;
  d.op_width = op_width;
  d.init_profile = init_profile;
  d.seq = seq;
  return pack_disc(d, key);
}

// The vtx_nonce out of the newest DISC_ACK the agent sent.
static uint32_t last_ack_vtx(const MockActuator& act) {
  REQUIRE(!act.controls.empty());
  auto ack = parse_disc_ack(act.controls.back().data(), act.controls.back().size());
  REQUIRE(ack.has_value());
  REQUIRE(ack->flags == 0);
  return ack->vtx_nonce;
}

// BOOT tick (if still in BOOT), then DISC -> ack ONLY: the pair is pending,
// nothing is linked. Returns the vtx_nonce the test's own first RCF must be
// tagged with -- that RCF is then the one that enters LINKED, exactly like
// the bare first RCF of the pre-pairing tests (entering_linked force, link-up
// IDR, link_established), so those tests keep their meaning unchanged.
static uint32_t ack_agent(RcAgent& agent, MockActuator& act, uint8_t op_channel = 136,
                          uint32_t vrx = kVrx, uint64_t t = 0) {
  if (agent.state() == RcAgent::State::BOOT) agent.tick(0, RadioHealth{});
  auto disc = make_disc_wire(vrx, op_channel, 20, 0, 1);
  agent.on_rc_frame(disc.data(), disc.size(), t);
  return last_ack_vtx(act);
}

// DISC -> ack -> read the vtx_nonce out of the ack -> first RCF under it.
// Returns the vtx_nonce every later RCF of the test must be tagged with.
// The link RCF is seq 1 (mcs0), so the test's own RCFs start at seq 2.
static uint32_t link_agent(RcAgent& agent, MockActuator& act, const Config& cfg,
                           uint8_t op_channel = 136, uint64_t t0 = 100) {
  agent.tick(0, RadioHealth{});
  auto disc = make_disc_wire(kVrx, op_channel, 20, 0, 1);
  agent.on_rc_frame(disc.data(), disc.size(), t0);
  REQUIRE(!act.controls.empty());
  auto ack = parse_disc_ack(act.controls.back().data(), act.controls.back().size());
  REQUIRE(ack.has_value());
  REQUIRE(ack->flags == 0);
  auto rcf = make_rcf_wire(1, encode_profile(PhyMode::HT, 0, 20), 8, ack->vtx_nonce);
  agent.on_rc_frame(rcf.data(), rcf.size(), t0 + 10);
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  (void)cfg;
  return ack->vtx_nonce;
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

// 2. Link pairing (spec 2026-10-01 §6): a DISC only elicits a DISC_ACK. It
// carries a vtx_nonce for the GS's vrx_nonce, agrees to the proposed
// channel, and reports the drone's OWN width (40 proposed here, to catch an
// ack that echoes the GS's request). No op, no retune, no LINKED entry: the
// first RCF verified under the acked pair does those.
TEST(disc_replies_disc_ack_and_does_not_link) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  const size_t applied0 = act.applied.size();
  const size_t bitrates0 = act.bitrates.size();
  auto wire = make_disc_wire(kVrx, 149, 40, 0, 2);     // proposes a MOVE to 149
  agent.on_rc_frame(wire.data(), wire.size(), 100);
  CHECK(agent.state() == RcAgent::State::RENDEZVOUS);
  REQUIRE(act.controls.size() == 1);
  auto ack = parse_disc_ack(act.controls[0].data(), act.controls[0].size());
  REQUIRE(ack.has_value());
  CHECK(ack->vrx_nonce == kVrx);
  CHECK(ack->vtx_nonce != 0);
  CHECK(ack->flags == 0);
  CHECK(ack->agreed_channel == 149);
  CHECK(ack->agreed_width == cfg.radio.width);
  CHECK(act.retunes.empty());          // no move on a DISC
  CHECK(act.applied.size() == applied0);   // no op on a DISC (BOOT's MAX_RANGE only)
  CHECK(act.bitrates.size() == bitrates0);
  // Same vrx_nonce again (lost-ack retry): same vtx_nonce.
  agent.on_rc_frame(wire.data(), wire.size(), 200);
  auto ack2 = parse_disc_ack(act.controls[1].data(), act.controls[1].size());
  CHECK(ack2->vtx_nonce == ack->vtx_nonce);
  // Different vrx_nonce: a different vtx_nonce.
  auto other = make_disc_wire(kVrx + 1, 149, 40, 0, 3);
  agent.on_rc_frame(other.data(), other.size(), 300);
  auto ack3 = parse_disc_ack(act.controls[2].data(), act.controls[2].size());
  CHECK(ack3->vtx_nonce != ack->vtx_nonce);
}

TEST(first_verified_rcf_links_and_defers_the_move_to_tick) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  auto disc = make_disc_wire(kVrx, 149, 40, 0, 1);
  agent.on_rc_frame(disc.data(), disc.size(), 100);
  auto ack = parse_disc_ack(act.controls[0].data(), act.controls[0].size());
  auto rcf = make_rcf_wire(1, encode_profile(PhyMode::HT, 2, 20), 8, ack->vtx_nonce);
  agent.on_rc_frame(rcf.data(), rcf.size(), 110);
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.take_session_promoted());
  CHECK(!agent.take_session_promoted());
  CHECK(agent.current_session().vrx_nonce == kVrx);
  CHECK(agent.current_session().vtx_nonce == ack->vtx_nonce);
  CHECK(act.retunes.empty());                       // not yet: main sends a Telem first
  agent.tick(200, RadioHealth{});
  REQUIRE(act.retunes.size() == 1);
  CHECK(act.retunes[0] == 149);
  CHECK(act.retune_reasons[0] == "disc");
  CHECK(agent.channel() == 149);
  CHECK(agent.current().ladder[0].mcs == 2);        // the RCF's op, not an init profile
  REQUIRE(!act.bitrates.empty());
}

TEST(unverified_rcf_is_dropped_and_flagged) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_agent(agent, act, cfg);
  CHECK(!agent.take_auth_reject());
  const size_t applied = act.applied.size();
  auto wrong_nonce = make_rcf_wire(2, encode_profile(PhyMode::HT, 5, 20), 8, vtx + 1);
  agent.on_rc_frame(wrong_nonce.data(), wrong_nonce.size(), 200);
  mabur::LinkKey other = mabur::kDefaultLinkKey; other[0] ^= 1;
  auto wrong_key = make_rcf_wire(3, encode_profile(PhyMode::HT, 5, 20), 8, 8, kNoProbeProfile, vtx, 0, kVrx, other);
  agent.on_rc_frame(wrong_key.data(), wrong_key.size(), 210);
  auto replay = make_rcf_wire(1, encode_profile(PhyMode::HT, 5, 20), 8, vtx);   // seq 1 again
  agent.on_rc_frame(replay.data(), replay.size(), 220);
  CHECK(act.applied.size() == applied);
  CHECK(agent.current().ladder[0].mcs == 0);
  CHECK(agent.take_auth_reject());
  CHECK(!agent.take_auth_reject());
  // The honest next RCF still works: a rejected frame never moved the seq tracker.
  auto ok = make_rcf_wire(2, encode_profile(PhyMode::HT, 5, 20), 8, vtx);
  agent.on_rc_frame(ok.data(), ok.size(), 230);
  CHECK(agent.current().ladder[0].mcs == 5);
}

TEST(seq32_extension_rejects_a_frame_from_one_wrap_ago) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_agent(agent, act, cfg);
  // Walk the wire seq across the 16-bit wrap in big steps (delta <= 32767).
  uint32_t s32 = 1;
  for (int i = 0; i < 4; ++i) {
    s32 += 30000;
    auto w = make_rcf_wire(static_cast<uint16_t>(s32), encode_profile(PhyMode::HT, 1, 20), 8, 8, kNoProbeProfile, vtx, s32);
    agent.on_rc_frame(w.data(), w.size(), 1000 + i);
  }
  CHECK(agent.current_session().last_seq32 == s32);         // 120001
  // A frame tagged with the SAME wire seq + 1 but seq32 from the previous
  // wrap (65536 lower): delta reads fresh, tag does not match.
  const uint32_t old32 = s32 + 1 - 65536;
  auto stale = make_rcf_wire(static_cast<uint16_t>(old32), encode_profile(PhyMode::HT, 6, 20), 8, 8, kNoProbeProfile, vtx, old32);
  agent.on_rc_frame(stale.data(), stale.size(), 2000);
  CHECK(agent.current().ladder[0].mcs == 1);
  CHECK(agent.take_auth_reject());
  auto fresh = make_rcf_wire(static_cast<uint16_t>(s32 + 1), encode_profile(PhyMode::HT, 6, 20), 8, 8, kNoProbeProfile, vtx, s32 + 1);
  agent.on_rc_frame(fresh.data(), fresh.size(), 2010);
  CHECK(agent.current().ladder[0].mcs == 6);
}

TEST(new_vrx_nonce_while_linked_swaps_session_without_touching_op) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_agent(agent, act, cfg);
  auto up = make_rcf_wire(2, encode_profile(PhyMode::HT, 4, 20), 8, vtx);
  agent.on_rc_frame(up.data(), up.size(), 200);
  const size_t applied = act.applied.size();
  // Restarted GS: new vrx_nonce, same channel proposal.
  auto disc = make_disc_wire(kVrx + 7, 136, 20, 0, 1);
  agent.on_rc_frame(disc.data(), disc.size(), 300);
  auto ack = parse_disc_ack(act.controls.back().data(), act.controls.back().size());
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(act.applied.size() == applied);                      // DISC changed nothing
  CHECK(agent.current_session().vrx_nonce == kVrx);          // still the old session
  // Old session's RCFs still verify until the new one proves itself.
  auto old_ok = make_rcf_wire(3, encode_profile(PhyMode::HT, 4, 20), 8, vtx);
  agent.on_rc_frame(old_ok.data(), old_ok.size(), 310);
  CHECK(!agent.take_auth_reject());
  // New GS's first RCF (its seq restarts at 1) promotes the pending pair.
  auto first = make_rcf_wire(1, encode_profile(PhyMode::HT, 4, 20), 8, 8, kNoProbeProfile, ack->vtx_nonce, 1, kVrx + 7);
  agent.on_rc_frame(first.data(), first.size(), 320);
  CHECK(agent.current_session().vrx_nonce == kVrx + 7);
  CHECK(agent.take_session_promoted());
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(act.retunes.empty());                                // same channel: no move
  agent.tick(400, RadioHealth{});
  CHECK(act.retunes.empty());
  CHECK(agent.current().ladder[0].mcs == 4);                 // op untouched by the swap
  // The old session is dead now.
  auto old_dead = make_rcf_wire(4, encode_profile(PhyMode::HT, 4, 20), 8, vtx);
  agent.on_rc_frame(old_dead.data(), old_dead.size(), 410);
  CHECK(agent.take_auth_reject());
}

// A promotion while LINKED whose pair was acked on a DIFFERENT channel still
// moves -- deferred to tick() like any promotion (the old LINKED ack-only
// branch honoured a channel move; spec 2026-10-01 §6 keeps it, but only
// once the new pair has proved itself with a verified RCF).
TEST(promotion_while_linked_to_a_new_channel_defers_the_move) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  link_agent(agent, act, cfg);
  const uint64_t gen = agent.current().generation;
  auto disc = make_disc_wire(kVrx + 7, 149, 20, 0, 1);
  agent.on_rc_frame(disc.data(), disc.size(), 300);
  CHECK(act.retunes.empty());                                // the DISC alone never moves
  const uint32_t vtx2 = last_ack_vtx(act);
  auto first = make_rcf_wire(1, encode_profile(PhyMode::HT, 0, 20), 8, 8, kNoProbeProfile, vtx2, 1, kVrx + 7);
  agent.on_rc_frame(first.data(), first.size(), 320);
  CHECK(agent.take_session_promoted());
  CHECK(act.retunes.empty());
  agent.tick(400, RadioHealth{});
  REQUIRE(act.retunes.size() == 1);
  CHECK(act.retunes[0] == 149);
  CHECK(act.retune_reasons[0] == "disc");
  CHECK(agent.channel() == 149);
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current().generation == gen + 1);              // just the RCF's own apply
}

TEST(wrong_key_disc_gets_flagged_ack_and_nothing_else) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  mabur::LinkKey other = mabur::kDefaultLinkKey; other[5] ^= 0x80;
  auto disc = make_disc_wire(kVrx, 149, 20, 0, 1, other);
  agent.on_rc_frame(disc.data(), disc.size(), 100);
  REQUIRE(act.controls.size() == 1);
  auto ack = parse_disc_ack(act.controls[0].data(), act.controls[0].size());
  REQUIRE(ack.has_value());
  CHECK(ack->flags == kAckKeyMismatch);
  CHECK(ack->vtx_nonce == 0);
  CHECK(ack->vrx_nonce == kVrx);
  CHECK(agent.state() == RcAgent::State::RENDEZVOUS);
  CHECK(act.retunes.empty());
  CHECK(agent.take_auth_reject());
  // No pending session was created for the bad DISC.
  auto rcf = make_rcf_wire(1, encode_profile(PhyMode::HT, 1, 20), 8, 0);
  agent.on_rc_frame(rcf.data(), rcf.size(), 110);
  CHECK(agent.state() == RcAgent::State::RENDEZVOUS);
}

// Spec 2026-10-01 §7: failsafe entry clears both sessions. Otherwise every
// RCF the GS sent during an uplink fade stays replayable (valid tag, seq32
// still ahead of the tracker) for the life of the process. Recovery is the
// GS's next keep-alive DISC, which gets a fresh pair.
TEST(failsafe_clears_the_session_and_the_keepalive_disc_relinks) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_agent(agent, act, cfg);           // link RCF at t=110
  agent.tick(110 + cfg.link.failsafe_ms + 1, RadioHealth{});
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  CHECK(!agent.current_session().valid);
  // An RCF the GS sent during the fade (old pair, seq ahead): rejected.
  auto back = make_rcf_wire(2, encode_profile(PhyMode::HT, 3, 20), 8, vtx);
  agent.on_rc_frame(back.data(), back.size(), 2000);
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  CHECK(agent.take_auth_reject());
  CHECK(agent.current().ladder[0].mcs == 0);
  // Keep-alive DISC from the same GS (same vrx): a NEW vtx_nonce.
  auto disc = make_disc_wire(kVrx, 136, 20, 0, 9);
  agent.on_rc_frame(disc.data(), disc.size(), 2100);
  const uint32_t vtx2 = last_ack_vtx(act);
  CHECK(vtx2 != vtx);
  CHECK(agent.state() == RcAgent::State::FAILSAFE);           // a DISC never links
  auto first = make_rcf_wire(1, encode_profile(PhyMode::HT, 3, 20), 8, vtx2);
  agent.on_rc_frame(first.data(), first.size(), 2110);
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current_session().vtx_nonce == vtx2);
  CHECK(!agent.take_auth_reject());
}

TEST(disc_for_current_session_while_not_linked_issues_a_fresh_vtx_nonce) {
  // > 1 h of silence would desync seq32; the drone is in FAILSAFE/RENDEZVOUS
  // by then, so a keep-alive DISC heard outside LINKED starts a NEW pending
  // pair for the same GS (the GS adopts the new nonce and resets its seq).
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_agent(agent, act, cfg);           // link RCF at t=110
  agent.tick(110 + cfg.link.failsafe_ms + 1, RadioHealth{});
  auto disc = make_disc_wire(kVrx, 136, 20, 0, 9);
  agent.on_rc_frame(disc.data(), disc.size(), 3000);
  auto ack = parse_disc_ack(act.controls.back().data(), act.controls.back().size());
  CHECK(ack->vtx_nonce != vtx);
  auto first = make_rcf_wire(1, encode_profile(PhyMode::HT, 2, 20), 8, ack->vtx_nonce);
  agent.on_rc_frame(first.data(), first.size(), 3010);
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current_session().vtx_nonce == ack->vtx_nonce);
}

TEST(verify_cal_frame_reads_the_current_session_from_another_thread) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  CalCmd c; c.nonce = 5; c.windows.push_back(CalWindow{3, -4, 4, 1});
  auto unlinked = pack_cal_cmd(c, mabur::kDefaultLinkKey, TagCtx{kVrx, 1, 0});
  CHECK(!agent.verify_cal_frame(unlinked.data(), unlinked.size()));
  const uint32_t vtx = link_agent(agent, act, cfg);
  auto good = pack_cal_cmd(c, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx, 0});
  auto bad = pack_cal_cmd(c, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx + 1, 0});
  bool ok = false, rej = true;
  std::thread th([&] { ok = agent.verify_cal_frame(good.data(), good.size());
                       rej = agent.verify_cal_frame(bad.data(), bad.size()); });
  th.join();
  CHECK(ok);
  CHECK(!rej);
  CHECK(agent.take_auth_reject());
}

// On hardware rx_callback hands the agent Packet.Data minus the 802.11
// header, which still carries devourer's trailing 4-byte FCS. The tag lives
// at the frame's structural offset, so those bytes must change nothing.
static std::vector<uint8_t> plus_fcs(std::vector<uint8_t> w) {
  for (uint8_t b : {0x11, 0x22, 0x33, 0x44}) w.push_back(b);
  return w;
}

TEST(disc_and_rcf_with_trailing_fcs_bytes_link_like_exact_length_frames) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  auto disc = plus_fcs(make_disc_wire(kVrx, 136, 20, 0, 1));
  agent.on_rc_frame(disc.data(), disc.size(), 100);
  const uint32_t vtx = last_ack_vtx(act);           // unflagged: the tag verified
  CHECK(!agent.take_auth_reject());
  auto rcf = plus_fcs(make_rcf_wire(1, encode_profile(PhyMode::HT, 2, 20), 8, vtx));
  agent.on_rc_frame(rcf.data(), rcf.size(), 110);
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current_session().vtx_nonce == vtx);
  CHECK(agent.current().ladder[0].mcs == 2);
  auto next = plus_fcs(make_rcf_wire(2, encode_profile(PhyMode::HT, 4, 20), 8, vtx));
  agent.on_rc_frame(next.data(), next.size(), 120);
  CHECK(agent.current().ladder[0].mcs == 4);
  CHECK(!agent.take_auth_reject());
}

TEST(verify_cal_frame_accepts_a_cal_cmd_with_trailing_fcs_bytes) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_agent(agent, act, cfg);
  CalCmd c; c.nonce = 5; c.windows.push_back(CalWindow{3, -4, 4, 1});
  auto cmd = plus_fcs(pack_cal_cmd(c, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx, 0}));
  CHECK(agent.verify_cal_frame(cmd.data(), cmd.size()));
  CalResult r; r.nonce = 5;
  auto res = plus_fcs(pack_cal_result(r, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx, 0}));
  CHECK(agent.verify_cal_frame(res.data(), res.size()));
  CHECK(!agent.take_auth_reject());
}

// Cal frames carry seq32 = 0, so the tag alone cannot stop a recorded
// CAL_CMD with an OLDER cal nonce from starting a fresh TX-power sweep (and
// its recorded CAL_RESULT from then applying walls). The drone refuses any
// cal nonce it has already seen in the current link session; a repeat of
// the CURRENT nonce (a retransmission, or the next phase) stays accepted.
TEST(cal_cmd_with_an_already_seen_nonce_is_refused_until_the_session_changes) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  uint32_t vtx = link_agent(agent, act, cfg);
  auto cmd = [&](uint32_t nonce, uint32_t v, uint8_t phase = 0) {
    CalCmd c; c.nonce = nonce; c.phase = phase; c.windows.push_back(CalWindow{3, -4, 4, 1});
    return pack_cal_cmd(c, mabur::kDefaultLinkKey, TagCtx{kVrx, v, 0});
  };
  auto n5 = cmd(5, vtx), n6 = cmd(6, vtx);
  CHECK(agent.verify_cal_frame(n5.data(), n5.size()));       // sweep 5
  CHECK(agent.verify_cal_frame(n5.data(), n5.size()));       // its retransmission
  auto n5p1 = cmd(5, vtx, 1);
  CHECK(agent.verify_cal_frame(n5p1.data(), n5p1.size()));   // its next phase
  CHECK(!agent.take_auth_reject());
  CHECK(agent.verify_cal_frame(n6.data(), n6.size()));       // sweep 6
  CHECK(!agent.verify_cal_frame(n5.data(), n5.size()));      // replayed 5: refused
  CHECK(agent.take_auth_reject());
  CHECK(agent.verify_cal_frame(n6.data(), n6.size()));       // 6 is still current
  // A CAL_RESULT is not a sweep start: CalSweep dedupes it against nonce_.
  CalResult r; r.nonce = 6;
  auto res = pack_cal_result(r, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx, 0});
  CHECK(agent.verify_cal_frame(res.data(), res.size()));
  CHECK(!agent.take_auth_reject());

  // A session change (failsafe clears it; the keep-alive DISC re-pairs)
  // forgets the seen nonces: 5 is accepted again under the new pair.
  agent.tick(110 + cfg.link.failsafe_ms + 1, RadioHealth{});
  REQUIRE(agent.state() == RcAgent::State::FAILSAFE);
  vtx = ack_agent(agent, act, 136, kVrx, 2000);
  auto first = make_rcf_wire(1, encode_profile(PhyMode::HT, 0, 20), 8, vtx);
  agent.on_rc_frame(first.data(), first.size(), 2010);
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  auto again = cmd(5, vtx);
  CHECK(agent.verify_cal_frame(again.data(), again.size()));
  CHECK(!agent.take_auth_reject());
}

// The GS is radio-silent for every sweep phase, so the drone always hits
// FAILSAFE (and clears its session, spec 2026-10-01 §7) partway through a
// calibration run. The run's later cal frames -- the next phase's CAL_CMD,
// and above all the CAL_RESULT sent straight into the silent verify window
// -- are still tagged under the pair the run was going under, and must
// verify for as long as the caller says a sweep is running. Bench repro
// 2026-10-03: every result auth-rejected, nothing ever applied.
TEST(cal_frames_verify_under_the_running_sweeps_session_after_failsafe_clears_it) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_agent(agent, act, cfg);           // link RCF at t=110
  CalCmd c; c.nonce = 7; c.windows.push_back(CalWindow{3, -4, 4, 1});
  auto open = pack_cal_cmd(c, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx, 0});
  CHECK(agent.verify_cal_frame(open.data(), open.size(), false));   // opens the sweep
  agent.tick(110 + cfg.link.failsafe_ms + 1, RadioHealth{});
  REQUIRE(agent.state() == RcAgent::State::FAILSAFE);
  REQUIRE(!agent.current_session().valid);

  c.phase = 1;
  auto fine = pack_cal_cmd(c, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx, 0});
  CHECK(agent.verify_cal_frame(fine.data(), fine.size(), true));
  CalResult r; r.nonce = 7;
  auto res = pack_cal_result(r, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx, 0});
  CHECK(agent.verify_cal_frame(res.data(), res.size(), true));
  CHECK(!agent.take_auth_reject());

  // Sweep over: the cleared pair is dead again, for results and commands.
  CHECK(!agent.verify_cal_frame(res.data(), res.size(), false));
  CHECK(!agent.verify_cal_frame(res.data(), res.size(), true));    // latch is gone
  CHECK(agent.take_auth_reject());
}

// The latch follows the newest pair a cal frame verified under: the GS can
// re-pair between phases (its DISC/RCF go out once a phase's slack ends),
// and the run then continues under -- and must survive the next failsafe
// with -- the new pair, not the one it opened under.
TEST(cal_latch_follows_a_mid_run_re_pair) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_agent(agent, act, cfg);
  CalCmd c; c.nonce = 9; c.windows.push_back(CalWindow{3, -4, 4, 1});
  auto open = pack_cal_cmd(c, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx, 0});
  CHECK(agent.verify_cal_frame(open.data(), open.size(), false));
  agent.tick(110 + cfg.link.failsafe_ms + 1, RadioHealth{});
  REQUIRE(agent.state() == RcAgent::State::FAILSAFE);
  const uint32_t vtx2 = ack_agent(agent, act, 136, kVrx, 4000);
  auto first = make_rcf_wire(1, encode_profile(PhyMode::HT, 0, 20), 8, vtx2);
  agent.on_rc_frame(first.data(), first.size(), 4010);
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  c.phase = 1;
  auto fine = pack_cal_cmd(c, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx2, 0});
  CHECK(agent.verify_cal_frame(fine.data(), fine.size(), true));
  agent.tick(4010 + cfg.link.failsafe_ms + 1, RadioHealth{});
  REQUIRE(!agent.current_session().valid);
  CalResult r; r.nonce = 9;
  auto res2 = pack_cal_result(r, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx2, 0});
  auto res1 = pack_cal_result(r, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx, 0});
  CHECK(agent.verify_cal_frame(res2.data(), res2.size(), true));
  CHECK(!agent.take_auth_reject());
  CHECK(!agent.verify_cal_frame(res1.data(), res1.size(), true));   // superseded pair
  CHECK(agent.take_auth_reject());
}

TEST(cal_nonce_ring_is_bounded_and_forgets_the_oldest) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_agent(agent, act, cfg);
  auto cmd = [&](uint32_t nonce) {
    CalCmd c; c.nonce = nonce; c.windows.push_back(CalWindow{3, -4, 4, 1});
    return pack_cal_cmd(c, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx, 0});
  };
  for (uint32_t n = 100; n < 100 + RcAgent::kCalNonceRing + 1; ++n) {
    auto w = cmd(n);
    CHECK(agent.verify_cal_frame(w.data(), w.size()));
  }
  auto oldest = cmd(100);            // pushed out of the ring
  CHECK(agent.verify_cal_frame(oldest.data(), oldest.size()));
  auto recent = cmd(100 + RcAgent::kCalNonceRing);   // still remembered
  CHECK(!agent.verify_cal_frame(recent.data(), recent.size()));
}

TEST(install_session_for_replay_accepts_a_pretagged_rcf) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  agent.install_session_for_replay(1, 1);
  auto rcf = make_rcf_wire(1, encode_profile(PhyMode::HT, 4, 20), 8, 8, kNoProbeProfile, 1, 1, 1);
  agent.on_rc_frame(rcf.data(), rcf.size(), 100);
  CHECK(agent.state() == RcAgent::State::LINKED);
}

// 2c. DiscAck.chip_caps always advertises CAP_FRAME_WIRE: the frame wire is
// the only video format maburd speaks since the pre-frame-shm path was
// deleted. The bit stays on the wire so a GS can refuse a peer without it.
TEST(disc_ack_advertises_frame_wire_cap) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS

  auto wire = make_disc_wire(0xCAFEF00D, /*op_channel=*/136,
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  // Link via RCF at a non-default rung (mcs2, ov 0.5).
  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto rcf = make_rcf_wire(1, profile_byte, 8, vtx);
  agent.on_rc_frame(rcf.data(), rcf.size(), 100);
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  const uint64_t gen = agent.current().generation;
  const size_t n_controls = act.controls.size();
  const size_t n_bitrates = act.bitrates.size();

  // Same channel proposed as the drone's own -> no move (channel moves are
  // covered separately by disc_foreign_channel_acks_then_retunes; this test
  // stays about the keep-alive no-op).
  auto disc = make_disc_wire(0xCAFEF00D, cfg.radio.channel, 20,
                             /*init_profile=*/0, /*seq=*/7);
  agent.on_rc_frame(disc.data(), disc.size(), 600);

  // Exactly one new control frame: a well-formed DiscAck echoing our real
  // caps and channel, and the DISC's nonce/seq.
  REQUIRE(act.controls.size() == n_controls + 1);
  auto parsed = parse_disc_ack(act.controls.back().data(),
                               act.controls.back().size());
  REQUIRE(parsed.has_value());
  CHECK(parsed->vrx_nonce == 0xCAFEF00D);
  CHECK(parsed->vtx_nonce == vtx);  // keep-alive for the current pair: same answer
  CHECK(parsed->flags == 0);
  CHECK(parsed->seq == 7);
  CHECK(parsed->chip_caps & mabur::rc::CAP_FRAME_WIRE);
  CHECK(parsed->agreed_channel == cfg.radio.channel);
  CHECK(parsed->agreed_width == cfg.radio.width);
  CHECK(act.retunes.empty());

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
// not an RCF. Failsafe entry is exactly that case: it clears the session
// (spec 2026-10-01 §7) AND rebases last_fb_ms_ to now, so a fresh-looking age paired
// with a stale echoed seq would let the GS fabricate an RTT sample from
// the wrong send time. (A keepalive DISC while LINKED changes nothing —
// feedback state included — so the echo correctly stays valid there.)
TEST(last_feedback_seq_tracks_rcf_and_invalidates_on_failsafe) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  CHECK(!agent.last_feedback_seq().has_value());  // no RCF ever
  const uint32_t vtx = ack_agent(agent, act);
  CHECK(!agent.last_feedback_seq().has_value());  // a DISC is not an RCF

  auto rcf = make_rcf_wire(4711, encode_profile(PhyMode::HT, 5, 20), 4, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(1, profile_byte, 8, vtx);
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

// 3b. Per-MCS efficiency (docs/bandwidth-sweep-findings-2026-09-17.md): the
// policy prices each layer at nominal × air_clock.efficiency_<bw>[mcs], so
// the budget is a fraction of the capacity the link DELIVERS. Same op as 3
// (20 MHz) with efficiency_20[2] = 0.5: both rate terms halve, kbps
// 8450 -> 4225 -> 4200. Entries for other MCS must not leak in.
TEST(bitrate_policy_prices_at_delivered_rate) {
  Config cfg = make_cfg();
  cfg.air_clock.efficiency_20 = {0.1, 0.1, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1};
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  auto wire = make_rcf_wire(1, encode_profile(PhyMode::HT, 2, 20), 8, vtx);
  agent.on_rc_frame(wire.data(), wire.size(), 100);
  REQUIRE(!act.bitrates.empty());
  CHECK(act.bitrates.back() == 4200);
}

// 3a. AppliedOp carries the RCF's base/enh overheads as a genuine PAIR, not
// folded into one value (Task 6, RC_VERSION 5): base=1.0, enh=0.5 must
// survive on_rc_frame -> apply_ladder_op -> AppliedOp distinctly.
TEST(applied_op_carries_distinct_base_enh_overhead_pair) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  // ov_base=1.0 (16/16), ov_enh=0.5 (8/16) -- distinct on the wire.
  auto wire = make_rcf_wire(1, profile_byte, 16, 8, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  auto wire = make_rcf_wire(1, profile_byte, 8, vtx);  // ov16=8 -> 0.5
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  // ov_base=0.5 (8/16), ov_enh=1.0 (16/16) -- distinct, so ovb/ove stay
  // observable in the weighted target instead of factoring out.
  auto wire = make_rcf_wire(1, profile_byte, 8, 16, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 3, 20);
  for (uint16_t seq = 1; seq <= 300; ++seq) {
    auto wire = make_rcf_wire(seq, profile_byte, 16, 4, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(10, profile_byte, 8, vtx);
  agent.on_rc_frame(wire.data(), wire.size(), 100);
  uint64_t gen_after_first = agent.current().generation;

  // Same seq again.
  agent.on_rc_frame(wire.data(), wire.size(), 200);
  CHECK(agent.current().generation == gen_after_first);

  // seq - 1 (stale/old).
  auto stale_wire = make_rcf_wire(9, profile_byte, 8, vtx);
  agent.on_rc_frame(stale_wire.data(), stale_wire.size(), 300);
  CHECK(agent.current().generation == gen_after_first);
}

// 5. Corrupt RCF (flip a byte) -> ignored.
TEST(corrupt_rcf_is_ignored) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(1, profile_byte, 8, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(1, profile_byte, 8, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(1, profile_byte, 8, vtx);
  agent.on_rc_frame(wire.data(), wire.size(), 0);
  agent.tick(1000, RadioHealth{});  // -> FAILSAFE
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  int idr_before = act.idr_calls;

  // Failsafe cleared the session (spec 2026-10-01 §7): the keep-alive DISC
  // re-pairs, and the first RCF under the new pair re-links.
  const uint32_t vtx2 = ack_agent(agent, act, 136, kVrx, 1400);
  auto wire2 = make_rcf_wire(2, profile_byte, 8, vtx2);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(1, profile_byte, 8, vtx);
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
  auto wire2 = make_rcf_wire(2, profile_byte, 8, vtx);
  agent.on_rc_frame(wire2.data(), wire2.size(), 900);
  auto wire3 = make_rcf_wire(3, profile_byte, 8, vtx);
  agent.on_rc_frame(wire3.data(), wire3.size(), 1800);

  // 2000ms clean (no new drops) -> level decrements 2->1, still sheding enh.
  agent.tick(2300, RadioHealth{0, 10});
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current().shed[1] == true);

  auto wire4 = make_rcf_wire(4, profile_byte, 8, vtx);
  agent.on_rc_frame(wire4.data(), wire4.size(), 2700);
  auto wire5 = make_rcf_wire(5, profile_byte, 8, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  auto wire = make_rcf_wire(1, profile_byte, 8, vtx);
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
  auto wire2 = make_rcf_wire(2, profile_byte, 8, vtx);
  agent.on_rc_frame(wire2.data(), wire2.size(), 900);
  auto wire3 = make_rcf_wire(3, profile_byte, 8, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(1, profile_byte, 8, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(1, profile_byte, 8, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire1 = make_rcf_wire(1, profile_byte, 8, vtx);
  agent.on_rc_frame(wire1.data(), wire1.size(), 0);
  size_t count_after_first = act.bitrates.size();
  REQUIRE(count_after_first >= 1);

  auto wire2 = make_rcf_wire(2, profile_byte, 8, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  // Enter LINKED at the top rung: mcs7, ov 2/16 -> clamp to max = 20000.
  auto top = make_rcf_wire(1, encode_profile(PhyMode::HT, 7, 20), 2, vtx);
  agent.on_rc_frame(top.data(), top.size(), 0);
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  REQUIRE(!act.bitrates.empty());
  CHECK(act.bitrates.back() == 20000);

  // Steady top-rung RCFs for 3s: no further calls (dedup), stamp expires.
  uint16_t seq = 2;
  for (uint64_t t = 100; t <= 3000; t += 100) {
    auto w = make_rcf_wire(seq++,
                            encode_profile(PhyMode::HT, 7, 20), 2, vtx);
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
    auto w = make_rcf_wire(seq++,
                            encode_profile(PhyMode::HT, d.mcs, 20), d.ov16, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  // Enter LINKED at mcs2/ov0.5 -> 8500 (forced entry call, stamp t=0; see
  // test 3/10b's blended-formula derivation for this mcs/ov combo).
  auto w0 = make_rcf_wire(1, encode_profile(PhyMode::HT, 2, 20), 8, vtx);
  agent.on_rc_frame(w0.data(), w0.size(), 0);
  REQUIRE(act.bitrates.back() == 8500);

  // Promote to mcs4/ov0.5 (16900, see 10b's derivation) once the stamp
  // expired: call fires. (ov0.5, not the old ov0.25 -- that combo now
  // clamps to 20000 under same-rate, same as the mcs7 step below, and
  // couldn't show a genuine further increase.)
  auto w1 = make_rcf_wire(2, encode_profile(PhyMode::HT, 4, 20), 8, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 1100);
  REQUIRE(act.bitrates.back() == 16900);
  size_t n = act.bitrates.size();

  // Further promote to mcs7 (20000) 100ms later: inside the throttle
  // window -> deferred.
  auto w2 = make_rcf_wire(3, encode_profile(PhyMode::HT, 7, 20), 2, vtx);
  agent.on_rc_frame(w2.data(), w2.size(), 1200);
  CHECK(act.bitrates.size() == n);

  // Same op re-sent after the window: goes out.
  auto w3 = make_rcf_wire(4, encode_profile(PhyMode::HT, 7, 20), 2, vtx);
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
  cfg.encoder.airtime_budget = 0.60;    // prod value (/etc/mabur.toml)
  cfg.encoder.bitrate_max_kbps = 10000; // prod value; this clamp is the trap
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links

  // Enter LINKED at rung 4: mcs4/ov1.5 -> 9360.0 -> 9400 (see the
  // derivation above).
  auto w0 = make_rcf_wire(1, encode_profile(PhyMode::HT, 4, 20), 24, vtx);
  agent.on_rc_frame(w0.data(), w0.size(), 0);
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  REQUIRE(!act.bitrates.empty());
  REQUIRE(act.bitrates.back() == 9400);

  // Promote to rung 5 well after the 1s throttle window: mcs5/ov1.0 ->
  // 15600.0, clamped to 10000. Only 600 above the last applied value.
  auto w1 = make_rcf_wire(2, encode_profile(PhyMode::HT, 5, 20), 16, vtx);
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
  cfg.encoder.airtime_budget = 0.60;    // prod value (/etc/mabur.toml)
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
  const uint32_t vtx = ack_agent(agent, act);
  CHECK(!agent.take_link_established());  // a DISC links nothing (spec 2026-10-01 §6)

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto rcf1 = make_rcf_wire(1, profile_byte, 8, vtx);
  agent.on_rc_frame(rcf1.data(), rcf1.size(), 10);  // RENDEZVOUS -> LINKED
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.take_link_established());
  CHECK(!agent.take_link_established());  // consumed

  agent.tick(1010, RadioHealth{});        // feedback silence -> FAILSAFE
  REQUIRE(agent.state() == RcAgent::State::FAILSAFE);
  const uint32_t vtx2 = ack_agent(agent, act, 136, kVrx, 1015);  // keep-alive re-pairs
  CHECK(!agent.take_link_established());  // the DISC itself latches nothing
  auto rcf2 = make_rcf_wire(2, profile_byte, 8, vtx2);
  agent.on_rc_frame(rcf2.data(), rcf2.size(), 1020);  // FAILSAFE -> LINKED
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  CHECK(!agent.take_link_established());  // flap, not a (re)start
}

// 11. RCF with probe_profile != kNoProbeProfile fills the dedicated probe
// slot (AppliedOp.probe/probe_profile), NOT the enh layer — ladder[1] stays
// on the base profile's scored mcs regardless of a probe (spec 2026-09-04
// probe-stream). agent.probe_on() reflects the last accepted RCF's
// probe_profile and clears the moment a follow-up RCF arrives at
// kNoProbeProfile.
TEST(probe_rcf_fills_the_probe_slot_not_the_enh_layer) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links
  const uint8_t probe_byte = encode_profile(PhyMode::HT, 6, 20);
  auto wire = make_rcf_wire(1, encode_profile(PhyMode::HT, 5, 20), 8, 8, probe_byte, vtx);
  agent.on_rc_frame(wire.data(), wire.size(), 100);
  REQUIRE(!act.applied.empty());
  const AppliedOp& op = act.applied.back();
  CHECK(op.ladder[0].mcs == 5);
  CHECK(op.ladder[1].mcs == 5);            // enh no longer moves for a probe
  CHECK(op.probe_profile == probe_byte);
  CHECK(op.probe.mcs == 6);
  CHECK(op.probe.bw == 20);
  CHECK(op.probe.ldpc && op.probe.stbc);
  CHECK(agent.probe_on());
  // A follow-up RCF without a probe clears the slot.
  auto wire2 = make_rcf_wire(2, encode_profile(PhyMode::HT, 5, 20), 8, 8, kNoProbeProfile, vtx);
  agent.on_rc_frame(wire2.data(), wire2.size(), 200);
  CHECK(act.applied.back().probe_profile == kNoProbeProfile);
  CHECK(!agent.probe_on());
}

// 11a. radio.ldpc = false (2026-10-02, RTL8821AU GS): every slot the agent
// applies flies BCC -- the BOOT MAX_RANGE op, the RCF-commanded rung and its
// probe slot. STBC stays on.
TEST(ldpc_off_config_clears_ldpc_on_every_applied_slot) {
  Config cfg = make_cfg();
  cfg.radio.ldpc = false;
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  REQUIRE(!act.applied.empty());
  for (const auto& l : act.applied.back().ladder) CHECK(!l.ldpc && l.stbc);
  const uint32_t vtx = ack_agent(agent, act);
  auto wire = make_rcf_wire(1, encode_profile(PhyMode::HT, 4, 20), 8, 8,
                            encode_profile(PhyMode::HT, 5, 20), vtx);
  agent.on_rc_frame(wire.data(), wire.size(), 100);
  const AppliedOp& op = act.applied.back();
  CHECK(op.ladder[0].mcs == 4);
  for (const auto& l : op.ladder) CHECK(!l.ldpc && l.stbc);
  CHECK(op.probe.mcs == 5);
  CHECK(!op.probe.ldpc && op.probe.stbc);
}

// 11b. Failsafe entry (MAX_RANGE) clears the probe slot even if the last
// RCF before the silence carried a probe_profile — a degraded/lost link
// must never report itself as still probing.
TEST(max_range_clears_the_probe_slot) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links
  auto wire = make_rcf_wire(1, encode_profile(PhyMode::HT, 5, 20), 8, 8,
                            encode_profile(PhyMode::HT, 6, 20), vtx);
  agent.on_rc_frame(wire.data(), wire.size(), 100);
  CHECK(agent.probe_on());
  agent.tick(100 + cfg.link.failsafe_ms + cfg.link.tick_ms, RadioHealth{});  // -> FAILSAFE
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  CHECK(!agent.probe_on());
  CHECK(act.applied.back().probe_profile == kNoProbeProfile);
}

// 11c. A probe must not move the encoder bitrate (spec 2026-08-05 s3-probe-
// promote, carried into 2026-09-04 probe-stream: the probe stream has its
// own slot and costs zero encoder writes — the bitrate stays keyed to the
// base profile's ladder[1]).
TEST(probe_rcf_does_not_change_bitrate) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links
  auto plain = make_rcf_wire(1, encode_profile(PhyMode::HT, 5, 20), 8, 8, vtx);
  agent.on_rc_frame(plain.data(), plain.size(), 100);
  REQUIRE(!act.bitrates.empty());
  const int before = act.bitrates.back();
  auto probed = make_rcf_wire(2, encode_profile(PhyMode::HT, 5, 20), 8, 8,
                              encode_profile(PhyMode::HT, 6, 20), vtx);
  agent.on_rc_frame(probed.data(), probed.size(), 1200);
  CHECK(act.bitrates.back() == before);
}

// 11d. The probe airs at the probe profile's OWN width, not the current
// op's — a 20/4 op with a 40/3 probe_profile (GS sitting on 20 with 40
// above it) must measure the 40 rung at 40, or a 20->40 promote is blind
// (controller Task 11b, 2026-09-24, docs/bw40.md).
TEST(probe_rcf_airs_at_the_probe_profiles_own_width) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links
  const uint8_t probe_byte = encode_profile(PhyMode::HT, 3, 40);
  auto wire = make_rcf_wire(1, encode_profile(PhyMode::HT, 4, 20), 8, 8, probe_byte, vtx);
  agent.on_rc_frame(wire.data(), wire.size(), 100);
  REQUIRE(!act.applied.empty());
  const AppliedOp& op = act.applied.back();
  CHECK(op.ladder[1].bw == 20);   // video ladder stays at the op's width
  CHECK(op.probe.mcs == 3);
  CHECK(op.probe.bw == 40);       // probe airs at its own width, not 20

  // Reverse: op already on 40, probe_profile also 40 (a different mcs) ->
  // probe stays 40 too, not silently coerced to the op's mode/width.
  auto wire2 = make_rcf_wire(2, encode_profile(PhyMode::HT, 3, 40), 8, 8,
                             encode_profile(PhyMode::HT, 4, 40), vtx);
  agent.on_rc_frame(wire2.data(), wire2.size(), 200);
  const AppliedOp& op2 = act.applied.back();
  CHECK(op2.ladder[1].bw == 40);
  CHECK(op2.probe.mcs == 4);
  CHECK(op2.probe.bw == 40);
}

// Link pairing (spec 2026-10-01 §6): a DISC no longer links, so it no longer
// latches link_established either -- the first verified RCF does.
TEST(link_established_latches_on_first_rcf_not_on_disc) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS
  const uint32_t vtx = ack_agent(agent, act, 136, kVrx, 10);
  CHECK(agent.state() == RcAgent::State::RENDEZVOUS);
  CHECK(!agent.take_link_established());
  auto rcf = make_rcf_wire(1, encode_profile(PhyMode::HT, 2, 20), 8, vtx);
  agent.on_rc_frame(rcf.data(), rcf.size(), 20);  // RENDEZVOUS -> LINKED
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.take_link_established());
  CHECK(!agent.take_link_established());
}

// Auto channel select (spec 2026-09-13), as amended by link pairing (spec
// 2026-10-01 §6): DISC with a foreign channel -> ack agreeing to the NEW
// channel from the current one; the retune waits for the first verified RCF
// and then runs from tick() (after main's promote Telem).
TEST(disc_foreign_channel_acks_then_retunes) {
  auto cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  auto wire = make_disc_wire(kVrx, /*op_channel=*/149, 20, 0, 1);
  agent.on_rc_frame(wire.data(), wire.size(), 100);
  REQUIRE(act.controls.size() == 1);
  auto ack = parse_disc_ack(act.controls[0].data(), act.controls[0].size());
  REQUIRE(ack.has_value());
  CHECK(ack->agreed_channel == 149);
  CHECK(act.retunes.empty());                         // a DISC never retunes
  auto rcf = make_rcf_wire(1, encode_profile(PhyMode::HT, 0, 20), 8, ack->vtx_nonce);
  agent.on_rc_frame(rcf.data(), rcf.size(), 150);
  CHECK(act.retunes.empty());                         // deferred to tick()
  agent.tick(200, RadioHealth{});
  REQUIRE(act.retunes.size() == 1);
  CHECK(act.retunes[0] == 149);
  REQUIRE(act.retune_reasons.size() == 1);
  CHECK(act.retune_reasons[0] == "disc");             // spec §7 reason
  CHECK(agent.channel() == 149);
  CHECK(agent.state() == RcAgent::State::LINKED);
}

// A keep-alive DISC for the CURRENT pair re-proposing a channel mid-session
// is ack-only now: the ack still reports the agreed channel, but nothing
// retunes and the op is untouched (spec 2026-10-01 §6: no DISC path
// retunes; an in-session channel change is the RCF hop order's job, and the
// GS only acts on an ack's channel on a NEW vtx_nonce).
TEST(linked_keepalive_disc_foreign_channel_acks_without_retune) {
  auto cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_agent(agent, act, cfg);          // -> LINKED on home
  const uint64_t gen_before = agent.current().generation;
  const size_t applied_before = act.applied.size();
  const size_t controls_before = act.controls.size();
  auto move = make_disc_wire(kVrx, 149, 20, 0, 2);
  agent.on_rc_frame(move.data(), move.size(), 200);          // LINKED keep-alive path
  REQUIRE(act.controls.size() == controls_before + 1);
  auto ack = parse_disc_ack(act.controls.back().data(), act.controls.back().size());
  REQUIRE(ack.has_value());
  CHECK(ack->agreed_channel == 149);
  CHECK(ack->vtx_nonce == vtx);
  agent.tick(300, RadioHealth{});
  CHECK(act.retunes.empty());
  CHECK(agent.channel() == 136);
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current().generation == gen_before);           // no op re-apply
  CHECK(act.applied.size() == applied_before);
}

TEST(disc_same_channel_never_retunes) {
  auto cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  auto wire = make_disc_wire(kVrx, 136, 20, 0, 1);
  agent.on_rc_frame(wire.data(), wire.size(), 100);
  auto rcf = make_rcf_wire(1, encode_profile(PhyMode::HT, 0, 20), 8, last_ack_vtx(act));
  agent.on_rc_frame(rcf.data(), rcf.size(), 150);
  agent.tick(160, RadioHealth{});
  agent.on_rc_frame(wire.data(), wire.size(), 200);   // LINKED keep-alive
  agent.tick(260, RadioHealth{});
  CHECK(act.retunes.empty());
  REQUIRE(act.controls.size() == 2);
  CHECK(parse_disc_ack(act.controls[1].data(), act.controls[1].size())->agreed_channel == 136);
}

TEST(unconfirmed_move_returns_home_after_move_confirm_ms) {
  auto cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  auto wire = make_disc_wire(kVrx, 149, 20, 0, 1);
  agent.on_rc_frame(wire.data(), wire.size(), 50);
  auto rcf = make_rcf_wire(1, encode_profile(PhyMode::HT, 0, 20), 8, last_ack_vtx(act));
  agent.on_rc_frame(rcf.data(), rcf.size(), 90);
  agent.tick(100, RadioHealth{});                      // deferred move at t=100
  agent.tick(1000, RadioHealth{});
  CHECK(act.retunes.size() == 1);
  agent.tick(2100, RadioHealth{});                     // 100 + 2000 elapsed
  REQUIRE(act.retunes.size() == 2);
  CHECK(act.retunes[1] == 136);
  REQUIRE(act.retune_reasons.size() == 2);
  CHECK(act.retune_reasons[1] == "move_unconfirmed");  // spec §7 reason
  CHECK(agent.channel() == 136);
  CHECK(agent.state() == RcAgent::State::RENDEZVOUS);
  // Leaving LINKED clears both sessions (spec §7), same as failsafe entry:
  // an RCF under the old pair is rejected ...
  CHECK(!agent.current_session().valid);
  const uint32_t old_vtx = parse_disc_ack(act.controls[0].data(), act.controls[0].size())->vtx_nonce;
  auto stale = make_rcf_wire(2, encode_profile(PhyMode::HT, 3, 20), 8, old_vtx);
  agent.on_rc_frame(stale.data(), stale.size(), 2200);
  CHECK(agent.state() == RcAgent::State::RENDEZVOUS);
  CHECK(agent.take_auth_reject());
  // ... and the keep-alive DISC (same vrx) gets a NEW pair that links.
  auto disc = make_disc_wire(kVrx, 136, 20, 0, 2);
  agent.on_rc_frame(disc.data(), disc.size(), 2300);
  const uint32_t vtx2 = last_ack_vtx(act);
  CHECK(vtx2 != old_vtx);
  auto first = make_rcf_wire(1, encode_profile(PhyMode::HT, 2, 20), 8, vtx2);
  agent.on_rc_frame(first.data(), first.size(), 2310);
  CHECK(agent.state() == RcAgent::State::LINKED);
}

// Reordered from the brief (which ticked 3000 then 1600, non-monotonic) to a
// monotonic timeline that still exercises all four checkpoints: the move
// (deferred to the tick at 100) is confirmed at 500; no fallback past
// move_confirm_ms (100+2000=2100, checked via retunes.size() staying at 1 --
// state has already moved on to FAILSAFE by then since failsafe_ms(1000) <
// move_confirm_ms(2000) from the RCF's last_fb_ms=500, which the same
// tick(2100) call also crosses); LINKED -> FAILSAFE without a home retune
// (channel stays on the op channel per spec §6: "on the op channel only
// while LINKED or FAILSAFE"); FAILSAFE -> RENDEZVOUS at +rendezvous_ms from
// the FAILSAFE-entry rebase, WITH a home retune.
TEST(gs_frame_confirms_move_and_rendezvous_entry_returns_home) {
  auto cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  auto wire = make_disc_wire(kVrx, 149, 20, 0, 1);
  agent.on_rc_frame(wire.data(), wire.size(), 50);
  const uint32_t vtx = last_ack_vtx(act);
  auto link = make_rcf_wire(1, encode_profile(PhyMode::HT, 0, 20), 8, vtx);
  agent.on_rc_frame(link.data(), link.size(), 90);
  agent.tick(100, RadioHealth{});                      // deferred move
  REQUIRE(act.retunes.size() == 1);
  auto rcf = make_rcf_wire(2, encode_profile(PhyMode::HT, 2, 20), 8, vtx);
  agent.on_rc_frame(rcf.data(), rcf.size(), 500);      // confirms
  agent.tick(2100, RadioHealth{});
  CHECK(act.retunes.size() == 1);                      // no fallback, no FAILSAFE-entry retune
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  CHECK(agent.channel() == 149);                       // FAILSAFE stays on the op channel
  agent.tick(2100 + 30000, RadioHealth{});
  CHECK(agent.state() == RcAgent::State::RENDEZVOUS);
  REQUIRE(act.retunes.size() == 2);
  CHECK(act.retunes[1] == 136);
  REQUIRE(act.retune_reasons.size() == 2);
  CHECK(act.retune_reasons[1] == "rendezvous");        // spec §7 reason
}

TEST(follow_gs_false_acks_home_and_never_retunes) {
  auto cfg = make_cfg();
  cfg.radio.follow_gs = false;
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  auto wire = make_disc_wire(kVrx, 149, 20, 0, 1);
  agent.on_rc_frame(wire.data(), wire.size(), 100);
  REQUIRE(act.controls.size() == 1);
  CHECK(parse_disc_ack(act.controls[0].data(), act.controls[0].size())->agreed_channel == 136);
  auto rcf = make_rcf_wire(1, encode_profile(PhyMode::HT, 0, 20), 8, last_ack_vtx(act));
  agent.on_rc_frame(rcf.data(), rcf.size(), 150);
  agent.tick(200, RadioHealth{});
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(act.retunes.empty());
}

MTEST_MAIN

// A restarted GS resets its RCF seq to ~1 while the drone's tracker holds
// the old session's high seq (bench 2026-07-12: a GS restart must never lock
// the drone out). Since link pairing (spec 2026-10-01 §6) a restarted GS is
// a NEW vrx_nonce: its DISC gets a fresh pair whose seq32 starts from its
// first RCF, so the old session's baseline never applies to it.
TEST(gs_restart_low_seq_accepted_after_failsafe) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});
  const uint32_t vtx = ack_agent(agent, act);

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto old_sess = make_rcf_wire(40000, profile_byte, 8, vtx);
  agent.on_rc_frame(old_sess.data(), old_sess.size(), 100);
  REQUIRE(agent.state() == RcAgent::State::LINKED);

  // GS dies; failsafe fires.
  agent.tick(1200, RadioHealth{});
  REQUIRE(agent.state() == RcAgent::State::FAILSAFE);

  // Restarted GS: new vrx_nonce, fresh seq numbering near zero accepted.
  const uint32_t vtx2 = ack_agent(agent, act, 136, kVrx + 1, 1250);
  auto new_sess = make_rcf_wire(3, profile_byte, 8, 8, kNoProbeProfile, vtx2, 0, kVrx + 1);
  agent.on_rc_frame(new_sess.data(), new_sess.size(), 1300);
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current_session().vrx_nonce == kVrx + 1);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links
  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto rcf = make_rcf_wire(1, profile_byte, 8, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links
  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto r1 = make_rcf_wire(1, profile_byte, 8, vtx);
  agent.on_rc_frame(r1.data(), r1.size(), 100);   // -> LINKED, forced apply
  REQUIRE(act.bitrates.size() == 2);
  const int wanted = act.bitrates.back();

  // Steady-state RCF, same operating point: with the refusal not latched the
  // target still counts as changed, and the throttle window never opened
  // (its timestamp is latched on success too), so the retry goes out at once.
  act.bitrate_ok = true;
  auto r2 = make_rcf_wire(2, profile_byte, 8, vtx);
  agent.on_rc_frame(r2.data(), r2.size(), 200);
  REQUIRE(act.bitrates.size() == 3);
  CHECK(act.bitrates.back() == wanted);

  // Now it IS latched: an unchanged target past the throttle window is not
  // re-sent, i.e. the success path still dedups exactly as before.
  auto r3 = make_rcf_wire(3, profile_byte, 8, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links
  REQUIRE(act.roi_qps.size() == 1);
  CHECK(act.roi_qps.back() == cfg.encoder.roi_qp_low);

  // Still "normal" as far as the agent knows, so the next policy run at the
  // same low bitrate re-attempts the same transition.
  act.roi_ok = true;
  uint8_t mcs0 = encode_profile(PhyMode::HT, 0, 20);
  auto r1 = make_rcf_wire(1, mcs0, 16, vtx);  // ov 1.00, same op
  agent.on_rc_frame(r1.data(), r1.size(), 100);
  REQUIRE(act.roi_qps.size() == 2);
  CHECK(act.roi_qps.back() == cfg.encoder.roi_qp_low);

  // Latched now: no further transition at the same operating point.
  auto r2 = make_rcf_wire(2, mcs0, 16, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links
  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto rcf = make_rcf_wire(1, profile_byte, 8, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links
  uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  auto r1 = make_rcf_wire(1, profile_byte, 4, vtx);
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
  const uint32_t vtx = ack_agent(agent, act);  // pending pair; the first RCF links
  uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  auto r1 = make_rcf_wire(1, profile_byte, 4, vtx);
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

// In-flight channel hop (spec 2026-09-14 §1): the RCF carries hop_ch/
// hop_epoch so the GS can move the drone mid-flight without a DISC
// round-trip. hop_ch 0 means "no hop order" (a pre-hop GS); a new
// (epoch, ch) pair retunes and arms move_pending_ like a DISC move, but
// the RCF that carries the order must not confirm its own move -- the
// NEXT RCF heard on the new channel does that.
// Tagged under (kVrx, vtx, seq). These tests link with link_agent (whose
// link RCF is seq 1), so their own RCFs start at seq 2.
static std::vector<uint8_t> make_rcf_wire_hop(uint16_t seq, uint8_t profile,
                                              double ovb, double ove, uint8_t hop_ch, uint8_t epoch,
                                              uint32_t vtx) {
  Rcf r; r.seq = seq; r.profile = profile; r.fec_overhead_base = ovb;
  r.fec_overhead_enh = ove; r.hop_ch = hop_ch; r.hop_epoch = epoch;
  return pack_rcf(r, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx, seq});
}

TEST(rcf_new_hop_pair_retunes_and_arms_move_confirm) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  auto w = make_rcf_wire_hop(2, 0x24, 1.0, 0.5, /*hop_ch=*/149, /*epoch=*/1, vtx);
  agent.on_rc_frame(w.data(), w.size(), 200);
  REQUIRE(act.retunes.size() == 1);
  CHECK(act.retunes[0] == 149);
  CHECK(act.retune_reasons[0] == "hop");
  CHECK(agent.channel() == 149);
  CHECK(agent.hop_epoch() == 1);
  // nothing heard on 149 -> home after move_confirm_ms
  agent.tick(200 + cfg.link.move_confirm_ms + 100, RadioHealth{});
  REQUIRE(act.retunes.size() == 2);
  CHECK(act.retunes[1] == 136);
  CHECK(act.retune_reasons[1] == "move_unconfirmed");
}

TEST(rcf_same_hop_pair_is_idempotent_and_next_rcf_confirms) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  auto w1 = make_rcf_wire_hop(2, 0x24, 1.0, 0.5, 149, 1, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 200);
  auto w2 = make_rcf_wire_hop(3, 0x24, 1.0, 0.5, 149, 1, vtx);   // repeat, heard on 149
  agent.on_rc_frame(w2.data(), w2.size(), 260);
  CHECK(act.retunes.size() == 1);
  agent.tick(260 + cfg.link.move_confirm_ms + 100, RadioHealth{});
  CHECK(act.retunes.size() == 1);            // confirmed: no move_unconfirmed
  CHECK(agent.channel() == 149);
}

TEST(rcf_hop_withdrawal_returns_to_previous_channel) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  auto w1 = make_rcf_wire_hop(2, 0x24, 1.0, 0.5, 149, 1, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 200);
  auto w2 = make_rcf_wire_hop(3, 0x24, 1.0, 0.5, 136, 2, vtx);   // withdraw: new epoch, old channel
  agent.on_rc_frame(w2.data(), w2.size(), 400);
  REQUIRE(act.retunes.size() == 2);
  CHECK(act.retunes[1] == 136);
  CHECK(act.retune_reasons[1] == "hop");
  CHECK(agent.hop_epoch() == 2);
}

// Unconfirmed hop falls back to the PRE-HOP channel, not home (spec
// 2026-09-14 §1 step 4: "both converge on the old channel"). A GS that
// withdraws at confirm_ms goes back to the old op; a drone that had already
// moved must go there too. Bench 2026-09-26 (GS session 0233): hop 112 ->
// 153 with home == 153, GS withdrew at 510 ms, the drone's fallback went
// "home" -- the channel it was already on -- and the pair sat split 60 s.
// Reverting to go_home_() here makes these fail: the first two stay on (or
// go to) home instead of 149.
static void hop_and_confirm(RcAgent& agent, const Config& cfg, uint8_t ch, uint8_t epoch,
                            uint16_t seq, uint64_t t, uint32_t vtx) {
  auto order = make_rcf_wire_hop(seq, 0x24, 1.0, 0.5, ch, epoch, vtx);
  agent.on_rc_frame(order.data(), order.size(), t);
  auto confirm = make_rcf_wire_hop(static_cast<uint16_t>(seq + 1), 0x24, 1.0, 0.5, ch, epoch, vtx);
  agent.on_rc_frame(confirm.data(), confirm.size(), t + 60);   // heard on ch: confirmed
}

TEST(unconfirmed_hop_into_home_reverts_to_pre_hop_channel) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  hop_and_confirm(agent, cfg, 149, 1, 2, 200, vtx);                  // op is now 149
  auto w = make_rcf_wire_hop(4, 0x24, 1.0, 0.5, /*hop_ch=*/136, 2, vtx);  // target == home
  agent.on_rc_frame(w.data(), w.size(), 1000);
  REQUIRE(act.retunes.size() == 2);
  CHECK(act.retunes[1] == 136);
  agent.tick(1000 + cfg.link.move_confirm_ms + 100, RadioHealth{});
  REQUIRE(act.retunes.size() == 3);
  CHECK(act.retunes[2] == 149);
  CHECK(act.retune_reasons[2] == "move_unconfirmed");
  CHECK(agent.channel() == 149);
}

TEST(unconfirmed_hop_reverts_to_pre_hop_channel_not_home) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  hop_and_confirm(agent, cfg, 149, 1, 2, 200, vtx);
  auto w = make_rcf_wire_hop(4, 0x24, 1.0, 0.5, 161, 2, vtx);
  agent.on_rc_frame(w.data(), w.size(), 1000);
  agent.tick(1000 + cfg.link.move_confirm_ms + 100, RadioHealth{});
  REQUIRE(act.retunes.size() == 3);
  CHECK(act.retunes[2] == 149);
  CHECK(agent.channel() == 149);
}

// The revert is one step: silence on the pre-hop channel too means the GS is
// gone, and the existing fallback (home, RENDEZVOUS) takes over.
TEST(silence_after_hop_revert_falls_back_home) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  hop_and_confirm(agent, cfg, 149, 1, 2, 200, vtx);
  auto w = make_rcf_wire_hop(4, 0x24, 1.0, 0.5, 161, 2, vtx);
  agent.on_rc_frame(w.data(), w.size(), 1000);
  const uint64_t t_revert = 1000 + cfg.link.move_confirm_ms + 100;
  agent.tick(t_revert, RadioHealth{});
  REQUIRE(act.retunes.size() == 3);
  agent.tick(t_revert + cfg.link.move_confirm_ms + 100, RadioHealth{});
  REQUIRE(act.retunes.size() == 4);
  CHECK(act.retunes[3] == 136);
  CHECK(act.retune_reasons[3] == "move_unconfirmed");
  CHECK(agent.state() == RcAgent::State::RENDEZVOUS);
}

// The withdrawing GS's RCF (new epoch, old channel) heard after the revert
// confirms it: no further retune, the drone stays on the pre-hop channel.
TEST(withdraw_rcf_after_hop_revert_confirms_it) {
  auto cfg = make_cfg();
  // The bundle's failsafe_ms (3000): the revert lands move_confirm_ms after
  // the hop order, inside failsafe. With make_cfg's 1000 the drone would be
  // in FAILSAFE by then, which clears the session (spec 2026-10-01 §7) and
  // makes the withdrawal wait for a keep-alive re-pair -- a different test.
  cfg.link.failsafe_ms = 3000;
  MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  hop_and_confirm(agent, cfg, 149, 1, 2, 200, vtx);
  auto w = make_rcf_wire_hop(4, 0x24, 1.0, 0.5, 161, 2, vtx);
  agent.on_rc_frame(w.data(), w.size(), 1000);
  const uint64_t t_revert = 1000 + cfg.link.move_confirm_ms + 100;
  agent.tick(t_revert, RadioHealth{});
  REQUIRE(act.retunes.size() == 3);
  auto wd = make_rcf_wire_hop(5, 0x24, 1.0, 0.5, 149, 3, vtx);
  agent.on_rc_frame(wd.data(), wd.size(), t_revert + 50);
  agent.tick(t_revert + cfg.link.move_confirm_ms + 100, RadioHealth{});
  CHECK(act.retunes.size() == 3);
  CHECK(agent.channel() == 149);
}

TEST(rcf_hop_ch_zero_is_ignored) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  auto w = make_rcf_wire_hop(2, 0x24, 1.0, 0.5, 0, 5, vtx);
  agent.on_rc_frame(w.data(), w.size(), 200);
  CHECK(act.retunes.empty());
  CHECK(agent.hop_epoch() == 0);
}

// Fix round 1, item 1 (reviewer): a restarted GS resets its hop epoch
// numbering along with everything else, so a stale latched hop_epoch_/
// hop_ch_ from the old session must not swallow the new session's first
// hop order. Every session boundary resets the hop latches (promotion, the
// FAILSAFE entry and the move-unconfirmed fallback in rc_agent.cpp) -- here
// pinned via the new-DISC session boundary, which is the one a fresh GS
// process actually takes.
TEST(new_disc_session_clears_stale_hop_state_so_the_next_hop_retunes) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  auto w1 = make_rcf_wire_hop(2, 0x24, 1.0, 0.5, 149, 1, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 200);
  REQUIRE(act.retunes.size() == 1);
  CHECK(act.retunes[0] == 149);

  // The GS restarts: nothing more arrives, so the unconfirmed move sends the
  // drone home (RENDEZVOUS) once move_confirm_ms elapses -- a restart long
  // enough to lose the link, which also clears the session (spec 2026-10-01
  // §7), not a same-session keep-alive DISC, which takes the LINKED
  // ack-only path and does not reach a reset site.
  agent.tick(200 + cfg.link.move_confirm_ms + 100, RadioHealth{});
  REQUIRE(act.retunes.size() == 2);
  CHECK(act.retunes[1] == 136);
  CHECK(act.retune_reasons[1] == "move_unconfirmed");
  CHECK(agent.state() == RcAgent::State::RENDEZVOUS);
  CHECK(agent.hop_epoch() == 0);  // already cleared by the fallback reset

  // The restarted GS re-establishes with a DISC (a NEW vrx_nonce: a fresh
  // pending pair) proposing the same home channel we're already on, so its
  // promotion is not itself a move.
  const uint32_t vtx2 = ack_agent(agent, act, 136, 0xFEEDFACE, 3000);
  CHECK(act.retunes.size() == 2);  // no move: proposed channel == current
  CHECK(agent.hop_epoch() == 0);

  // The restarted GS's own hop epoch numbering restarts too, so it repeats
  // (epoch=1, ch=149) verbatim -- without the reset this would be swallowed
  // as "already applied" (stale hop_epoch_==1 latched from the old
  // session) and the drone would silently stay on home while the GS
  // believes it has moved to 149.
  Rcf r2; r2.seq = 1; r2.profile = 0x24; r2.fec_overhead_base = 1.0;
  r2.fec_overhead_enh = 0.5; r2.hop_ch = 149; r2.hop_epoch = 1;
  auto w2 = pack_rcf(r2, mabur::kDefaultLinkKey, TagCtx{0xFEEDFACE, vtx2, 1});
  agent.on_rc_frame(w2.data(), w2.size(), 3100);
  REQUIRE(act.retunes.size() == 3);
  CHECK(act.retunes[2] == 149);
  CHECK(act.retune_reasons[2] == "hop");
  CHECK(agent.channel() == 149);
  CHECK(agent.hop_epoch() == 1);
}

// Fix round 1, item 2 (reviewer): the spec's freshness guard compares the
// (hop_epoch, hop_ch) PAIR, not the epoch alone -- a same-epoch RCF
// commanding a different channel must still be applied.
TEST(rcf_same_epoch_different_channel_is_applied_not_ignored) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  auto w1 = make_rcf_wire_hop(2, 0x24, 1.0, 0.5, 149, 1, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 200);
  REQUIRE(act.retunes.size() == 1);
  CHECK(act.retunes[0] == 149);

  auto w2 = make_rcf_wire_hop(3, 0x24, 1.0, 0.5, /*hop_ch=*/40, /*epoch=*/1, vtx);
  agent.on_rc_frame(w2.data(), w2.size(), 260);
  REQUIRE(act.retunes.size() == 2);
  CHECK(act.retunes[1] == 40);
  CHECK(act.retune_reasons[1] == "hop");
  CHECK(agent.channel() == 40);
  CHECK(agent.hop_epoch() == 1);
}

// VTX onboard recorder (spec 2026-09-26): RcAgent applies the RCF `rec` wish.

static std::vector<uint8_t> make_rcf_wire_rec(uint16_t seq, uint8_t rec, uint32_t vtx) {
  Rcf r; r.seq = seq; r.profile = 0x24;
  r.fec_overhead_base = 1.0; r.fec_overhead_enh = 0.5; r.rec = rec;
  return pack_rcf(r, mabur::kDefaultLinkKey, TagCtx{kVrx, vtx, seq});
}

TEST(rcf_rec_known_on_calls_set_record_once) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  auto w1 = make_rcf_wire_rec(2, kRecKnown | kRecOn, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 200);
  auto w2 = make_rcf_wire_rec(3, kRecKnown | kRecOn, vtx);
  agent.on_rc_frame(w2.data(), w2.size(), 210);
  REQUIRE(act.records.size() == 1);
  CHECK(act.records[0] == true);
}

TEST(rcf_rec_unknown_leaves_recorder) {
  // maburgs restarted: its wish is unknown until the player re-sends. The
  // onboard recording must NOT stop on that.
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  auto on = make_rcf_wire_rec(2, kRecKnown | kRecOn, vtx);
  agent.on_rc_frame(on.data(), on.size(), 200);
  auto unk = make_rcf_wire_rec(3, 0, vtx);
  agent.on_rc_frame(unk.data(), unk.size(), 210);
  CHECK(act.records.size() == 1);
}

TEST(rcf_rec_known_off_stops_and_link_loss_does_not) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  auto on = make_rcf_wire_rec(2, kRecKnown | kRecOn, vtx);
  agent.on_rc_frame(on.data(), on.size(), 200);
  agent.tick(200 + cfg.link.failsafe_ms + 100, RadioHealth{});   // link lost
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  CHECK(act.records.size() == 1);                                // no timer stop
  const uint32_t vtx2 = ack_agent(agent, act, 136, kVrx, 1900);  // keep-alive re-pairs
  CHECK(act.records.size() == 1);                                // nor a session clear
  auto off = make_rcf_wire_rec(3, kRecKnown, vtx2);
  agent.on_rc_frame(off.data(), off.size(), 2000);
  REQUIRE(act.records.size() == 2);
  CHECK(act.records[1] == false);
}

TEST(rcf_rec_retries_until_the_actuator_takes_it) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg);
  act.record_ok = false;
  auto w1 = make_rcf_wire_rec(2, kRecKnown | kRecOn, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 200);
  act.record_ok = true;
  auto w2 = make_rcf_wire_rec(3, kRecKnown | kRecOn, vtx);
  agent.on_rc_frame(w2.data(), w2.size(), 210);
  auto w3 = make_rcf_wire_rec(4, kRecKnown | kRecOn, vtx);
  agent.on_rc_frame(w3.data(), w3.size(), 220);
  CHECK(act.records.size() == 2);   // failed once, taken once, then latched
}

// GS-requested IDR (spec 2026-09-28 web-idr-request): an idr_epoch CHANGE
// raises one pending IDR, served from tick() through the shared pacer and
// deferred (not dropped) when the 100 ms floor refuses it.

static std::vector<uint8_t> make_rcf_wire_idr(uint16_t seq, uint8_t epoch, uint32_t vtx,
                                              uint32_t vrx = kVrx) {
  Rcf r; r.seq = seq; r.profile = 0x24;
  r.fec_overhead_base = 1.0; r.fec_overhead_enh = 0.5; r.idr_epoch = epoch;
  return pack_rcf(r, mabur::kDefaultLinkKey, TagCtx{vrx, vtx, seq});
}

TEST(gs_idr_epoch_bump_fires_exactly_one_idr) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg, 136, /*t0=*/0);
  const int base = act.idr_calls;
  auto w1 = make_rcf_wire_idr(2, 0, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 200);
  agent.tick(210, RadioHealth{});
  CHECK(act.idr_calls == base);                 // epoch 0 == seen 0: nothing
  auto w2 = make_rcf_wire_idr(3, 1, vtx);
  agent.on_rc_frame(w2.data(), w2.size(), 250);
  CHECK(act.idr_calls == base);                 // served from tick(), not intake
  agent.tick(260, RadioHealth{});
  CHECK(act.idr_calls == base + 1);
  CHECK(agent.idr_gs_total() == 1);
  // The same epoch repeated in every later RCF is not a new request.
  auto w3 = make_rcf_wire_idr(4, 1, vtx);
  agent.on_rc_frame(w3.data(), w3.size(), 300);
  agent.tick(400, RadioHealth{});
  CHECK(act.idr_calls == base + 1);
  CHECK(agent.idr_gs_total() == 1);
}

TEST(gs_idr_inside_pacer_floor_is_deferred_not_dropped) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg, 136, /*t0=*/0);
  const int base = act.idr_calls;
  agent.note_chain_break();
  agent.tick(200, RadioHealth{});               // chain-break IDR at t=200
  REQUIRE(act.idr_calls == base + 1);
  auto w = make_rcf_wire_idr(2, 1, vtx);
  agent.on_rc_frame(w.data(), w.size(), 240);
  agent.tick(250, RadioHealth{});               // 50 ms after the last IDR: refused
  CHECK(act.idr_calls == base + 1);
  agent.tick(300, RadioHealth{});               // floor passed: the SAME request fires
  CHECK(act.idr_calls == base + 2);
  CHECK(agent.idr_gs_total() == 1);
  // REVERT CHECK: clearing idr_gs_pending_ before idr_due() (drop semantics)
  // leaves idr_calls at base + 1 here.
}

TEST(gs_idr_bumps_collapse_into_one_pending_idr) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg, 136, /*t0=*/0);
  const int base = act.idr_calls;
  auto w1 = make_rcf_wire_idr(2, 1, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 200);
  auto w2 = make_rcf_wire_idr(3, 2, vtx);
  agent.on_rc_frame(w2.data(), w2.size(), 205);
  agent.tick(210, RadioHealth{});
  agent.tick(400, RadioHealth{});
  CHECK(act.idr_calls == base + 1);
  CHECK(agent.idr_gs_total() == 1);
}

TEST(gs_idr_epoch_wrap_is_still_a_change) {
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg, 136, /*t0=*/0);
  const int base = act.idr_calls;
  auto w1 = make_rcf_wire_idr(2, 255, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 200);
  agent.tick(210, RadioHealth{});
  auto w2 = make_rcf_wire_idr(3, 0, vtx);
  agent.on_rc_frame(w2.data(), w2.size(), 400);
  agent.tick(410, RadioHealth{});
  CHECK(act.idr_calls == base + 2);
}

TEST(gs_idr_nonzero_epoch_on_first_rcf_after_disc_is_served) {
  // A page that asked before the link came up (epoch already nonzero) is
  // served by the first RCF's own link-up IDR -- exactly one, not a second
  // GS-attributed one 100 ms later (the DISC itself issues none).
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act);
  const uint32_t vtx = ack_agent(agent, act);
  const int base = act.idr_calls;
  auto w = make_rcf_wire_idr(1, 7, vtx);
  agent.on_rc_frame(w.data(), w.size(), 200);
  agent.tick(210, RadioHealth{});
  agent.tick(400, RadioHealth{});
  CHECK(act.idr_calls == base + 1);
}

TEST(gs_idr_seen_epoch_resets_on_session_promotion) {
  // A restarted page is a new GS session (new vrx_nonce). Its pair's
  // promotion -- here while still LINKED, so no link-up IDR masks it --
  // resets the seen epoch, so the new page's epoch 3 (the same value the old
  // page last sent) is still a change and gets its own IDR.
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act);
  const uint32_t vtx = link_agent(agent, act, cfg, 136, /*t0=*/0);
  auto w1 = make_rcf_wire_idr(2, 3, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 200);
  agent.tick(210, RadioHealth{});
  const int after_first = act.idr_calls;
  REQUIRE(agent.idr_gs_total() == 1);
  const uint32_t vtx2 = ack_agent(agent, act, 136, 0xBEEF0001, 300);
  REQUIRE(act.idr_calls == after_first);              // a DISC issues no IDR
  auto w2 = make_rcf_wire_idr(1, 3, vtx2, 0xBEEF0001);
  agent.on_rc_frame(w2.data(), w2.size(), 400);
  REQUIRE(agent.current_session().vrx_nonce == 0xBEEF0001);
  agent.tick(410, RadioHealth{});
  CHECK(act.idr_calls == after_first + 1);      // seen reset to 0, so 3 is a change
  CHECK(agent.idr_gs_total() == 2);
  // REVERT CHECK: drop `idr_epoch_seen_ = 0;` from the promotion block and
  // idr_calls stays at after_first.
}

TEST(gs_idr_served_on_the_tick_that_enters_failsafe) {
  // Serve runs at tick ENTRY like the chain-break consumer: a request whose
  // floor has passed still goes out on the tick that notices lost feedback.
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg, 136, /*t0=*/0);
  const int base = act.idr_calls;
  auto w1 = make_rcf_wire_idr(2, 1, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 250);
  agent.tick(250 + cfg.link.failsafe_ms + 10, RadioHealth{});   // first tick since the RCF
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  CHECK(act.idr_calls == base + 1);
  CHECK(agent.idr_gs_total() == 1);
}

TEST(gs_idr_rcf_failsafe_recovery_sends_exactly_one_idr) {
  // Fix round 1 (spec 2026-09-28): FAILSAFE entry resets idr_epoch_seen_ to
  // 0, so the first RCF back reads the page's UNCHANGED epoch as a fresh
  // change and re-arms idr_gs_pending_ on the very RCF whose entering_linked
  // path already issues a link-up IDR. Without satisfying the pending
  // request there, it survives to the next tick and the 100 ms floor defers
  // it into a redundant second IDR ~100 ms after recovery.
  auto cfg = make_cfg(); MockActuator act; RcAgent agent(cfg, act); const uint32_t vtx = link_agent(agent, act, cfg, 136, /*t0=*/0);
  const int base = act.idr_calls;
  auto w1 = make_rcf_wire_idr(2, 5, vtx);
  agent.on_rc_frame(w1.data(), w1.size(), 200);
  agent.tick(210, RadioHealth{});                // GS request served
  REQUIRE(act.idr_calls == base + 1);
  agent.tick(200 + cfg.link.failsafe_ms + 10, RadioHealth{});
  REQUIRE(agent.state() == RcAgent::State::FAILSAFE);
  const int after_failsafe = act.idr_calls;
  // FAILSAFE -> LINKED via RCF, same epoch as before: entering_linked's own
  // link-up IDR must cover the request the intake just re-armed, in the
  // same RCF -- not a second one 100 ms later. FAILSAFE cleared the
  // session (spec 2026-10-01 §7): the keep-alive DISC re-pairs first.
  const uint32_t vtx2 = ack_agent(agent, act, 136, kVrx, 1300);
  auto w2 = make_rcf_wire_idr(3, 5, vtx2);
  agent.on_rc_frame(w2.data(), w2.size(), 1400);
  agent.tick(1450, RadioHealth{});
  agent.tick(1600, RadioHealth{});
  CHECK(act.idr_calls == after_failsafe + 1);    // exactly one, not two
  CHECK(agent.idr_gs_total() == 1);              // unchanged: the link-up
                                                  // IDR isn't GS-attributed
  // REVERT CHECK: remove the `idr_gs_pending_ = false;` added inside the
  // entering_linked block and act.idr_calls reads after_failsafe + 2 here.
}

// Low-power (pre-arm) mode, spec 2026-09-20.

// Helper: BOOT tick, DISC/ack, then a LINKED mcs5 session entered by the
// first RCF at t=100 (ov 0.5/0.5). Returns the session's vtx_nonce; the
// tests' own RCFs continue at seq 2.
static uint32_t link_up_mcs5(RcAgent& agent, MockActuator& act, const Config& cfg) {
  agent.tick(0, RadioHealth{});
  const uint32_t vtx = ack_agent(agent, act);
  uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  auto r1 = make_rcf_wire(1, profile_byte, 8, vtx);
  agent.on_rc_frame(r1.data(), r1.size(), 100);
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  (void)cfg;
  return vtx;
}

// LP-1. A fresh DISARMED report enters low power: fps first, then the
// bitrate clamped to low_power.bitrate_kbps, and never under the encoder
// floor (test 10e's structural minimum still holds).
TEST(low_power_enters_on_fresh_disarmed_fps_then_capped_bitrate) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  link_up_mcs5(agent, act, cfg);
  REQUIRE(!act.bitrates.empty());
  const int full = act.bitrates.back();
  REQUIRE(full > cfg.low_power.bitrate_kbps);
  CHECK(!act.fps.empty());
  CHECK(act.fps.back() == 60);          // boot/link-up asserted the config fps
  const size_t nf = act.fps.size(), nb = act.bitrates.size();

  agent.note_arm_state(false, 150);
  agent.tick(200, RadioHealth{});
  CHECK(agent.low_power());
  REQUIRE(act.fps.size() == nf + 1);
  CHECK(act.fps.back() == 15);
  REQUIRE(act.bitrates.size() == nb + 1);
  CHECK(act.bitrates.back() == cfg.low_power.bitrate_kbps);
  CHECK(act.bitrates.back() >= cfg.encoder.bitrate_min_kbps);
  REQUIRE(act.verbs.size() >= 2);
  CHECK(act.verbs[act.verbs.size() - 2] == "fps");
  CHECK(act.verbs.back() == "bitrate");
}

// LP-2. ARMED exits low power and latches: a later DISARMED report never
// re-enters, and sends nothing.
// LP-2. ARMED exits low power; a later DISARMED report RE-ENTERS it. The
// arm state is followed both ways, so a downed-but-powered aircraft drops
// back to the thin stream (the post-crash case: mcs0, 15 fps reaches the GS
// where full rate does not). armed_latched() still records that this process
// saw an arm, but is observability only and must not gate the mode.
TEST(low_power_exits_on_armed_and_reenters_on_disarm) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  link_up_mcs5(agent, act, cfg);
  agent.note_arm_state(false, 150);
  agent.tick(200, RadioHealth{});
  REQUIRE(agent.low_power());

  agent.note_arm_state(true, 300);
  agent.tick(400, RadioHealth{});
  CHECK(!agent.low_power());
  CHECK(agent.armed_latched());
  CHECK(act.fps.back() == 60);
  CHECK(act.bitrates.back() > cfg.low_power.bitrate_kbps);

  // Disarm again: back to the low-power operating point, fps before bitrate,
  // even though the latch is set.
  const size_t nf = act.fps.size(), nb = act.bitrates.size();
  agent.note_arm_state(false, 500);
  agent.tick(600, RadioHealth{});
  CHECK(agent.low_power());
  CHECK(agent.armed_latched());          // still true: it records history only
  REQUIRE(act.fps.size() == nf + 1);
  CHECK(act.fps.back() == 15);
  REQUIRE(act.bitrates.size() == nb + 1);
  CHECK(act.bitrates.back() == cfg.low_power.bitrate_kbps);
  REQUIRE(act.verbs.size() >= 2);
  CHECK(act.verbs[act.verbs.size() - 2] == "fps");
  CHECK(act.verbs.back() == "bitrate");

  // And arming a second time exits again -- the cycle is repeatable.
  agent.note_arm_state(true, 700);
  agent.tick(800, RadioHealth{});
  CHECK(!agent.low_power());
  CHECK(act.fps.back() == 60);
}

// LP-2b. Fail open is unchanged by re-entry: after an arm, the FC going
// SILENT (crash kills its power, UART dies) must NOT re-enter low power --
// staleness means full rate, however long the silence lasts.
TEST(low_power_silence_after_arming_never_reenters) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_up_mcs5(agent, act, cfg);
  agent.note_arm_state(true, 100);
  agent.tick(200, RadioHealth{});
  REQUIRE(!agent.low_power());

  // No further reports, ever. RCFs keep the link LINKED so the only policy
  // runs are the ones this test is about.
  const uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  uint16_t seq = 2;
  for (uint64_t t = 300; t <= 9000; t += 300) {
    auto r = make_rcf_wire(seq++, profile_byte, 8, vtx);
    agent.on_rc_frame(r.data(), r.size(), t);
    agent.tick(t, RadioHealth{});
    CHECK(!agent.low_power());
  }
  CHECK(act.fps.back() == 60);
}

// LP-3. Silence: a DISARMED report older than stale_ms means full power
// (fail open). A report stamped slightly AFTER the tick clock (the MSP
// thread stamps on its own read) still counts as fresh.
TEST(low_power_exits_when_the_disarmed_report_goes_stale) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_up_mcs5(agent, act, cfg);
  const uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  agent.note_arm_state(false, 250);     // ahead of the next tick's 200
  agent.tick(200, RadioHealth{});
  CHECK(agent.low_power());
  // RCFs keep the link LINKED (failsafe_ms 1000) so the only policy runs
  // below are the ones this test is about.
  auto r2 = make_rcf_wire(2, profile_byte, 8, vtx);
  agent.on_rc_frame(r2.data(), r2.size(), 900);
  agent.tick(1000, RadioHealth{});      // report 750 ms old: still fresh
  CHECK(agent.low_power());
  auto r3 = make_rcf_wire(3, profile_byte, 8, vtx);
  agent.on_rc_frame(r3.data(), r3.size(), 1800);
  agent.tick(2300, RadioHealth{});      // 2050 ms old: stale
  CHECK(!agent.low_power());
  CHECK(act.fps.back() == 60);
  CHECK(act.bitrates.back() > cfg.low_power.bitrate_kbps);
}

// LP-4. enable = false: no fps verb is ever sent and no clamp applies,
// whatever the FC says.
TEST(low_power_disabled_never_touches_fps_or_clamps) {
  Config cfg = make_cfg();
  cfg.low_power.enable = false;
  MockActuator act;
  RcAgent agent(cfg, act);
  link_up_mcs5(agent, act, cfg);
  CHECK(act.fps.empty());
  const int full = act.bitrates.back();
  agent.note_arm_state(false, 150);
  agent.tick(200, RadioHealth{});
  agent.tick(300, RadioHealth{});
  CHECK(!agent.low_power());
  CHECK(act.fps.empty());
  CHECK(act.bitrates.back() == full);
}

// LP-5. A refused set_fps is retried on the next tick -- in RENDEZVOUS, on
// the ground, where there are no RCFs to carry a retry -- and latched only
// on success, then the retry stops.
TEST(low_power_refused_fps_is_retried_in_rendezvous_then_latched) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});         // BOOT -> RENDEZVOUS
  REQUIRE(agent.state() == RcAgent::State::RENDEZVOUS);
  const size_t nf0 = act.fps.size();

  act.fps_ok = false;
  agent.note_arm_state(false, 150);
  agent.tick(200, RadioHealth{});       // transition: attempt 1, refused
  CHECK(agent.low_power());
  REQUIRE(act.fps.size() == nf0 + 1);
  CHECK(act.fps.back() == 15);
  agent.tick(300, RadioHealth{});       // retry
  REQUIRE(act.fps.size() == nf0 + 2);
  act.fps_ok = true;
  agent.tick(400, RadioHealth{});       // lands
  REQUIRE(act.fps.size() == nf0 + 3);
  agent.tick(500, RadioHealth{});
  agent.tick(600, RadioHealth{});
  CHECK(act.fps.size() == nf0 + 3);     // latched: no more retries
}

// LP-6. The 5 s re-assert restates the fps too (force), bounding any
// override of the encoder fps behind RcAgent's back.
TEST(low_power_reassert_restates_fps) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  const uint32_t vtx = link_up_mcs5(agent, act, cfg);
  agent.note_arm_state(false, 150);
  agent.tick(200, RadioHealth{});
  REQUIRE(agent.low_power());
  const size_t nf = act.fps.size();
  const uint8_t profile_byte = encode_profile(PhyMode::HT, 5, 20);
  uint16_t seq = 2;
  for (uint64_t t = 300; t <= 5100; t += 100) {
    agent.note_arm_state(false, t);     // FC keeps answering DISARMED
    // Same-op RCFs keep the link LINKED; with force=false an unchanged
    // target sends nothing (fps or bitrate).
    auto r = make_rcf_wire(seq++, profile_byte, 8, vtx);
    agent.on_rc_frame(r.data(), r.size(), t);
    agent.tick(t, RadioHealth{});
  }
  CHECK(act.fps.size() == nf);          // nothing inside the interval
  agent.note_arm_state(false, 5300);
  agent.tick(5300, RadioHealth{});      // >= 5000 since the accepted bitrate at 200; no RCF this ms
  CHECK(act.fps.size() == nf + 1);
  CHECK(act.fps.back() == 15);
}

// LP-7. FAILSAFE entry while in low power keeps the cap: the floor is
// min(max-range target, low_power.bitrate_kbps).
TEST(low_power_cap_survives_failsafe_entry) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  link_up_mcs5(agent, act, cfg);
  agent.note_arm_state(false, 150);
  agent.tick(200, RadioHealth{});
  REQUIRE(agent.low_power());
  agent.note_arm_state(false, 1100);
  agent.tick(1200, RadioHealth{});      // 1100 ms since the RCF at 100: FAILSAFE
  REQUIRE(agent.state() == RcAgent::State::FAILSAFE);
  CHECK(agent.low_power());
  CHECK(act.bitrates.back() <= cfg.low_power.bitrate_kbps);
  CHECK(act.fps.back() == 15);
}
