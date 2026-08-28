#include <cstdint>
#include <memory>
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

// Records every Actuator call for inspection by the tests.
struct MockActuator : Actuator {
  std::vector<AppliedOp> applied;
  std::vector<std::vector<uint8_t>> controls;
  std::vector<int> bitrates;
  std::vector<int> roi_qps;
  int idr_calls = 0;

  void apply_op(const AppliedOp& op) override { applied.push_back(op); }
  void send_control(const std::vector<uint8_t>& body) override { controls.push_back(body); }
  void set_bitrate_kbps(int kbps) override { bitrates.push_back(kbps); }
  void set_roi_qp(int qp) override { roi_qps.push_back(qp); }
  void request_idr() override { ++idr_calls; }
};

Config make_cfg() {
  Config cfg;
  cfg.link.vtx_id = 1;
  cfg.link.failsafe_ms = 1000;
  cfg.link.rendezvous_ms = 30000;
  cfg.link.tick_ms = 100;
  cfg.waybeam.airtime_budget = 0.65;
  cfg.waybeam.bitrate_min_kbps = 1000;
  cfg.waybeam.bitrate_max_kbps = 20000;
  cfg.waybeam.roi_threshold_kbps = 3000;
  cfg.waybeam.roi_qp_low = 8;
  cfg.waybeam.roi_qp_normal = 0;
  return cfg;
}

// Builds a CRC-valid RCF wire frame for vtx_id/seq/profile/fec_overhead.
std::vector<uint8_t> make_rcf_wire(uint32_t vtx_id, uint16_t seq, uint8_t profile,
                                    uint8_t fec_overhead_16ths) {
  Rcf r;
  r.vtx_id = vtx_id;
  r.seq = seq;
  r.profile = profile;
  r.fec_overhead_16ths = fec_overhead_16ths;
  return pack_rcf(r);
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

// 1. BOOT->RENDEZVOUS on first tick; op is MAX_RANGE (mcs0 ladder, shed
// T1+T2, ov 1.0).
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
  for (int i = 0; i < 4; ++i) {
    CHECK(op.ladder[static_cast<size_t>(i)].mode == expect_ladder[static_cast<size_t>(i)].mode);
    CHECK(op.ladder[static_cast<size_t>(i)].mcs == expect_ladder[static_cast<size_t>(i)].mcs);
    CHECK(op.ladder[static_cast<size_t>(i)].bw == expect_ladder[static_cast<size_t>(i)].bw);
  }
  CHECK(op.fec_overhead > 0.999 && op.fec_overhead < 1.001);
  CHECK(op.shed[0] == false);
  CHECK(op.shed[1] == false);
  CHECK(op.shed[2] == true);
  CHECK(op.shed[3] == true);

  // BOOT's initial MAX_RANGE apply forces the bitrate policy to the robust
  // MCS0 floor immediately (T0=6.5Mbps, ov=uep_layer_overhead(1,1.0)=2.0
  // clamped -> 6500*0.65/3.0 = 1408.33 -> rounds to 1400).
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

// 2b. Keep-alive DISC while LINKED is ignored end-to-end — Python parity
// (rendezvous.feed_disc: `if self.state not in (RC_LOST, DISCOVERY): return
// None`). The GS sends a SESSION keep-alive DISC (~1 Hz, init_profile 0 =
// MAX_RANGE row) so a drone that silently fell back re-links immediately; a
// healthy LINKED drone must not ACK it, must not apply the init profile
// (which would yank the op to MAX_RANGE/floor every second — bench-observed
// op thrash, 2026-07-12), and must not refresh the failsafe watchdog.
TEST(keepalive_disc_while_linked_is_ignored) {
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

  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current().generation == gen);   // op untouched (no MAX_RANGE yank)
  CHECK(agent.current().ladder[1].mcs == 2);  // still the RCF rung
  CHECK(agent.current().ladder[2].mcs == 2);  // T1 rides the same base mcs
  CHECK(act.controls.size() == n_controls);   // no DISC_ACK
  CHECK(act.bitrates.size() == n_bitrates);   // bitrate policy not re-forced

  // The ignored DISC must not have refreshed the failsafe watchdog: last
  // real feedback was the RCF at t=100, so failsafe_ms=1000 fires at t=1100.
  agent.tick(1099, RadioHealth{});
  CHECK(agent.state() == RcAgent::State::LINKED);
  agent.tick(1100, RadioHealth{});
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
}

// 3. RCF profile HT mcs2/20, fec16=8 (ov=0.5) -> op ladder all four rungs at
// mcs2 (base rate; FEC overhead is the sole per-layer differentiator), ov
// 0.5; set_bitrate_kbps called with ~5100.
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
  CHECK(op.ladder[0].mcs == 2);  // CRIT
  CHECK(op.ladder[1].mcs == 2);  // T0
  CHECK(op.ladder[2].mcs == 2);  // T1 — same base mcs
  CHECK(op.ladder[3].mcs == 2);  // T2 — same base mcs
  CHECK(op.fec_overhead > 0.499 && op.fec_overhead < 0.501);

  REQUIRE(!act.bitrates.empty());
  CHECK(act.bitrates.back() == 5100);
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
  CHECK(op.fec_overhead > 0.999 && op.fec_overhead < 1.001);

  // LINKED->FAILSAFE entry forces the bitrate policy to the MCS0 floor
  // (1400 kbps) immediately, bypassing the steady-state throttle/hysteresis
  // — the encoder must not keep flooding at the last LINKED bitrate (5100)
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

// 9. tx_drops rising 2 ticks -> shed[3] then shed[2] true; 2s clean -> back
// off. Refreshed with a periodic RCF (every <failsafe_ms=1000ms) throughout
// so the agent stays LINKED for the whole scenario — without it, the last
// two ticks (2300/4400ms with no RCF since t=0) land past failsafe_ms and
// silently transition LINKED->FAILSAFE, which forces shed[2]/shed[3] true
// via failsafe_shed_ (see C2(b)) and would make this congestion-only
// decay test spuriously fail/pass for the wrong reason.
TEST(congestion_shed_escalates_and_recovers) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(wire.data(), wire.size(), 0);
  CHECK(agent.current().shed[3] == false);
  CHECK(agent.current().shed[2] == false);

  agent.tick(100, RadioHealth{0, 5});  // drops rose 0->5: level 1 (shed T2)
  CHECK(agent.current().shed[3] == true);
  CHECK(agent.current().shed[2] == false);

  agent.tick(200, RadioHealth{0, 10});  // drops rose 5->10: level 2 (shed T1 too)
  CHECK(agent.current().shed[3] == true);
  CHECK(agent.current().shed[2] == true);

  // Keep LINKED alive with fresh RCFs (seq increasing) before the failsafe
  // window (1000ms since last feedback) would otherwise elapse.
  auto wire2 = make_rcf_wire(cfg.link.vtx_id, 2, profile_byte, 8);
  agent.on_rc_frame(wire2.data(), wire2.size(), 900);
  auto wire3 = make_rcf_wire(cfg.link.vtx_id, 3, profile_byte, 8);
  agent.on_rc_frame(wire3.data(), wire3.size(), 1800);

  // 2000ms clean (no new drops) -> level decrements back off.
  agent.tick(2300, RadioHealth{0, 10});
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current().shed[2] == false);
  CHECK(agent.current().shed[3] == true);

  auto wire4 = make_rcf_wire(cfg.link.vtx_id, 4, profile_byte, 8);
  agent.on_rc_frame(wire4.data(), wire4.size(), 2700);
  auto wire5 = make_rcf_wire(cfg.link.vtx_id, 5, profile_byte, 8);
  agent.on_rc_frame(wire5.data(), wire5.size(), 3600);

  agent.tick(4400, RadioHealth{0, 10});
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current().shed[3] == false);
}

// 9b. FAILSAFE entry forces shed[2]/shed[3]; a subsequent congestion-guard
// reapply (which recomputes shed[2]/shed[3] from shed_level_ alone) must not
// clobber the failsafe-forced shed — it has to OR failsafe_shed_ in. Also
// covers: reapply_with_shed() must publish (act.apply_op called again) even
// though it never bumps generation.
TEST(failsafe_shed_survives_congestion_reapply) {
  Config cfg = make_cfg();
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});  // BOOT -> RENDEZVOUS, MAX_RANGE: shed[2]=shed[3]=true

  uint8_t profile_byte = encode_profile(PhyMode::HT, 2, 20);
  auto wire = make_rcf_wire(cfg.link.vtx_id, 1, profile_byte, 8);
  agent.on_rc_frame(wire.data(), wire.size(), 0);  // -> LINKED, shed[2]=shed[3]=false
  CHECK(agent.state() == RcAgent::State::LINKED);
  CHECK(agent.current().shed[2] == false);
  CHECK(agent.current().shed[3] == false);

  agent.tick(1000, RadioHealth{});  // silence -> FAILSAFE, MAX_RANGE reapplied
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  CHECK(agent.current().shed[2] == true);
  CHECK(agent.current().shed[3] == true);
  uint64_t gen_at_failsafe = agent.current().generation;
  size_t applies_at_failsafe = act.applied.size();

  // Congestion guard reapply while still in FAILSAFE: generation must NOT
  // bump (reapply_with_shed never touches it), but the actuator must see a
  // fresh apply_op (a new object every time, per the AppliedOp::generation
  // doc comment) and shed[2]/shed[3] must remain forced true — NOT
  // recomputed down to shed_level_'s (0) sheds.
  agent.tick(1100, RadioHealth{0, 5});  // tx_drops rose 0->5
  CHECK(agent.current().generation == gen_at_failsafe);
  CHECK(act.applied.size() > applies_at_failsafe);
  CHECK(agent.current().shed[2] == true);
  CHECK(agent.current().shed[3] == true);
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
  CHECK(agent.current().shed[2] == false);
  CHECK(agent.current().shed[3] == false);

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
  agent.tick(100, RadioHealth{0, 5});  // tx_drops rose 0->5 -> shed[3]=true, no gen bump
  CHECK(agent.current().shed[3] == true);
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
  const Step down[] = {{4, 4, 14500}, {2, 8, 5100}, {0, 16, 1400}};
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

  // Enter LINKED at mcs2/ov0.5 -> 5100 (forced entry call, stamp t=0).
  auto w0 = make_rcf_wire(cfg.link.vtx_id, 1, encode_profile(PhyMode::HT, 2, 20), 8);
  agent.on_rc_frame(w0.data(), w0.size(), 0);
  REQUIRE(act.bitrates.back() == 5100);

  // Promote to mcs4/ov0.25 (14500) once the stamp expired: call fires.
  auto w1 = make_rcf_wire(cfg.link.vtx_id, 2, encode_profile(PhyMode::HT, 4, 20), 4);
  agent.on_rc_frame(w1.data(), w1.size(), 1100);
  REQUIRE(act.bitrates.back() == 14500);
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
// bench 2026-08-28: prod runs airtime_budget 0.60 / bitrate_max 10000, so
// rung4 (mcs4/ov0.50) commands 39000*0.60/2.5 = 9360 -> 9400, and rung5's
// 52000*0.60/2.5 = 12480 clamps to 10000. |10000-9400| = 600 is inside the
// v1 `changed_enough` deadband (last/10 = 940), so the promote was
// swallowed and the encoder stayed on the mcs4 bitrate forever while the
// link ran mcs5 -- a permanent 6% undershoot at the rung the link sits on
// nearly all the time. The deadband is a filter on an absolute target with
// no accumulator, so the error can never grow into the band: 9400 is a
// fixed point. Dedup of repeated identical RCFs is the 1s throttle's job
// (test 10 and 10b's steady window pin that); this test pins that a
// genuinely CHANGED target is never discarded for being too small a step.
TEST(promote_reaches_encoder_when_clamp_puts_target_inside_deadband) {
  Config cfg = make_cfg();
  cfg.waybeam.airtime_budget = 0.60;    // prod value (/etc/mabur.json)
  cfg.waybeam.bitrate_max_kbps = 10000; // prod value; this clamp is the trap
  MockActuator act;
  RcAgent agent(cfg, act);
  agent.tick(0, RadioHealth{});

  // Enter LINKED at rung 4: mcs4/ov0.50 -> 39000*0.60/2.5 = 9360 -> 9400.
  auto w0 = make_rcf_wire(cfg.link.vtx_id, 1, encode_profile(PhyMode::HT, 4, 20), 8);
  agent.on_rc_frame(w0.data(), w0.size(), 0);
  REQUIRE(agent.state() == RcAgent::State::LINKED);
  REQUIRE(!act.bitrates.empty());
  REQUIRE(act.bitrates.back() == 9400);

  // Promote to rung 5 well after the 1s throttle window: mcs5/ov0.50 ->
  // 12480, clamped to 10000. Only 600 above the last applied value.
  auto w1 = make_rcf_wire(cfg.link.vtx_id, 2, encode_profile(PhyMode::HT, 5, 20), 8);
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
  cfg.waybeam.airtime_budget = 0.60;    // prod value (/etc/mabur.json)
  cfg.waybeam.bitrate_max_kbps = 10000; // prod value
  MockActuator act;
  RcAgent agent(cfg, act);

  // BOOT tick applies MAX_RANGE (mcs0/ov1.00) -> 6500*0.60/3 = 1300, and
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
  CHECK(lowest >= cfg.waybeam.bitrate_min_kbps);
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

// 11. RCF with probe3=true overrides only s3 (ladder[3]) with probe_profile's
// MCS while T0/T1/CRIT stay on the base profile's mcs; agent.probing()
// reflects the last accepted RCF's probe3 bit and reverts (both op.ladder[3]
// and probing()) the moment a follow-up RCF arrives without the flag.
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
  r.fec_overhead_16ths = 8;
  r.probe3 = true;
  r.probe_profile = probe_byte;
  auto wire = pack_rcf(r);
  agent.on_rc_frame(wire.data(), wire.size(), 0);

  CHECK(agent.state() == RcAgent::State::LINKED);
  const AppliedOp& op = agent.current();
  CHECK(op.ladder[0].mcs == 5);
  CHECK(op.ladder[1].mcs == 5);
  CHECK(op.ladder[2].mcs == 5);
  CHECK(op.ladder[3].mcs == 6);  // s3 at probe MCS
  CHECK(agent.probing());

  // A follow-up RCF without the flag reverts s3 to the base profile's mcs
  // and clears probing().
  r.probe3 = false;
  r.seq = 2;
  auto wire2 = pack_rcf(r);
  agent.on_rc_frame(wire2.data(), wire2.size(), 100);
  CHECK(agent.current().ladder[3].mcs == 5);
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
  r.fec_overhead_16ths = 8;
  r.probe3 = true;
  r.probe_profile = encode_profile(PhyMode::HT, 6, 20);
  auto wire = pack_rcf(r);
  agent.on_rc_frame(wire.data(), wire.size(), 0);
  CHECK(agent.probing());

  agent.tick(1000, RadioHealth{});  // silence -> FAILSAFE (MAX_RANGE reapplied)
  CHECK(agent.state() == RcAgent::State::FAILSAFE);
  CHECK(!agent.probing());
}

// 2d. DiscAck.chip_caps advertises CAP_S3_PROBE alongside the existing
// caps — this drone accepts RCF_F_PROBE3.
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
  CHECK(parsed->chip_caps & mabur::rc::CAP_S3_PROBE);
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
