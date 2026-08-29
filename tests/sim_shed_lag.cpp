// Validation sim for the encoder-shed-lag hypothesis (ctl-0020, 2026-08-09).
//
// Replays episode B of /media/dvr ctl-0020 (demote cascade rung 5->0 at
// t=598.9..600.1s) into the REAL RcAgent at the real 100ms RCF cadence and
// records when the encoder actually receives a bitrate command vs when the
// radio MCS changes. Then a coarse TxQueue/air model turns the measured
// encoder timeline into predicted oversubscription at rung 0, compared
// against the utilization decay the ctl log recorded (u = 0.97, 0.63, 0.05
// at 1 Hz after reaching rung 0). A counterfactual run models the proposed
// fix (bitrate call forced on any decrease, ~150ms apply latency).
//
// This is a hypothesis-validation harness, not a regression gate: the
// CHECKs pin the mechanism (throttle+hysteresis swallow the cascade's
// intermediate sheds; the floor shed lands >1s after the cascade starts),
// and the tables are for the human reading the output.
//
// Model assumptions (coarse, stated so the output can be judged):
//  - air bytes = video * (1 + eff1) with eff1 = uep_layer_overhead(1, ov)
//    of the APPLIED op; the s3 enhancement stream is ignored (it is shed by
//    the congestion guard early in a real overload), so offered load here
//    is an UNDERESTIMATE.
//  - channel service rate = phy_rate_mbps(mcs) * ETA, ETA = 0.75 MAC
//    efficiency (sensitivity printed for 0.65/0.85).
//  - TxQueue = 256 bodies x 1328B payload, drop-oldest.
//  - waybeam HTTP + encoder rate-control latency = 50ms after the call.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
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

// ---- episode B ground truth (ms, ctl-0020 timebase) ------------------

struct RungDef { int mcs; double ov; };
// ctl-0020 header: ladder=0/100,2/50,4/25,5/25,6/25,7/10
const RungDef kLadder[6] = {{0, 1.00}, {2, 0.50}, {4, 0.25},
                            {5, 0.25}, {6, 0.25}, {7, 0.10}};

struct Transition { double t_ms; int to; };
// E lines, episode B (5->4->3->2->1->0), plus observed rung-0 u decay.
const Transition kCascade[] = {{598931, 4}, {598981, 3}, {599143, 2},
                               {599445, 1}, {600107, 0}};
const double kObservedU[][2] = {  // {t_ms, u} S lines after reaching rung 0
    {600802, 0.9715}, {601803, 0.6313}, {602805, 0.0488}, {603812, 0.0170}};

int rung_at(double t_ms) {
  int r = 5;
  for (const auto& tr : kCascade)
    if (t_ms >= tr.t_ms) r = tr.to;
  return r;
}

// ---- recording actuator ----------------------------------------------

struct TimedActuator : Actuator {
  uint64_t now = 0;  // sim writes this before every RcAgent call
  struct BitrateCall { uint64_t t; int kbps; };
  struct McsChange { uint64_t t; int mcs; };
  std::vector<BitrateCall> bitrates;
  std::vector<McsChange> mcs_changes;
  int last_mcs = -1;
  int idr_calls = 0;

  void apply_op(const AppliedOp& op) override {
    if (op.ladder[1].mcs != last_mcs) {
      last_mcs = op.ladder[1].mcs;
      mcs_changes.push_back({now, last_mcs});
    }
  }
  void send_control(const std::vector<uint8_t>&) override {}
  bool set_bitrate_kbps(int kbps) override { bitrates.push_back({now, kbps}); return true; }
  bool set_roi_qp(int) override { return true; }
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

std::vector<uint8_t> rcf_wire(uint32_t vtx_id, uint16_t seq, int rung) {
  Rcf r;
  r.vtx_id = vtx_id;
  r.seq = seq;
  r.profile = encode_profile(PhyMode::HT, static_cast<uint8_t>(kLadder[rung].mcs), 20);
  r.fec_overhead = kLadder[rung].ov;
  return pack_rcf(r);
}

// Drives the real RcAgent through link-up, 8s of steady rung 5, then the
// episode-B cascade, at the real 100ms RCF/tick cadence.
TimedActuator replay_real_agent() {
  Config cfg = make_cfg();
  static std::vector<TimedActuator> keep;  // survive return by value copies
  TimedActuator act;
  RcAgent agent(cfg, act);

  const uint64_t t_link = 590000;
  act.now = t_link;
  agent.tick(t_link, RadioHealth{});  // BOOT -> RENDEZVOUS

  Disc d;
  d.vtx_id = cfg.link.vtx_id;
  d.vrx_nonce = 0x1234;
  d.op_channel = 149;
  d.op_width = 20;
  d.init_profile = encode_profile(PhyMode::HT, 7, 20);
  d.seq = 1;
  auto disc = pack_disc(d);
  agent.on_rc_frame(disc.data(), disc.size(), t_link);  // -> LINKED

  uint16_t seq = 2;
  for (uint64_t t = t_link + 100; t <= 605000; t += 100) {
    act.now = t;
    agent.tick(t, RadioHealth{});
    auto wire = rcf_wire(cfg.link.vtx_id, seq++, rung_at(static_cast<double>(t)));
    agent.on_rc_frame(wire.data(), wire.size(), t);
  }
  return act;
}

// ---- transport model --------------------------------------------------

double phy_mbps(int mcs) {
  static const double t[8] = {6.5, 13, 19.5, 26, 39, 52, 58.5, 65};
  return t[mcs];
}
double eff1(double ov) { return uep_layer_overhead(1, ov); }

struct QueueSample { double t, video_kbps, offered_kbps, cap_kbps, queue_ms, drop_frac; };

// encoder timeline: piecewise-constant video kbps from (t, kbps) points.
struct Step { double t; int kbps; };

struct QueueVerdict {
  double drop_frac = 0;      // dropped bytes / offered bytes over the episode
  double drained_at_ms = 0;  // when the queue first empties after rung 0
};

std::vector<QueueSample> run_queue_model(const std::vector<Step>& enc,
                                         double eta, QueueVerdict* verdict) {
  const double cap_bytes = 256.0 * 1328.0;
  const double dt = 10.0;  // ms
  double q = 0, dropped = 0, offered_tot = 0;
  double drained_at = 0;
  std::vector<QueueSample> out;
  for (double t = 597000; t <= 606000; t += dt) {
    int video = enc.front().kbps;
    for (const auto& s : enc)
      if (t >= s.t) video = s.kbps;
    int r = rung_at(t);
    // applied ov follows the rung the drone has heard about (next 100ms grid)
    double ov = kLadder[rung_at(t - 100)].ov;
    double offered = video * (1.0 + eff1(ov));                   // kbps
    double cap = phy_mbps(kLadder[r].mcs) * 1000.0 * eta;        // kbps
    double arr = offered / 8.0 * dt / 1000.0 * 1000.0;           // bytes in dt
    double srv = cap / 8.0 * dt / 1000.0 * 1000.0;
    q = std::max(0.0, q + arr - srv);
    offered_tot += arr;
    if (q > cap_bytes) { dropped += q - cap_bytes; q = cap_bytes; }
    if (r == 0 && q <= 0.0 && drained_at == 0) drained_at = t;
    if (static_cast<int64_t>(t) % 100 == 0)
      out.push_back({t, static_cast<double>(video), offered, cap,
                     q * 8.0 / cap, offered_tot > 0 ? dropped / offered_tot : 0});
  }
  if (verdict) {
    verdict->drop_frac = offered_tot > 0 ? dropped / offered_tot : 0;
    verdict->drained_at_ms = drained_at;
  }
  return out;
}

void print_timeline(const char* title, const std::vector<QueueSample>& qs) {
  std::printf("\n%s\n", title);
  std::printf("      t(s)  rung  video_cmd  offered   capacity  queue_lag  drops\n");
  for (const auto& s : qs) {
    if (s.t < 598500 || s.t > 603500) continue;
    std::printf("  %8.1f  %4d  %7.0fk  %7.0fk  %7.0fk  %6.0fms  %4.0f%%\n",
                s.t / 1000.0, rung_at(s.t), s.video_kbps, s.offered_kbps,
                s.cap_kbps, s.queue_ms, s.drop_frac * 100.0);
  }
}

}  // namespace

TEST(shed_lag_replay_and_model) {
  // ---- part 1: real RcAgent replay -----------------------------------
  TimedActuator act = replay_real_agent();

  std::printf("\n=== PART 1: real RcAgent replay of ctl-0020 episode B ===\n");
  std::printf("cascade: 5->4 @598.93  4->3 @598.98  3->2 @599.14  "
              "2->1 @599.45  1->0 @600.11 (s)\n");
  std::printf("\nMCS changes applied by drone:\n");
  for (const auto& m : act.mcs_changes)
    if (m.t > 598000)
      std::printf("  t=%8.2fs  mcs%d\n", m.t / 1000.0, m.mcs);
  std::printf("\nencoder bitrate calls (set_bitrate_kbps):\n");
  std::vector<Step> enc = {{0, 0}};
  for (const auto& b : act.bitrates) {
    if (b.t > 590000)
      std::printf("  t=%8.2fs  %d kbps\n", b.t / 1000.0, b.kbps);
    if (b.t <= 598000) enc[0] = {0, b.kbps};  // steady-state value
    else enc.push_back({static_cast<double>(b.t) + 50.0, b.kbps});  // +waybeam
  }

  // Fix pins (shed-sync design 2026-08-09): every decreased target goes
  // out with the rung change — the floor value within one RCF+tick of the
  // rung-0 commit, and the walk's intermediate sheds (19300, 14500, 5100)
  // no longer swallowed by the throttle/hysteresis.
  uint64_t t_1400 = 0;
  int calls_in_cascade = 0;
  for (const auto& b : act.bitrates) {
    if (b.t >= 598900 && b.kbps == 1400 && !t_1400) t_1400 = b.t;
    if (b.t >= 598900 && b.t <= 600300) ++calls_in_cascade;
  }
  REQUIRE(t_1400 != 0);
  std::printf("\nfloor shed (1400) landed at t=%.2fs: %+.0fms after cascade "
              "start, %+.0fms after rung-0 commit\n",
              t_1400 / 1000.0, t_1400 - 598931.0, t_1400 - 600107.0);
  CHECK(t_1400 - 600107 <= 150);   // floor shed rides the rung-0 RCF
  CHECK(calls_in_cascade >= 4);    // 19300, 14500, 5100, 1400 all applied

  // ---- part 2: queue/air model on the measured encoder timeline ------
  QueueVerdict real_v;
  auto qs = run_queue_model(enc, 0.75, &real_v);
  print_timeline("=== PART 2: transport model, measured encoder timeline "
                 "(ETA=0.75) ===", qs);
  for (double eta : {0.65, 0.85}) {
    QueueVerdict v;
    run_queue_model(enc, eta, &v);
    std::printf("  sensitivity: ETA=%.2f -> drops %.0f%%, queue drained at "
                "t=%.1fs\n", eta, v.drop_frac * 100, v.drained_at_ms / 1000.0);
  }
  std::printf("  observed ctl-0020 u at rung 0: ");
  for (const auto& o : kObservedU) std::printf("%.2f@%.1fs  ", o[1], o[0] / 1000.0);
  std::printf("\n");

  // ---- part 3: counterfactual (shed forced on decrease, 150ms apply) --
  std::vector<Step> fix = {{0, 20000}};
  int prev = 20000;
  for (const auto& tr : kCascade) {
    double mbps = phy_mbps(kLadder[tr.to].mcs);
    double kbps = mbps * 1000.0 * 0.65 / (1.0 + eff1(kLadder[tr.to].ov));
    int k = static_cast<int>(std::clamp(kbps, 1000.0, 20000.0) / 100.0 + 0.5) * 100;
    if (k < prev) fix.push_back({tr.t_ms + 150.0, k});  // RCF+poll+waybeam
    prev = std::min(prev, k);
  }
  QueueVerdict fix_v;
  auto qf = run_queue_model(fix, 0.75, &fix_v);
  print_timeline("=== PART 3: counterfactual, shed forced on every decrease ===",
                 qf);
  std::printf("\nverdict: body drops %.0f%% (reference immediate-shed model "
              "%.0f%%); queue drained at t=%.1fs / t=%.1fs (rung 0 reached "
              "600.11s)\n",
              real_v.drop_frac * 100, fix_v.drop_frac * 100,
              real_v.drained_at_ms / 1000.0, fix_v.drained_at_ms / 1000.0);
  // With the synchronized shed, the replayed agent's own timeline must not
  // overflow the TxQueue any more (the freeze ingredient). The remaining
  // queue-drain tail after a full collapse is bounded and measured by the
  // hardware campaign, not modeled here.
  CHECK(real_v.drop_frac < 0.01);
}

MTEST_MAIN
