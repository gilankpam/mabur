// maburgs — mabur ground station daemon.
// Plan 1 scope: the dry-run datapath (frame file -> aggregator -> frame tail
// -> AU records; the original RTP output was deleted in PR C).
// Plan 2 scope: real-radio mode (N-card front-ends, control loop, card failover).
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "aggregator.h"
#include "au_doorbell.h"
#include "au_log.h"
#include "au_ring.h"
#include "body_queue.h"
#include "cal_control.h"
#include "cal_log.h"
#include "cal_session.h"
#include "card_scan.h"
#include "channel_plan.h"
#include "channel_scout.h"
#include "config.h"
#include "ctl_log.h"
#include "drone_restart.h"
#include "debug_session.h"
#include "frame_file_source.h"
#include "frame_stream.h"
#include "gap_timeout_policy.h"
#include "hop_controller.h"
#include "hop_blank.h"
#include "hop_burst_gate.h"
#include "hop_ranker.h"
#include "hop_verdict.h"
#include "inflight_scout.h"
#include "ladder_residual.h"
#include "lat_window.h"
#include "log_writer.h"
#ifdef MABUR_LOSS_SIM
#include "loss_control.h"
#endif
#include "mabur/cal_wire.h"
#include "mabur/probe_wire.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"
#include "mabur/sbi.h"
#include "mabur/sw_wire.h"
#include "mabur/uep_encoder.h"
#include "msp_sink.h"
#include "pair_pick.h"
#include "pts_anchor.h"
#include "probe_log.h"
#include "fec_log.h"
#include "probe_track.h"
#include "rcf_slot.h"
#include "rtt_estimator.h"
#include "radio_frontend.h"
#include "rf_labels.h"
#include "s1_loss.h"
#include "scan_log.h"
#include "snr_units.h"
#include "stats_exporter.h"
#include "stats_sink.h"
#include "transition_edge.h"
#include "tx_selector.h"
#include "udp_sink.h"
#include "vrx_controller.h"

namespace {

std::atomic<bool> g_stop{false};
std::atomic<bool> g_dump{false};
void on_signal(int) { g_stop.store(true); }
void on_usr1(int) { g_dump.store(true); }

uint64_t mono_ms() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

// Same clock the au ring writer stamps t_first_us/t_complete_us with
// (clock_gettime CLOCK_MONOTONIC) -- steady_clock == CLOCK_MONOTONIC on
// this glibc/Linux target, so this µs value and the ring's µs stamps share
// one timebase and are directly subtractable (see the fec-segment comment
// in run_radio's end_frame lambda).
uint64_t mono_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

// hop.state on the sideport (Task 12): HopState has no to_string() of its
// own (hop_controller.h/.cpp aren't in this task's file list), so map it
// here, next to the exporter feed's other ad hoc conversions.
const char* hop_state_name(maburgs::HopState s) {
  switch (s) {
    case maburgs::HopState::Idle:      return "idle";
    case maburgs::HopState::Ordered:   return "ordered";
    case maburgs::HopState::Verifying: return "verifying";
    case maburgs::HopState::Hold:      return "hold";
  }
  return "idle";
}

// Shared by run_radio()'s live loop and (under MABUR_TEST)
// run_hop_inject_test(): the 4-case HopAction handler, extracted so gs_e2e's
// hop scenario exercises the SAME code run_radio() runs instead of a
// hand-copied duplicate (Task 15 fix round 1 -- the duplicate's deliberate-
// break check was found to validate only itself, since --dry-run returns
// before run_radio() is ever reached). Deliberately excludes run_radio()'s
// three cross-thread bookkeeping lines (hopping_atomic x2,
// rcf_sent_at_order) -- those have no meaning outside run_radio()'s own
// atomics/counters, so the caller still runs them immediately after this
// call, in the same relative order as before the extraction.
void apply_hop_action(const maburgs::HopAction& act, double now_ms, int confirm_ms,
                      maburgs::VrxController& vrx, maburgs::ChannelPlan& plan,
                      maburgs::HopVerdict& verdict) {
  switch (act.kind) {
    case maburgs::HopAction::Order:
      vrx.set_hop(act.target, act.epoch);
      vrx.restore_rung(act.restore_rung, now_ms);
      vrx.blank_store(now_ms + confirm_ms + 150.0);
      // Two-card (act.lead_card >= 0): retune the lead card right now --
      // the trailing card keeps video alive on op_ throughout, so there is
      // nothing to wait for. One-card (act.lead_card < 0): do NOT retune
      // yet. The sole radio must stay on the OLD channel while the order
      // rides hop.one_card_repeats RCFs (spec section 1 step 1) --
      // HopController already counts those repeats and only emits
      // OneCardRetune once enough have gone out; calling plan.hop_order()
      // here too would move plan.desired() (and so this radio) before the
      // drone could possibly have heard the order, abandoning the only
      // channel it can still be reached on. vrx.set_hop/restore_rung/
      // blank_store above still run unconditionally: the RCF has to start
      // carrying the order immediately, which is the whole point of the
      // repeats.
      if (act.lead_card >= 0) plan.hop_order(now_ms, act.target, act.lead_card);
      break;
    case maburgs::HopAction::OneCardRetune:
      plan.hop_order(now_ms, act.target, -1);
      break;
    case maburgs::HopAction::Confirm:
      plan.hop_confirmed(now_ms);
      break;
    case maburgs::HopAction::Withdraw:
      vrx.set_hop(act.target, act.epoch);
      plan.hop_withdraw(now_ms);
      break;
    case maburgs::HopAction::VerifyPass:
      // The hop stands: thaw the verdict engine's frozen references (spec
      // section 2, "or after a hop's verify window ends"). Before this
      // HopVerdict::reset() had ZERO callers anywhere in the tree, so the
      // per-card RSSI median and the recovered-rate mean stayed frozen at
      // the OLD channel's values for the rest of the flight -- `fading` on
      // the channel we had just moved to was being measured against the
      // one we left. Only verify_pass thaws: a verify_fail or a withdraw
      // RE-ORDERS, and spec section 5 has the retry REUSE the pre-onset
      // ref_rung rather than re-snapshot it. (The retry's own Order is
      // unaffected either way -- order() reads restore_rung off the cached
      // VerdictOut and hopc.tick() has already returned by the time this
      // runs. The cost lands one window later: after a thaw the next
      // impaired window re-freezes ref_rung at the MID-HOP rung, already
      // demoted or already restored, and the pre-onset value section 5
      // wants reused is gone.)
      verdict.reset();
      break;
    case maburgs::HopAction::Hold:
    case maburgs::HopAction::None:
      break;
  }
}

#ifdef MABUR_TEST
// In-flight hop injection seam (Task 15, 2026-09-14-inflight-channel-hop).
// Exists ONLY when MABUR_BUILD_TESTS compiled MABUR_TEST into this binary
// (gs/CMakeLists.txt option; never a production/device build, which passes
// -DMABUR_BUILD_TESTS=OFF -- tools/build-arm.sh, tools/build-arm64.sh) and
// only runs when MABUR_HOP_INJECT is actually set, so a bare `--dry-run`
// invocation (every other gs_e2e scenario) is byte-for-byte unaffected.
//
// Drives the SAME classes run_radio()'s live loop wires together
// (ChannelPlan, HopController, VrxController/LadderController, ScanLog,
// CtlLog) through the real action-handling/RCF-building/log-writing code,
// proving the WIRING between them -- every module underneath already has
// its own unit tests. There is no real radio and no wall clock here: the
// scenario runs on a private double `t` instead, advanced explicitly by
// this function, which is what lets a 1 s verify window finish instantly
// instead of sleeping. `agg` is the SAME Aggregator the fixture bodies were
// fed into by the caller, so the confirming AU goes through the real
// decode path (Aggregator::on_rx_body), not a bypass; `confirm_template` is
// a genuine video body captured from that same fixture stream, replayed
// with its card_id overwritten to the card under test.
//
// MABUR_HOP_INJECT=<target_ch>:<score> is the whole seam: everything else
// (candidate ranking, RF windows, dwells) is exactly what test_hop_ranker.cpp
// and test_hop_verdict.cpp already cover in isolation, so this test skips
// straight to the one decision -- "hop to <target_ch>" -- HopController
// would otherwise reach only after a real ranker sweep.
int run_hop_inject_test(const maburgs::Config& cfg, int n_cards,
                        maburgs::Aggregator& agg,
                        const mabur::node::RxBody& confirm_template) {
  const char* inject = std::getenv("MABUR_HOP_INJECT");
  unsigned target_u = 0, score_u = 0;
  if (std::sscanf(inject, "%u:%u", &target_u, &score_u) != 2) {
    std::fprintf(stderr, "hop-test: MABUR_HOP_INJECT must be '<ch>:<score>', got '%s'\n", inject);
    return 2;
  }
  const uint8_t target = static_cast<uint8_t>(target_u);
  const uint32_t score = score_u;
  const int lead_card = n_cards >= 2 ? n_cards - 1 : -1;
  const int confirm_card = lead_card >= 0 ? lead_card : 0;

  maburgs::DebugSession debug(cfg.debug_log.dir, cfg.debug_log.enable);
  if (!debug.ok()) {
    std::fprintf(stderr,
                 "hop-test: MABUR_HOP_INJECT needs debug_log.enable=true and a "
                 "writable debug_log.dir\n");
    return 2;
  }
  maburgs::LogWriter writer;
  maburgs::CtlLog ctl_log(writer, debug.dir(), "hop-e2e");
  maburgs::ScanLog scan_log(writer, debug.dir(), "hop-e2e");

  maburgs::VrxCfg vcfg;
  vcfg.vtx_id = cfg.link.vtx_id;
  vcfg.op_channel = cfg.radio.channel;
  vcfg.feedback_ms = cfg.link.feedback_ms;
  vcfg.beacon_keepalive_ms = cfg.link.beacon_keepalive_ms;
  vcfg.ladder = cfg.link.ladder_cfg;
  vcfg.pin_mcs = cfg.link.static_mcs;
  vcfg.pin_bw = cfg.link.static_bw;
  vcfg.pin_overhead_base = cfg.link.static_overhead_base;
  vcfg.pin_overhead_enh = cfg.link.static_overhead_enh;
  vcfg.probe_pin_mcs = cfg.link.ladder_cfg.probe.pin_mcs;
  maburgs::VrxController vrx(vcfg);

  maburgs::ChannelPlan plan(maburgs::ChannelPlanCfg{
      cfg.radio.channel, n_cards, cfg.radio.scan.split_after_ms,
      cfg.radio.scan.home_window_ms, 20});
  maburgs::HopController hopc(cfg.hop, cfg.radio.channel);
  // The real verdict engine, so the VerifyPass -> HopVerdict::reset()
  // wiring is exercised end to end rather than asserted by reading the
  // code. It is driven straight below to the state a real interference
  // episode leaves it in: references frozen at the OLD channel.
  maburgs::HopVerdict hop_verdict(cfg.hop, n_cards);
  std::vector<uint8_t> cur_ch(static_cast<size_t>(n_cards), cfg.radio.channel);
  double t = 0.0;
  double last_ctl_t = vrx.ctl().last_event().t_ms;
  const maburgs::LinkHealth healthy{true, 0.0, 0.0, false};

  // Log the ladder's ctl.log transition line whenever LadderController's
  // own last_event() changes -- same t_ms-change-detect pattern main.cpp's
  // live loop uses (ctl_log->event(...) site above run_radio's `E` line).
  auto drain_ctl_event = [&] {
    if (const auto& e = vrx.ctl().last_event(); e.t_ms != last_ctl_t) {
      last_ctl_t = e.t_ms;
      ctl_log.event(e.t_ms, e.from, e.to, maburgs::to_string(e.reason), e.u,
                   e.snr_db, e.evm_db);
    }
  };
  // Apply a HopAction through the SAME apply_hop_action() run_radio() calls
  // (Task 15 fix round 1 -- this used to be a hand-copied duplicate switch,
  // which meant the deliberate-break check below only ever validated the
  // duplicate, not the shipped code path), then drain both controllers'
  // event queues into scan.log/ctl.log. Returns the taken HopEvents so the
  // caller can check for a specific kind (e.g. "order" or "verify_pass")
  // without re-deriving controller state.
  auto apply_action = [&](const maburgs::HopAction& act) {
    apply_hop_action(act, t, cfg.hop.confirm_ms, vrx, plan, hop_verdict);
    for (int i = 0; i < n_cards; ++i)
      cur_ch[static_cast<size_t>(i)] = plan.desired(i);
    drain_ctl_event();
    std::vector<maburgs::HopEvent> events = hopc.take_events();
    for (const auto& e : events) scan_log.hop(e);
    for (const auto& ev : plan.take_events()) scan_log.move(ev);
    return events;
  };
  auto has_kind = [](const std::vector<maburgs::HopEvent>& evs, const char* kind) {
    for (const auto& e : evs)
      if (e.kind == kind) return true;
    return false;
  };

  // 1) Settle: a synthetic DiscAck (mirroring vrx.on_rc_frame() off a real
  // drone reply) so the rendezvous reaches SESSION and step() starts
  // building real RCFs -- the same wiring point a genuine drone drives.
  {
    mabur::rc::DiscAck ack;
    ack.vtx_id = cfg.link.vtx_id;
    ack.vrx_nonce = vrx.rz_nonce();
    ack.chip_caps = mabur::rc::CAP_FRAME_WIRE;
    ack.agreed_channel = cfg.radio.channel;
    ack.seq = 1;
    const auto wire = mabur::rc::pack_disc_ack(ack);
    vrx.on_rc_frame(wire.data(), wire.size(), t);
  }
  bool session_up = false;
  for (int i = 0; i < 200 && !session_up; ++i) {
    t += 10.0;
    vrx.on_video(t);
    vrx.step(t, healthy);
    session_up = vrx.link_state() == maburgs::VrxState::SESSION;
  }
  if (!session_up) {
    std::fprintf(stderr, "hop-test: synthetic link never reached SESSION\n");
    return 2;
  }
  plan.tick(t, true);
  for (int i = 0; i < n_cards; ++i) cur_ch[static_cast<size_t>(i)] = plan.desired(i);

  // 2) Inject the verdict: `interfered`, best candidate = target/score,
  // bypassing HopVerdict/HopRanker -- this IS the injection seam.
  maburgs::VerdictOut vo;
  vo.v = maburgs::Verdict::Interfered;
  vo.trigger = true;
  vo.ref_rung = vrx.ctl().rung();
  // The measurement span a real HopVerdict::window() would have stamped:
  // this verdict was computed BEFORE the hop, on the old channel. main.cpp
  // caches exactly this object and re-feeds it to HopController on every
  // ~10 ms control tick until the next 150 ms window replaces it, so the
  // first thing verifying_tick() ever sees after a Confirm is this -- step
  // 5 below replays it deliberately.
  vo.t_start_ms = t - cfg.hop.window_ms;
  vo.t_ms = t;
  const std::vector<maburgs::VerdictCardIn> vc(static_cast<size_t>(n_cards));
  const maburgs::VerdictLinkIn vl;
  scan_log.verdict(t, vo, vc, vl);

  // Freeze the verdict engine's references the way a real impaired window
  // does (spec section 2), so there is something for verify_pass to thaw.
  {
    std::vector<maburgs::VerdictCardIn> impaired(static_cast<size_t>(n_cards));
    for (auto& c : impaired) {
      c.valid = true;
      c.rssi_dbm = -55; c.snr_db = 30;
      c.foreign = 40;                       // > foreign_pps * window_ms/1000
    }
    maburgs::VerdictLinkIn il;
    il.pre_fec_loss = 0.10;                 // > loss_pct 3 %
    hop_verdict.window(t, impaired, il, vrx.ctl().rung());
  }
  if (hop_verdict.ref_rung() < 0) {
    std::fprintf(stderr, "hop-test: verdict references never froze\n");
    return 2;
  }

  maburgs::HopTick ht;
  ht.now_ms = t;
  ht.verdict = vo;
  ht.cur_op = plan.op();
  ht.n_cards = n_cards;
  ht.lead_card = lead_card;
  ht.best = target;
  ht.best_score = score;
  plan.tick(t, true);
  maburgs::HopAction act = hopc.tick(ht);
  if (act.kind != maburgs::HopAction::Order) {
    std::fprintf(stderr, "hop-test: expected Order, got kind=%d\n",
                 static_cast<int>(act.kind));
    return 2;
  }
  apply_action(act);

  // 3) Dump the first RCF built after the order -- it must already carry
  // hop_ch/hop_epoch (VrxController::build_rcf() stamps them from
  // set_hop() above). One-card (lead_card < 0): keep sending on the OLD
  // channel and re-ticking the controller after every send, exactly the
  // `rcf_sent_since_order` count ordered_tick() escalates on, until it
  // fires OneCardRetune -- the spec's "one_card_repeats RCFs on the old
  // channel before the radio moves".
  int rcf_sent = 0;
  bool dumped = false;
  for (int i = 0; i < 40 && (n_cards >= 2 ? !dumped
                                          : hopc.state() == maburgs::HopState::Ordered);
       ++i) {
    t += cfg.link.feedback_ms;
    vrx.on_video(t);
    auto out = vrx.step(t, healthy);
    if (!out || out->is_disc) continue;
    ++rcf_sent;
    if (const char* path = std::getenv("MABUR_HOP_RCF_OUT"); path && !dumped) {
      if (FILE* f = std::fopen(path, "wb")) {
        std::fwrite(out->frame.data(), 1, out->frame.size(), f);
        std::fclose(f);
      }
    }
    dumped = true;
    if (n_cards == 1) {
      ht.now_ms = t;
      ht.rcf_sent_since_order = rcf_sent;
      ht.video_on_target = false;
      act = hopc.tick(ht);
      apply_action(act);
      // Stop as soon as the radio actually moves: ordered_tick() stays in
      // HopState::Ordered after OneCardRetune (only a Confirm/Withdraw
      // leaves it), so without this the loop would keep re-ticking with
      // video_on_target still false all the way to the confirm_ms
      // timeout and withdraw before step 4 below ever gets to inject the
      // confirming AU.
      if (act.kind == maburgs::HopAction::OneCardRetune) break;
    }
  }
  if (!dumped) {
    std::fprintf(stderr, "hop-test: no RCF built after the order\n");
    return 2;
  }
  if (n_cards == 1 && cur_ch[static_cast<size_t>(confirm_card)] != target) {
    std::fprintf(stderr,
                 "hop-test: one-card radio never reached the target (OneCardRetune "
                 "did not fire within %d RCF sends)\n",
                 rcf_sent);
    return 2;
  }

  // 4) Confirm: replay a genuine video body (captured from the fixture
  // stream the caller already fed through this same Aggregator) on the
  // lead/only card, and check the SAME condition run_radio's batch-drain
  // loop uses to decide "this is video, not RC/MSP/probe traffic" before
  // it updates last_video_ch.
  mabur::node::RxBody cb = confirm_template;
  cb.card_id = static_cast<uint8_t>(confirm_card);
  cb.mono_us = static_cast<uint64_t>(t) * 1000;
  const int sid_peek = mabur::sbi_peek_stream_id(cb.body.data(), cb.body.size());
  const bool is_video = cb.crc_ok &&
                        mabur::rc::frame_type(cb.body.data(), cb.body.size()) < 0 &&
                        sid_peek != mabur::kMspStreamId &&
                        sid_peek != mabur::kProbeStreamId;
  if (!is_video) {
    std::fprintf(stderr, "hop-test: confirm_template is not a video body\n");
    return 2;
  }
  // 4a) The body a real card actually produces first: one received on the
  // OLD channel before the retune landed, drained a tick later. The card
  // is ALREADY tuned to the target by now (cur_ch[confirm_card] == target,
  // the plan moved it), so the pre-fix "where is this card tuned now" read
  // confirmed the hop off it -- before the order's RCF had left the
  // slotter, and on a link that documents 30-50 % RCF uplink loss. Only
  // RxBody::rx_channel, stamped when the frame was lifted off the card,
  // can tell the two apart.
  // Not fed to `agg`: this is a second copy of the SAME body (there is one
  // template), which the decoder would rightly dedupe, and the decode path
  // is not what 4a exercises -- the confirm gate is.
  mabur::node::RxBody stale = cb;
  stale.rx_channel = cfg.radio.channel;   // received before the lead card moved
  vrx.on_video(t);
  ht.now_ms = t;
  ht.video_on_target = plan.hopping() && stale.rx_channel == plan.hop_target();
  if (ht.video_on_target) {
    std::fprintf(stderr,
                 "hop-test: a body received on the OLD channel (%u) confirmed the "
                 "hop to %u\n",
                 static_cast<unsigned>(stale.rx_channel), target);
    return 2;
  }
  act = hopc.tick(ht);
  if (act.kind != maburgs::HopAction::None ||
      hopc.state() != maburgs::HopState::Ordered) {
    std::fprintf(stderr,
                 "hop-test: stale-channel body moved the controller (kind=%d state=%s)\n",
                 static_cast<int>(act.kind), hop_state_name(hopc.state()));
    return 2;
  }
  apply_action(act);

  // 4b) The genuine confirmation: a body stamped with the target, i.e. one
  // the card really did receive after its retune.
  cb.rx_channel = target;
  agg.on_rx_body(cb);
  const uint8_t last_video_ch = cb.rx_channel;
  vrx.on_video(t);
  ht.now_ms = t;
  ht.video_on_target = plan.hopping() && last_video_ch == plan.hop_target();
  if (!ht.video_on_target) {
    std::fprintf(stderr, "hop-test: confirm body did not land on the hop target\n");
    return 2;
  }
  act = hopc.tick(ht);
  if (act.kind != maburgs::HopAction::Confirm) {
    std::fprintf(stderr, "hop-test: expected Confirm, got kind=%d\n",
                 static_cast<int>(act.kind));
    return 2;
  }
  apply_action(act);

  // 5a) The tick right after the Confirm, which in run_radio() is ~10 ms
  // later and carries the SAME cached VerdictOut the order was placed on:
  // Interfered, measured on the old channel. It must not break the verify
  // window (C1 -- unguarded this backed the just-landed clean channel off
  // for 30 s and ordered the next candidate ~10 ms after arriving).
  ht.now_ms = t + 10.0;
  ht.verdict = vo;
  ht.video_on_target = false;
  act = hopc.tick(ht);
  if (act.kind != maburgs::HopAction::None ||
      hopc.state() != maburgs::HopState::Verifying) {
    std::fprintf(stderr,
                 "hop-test: stale pre-hop verdict broke the verify window "
                 "(kind=%d state=%s)\n",
                 static_cast<int>(act.kind), hop_state_name(hopc.state()));
    return 2;
  }
  apply_action(act);

  // 5b) Verify: healthy windows until verify_ms has elapsed since confirm.
  bool verify_pass = false;
  maburgs::VerdictOut healthy_vo;  // default Healthy, trigger=false
  for (int i = 0; i < 200 && !verify_pass; ++i) {
    healthy_vo.t_start_ms = t;
    t += cfg.hop.window_ms;
    healthy_vo.t_ms = t;
    vrx.on_video(t);
    vrx.step(t, healthy);
    scan_log.verdict(t, healthy_vo, vc, vl);
    ht.now_ms = t;
    ht.verdict = healthy_vo;
    ht.video_on_target = false;
    act = hopc.tick(ht);
    verify_pass = has_kind(apply_action(act), "verify_pass");
  }
  if (!verify_pass) {
    std::fprintf(stderr, "hop-test: verify_pass never landed in scan.log\n");
    return 2;
  }
  // Spec section 2's second thaw rule, driven through the SAME
  // apply_hop_action() run_radio() uses: the references frozen on the old
  // channel above must be gone now the hop has stood. Without the
  // HopAction::VerifyPass wiring, HopVerdict::reset() has no caller at all
  // and this still reads the old channel's snapshot.
  if (hop_verdict.ref_rung() >= 0) {
    std::fprintf(stderr,
                 "hop-test: verify_pass did not thaw the verdict references "
                 "(ref_rung still %d)\n",
                 hop_verdict.ref_rung());
    return 2;
  }
  writer.flush_now();
  std::fprintf(stderr, "hop-test: OK (n_cards=%d target=%u)\n", n_cards, target);
  return 0;
}
#endif  // MABUR_TEST

void usage() {
  std::fprintf(stderr,
               "usage: maburgs -c <config.toml> --dry-run --in <frames.bin>\n"
               "               [--cards N] [--drop-pct P] [--seed S] [--out-aus <file>]\n"
#ifdef MABUR_LOSS_SIM
               "       maburgs -c <config.toml> [--loss-sim [port]]\n"
               "\n"
               "  --loss-sim [port]  BENCH ONLY: bind a loopback UDP command\n"
               "                     socket (default port 8302) for injecting\n"
               "                     per-stream loss. Starts at zero; see\n"
               "                     tools/bench/losssim.py.\n"
               "                     Injection is INDEPENDENT PER CARD, so a\n"
               "                     body only reaches the decoder as lost when\n"
               "                     every card drops it: `sN loss=X` sets the\n"
               "                     per-card rate, `sN eff=X` sets the nominal\n"
               "                     union rate (percard^ncards) and solves for\n"
               "                     per-card. Replies always state both.\n"
               "                     `eff` is NOMINAL -- it assumes every card\n"
               "                     heard every body, which only holds on a\n"
               "                     clean link; on a link already losing\n"
               "                     bodies the true injected loss is higher.\n"
               "                     Record real loss from the stats sideport's\n"
               "                     per-stream counters, never from the dial.\n"
#endif
               );
}

// Dry-run AU capture for the e2e: one LP record per reassembled AU --
// u32 total_len | u8 sid | u8 flags (framewire idr|discont, bit 0x04 =
// complete here (NOT the ring's 0x80 -- this LP format is local to the
// dry-run/e2e pair)) | u32 pts_us | Annex-B bytes. Parsed by
// tests/integration/verify_aus.py; keep the two in sync.
struct AuFileOut {
  FILE* f = nullptr;
  uint64_t written = 0;
  std::vector<uint8_t> au;
  mabur::framewire::FrameHdr hdr{};
  uint8_t sid = 0;
  bool in_au = false;
  bool open(const char* path) { f = fopen(path, "wb"); return f != nullptr; }
  ~AuFileOut() { if (f) fclose(f); }
  void begin(const mabur::framewire::FrameHdr& h, uint8_t s) {
    if (!f) return;
    hdr = h; sid = s; au.clear(); in_au = true;
  }
  void append(const uint8_t* d, size_t n) {
    if (f && in_au) au.insert(au.end(), d, d + n);
  }
  void finish(bool complete) {
    // Host-endian fwrite of the u32 fields; verify_aus.py unpacks "<I".
    // Fine on every LE host in play (the dry-run only runs on the x86-64
    // dev box); explicit LE serialization needed if that ever changes.
    if (!f || !in_au) return;
    in_au = false;
    const uint32_t len = static_cast<uint32_t>(au.size());
    const uint8_t flags = static_cast<uint8_t>(hdr.flags | (complete ? 0x04 : 0));
    fwrite(&len, 4, 1, f);
    fwrite(&sid, 1, 1, f);
    fwrite(&flags, 1, 1, f);
    fwrite(&hdr.pts_us, 4, 1, f);
    if (len) fwrite(au.data(), 1, len, f);
    ++written;
  }
};

#ifdef MABUR_LOSS_SIM
static int run_radio(const maburgs::Config& cfg, int loss_sim_port) {
#else
static int run_radio(const maburgs::Config& cfg) {
#endif
  std::fprintf(stderr, "fec: symbol_size=[%d,%d] seq_horizon=%d\n",
               cfg.fec.symbol_size[0], cfg.fec.symbol_size[1],
               cfg.fec.seq_horizon);

  // Ladder feasibility log: one line per effective (post-max_mcs-filter)
  // rung, so a boot log alone tells you whether the configured ladder can
  // physically carry the video the encoder is about to be told to produce.
  for (size_t i = 0; i < cfg.link.ladder_cfg.ladder.size(); ++i) {
    const maburgs::Rung& rung = cfg.link.ladder_cfg.ladder[i];
    const auto spec = mabur::rc::ladder_from(mabur::rc::PhyMode::HT,
                                              static_cast<uint8_t>(rung.mcs), 20);
    // Same-rate-fixed-pairs (Task 4): both sids run the same PHY rate now,
    // so one `rate` covers the whole rung; only the per-sid overhead
    // (hence per-sid budget) still differs.
    const double rate = mabur::rc::phy_rate_mbps(spec[0]);
    const double denom = 0.5 * (1 + rung.overhead_base) / rate +
                         0.5 * (1 + rung.overhead_enh) / rate;
    const double src_mbps = 0.65 / denom;
    std::fprintf(stderr,
                 "ladder[%zu]: mcs%d ov %.2f/%.2f budgets=%.0f%%/%.0f%% ~%.1f Mbps src\n",
                 i, rung.mcs, rung.overhead_base, rung.overhead_enh,
                 100.0 * rung.overhead_base / (1 + rung.overhead_base),
                 100.0 * rung.overhead_enh / (1 + rung.overhead_enh), src_mbps);
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::signal(SIGUSR1, on_usr1);

  // Card discovery. With no [[radio.cards]] in the config the bus is the
  // source of truth: probe every device the chip itself claims as a
  // supported radio and receive on all of them. The wait exists because
  // maburgs starts during boot, with USB enumeration still in flight --
  // deciding on the first poll is how a two-card GS silently becomes a
  // one-card GS for the whole flight. Fixed for the process lifetime once
  // settled: Aggregator, TxSelector and the sideport all size off it.
  std::vector<maburgs::ScannedCard> scanned;
  if (cfg.radio.auto_scan) {
    libusb_context* scan_ctx = nullptr;
    if (libusb_init(&scan_ctx) != 0) {
      std::fprintf(stderr, "error: libusb_init failed for the card scan\n");
      return 1;
    }
    const maburgs::ScanPolicy policy;
    std::fprintf(stderr, "cards: scanning USB (settle %d ms, timeout %d ms)\n",
                 policy.settle_ms, policy.timeout_ms);
    scanned = maburgs::scan_until_settled(
        policy, [scan_ctx] { return maburgs::enumerate_supported_cards(scan_ctx); },
        [] { return mono_ms(); },
        [](int ms) {
          std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        });
    libusb_exit(scan_ctx);
    if (scanned.empty()) {
      // Nothing to receive on. Exit rather than run blind: S96maburgs
      // respawns at 2 s, which is the retry a late-appearing card needs.
      std::fprintf(stderr, "error: no supported radio found on USB\n");
      return 1;
    }
    for (size_t i = 0; i < scanned.size(); ++i)
      std::fprintf(stderr, "cards: card %zu = %04x:%04x at usb %s\n", i,
                   scanned[i].usb_vid, scanned[i].usb_pid,
                   maburgs::port_name(scanned[i]).c_str());
  }

  const int n_cards = cfg.radio.auto_scan
                          ? static_cast<int>(scanned.size())
                          : static_cast<int>(cfg.radio.cards.size());
  maburgs::BodyQueue queue;  // all cards share one queue; card_id tags origin
  std::vector<std::unique_ptr<maburgs::RadioFrontend>> fronts;
  // The boot scout card (spare card, or the only card) scans the 20 MHz
  // halves at 20 MHz tuning and joins the link at radio.width once the
  // pick freezes (ChannelScout::run -> retune_width). Every other card,
  // and every card when the scan is off, tunes radio.width from the start.
  const int boot_scout_card = n_cards == 1 ? 0 : n_cards - 1;
  for (int i = 0; i < n_cards; ++i) {
    maburgs::RadioFrontend::Cfg fc;
    if (cfg.radio.auto_scan) {
      fc.by_port = true;
      fc.port = scanned[static_cast<size_t>(i)];
      fc.usb_vid = fc.port.usb_vid;
      fc.usb_pid = fc.port.usb_pid;
    } else {
      fc.usb_vid = cfg.radio.cards[static_cast<size_t>(i)].usb_vid;
      fc.usb_pid = cfg.radio.cards[static_cast<size_t>(i)].usb_pid;
      fc.index = cfg.radio.cards[static_cast<size_t>(i)].index;
    }
    fc.channel = cfg.radio.channel;
    fc.width_mhz = (cfg.radio.scan.enable && i == boot_scout_card) ? 20 : cfg.radio.width;
    fc.card_id = static_cast<uint8_t>(i);
    fronts.push_back(std::make_unique<maburgs::RadioFrontend>(fc, queue));
  }

  // A pin that outruns the cards actually found is a missing antenna, not a
  // config error: fall back to auto-select rather than lose the uplink.
  const int tx_card_pin = maburgs::effective_tx_card(cfg.radio.tx_card, n_cards);
  if (tx_card_pin != cfg.radio.tx_card)
    std::fprintf(stderr,
                 "warning: radio.tx_card %d but only %d card(s) found; "
                 "falling back to auto-select\n",
                 cfg.radio.tx_card, n_cards);

  maburgs::Aggregator agg(cfg.uep_layers(),
                          static_cast<uint32_t>(cfg.fec.seq_horizon), n_cards,
                          static_cast<uint32_t>(cfg.link.arrival_guard_syms));

#ifdef MABUR_LOSS_SIM
  // BENCH RIG (MABUR_LOSS_SIM). Off unless --loss-sim was given; rates
  // always start at zero and are set live, so no config file can carry an
  // injection rate across a reboot.
  maburgs::LossControl loss_ctl;
  if (loss_sim_port > 0) {
    // n_cards is what converts the per-card injection rate into the effective
    // (union) rate the decoder actually sees, so the control socket needs it.
    if (loss_ctl.open(loss_sim_port, agg.n_cards())) {
      std::fprintf(stderr,
                   "maburgs: LOSS-SIM control on udp 127.0.0.1:%d "
                   "(all streams zero, ncards=%d; `loss=` is PER-CARD, `eff=` "
                   "is the NOMINAL union rate = percard^ncards -- nominal "
                   "because it assumes every card heard every body, so record "
                   "real loss from the stats sideport, not from the dial)\n",
                   loss_sim_port, agg.n_cards());
    } else {
      std::fprintf(stderr, "warning: loss-sim port %d unusable; disabled\n",
                   loss_sim_port);
    }
  }
#endif
  // TX-power wall calibration (2026-09-10-tx-power-calibration): the
  // loopback command listener. Unlike loss_ctl above, this is compiled
  // into every prod build -- calibration is a real operator workflow, not
  // a bench-only scaffold -- so there is no MABUR_LOSS_SIM-style guard
  // (cal_control.h). Port 8400 is pinned by the design doc: the 830x
  // block belongs to the stats sideport, its UDP sinks and the OSD feed.
  // CalSession itself is constructed further down, once cal_log exists --
  // it needs a CalLog* to write into.
  maburgs::CalControl cal_ctl;
  if (!cal_ctl.open(8400))
    std::fprintf(stderr,
                 "warning: calibration control port 8400 unusable; "
                 "`maburcal start` will not reach this daemon\n");
  // Telem (the drone's only calibration ack signal, flags bit6 cal_active)
  // carries no nonce of its own, so this is what the T_TELEM handler below
  // hands back to CalSession::on_ack() -- stashed from the CalCmd the last
  // due_cmd() call actually sent.
  uint32_t cal_pending_nonce = 0;
  // cal.log's R line is written once per SESSION (cal_log.h), but the ack
  // above fires once per PHASE within a session (coarse, then fine) --
  // this dedups by nonce, which is constant across a session and freshly
  // randomized per `maburcal start` (cal_control.h).
  std::optional<uint32_t> cal_log_run_nonce;
  // Debug-log session (2026-09-06 consolidation): one directory for
  // ctl.log/probe.log/au.log/flight.jsonl, one writer thread feeding all of
  // them. Declared here (ahead of the stats block and the FrameStream
  // construction below) so `log_writer` outlives every log object that binds
  // to it.
  maburgs::DebugSession debug(cfg.debug_log.dir, cfg.debug_log.enable);
  std::optional<maburgs::LogWriter> log_writer;
  if (debug.ok()) {
    log_writer.emplace();
    std::fprintf(stderr, "debug-log: session %s%s\n", debug.dir().c_str(),
                 debug.rejoined() ? " (rejoined)" : "");
  }
  // cal.log (callog 1): TX-power calibration's raw per-cell record (spec
  // 2026-09-10-tx-power-calibration-design.md). Unlike ctl.log/probe.log/
  // au.log below, this uses its own private LogWriter (cal_log.h) rather
  // than the shared `log_writer` -- a calibration run is rare and
  // short-lived, so there is nothing to gain from the fixed-slot session
  // writer. header() writes the file's format marker at most once per
  // FILE.
  //
  // Deliberately NOT gated on debug.ok()/debug_log.enable, unlike every
  // log above -- that flag governs CONTINUOUS per-second flight logging,
  // where the risk is disk volume and flash wear across every second of
  // every flight. A calibration run is a different risk profile: a
  // bounded (~380-line) trace, once per unit, on a deliberate operator
  // action, and the sole record of a measurement written straight into
  // the drone's config. Task 13 deleted bench/txagcbench's Python
  // analyzer; cal.log + `maburcal report` is the ENTIRE replacement for
  // examining a run's data after the fact -- shipping with debug_log off
  // (as gs/bundle/maburgs.default.toml does since 671c848) must not mean
  // every calibration runs silently and leaves no trace. When debug
  // logging is on, reuse its session directory (one place to look, and
  // one session's worth of context around a run); when it's off, fall back
  // to <debug_log.dir>/cal -- `dir` is present in config independent of
  // `enable`. Either way the once-per-FILE marker decision is the file's
  // own presence (cal_log_header_due), never the session's rejoin state.
  std::string cal_log_dir = debug.dir();
  if (!debug.ok()) {
    cal_log_dir = cfg.debug_log.dir + "/cal";
    // Best-effort, mirroring DebugSession::allocate_(): debug_log.dir may
    // legitimately not exist yet on a unit that has never turned on
    // flight logging. mkdir the parent, then the leaf; EEXIST on either
    // is the expected steady-state, not a failure.
    ::mkdir(cfg.debug_log.dir.c_str(), 0755);
    ::mkdir(cal_log_dir.c_str(), 0755);
  }
  // Ask the FILE, in both branches, and ask before constructing the CalLog
  // that would create it (cal_log.h). A rejoined session directory is not
  // evidence that a cal.log exists in it -- that inference shipped, and the
  // first calibration after any maburgs restart lost its `callog 1` marker
  // and with it every operator-facing rendering of the run.
  const bool cal_log_new_file = maburgs::cal_log_prepare(cal_log_dir);
  std::optional<maburgs::CalLog> cal_log;
  cal_log.emplace(cal_log_dir);
  if (cal_log->ok()) {
    if (cal_log_new_file) cal_log->header();
  } else {
    // Non-fatal by cal_log.h's own contract: the walls still get measured
    // and applied with no trace, which is bad but not as bad as refusing
    // to calibrate over a logging directory problem.
    std::fprintf(stderr,
                 "warning: calibration log directory %s unusable; a "
                 "calibration will still run and apply, but its cal.log "
                 "trace will be lost\n",
                 cal_log_dir.c_str());
  }
  // The session brain. cal_log always exists by now (emplace() above is
  // unconditional); CalLog::ok() being false just makes every record
  // method a silent no-op (cal_log.h's own contract), so handing over
  // &*cal_log unconditionally is safe even when the directory resolution
  // above failed.
  maburgs::CalSession cal_session(maburgs::CalSessionCfg{}, &*cal_log);
  // Stats sideport (spec: docs/superpowers/specs/2026-07-25-gs-stats-sideport-design.md)
  // and flight.jsonl (debug-log consolidation, 2026-09-06) share one
  // StatsExporter snapshot, so the snapshot builder runs whenever either
  // wants it: `cfg.stats.enable` gates the UDP sinks ONLY (:8302 feeds
  // maburplay's GS OSD, so debug logging must never blank it), `debug.ok()`
  // gates the file. Declared here (ahead of the FrameStream construction
  // below) so the FrameStream end_frame lambda can capture `stats` by
  // reference; both must also outlive every lambda that captures them.
  // session id: nonzero random u32 so consumers detect restarts.
  // UdpSink has a user-declared destructor and a deleted copy constructor
  // with no declared move constructor, so it has neither -- std::vector
  // growth (even reserve() on an empty vector) instantiates a copy/move
  // path unconditionally and fails to compile against it. unique_ptr sidesteps
  // that: the vector moves pointers, never UdpSink objects.
  std::vector<std::unique_ptr<maburgs::UdpSink>> stats_udp;
  std::optional<maburgs::StatsExporter> stats;
  maburgs::LogWriter::Stream flight_jsonl = maburgs::LogWriter::kBadStream;
  if (debug.ok())
    flight_jsonl = log_writer->open(debug.dir(), "flight.jsonl", "",
                                    /*mark_drops=*/false);
  if (cfg.stats.enable || debug.ok()) {
    std::function<bool(const std::string&)> to_udp;
    if (cfg.stats.enable) {
      stats_udp.reserve(cfg.stats.out.size());
      for (const auto& o : cfg.stats.out)
        stats_udp.push_back(std::make_unique<maburgs::UdpSink>(o.host, o.port));
      to_udp = [&stats_udp](const std::string& s) {
        // Every destination gets the same buffer. One failing sink must not
        // stop the others -- a dead consumer is not a reason to blind the
        // live ones.
        bool any = false;
        for (auto& u : stats_udp)
          if (u->send(reinterpret_cast<const uint8_t*>(s.data()), s.size()))
            any = true;
        return any;
      };
    }
    std::function<void(const std::string&)> to_file;
    if (flight_jsonl != maburgs::LogWriter::kBadStream)
      to_file = [&log_writer, flight_jsonl](const std::string& s) {
        log_writer->line(flight_jsonl, s.data(), s.size());
      };
    uint32_t session = 0;
    std::random_device rd;
    while (session == 0) session = rd();
    stats.emplace(
        session, cfg.stats.interval_ms,
        maburgs::make_stats_sink(std::move(to_udp), std::move(to_file)));
    if (cfg.stats.enable)
      for (const auto& o : cfg.stats.out)
        std::fprintf(stderr, "maburgs: stats sideport -> udp %s:%d every %d ms\n",
                     o.host.c_str(), o.port, cfg.stats.interval_ms);
    if (flight_jsonl != maburgs::LogWriter::kBadStream)
      std::fprintf(stderr, "debug-log: flight jsonl -> %s every %d ms\n",
                   log_writer->path(flight_jsonl).c_str(), cfg.stats.interval_ms);
  }

  // Drone telemetry (T_TELEM): display-only, not rendezvous traffic — held
  // here for the DRONE display region rather than forwarded to the vrx
  // controller. Core-thread-owned, like everything else in this loop.
  // rx_ms is the GS-side mono stamp the aggregator carries on every rc frame.
  // Spec 2026-07-26 drone-telemetry.
  struct { std::optional<mabur::rc::Telem> t; uint64_t rx_ms = 0; } latest_telem;

  // Auto channel selection (spec 2026-09-13-auto-channel-select). The plan
  // owns where the link lives; the scout (only while radio.scan.enable and
  // until the first ack) measures candidates on the spare card, or on the
  // only card between home windows.
  const maburgs::ScanCfg& scfg = cfg.radio.scan;
  maburgs::ChannelPlan plan(maburgs::ChannelPlanCfg{
      cfg.radio.channel, n_cards, scfg.split_after_ms, scfg.home_window_ms, 20});
  // Live channel per card, as far as this thread knows: InitWrite tunes to
  // home, retune() moves it, and the scout owns its card's entry until the
  // scout thread is joined.
  std::vector<uint8_t> cur_ch(static_cast<size_t>(n_cards), cfg.radio.channel);
  const bool one_card = n_cards == 1;
  const int scout_card = boot_scout_card;
  std::unique_ptr<maburgs::ChannelScout> scout;
  std::thread scout_thread;
  bool scout_joined = true;  // no thread running
  if (scfg.enable) {
    maburgs::ScoutCfg sc;
    sc.home = cfg.radio.channel;
    sc.candidates = scfg.candidates;
    sc.dwell_ms = scfg.dwell_ms;
    sc.settle_ms = scfg.settle_ms;
    sc.min_rounds = scfg.min_rounds;
    sc.home_window_ms = scfg.home_window_ms;
    sc.beacon_period_ms = 20;
    sc.home_margin = static_cast<uint32_t>(scfg.home_margin);
    sc.one_card = one_card;
    sc.link_width_mhz = cfg.radio.width;
    scout = std::make_unique<maburgs::ChannelScout>(
        sc, *fronts[static_cast<size_t>(scout_card)],
        [] { return static_cast<int64_t>(mono_ms()); },
        [](int ms) {
          if (ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        });
  }
  // Freeze + join the scout thread (it exits at the end of the current
  // dwell, after parking its card on `target`). Idempotent.
  auto join_scout = [&](uint8_t target) {
    if (scout_joined) return;
    scout->freeze(target);
    scout_thread.join();
    scout_joined = true;
  };

  // In-flight channel hop (spec 2026-09-14-inflight-channel-hop): verdict
  // engine, candidate ranker, hop state machine, and the spare-card scout
  // that runs for the rest of the flight (unlike `scout` above, which is
  // boot-only). HopRanker is built over scfg.candidates PLUS home (its own
  // constructor appends home when it is not already a candidate, hop_ranker.cpp)
  // -- a strict superset of the in-flight scout's own candidate list just
  // below, so HopRanker::add() (which silently drops a visit for any
  // channel outside its own list, Task 5's carried finding) can never
  // discard a scout visit: every channel the scout can ever dwell on is
  // already ranked.
  const maburgs::HopCfg& hcfg = cfg.hop;
  maburgs::HopVerdict verdict(hcfg, n_cards);
  // boot_pick 0 = "no boot-time pick yet" (no channel is ever 0), so the
  // ranked tiebreak falls through to home until the boot scan actually
  // resolves; ranker.set_boot_pick() below publishes the real one at the
  // first DiscAck. Passing home as BOTH arguments (as this did) collapsed
  // spec section 3's "ties -> boot-time pick, then home" into "ties ->
  // home" and made the tiebreak's first term dead code.
  //
  // At 40 MHz the boot scan visits every HALF (pair_pick.h); the ranker's
  // candidate set must hold them all or add() drops those visits. In
  // session the scout only ever dwells on primaries.
  maburgs::HopRanker ranker(hcfg,
                            cfg.radio.width == 40
                                ? maburgs::scan_half_set(cfg.radio.channel, scfg.candidates)
                                : scfg.candidates,
                            cfg.radio.channel, 0);
  maburgs::HopController hopc(hcfg, cfg.radio.channel);
  // Constructed against card 0 as a placeholder radio -- harmless, since the
  // scout thread below always repoints this via set_radio() (Task 10)
  // before every dwell/burst and never touches it beforehand.
  maburgs::InflightScout inflight(
      maburgs::InflightScoutCfg{hcfg.dwell_observe_ms, hcfg.dwell_period_ms, scfg.candidates,
                                cfg.radio.width},
      *fronts[static_cast<size_t>(scout_card)],
      [] { return static_cast<int64_t>(mono_us()); },
      [](int ms) {
        if (ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(ms));
      });

  // Cross-thread state for the in-flight scout thread (spec section 3/6).
  // Everything the scout thread and the core loop both touch is either an
  // atomic here or guarded by dwell_mu (the two drained vectors) /
  // inflight_mu (every call into `inflight`, which drives a RadioFrontend
  // the core loop's own mechanical retune loop and TX selector must not
  // race). dwell_busy/dwell_card name the card (if any) the scout thread
  // currently has off on a candidate; every core-loop site that can call
  // fe.retune()/sel.update() on that card must hold off until it clears.
  std::atomic<bool> dwell_busy{false};
  std::atomic<bool> scout_run{false};
  std::atomic<bool> in_session_atomic{false};
  std::atomic<bool> hopping_atomic{false};
  std::atomic<int> tx_card_now{tx_card_pin < 0 ? 0 : tx_card_pin};
  std::atomic<int> dwell_card{-1};
  // Bumped once per completed AU (end-of-AU FrameStream callback, below):
  // the scout thread's alignment wait, so a dwell starts on an AU boundary
  // rather than mid-burst.
  std::atomic<uint64_t> au_seq{0};
  std::mutex dwell_mu;
  std::vector<std::pair<int, maburgs::ScoutDwell>> dwell_recs;  // {card, record}
  std::vector<maburgs::HopVisit> dwell_visits;
  // Sideport dwell snapshot per card (Task 12). NOT guarded by dwell_mu --
  // safe anyway because both the write (at the dwell_recs/dwell_visits
  // drain below, AFTER that drain's lock_guard scope has already closed
  // and swapped the shared vectors into purely local ones) and the read
  // (at the sideport feed) run on the core thread only, same as the
  // pre-existing cur_ch update in that same drain loop -- there is no
  // second thread that ever touches dwell_stats for a lock to arbitrate.
  // nullopt = this card has never completed a dwell. visits is cumulative
  // over every drained dwell (success or not); score is the last
  // SUCCESSFUL dwell's HopRanker::score() (a failed retune produces no
  // HopVisit, so a stale score is kept rather than zeroed); cost_us is
  // always the last dwell's to_us+read_us+back_us, success or not.
  std::vector<std::optional<maburgs::StatsDwellIn>> dwell_stats(
      static_cast<size_t>(n_cards));
  // Serializes every call into `inflight` (and, transitively, whichever
  // RadioFrontend it currently points at) across the scout thread's
  // periodic dwell and the core thread's own synchronous freshness burst
  // (Step 4, just before a hop order) -- the brief's sketch has the core
  // thread call inflight.burst() directly with no such guard, which would
  // let the two threads drive the same InflightScout/RadioFrontend at
  // once; this mutex is the fix.
  std::mutex inflight_mu;
  bool inflight_started = false;
  std::thread scout_thread2;
  auto scout_loop = [&] {
    while (scout_run.load() && !g_stop.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(hcfg.dwell_period_ms));
      if (!in_session_atomic.load() || hopping_atomic.load() || n_cards < 2) continue;
      const int card = tx_card_now.load() == 0 ? 1 : 0;
      auto& fe = *fronts[static_cast<size_t>(card)];
      if (!fe.ready()) continue;
      const uint64_t s0 = au_seq.load();  // align to the next AU boundary (<= 17 ms wait)
      for (int i = 0; i < 20 && au_seq.load() == s0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      std::lock_guard<std::mutex> ilk(inflight_mu);
      inflight.set_radio(fe);
      dwell_card.store(card);
      dwell_busy.store(true);
      maburgs::ScoutDwell d;
      maburgs::HopVisit v;
      const bool ok = inflight.dwell(inflight.next_candidate(), fe.channel(), d, v);
      dwell_busy.store(false);
      dwell_card.store(-1);
      std::lock_guard<std::mutex> lk(dwell_mu);
      dwell_recs.emplace_back(card, d);
      if (ok) dwell_visits.push_back(v);
    }
  };

  // Verdict-window bookkeeping (spec section 2): trailing per-card frame
  // snapshots so every window reads a DELTA, not a cumulative count -- the
  // same pattern crc_fail/foreign deltas use elsewhere in this loop.
  std::vector<maburgs::ScoutFrames> window_prev(static_cast<size_t>(n_cards));
  std::vector<uint64_t> window_prev_crc(static_cast<size_t>(n_cards), 0);
  std::vector<bool> window_prev_ok(static_cast<size_t>(n_cards), false);
  uint64_t last_window_ms = 0;
  uint64_t recovered_prev_window = 0;
  bool hop_was_active = false;   // hop_active() edge tracker (hop_burst_gate.h)
  maburgs::Verdict last_verdict = maburgs::Verdict::Healthy;
  maburgs::VerdictOut last_verdict_out;
  // hop.last_ms (Task 12): elapsed_ms of the most recent HopEvent
  // (HopController::take_events(), drained below) -- nullopt until any
  // hop event (order/confirm/withdraw/hold) has fired this session.
  std::optional<uint64_t> last_hop_event_ms;

  // RCF send counter: the hop controller's one-card-retune escalation
  // reads the delta since the order. Bumped in send_control_frame below
  // wherever stamp_rtt is set, i.e. exactly the periodic op-point RCF --
  // never a DISC beacon or a calibration frame, neither of which carries
  // hop_ch/hop_epoch.
  uint64_t rcf_sent_total = 0, rcf_sent_at_order = 0;
  // Channel the most recent video body arrived on (batch-drain loop,
  // below): confirms a hop landed by comparing against plan.hop_target().
  uint8_t last_video_ch = cfg.radio.channel;
  // Freshness-burst rate limiter (fix round 3): the burst's own gate
  // (Idle/Hold + trigger) has nothing else pacing it -- Hold re-enters on
  // every tick with the trigger latched true, and neither cooldown_ms
  // (guards only a POST-hop re-trigger, stamped at verify_pass) nor
  // max_hops_per_min (counted only inside order()) apply to a burst that
  // never results in an order. Reusing dwell_period_ms caps a sustained
  // Hold to the same off-air duty cycle the periodic scout thread already
  // runs at (one ~30 ms burst per ~333 ms, ~9%), rather than back-to-back
  // (~100%, and the process's own core-thread RX drain along with it).
  double last_burst_ms = -1e18;

  // Control-path RTT + pts-offset estimator (link-rtt, 2026-09-02). Fed
  // from the same core thread as latest_telem: RCF send stamps below,
  // telem echoes in the rc sink.
  maburgs::RttEstimator rtt_est;
  // RCF slotting (gs-uplink-self-blanking findings 2026-09-02): control
  // frames wait for the end-of-AU callback (FrameStream sink below) so the
  // send lands in the drone's inter-AU idle. See rcf_slot.h.
  maburgs::RcfSlotter rcf_slot(
      maburgs::RcfSlotCfg{cfg.link.rcf_slot_hold_ms, 100, 2, 3, 1});
  // One card + a running scout: sends that leave while the card is off
  // measuring a candidate would race the scout thread's FastRetune /
  // GetRxEnergy on the same device. The DISC routing ladder checks
  // at_home() when the frame is OFFERED, but the slotter releases it
  // later with no re-check, and RCFs (which exist from the first ack
  // until the join) never consulted it at all. This is the one gate that
  // covers both, at the single site every control frame passes through.
  // Two-card mode trips it while the scout has a silent dwell in progress
  // (quiet()): the beaconing card's TX leaks into the adjacent scout on
  // every channel and dominated the readings otherwise.
  uint64_t scout_gated_sends = 0;
  // The one place a control frame leaves the GS (direct or via the RCF
  // slotter): card + RTT stamp travel with the frame (SlotFrame).
  auto send_control_frame = [&](const maburgs::SlotFrame& f) {
    if (scout && !scout_joined &&
        ((one_card && !scout->at_home()) || scout->quiet())) {
      ++scout_gated_sends;
      return;
    }
    static const bool gaplog = std::getenv("MABUR_GAPLOG") != nullptr;
    if (gaplog) {
      const uint64_t now_ms = mono_ms();
      std::fprintf(stderr,
                   "gstx card=%d mono=%llu reason=%d hold=%llu since_au=%llu\n",
                   f.card, static_cast<unsigned long long>(mono_us()),
                   static_cast<int>(f.reason),
                   static_cast<unsigned long long>(now_ms - f.offered_ms),
                   static_cast<unsigned long long>(now_ms - rcf_slot.last_au_ms()));
    }
    fronts[static_cast<size_t>(f.card)]->send_control(f.frame);
    // stamp_rtt is set only for the periodic op-point RCF (never a DISC
    // beacon or a calibration frame) -- exactly the frame that carries
    // hop_ch/hop_epoch, so this is the count HopController's one-card-
    // retune escalation reads the delta of.
    if (f.stamp_rtt) {
      rtt_est.on_rcf_sent(f.seq, mono_us());
      ++rcf_sent_total;
    }
  };

  maburgs::AuRingWriter au_ring;
  maburgs::AuDoorbell au_bell;
  bool au_on = false;
  if (cfg.au_ring.enable) {
    const maburgs::AuRingGeom geom{
        static_cast<uint32_t>(cfg.au_ring.slot_kb) * 1024u,
        static_cast<uint32_t>(cfg.au_ring.slot_count)};
    au_on = au_ring.open(cfg.au_ring.path, geom);
    if (au_on && !au_bell.open(cfg.au_ring.socket, au_ring.geom()))
      std::fprintf(stderr, "warning: au_ring doorbell %s unusable\n",
                   cfg.au_ring.socket.c_str());
    if (!au_on)
      std::fprintf(stderr, "warning: au_ring %s unusable; disabled\n",
                   cfg.au_ring.path.c_str());
  } else {
    // PR C: the ring IS the video output. A disabled ring means every
    // reassembled frame is decoded and thrown away -- legal for FEC-only
    // bench work, but never silently.
    std::fprintf(stderr,
                 "warning: au_ring disabled -- NO video output (frames are "
                 "reassembled and discarded)\n");
  }

  // Head-segment latency aggregates (sideport link.video.lat, spec
  // 2026-08-30-latency-accounting Task 10): pts->mono anchor + rolling
  // percentile window, both core-loop-owned like everything else here.
  // cur_au_pts/cur_au_sid are set in begin_frame and read back in end_frame
  // -- legal because FrameStream's contract pairs begin/end for the SAME AU
  // with no other AU's begin in between (single-threaded core loop).
  maburgs::PtsAnchor lat_anchor;
  maburgs::LatWindow lat_win;
  uint32_t cur_au_pts = 0;
  uint8_t cur_au_sid = 0;

  // Probe stream (spec 2026-09-04 section 3): scored by ProbeTrack against
  // the enh AU count; the ENH layer's geometry gives bpb/block_payload, so
  // the same array the Aggregator was built from decides how a probe body
  // is parsed. Core-thread-owned like every other window in this loop.
  const auto probe_layer = cfg.uep_layers()[1];  // ENH layer geometry
  const int probe_bpb = probe_layer.blocks_per_body;
  const int probe_block_payload =
      static_cast<int>(mabur::sw::kSwHeaderLen) + probe_layer.fec.symbol_size;
  maburgs::ProbeTrack probe_track(maburgs::ProbeTrackCfg{probe_bpb, 100, n_cards});
  maburgs::S1LossWindow probe_loss;  // union, commanded profile
  std::vector<maburgs::S1LossWindow> probe_card_loss(static_cast<size_t>(n_cards));
  uint8_t probe_cmd_last = mabur::rc::kNoProbeProfile;
  // A commanded-profile change blanks the windows: bodies already in flight
  // carry the OLD profile and ProbeTrack stops scoring them, so the window
  // must refill from the new profile's bodies only (RCF lag + finalize).
  constexpr double kProbeSwitchBlankMs = 150.0;
  // Per-body probe log, alongside ctl.log and au.log in the same session
  // directory (DebugSession). Declared here, before FrameStream/au_log
  // below, and emplaced later once debug_log.enable is known.
  std::optional<maburgs::ProbeLog> probe_log;
  // Per-episode FEC loss log (fec.log), same directory and lifetime.
  std::optional<maburgs::FecLog> fec_log;
  // Per-AU meta log; forward-declared here so the FrameStream callbacks
  // just below can reference it by [&] capture, even though it is only
  // emplaced once debug.ok() is known (ctl/probe/au construction, below,
  // next to ctl_log).
  std::optional<maburgs::AuLog> au_log;

  // Video tail: FrameStream reassembles whole frames from the raw FRAG
  // fragments the decoder emits; whole access units leave maburgs through
  // the shm AU ring (PR C: the RTP packetizer/UDP path is gone — maburplay
  // is the consumer, ausniff the external gate).
  maburgs::FrameStream fstream(
      {static_cast<uint64_t>(cfg.video.frame_gap_timeout_ms),
       cfg.video.frame_lookahead},
      {[&](const mabur::framewire::FrameHdr& h, uint8_t sid) {
         cur_au_pts = h.pts_us;
         cur_au_sid = sid;
         if (au_on) au_ring.begin(h, sid);
         if (au_log) au_log->begin();
         rcf_slot.on_au_first(mono_ms());
         // One probe expectation per video access unit, base and enh alike
         // (probe per AU, 2026-09-16): the probe body rides the AU's send
         // opportunity, so the AU count is what "expected" means
         // (probe_track.h explains why seq gaps cannot be).
         probe_track.on_au(sid, h.frame_id, static_cast<double>(mono_ms()));
       },
       [&](const uint8_t* d, size_t n) {
         if (au_on) au_ring.append(d, n);
         if (au_log) au_log->payload(d, n);
       },
       [&](bool c, const maburgs::AuLatMeta& lat) {
         // In-flight scout thread's AU-boundary alignment wait (spec
         // 2026-09-14-inflight-channel-hop section 3): every AU end, clean
         // or truncated, counts -- the scout only needs a boundary to start
         // its dwell on, not a specific outcome.
         au_seq.fetch_add(1, std::memory_order_relaxed);
         if (au_on) {
           const uint64_t rec = au_ring.finish(c, lat);
           if (rec != UINT64_MAX) au_bell.notify(rec);
           if (rec != UINT64_MAX && au_log)
             au_log->row(mono_us(), au_ring.last_record());
         }
         // Every video AU is trailed by a probe while one is commanded
         // (probe per AU, 2026-09-16), so a base completion releases
         // nothing by itself either: the release is the probe's arrival,
         // or the learned tail deadline if it is lost (rcf_slot.h).
         rcf_slot.on_au_complete(
             mono_ms(),
             cur_au_sid < mabur::UepEncoder::kNumStreams &&
                 probe_cmd_last != mabur::rc::kNoProbeProfile);
         {
           static const bool gaplog_au = std::getenv("MABUR_GAPLOG") != nullptr;
           if (gaplog_au)
             std::fprintf(stderr, "auc mono=%llu complete=%d\n",
                          static_cast<unsigned long long>(mono_us()), c ? 1 : 0);
         }
         if (stats) stats->on_frame(mono_ms());
         // au_tail gauge (usb-feed probe 2026-09-01): fec = arrival span
         // (last body mono - first body mono) + publish tail (now - last
         // body: repair/decode/assembly/ring write). Core-thread-owned,
         // 5 s stderr windows. Names where the fec residual lives when the
         // air span is known from rx_pace.
         if (lat.t_first_us && lat.t_last_arr_us >= lat.t_first_us) {
           static uint64_t at_n = 0, at_span_sum = 0, at_span_max = 0;
           static uint64_t at_tail_sum = 0, at_tail_max = 0, at_last_rep = 0;
           const uint64_t now = mono_us();
           const uint64_t span = lat.t_last_arr_us - lat.t_first_us;
           const uint64_t tail =
               now > lat.t_last_arr_us ? now - lat.t_last_arr_us : 0;
           ++at_n;
           at_span_sum += span;
           at_tail_sum += tail;
           if (span > at_span_max) at_span_max = span;
           if (tail > at_tail_max) at_tail_max = tail;
           if (at_last_rep == 0) at_last_rep = now;
           if (now - at_last_rep >= 5000000 && at_n > 0) {
             std::fprintf(stderr,
                          "maburgs au_tail: n=%llu span_us mean=%llu max=%llu "
                          "tail_us mean=%llu max=%llu\n",
                          (unsigned long long)at_n,
                          (unsigned long long)(at_span_sum / at_n),
                          (unsigned long long)at_span_max,
                          (unsigned long long)(at_tail_sum / at_n),
                          (unsigned long long)at_tail_max);
             at_n = at_span_sum = at_span_max = 0;
             at_tail_sum = at_tail_max = 0;
             at_last_rep = now;
           }
         }
         // Head-segment latency: t_first_us is the radio's mono stamp on
         // the AU's first body and t_complete_us (via mono_us() below,
         // the "now" at this closure) shares its timebase -- steady_clock
         // == CLOCK_MONOTONIC on this glibc/Linux target, so the two are
         // directly subtractable with no clock-domain conversion.
         if (lat.t_first_us) {
           // Enc-excess air fix (2026-08-31), mirrored in the player's
           // LatTracker::on_submit: the anchor consumes the encode/queue-
           // corrected arrival so its floor means "best post-encoder,
           // post-queue transit"; enc/dq report their full wire values and
           // air is the transit excess. enc + dq + air == t_first - map(pts)
           // exactly (additive invariant unchanged). Corrections come only
           // from FCS-clean bodies and are plausibility-capped; an
           // implausible one withholds the sample rather than dragging the
           // snap-down floor.
           const int64_t adjust = static_cast<int64_t>(lat.enc_us) +
                                  static_cast<int64_t>(lat.drone_q_ms) * 1000;
           if (adjust <= maburgs::PtsAnchor::kMaxAnchorAdjustUs &&
               static_cast<int64_t>(lat.t_first_us) > adjust) {
             const uint64_t adj_arrival =
                 lat.t_first_us - static_cast<uint64_t>(adjust);
             const auto obs = lat_anchor.observe(cur_au_pts, adj_arrival);
             if (!obs.discont && lat_anchor.usable()) {
               const int64_t excess =
                   static_cast<int64_t>(adj_arrival) -
                   static_cast<int64_t>(lat_anchor.map_us(obs.pts64));
               const uint64_t t_done = mono_us();
               const uint32_t fec = static_cast<uint32_t>(
                   t_done > lat.t_first_us ? t_done - lat.t_first_us : 0);
               lat_win.add(lat.enc_us,
                           static_cast<uint32_t>(lat.drone_q_ms) * 1000,
                           static_cast<uint32_t>(excess > 0 ? excess : 0), fec);
             }
           }
         }
       }});
  // Only fragments from a peer that advertised the frame wire format may reach
  // FrameStream: an older drone's bodies carry a mutually unparseable frag
  // header, and feeding them here would produce garbage video rather than an
  // obvious failure. Core-thread-owned, like everything else in this loop.
  bool frame_wire = false;
  bool refused_peer = false;  // one loud line per run, not per tick

  agg.set_frag_sink([&](const mabur::DecodedFrag& f) {
    if (frame_wire)
      fstream.push_fragment(f.stream_id, f.frag.data(), f.frag.size(), mono_ms(),
                            {f.body_mono_us, f.q_ms, f.enc_us, f.air_ms});
  });

  maburgs::VrxCfg vcfg;
  vcfg.vtx_id = cfg.link.vtx_id;
  vcfg.op_channel = cfg.radio.channel;
  vcfg.feedback_ms = cfg.link.feedback_ms;
  vcfg.beacon_keepalive_ms = cfg.link.beacon_keepalive_ms;
  vcfg.ladder = cfg.link.ladder_cfg;
  vcfg.pin_mcs = cfg.link.static_mcs;
  vcfg.pin_bw = cfg.link.static_bw;
  vcfg.pin_overhead_base = cfg.link.static_overhead_base;
  vcfg.pin_overhead_enh = cfg.link.static_overhead_enh;
  vcfg.probe_pin_mcs = cfg.link.ladder_cfg.probe.pin_mcs;
  maburgs::VrxController vrx(vcfg);

  // Dedicated adaptive-link log (spec 2026-08-05-s3-probe-promote-design.md
  // section 5): maburgs' own compact S/E/P/N record of every rung decision,
  // independent of the stats sideport so the learning dataset survives a
  // dead/absent consumer (2026-08-04: statsrec wasn't running and the
  // flight jsonl froze hours before the session).
  //
  // Opened whenever debug_log.enable is set, INCLUDING static-pin mode
  // (static_mcs >= 0). It used to be skipped there -- a pinned link never
  // ticks the adaptive controller, so there were no rung decisions to
  // record -- but the probe stream changed that (spec 2026-09-04 section
  // 8.3 steps 1-2): the pinned bench runs are exactly the ones whose
  // per-body probe.log matters, and it lives alongside ctl.log in the same
  // session directory so the two files from one boot always pair up.
  //
  // What a pinned S line actually contains: the controller is never
  // updated, so u/util read 0 and E/P/N/R records never fire at all. The
  // three probe columns are `<rung> nan <probe_n>`, and only the last is a
  // measurement. `rung` is probe_rung() off a frozen idx_ == 0 -- pinning
  // does not disable link.probe.enable -- so it prints
  // min(probe.rung_offset, top), i.e. 1 on a normal multi-rung ladder, NOT
  // -1. `u` is nan because the gate never leaves Off. `probe_n` is the real
  // count of expected blocks in the window: with link.probe.pin_mcs >= 0
  // the RCF carries a probe profile, so ProbeTrack books bpb per enh AU and
  // this is nonzero -- exactly the number the pinned bench runs want. It is
  // 0 only when pin_mcs < 0, where nothing commands a probe profile.
  // au_log (AuLog) is forward-declared above, next to probe_log, so the
  // FrameStream callbacks can capture it by reference; only ctl_log is new
  // here.
  std::optional<maburgs::CtlLog> ctl_log;
  if (debug.ok()) {
    std::string header = "ladder=";
    for (size_t i = 0; i < cfg.link.ladder_cfg.ladder.size(); ++i) {
      const maburgs::Rung& r = cfg.link.ladder_cfg.ladder[i];
      if (i) header += ",";
      header += std::to_string(r.mcs) + "/" +
                std::to_string(static_cast<int>(std::lround(r.overhead_base * 100))) +
                ":" +
                std::to_string(static_cast<int>(std::lround(r.overhead_enh * 100)));
    }
    char tail[96];
    std::snprintf(tail, sizeof(tail), " down_util=%.2f up_util=%.2f probe_offset=%d",
                  cfg.link.ladder_cfg.down_util, cfg.link.ladder_cfg.up_util,
                  cfg.link.ladder_cfg.probe.rung_offset);
    header += tail;
    ctl_log.emplace(*log_writer, debug.dir(), header);
    probe_log.emplace(*log_writer, debug.dir(), probe_bpb);
    fec_log.emplace(*log_writer, debug.dir());
    // Gated on au_on too (not just debug.ok()): with au_ring.enable=false
    // there are never any rows to write, and an emplace here would leave
    // au.log containing only its "# aulog 4" header -- reads as "the
    // logger is broken" rather than "the ring is off".
    if (au_on) au_log.emplace(*log_writer, debug.dir());
  }

  // scan.log (scanlog 2): the channel-selection record -- card caps, scout
  // dwells (boot-time AND in-flight), the pick, every link move, and the
  // in-flight hop verdict/hop-event lines (spec 2026-09-13-auto-channel-
  // select section 7; V/H lines added by 2026-09-14-inflight-channel-hop).
  // Same session directory and writer as ctl.log, same debug_log.enable gate.
  std::optional<maburgs::ScanLog> scan_log;
  if (debug.ok()) {
    std::string h = "home=" + std::to_string(cfg.radio.channel) + " candidates=";
    for (size_t i = 0; i < scfg.candidates.size(); ++i)
      h += (i ? "," : "") + std::to_string(scfg.candidates[i]);
    h += " dwell_ms=" + std::to_string(scfg.dwell_ms) +
         " min_rounds=" + std::to_string(scfg.min_rounds) +
         " enable=" + std::string(scfg.enable ? "1" : "0") +
         " cards=" + std::to_string(n_cards);
    scan_log.emplace(*log_writer, debug.dir(), h);
  }

  // Measured-loss ladder feedback: stream 1 (base layer)'s cumulative
  // (expected, arrived) symbol totals, pre-FEC-repair. expected = source
  // symbols ever seen by seq framing (delivered directly + recovered by FEC
  // + abandoned as unrecoverable); arrived = delivered directly PLUS
  // recovered symbols whose direct copy landed afterwards
  // (syms_recovered_arrived) — a repair that merely wins an arrival race is
  // not channel loss. Without that term, rung 0's 2x parity read a clean
  // bench as 19-26% pre-FEC loss and pinned u above up_util forever (stuck
  // at mcs0, 2026-07-27). This is the PRE-FEC window; the post-FEC residual
  // below is a different question over the same symbol counters (what FEC
  // could not repair at all) -- see gs/src/ladder_residual.cpp.
  maburgs::S1LossWindow s1_loss;
  // Sideport/OSD gauge (2026-09-23): BOTH video layers' current-only
  // arrival-tracker counts pooled in one window, so the LOSS row reads the
  // whole downlink; the ladder keeps deciding on s1_loss_cur (base only).
  maburgs::S1LossWindow pre_loss_all;
  // s3 probe-before-promote feedback (same windowing machinery as s1_loss,
  // stream 3): pre-FEC loss for probe/s3-demote decisions. The steady-state
  // demote path's residual (abandoned/expected) window is s3_resid_cur
  // below -- attribution is unconditional, so there is no total-based
  // sibling left to keep here.
  maburgs::S1LossWindow s3_loss;
  // Current-rung-only siblings (transition attribution, spec 2026-08-14):
  // same machinery, fed from the attributed counters. s1_loss/s3_loss stay
  // as the observability totals (residual_loss, the artifact-rate meter).
  maburgs::S1LossWindow s1_loss_cur, s1_resid_cur, s3_loss_cur, s3_resid_cur;
  // Post-transition settle for s1_resid_cur (the block-4 instant-demote
  // input; see the blank_until call at the sid-0 transition edge-detect).
  // Budget: one 50 ms edge-detect tick + the ~80 ms abandonment-horizon
  // booking lag + margin. Deliberately half of s3_settle_ms: a genuine
  // continuing fade then steps ~200 ms/rung, inside the ~410-440 ms/rung
  // cadence flight-validated 2026-08-14, and nowhere near the broken
  // 50 ms/rung debris cascade this exists to stop.
  // (settle constant now lives in TransitionEdge::kResidSettleMs)
  // Observability siblings of s1_resid_cur, pooled over both layers: the
  // sideport's link.residual_loss / link.attrib.residual_cur and the ctl
  // log's S-line resid/resid_cur. These MUST be windowed, not cumulative --
  // the decoder's abandonment counters are monotonic since boot, and a
  // lifetime average never returns to zero, which would break
  // flightreport.py's residual-episode detection and turn the player OSD's
  // post-loss row into a number that barely moves during a real burst.
  maburgs::S1LossWindow pool_resid, pool_resid_cur;
  // Per-layer delivery percent (sideport link.layer_delivery_pct), windowed
  // for the same reason.
  std::array<maburgs::S1LossWindow, 2> layer_resid;
  // Fade-trigger staleness gate (spec 2026-08-14 §3, source repointed to
  // the s1+s3 pool 2026-08-15): per-card pooled frame counts snapshotted
  // once per feedback window (same cadence as the prev_pkts_out snapshot
  // below), so "zero
  // s1+s3 frames this window" can NaN the RF labels before they reach the
  // controller. A frozen EMA must never read as a live signal. Snapshotted
  // for EVERY card, not just the chosen one: the choice itself is
  // freshness-gated, so which card is chosen can change between windows
  // and each needs its own baseline.
  std::vector<uint64_t> prev_pool_frames(static_cast<size_t>(n_cards), 0);
  // Scratch for select_label_card(), hoisted so the per-window refill
  // allocates nothing after the first pass.
  std::vector<maburgs::CardLabelInput> label_card_inputs(
      static_cast<size_t>(n_cards));
  // Completed-packet high-water mark at the controller's last non-disc step;
  // the starvation gate diffs against it (see the control step below).
  uint64_t prev_pkts_out = 0;
  // Commanded-MCS edge detect for boundary marking. -1 forces a first mark
  // (which finds cur_mcs unknown -> closed plain-fallback boundary).
  maburgs::TransitionEdge edge;
  // Change-detect on ctl().last_event(): initialize to the pre-any-event
  // default (t_ms 0) so boot doesn't print a phantom transition line.
  double last_ctl_event_ms = vrx.ctl().last_event().t_ms;
  // Same change-detect pattern for the ctl log's probe/penalty records
  // (initialized to the pre-any-event default so boot doesn't print one).
  double last_probe_t_ms = vrx.ctl().last_probe_edge().t_ms;
  double last_penalty_t_ms = vrx.ctl().last_penalty().t_ms;
  double last_rung_log_ms = 0;
  // R-line snapshot of every rung with any data (spec 2026-08-13).
  auto emit_rung_lines = [&](double t_ms) {
    if (!ctl_log) return;
    const maburgs::RungStore& st = vrx.ctl().rungs();
    for (int i = 0; i < static_cast<int>(st.size()); ++i) {
      const maburgs::RungStat& rs = st.stat(i);
      if (rs.u.n == 0 && rs.probe_u.n == 0) continue;
      const double age_s =
          rs.last_sample_ms < 0 ? -1.0 : (t_ms - rs.last_sample_ms) / 1000.0;
      const double sd = rs.evm_n ? std::sqrt(rs.evm_var_db2)
                                 : std::numeric_limits<double>::quiet_NaN();
      ctl_log->rung(t_ms, i, rs.u.v, rs.resid.v, rs.u3.v, rs.s3_resid.v,
                     rs.evm_db, sd, rs.u.n, age_s, rs.probe_u.v,
                     rs.probe_u.n);
    }
  };
  // Drone restart = new flight: rotate the debug-log session in place so
  // each maburd start gets its own NNNN directory (ctl/probe/au/flight, and
  // maburplay's lat.log follows the marker within 5 s). Detected from
  // tlm_seq restarting (drone_restart.h); nothing else here resets.
  maburgs::DroneRestartDetector drone_restart;
  auto rotate_session = [&](uint16_t from_seq, uint16_t to_seq) {
    if (!debug.ok()) return;
    const std::string old_dir = debug.dir();
    if (!debug.rotate()) {
      std::fprintf(stderr, "debug-log: rotate failed, staying in %s\n",
                   old_dir.c_str());
      return;
    }
    if (flight_jsonl != maburgs::LogWriter::kBadStream)
      log_writer->reopen(flight_jsonl, debug.dir());
    if (ctl_log) ctl_log->rotate(debug.dir());
    if (probe_log) probe_log->rotate(debug.dir());
    if (fec_log) fec_log->rotate(debug.dir());
    if (au_log) au_log->rotate(debug.dir());
    if (scan_log) scan_log->rotate(debug.dir());
    std::fprintf(stderr,
                 "debug-log: drone restart (tlm_seq %u -> %u): session %s -> %s\n",
                 static_cast<unsigned>(from_seq), static_cast<unsigned>(to_seq),
                 old_dir.c_str(), debug.dir().c_str());
  };
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>& f, uint64_t us) {
    if (mabur::rc::frame_type(f.data(), f.size()) == mabur::rc::T_TELEM) {
      // A CRC-clean frame can still fail to parse as a valid Telem (e.g. a
      // corrupted T_TELEM whose CRC happens to pass this layer but whose
      // internal fields don't parse) — only overwrite the holder on success,
      // so a bad frame leaves the last good telemetry (and its rx_ms stamp)
      // untouched rather than clobbering it with nullopt.
      if (auto t = mabur::rc::parse_telem(f.data(), f.size())) {
        const uint16_t prev_seq = latest_telem.t ? latest_telem.t->tlm_seq : 0;
        if (drone_restart.on_telem(t->tlm_seq, static_cast<double>(us) / 1000.0))
          rotate_session(prev_seq, t->tlm_seq);
        latest_telem.t = t;
        latest_telem.rx_ms = us / 1000;
        // link-rtt: every telem is a sync sample. `us` is the radio
        // frontend's steady_clock stamp — same base as the mono_us() send
        // stamps below, so the subtraction is one clock throughout.
        rtt_est.on_telem(t->rcf_seq_echo, (t->flags & 0x08) != 0,
                         t->rcf_age_ms, t->pts_at_build, us);
        // Calibration ack (spec: Telem flags bit6 cal_active) -- the only
        // acknowledgment signal the wire carries for a T_CAL_CMD. Telem has
        // no nonce field, so cal_pending_nonce (stashed from the CalCmd the
        // last due_cmd() call sent) is what on_ack() checks against; a
        // stale/mismatched nonce or an ack outside AwaitAck is a no-op
        // inside CalSession (cal_session.h).
        if ((t->flags & 0x40) != 0) {
          cal_session.on_ack(cal_pending_nonce, us / 1000);
          // cal.log's R line is once per SESSION; the ack fires once per
          // PHASE -- dedup by nonce (cal_log.h).
          if (cal_log && cal_log_run_nonce != cal_pending_nonce) {
            cal_log->run(cal_pending_nonce, cal_session.margin_db());
            cal_log_run_nonce = cal_pending_nonce;
          }
        }
      }
      return;
    }
    vrx.on_rc_frame(f.data(), f.size(), static_cast<double>(us) / 1000.0);
  });

  std::unique_ptr<maburgs::UdpSink> msp_udp;
  std::unique_ptr<maburgs::MspSink> msp_sink;
  if (cfg.msp.enable) {
    const int bp = cfg.msp.symbol_size + static_cast<int>(mabur::sw::kSwHeaderLen);
    msp_udp = std::make_unique<maburgs::UdpSink>(cfg.msp.out_host, cfg.msp.out_port);
    maburgs::MspSink::EmitFn emit = [&](const uint8_t* d, size_t n) { msp_udp->send(d, n); };
    std::fprintf(stderr,
                 "maburgs: MSP OSD -> udp %s:%d symbol_size=%d window=%d block_payload=%d\n",
                 cfg.msp.out_host.c_str(), cfg.msp.out_port, cfg.msp.symbol_size,
                 cfg.msp.window, bp);
    msp_sink = std::make_unique<maburgs::MspSink>(cfg.msp.symbol_size, cfg.msp.window, emit);
    agg.set_msp_sink([&](const uint8_t* b, size_t n, uint64_t us) {
      msp_sink->on_body(b, n, us / 1000);
    });
  }

  // Probe stream bodies (SBI stream 5). Same core-thread contract as the MSP
  // sink: the aggregator calls this from the drain loop below. Every body is
  // scored per-card; ProbeTrack merges the cards into the union the ladder
  // reads. The RF labels are this body's own, not the card's EMA -- a probe
  // sample must carry the radio conditions it actually flew through.
  agg.set_probe_sink([&](uint8_t card, const mabur::node::RxBody& m) {
    // The probe is the last PPDU of its ENH burst: seeing it (any card,
    // any profile, parseable or not -- the aggregator routed it here by
    // stream id) is the slotter's "burst off air" release (rcf_slot.h).
    rcf_slot.on_probe_tail(mono_ms());
    mabur::probe::ProbeRx rx;
    if (!mabur::probe::parse_probe_body(m.body.data(), m.body.size(),
                                        probe_block_payload, &rx))
      return;
    const double snr = m.phy_valid
                            ? std::max(m.snr[0], m.snr[1]) * maburgs::kSnrRawToDb
                            : std::nan("");
    // EVM: raw half-dB, negative = clean, 0 = not sampled (node.h) -- pick
    // the better sampled chain, NaN when neither chain reported.
    const int8_t evm_raw =
        (m.evm[0] != 0 && (m.evm[1] == 0 || m.evm[0] < m.evm[1])) ? m.evm[0]
                                                                  : m.evm[1];
    const double evm = evm_raw != 0 ? evm_raw * maburgs::kEvmRawToDb : std::nan("");
    probe_track.on_body(card, rx, snr, evm,
                        static_cast<double>(m.mono_us) / 1000.0);
  });

  maburgs::TxSelector sel(
      maburgs::TxSelectorCfg{tx_card_pin, 3.0, 2000, 1500}, n_cards);

  std::vector<uint64_t> retry_at_ms(static_cast<size_t>(n_cards), 0);
  // scan.log C record: once per card, the first time it reports ready.
  std::vector<bool> caps_logged(static_cast<size_t>(n_cards), false);
  // Last sample mirrored into the sideport. Nothing writes this since the
  // A-record block (1 Hz in-flight energy poll, spec section 7) was removed
  // with radio.scan.energy_period_ms -- cards[i].energy reads null on the
  // sideport until Task 11's verdict windows refill it.
  std::vector<std::optional<maburgs::StatsEnergyIn>> energy_last(
      static_cast<size_t>(n_cards));
  uint64_t last_stats_ms = 0;
  // Separate from last_stats_ms since 2026-08-15: the ctl log runs on
  // debug_log.ctl_period_ms, the stderr line stays at 1 Hz.
  uint64_t last_ctl_sample_ms = 0;
  // Foreign-RC_VERSION warning throttle. Separate `logged` flag rather than a
  // 0 sentinel: mono_ms() is small but not guaranteed non-zero at startup.
  uint64_t last_foreign_rc_ms = 0;
  bool foreign_rc_logged = false;
  std::vector<mabur::node::RxBody> batch;

  // Rate-aware gap timeout: updated 1 Hz from the per-stream seq-advance
  // rate, pushed into FrameStream (gap_timeout_policy.h).
  maburgs::GapTimeoutPolicy gap_policy(cfg.video.frame_gap_timeout_ms,
                                       cfg.video.frame_gap_timeout_max_ms);
  uint64_t gap_update_ms = 0;

  while (!g_stop.load()) {
    const uint64_t now_ms_u = mono_ms();
    const double now_ms = static_cast<double>(now_ms_u);

    // Card lifecycle: (re)open dead front-ends with 2 s backoff.
    for (int i = 0; i < n_cards; ++i) {
      auto& fe = *fronts[static_cast<size_t>(i)];
      // The scout thread owns this card's control plane, so it must be gone
      // before the card is stopped and reopened underneath it. A dead card
      // also fails every retune, which costs the scout its dwell sleeps and
      // spins it; freezing on what it has measured so far ends that.
      if (i == scout_card && !scout_joined && !fe.alive()) {
        const uint64_t rounds = scout->rounds();
        join_scout(scout->proposal());
        std::fprintf(stderr,
                     "maburgs channel: scout card %d died, scan abandoned at "
                     "%llu rounds, card stays at 20 MHz\n",
                     i, static_cast<unsigned long long>(rounds));
      }
      if (!fe.alive() && now_ms_u >= retry_at_ms[static_cast<size_t>(i)]) {
        fe.stop();
        if (!fe.open_and_start())
          std::fprintf(stderr, "card %d: open failed, retrying\n", i);
        else
          cur_ch[static_cast<size_t>(i)] = cfg.radio.channel;  // InitWrite tunes home
        retry_at_ms[static_cast<size_t>(i)] = now_ms_u + 2000;
      }
      if (fe.ready() && !caps_logged[static_cast<size_t>(i)]) {
        const maburgs::CardCaps c = fe.caps();
        if (scan_log) scan_log->caps(now_ms, i, c);
        caps_logged[static_cast<size_t>(i)] = true;
        std::fprintf(stderr,
                     "maburgs radio card %d: %s %s %dx%d fast_retune=%d "
                     "sensors fa=%d igi=%d nhm=%d floor=%d\n",
                     i, c.chip.c_str(), c.gen.c_str(), c.tx_chains, c.rx_chains,
                     c.fast_retune ? 1 : 0, c.fa_ok ? 1 : 0, c.igi_ok ? 1 : 0,
                     c.nhm_ok ? 1 : 0, c.floor_ok ? 1 : 0);
      }
    }
    // The scout starts once (and only once) its card is up: from here until
    // join_scout() that card's retune/read_energy belong to the scout thread.
    if (scout && scout_joined && !scout->frozen() &&
        fronts[static_cast<size_t>(scout_card)]->ready()) {
      scout_thread = std::thread([&] { scout->run(); });
      scout_joined = false;
    }

    // Drain (blocks <=10 ms: this IS the control tick cadence).
    batch.clear();
    queue.drain(batch, 10);
    for (const auto& m : batch) {
      // Calibration sweep frames (cal_wire.h) are a distinct body type, not
      // video, and must be recognized BEFORE agg.on_rx_body() routes the
      // body -- nothing downstream of on_rx_body() (the decoder, the RC
      // dispatch, the video-silence timer) knows what a sweep frame is, and
      // a frame stamped at TXAGC idx 127 fed to the decoder as garbage
      // video would be silently wrong rather than loudly ignored. Must run
      // even for CRC-bad frames: maburgs sets rx.keep_corrupted
      // unconditionally, so corrupt sweep frames arrive and are tallied as
      // `corrupt`, not silently dropped (design "CRC-bad frames now
      // arrive"). A frame whose corruption flips rate/idx/phase/fill fails
      // parse_cal_payload's fill guard and falls through to the ordinary
      // path below instead of being misattributed to a cell.
      mabur::cal::CalFrameInfo cal_info;
      if (mabur::cal::parse_cal_payload(m.body.data(), m.body.size(), &cal_info)) {
        const int rssi_dbm =
            static_cast<int>(std::max(m.rssi[0], m.rssi[1])) - 110;
        cal_session.on_cal_frame(m.card_id, cal_info, rssi_dbm, m.crc_ok,
                                 m.mono_us / 1000);
        continue;
      }
      agg.on_rx_body(m);
      // A drone at a different RC_VERSION is invisible to every RC path here:
      // frame_type() returns -1 for it, which is this loop's affirmative "this
      // is video" signal, so its T_TELEM/DISC_ACK is counted into the card
      // totals AND fed to vrx.on_video() below, holding off the rendezvous
      // video-silence fallback on traffic the decoder never sees. The
      // visible end state is no video at all, indistinguishable from the
      // stale-caps restart deadlock. Log it (rate-limited to 1/5s: a
      // mismatched peer transmits continuously and /tmp is tmpfs) and change
      // nothing else -- the frame is still handled exactly as before.
      //
      // Gated on crc_ok because RC_MAGIC is only two bytes: roughly 1 in 65536
      // corrupt video bodies matches it by chance, and a corrupt frame must
      // not raise a version-mismatch alarm.
      if (m.crc_ok &&
          mabur::rc::is_foreign_rc_version(m.body.data(), m.body.size()) &&
          (!foreign_rc_logged || now_ms_u - last_foreign_rc_ms >= 5000)) {
        foreign_rc_logged = true;
        last_foreign_rc_ms = now_ms_u;
        std::fprintf(stderr,
                     "maburgs: heard an RC frame at RC_VERSION %u but this "
                     "build speaks %u, so no RC path can read it and it is "
                     "being miscounted as video (rate-limited to 1/5s). "
                     "The pair is half-deployed: no control link and, because "
                     "DISC_ACK carries CAP_FRAME_WIRE, no video either. "
                     "Finish the deploy on BOTH ends; restarting maburd will "
                     "not help.\n",
                     static_cast<unsigned>(m.body[2]),
                     static_cast<unsigned>(mabur::rc::RC_VERSION));
      }
      // Exclude RC frames, MSP frames (SBI stream_id == kMspStreamId) AND
      // probe frames (stream_id == kProbeStreamId), mirroring the
      // aggregator's routing: only real video may refresh the rendezvous
      // video-silence timer, or a link carrying nothing but MSP telemetry
      // or probe canaries would never fall back to BEACONING. This keeps
      // MSP and probe traffic invisible to the adaptive-link controller
      // end-to-end.
      const int sid_peek =
          mabur::sbi_peek_stream_id(m.body.data(), m.body.size());
      if (m.crc_ok &&
          mabur::rc::frame_type(m.body.data(), m.body.size()) < 0 &&
          sid_peek != mabur::kMspStreamId && sid_peek != mabur::kProbeStreamId) {
        vrx.on_video(static_cast<double>(m.mono_us) / 1000.0);
        // In-flight hop confirmation (spec section 4): the channel THIS
        // video body was actually received on, stamped by the producing
        // RadioFrontend at the instant it lifted the frame off the card
        // (mabur/node.h's RxBody::rx_channel).
        //
        // NOT cur_ch[m.card_id], which is where that card is tuned NOW:
        // this drain runs up to a full control tick behind the RX threads
        // and RadioFrontend::retune() does not flush BodyQueue, so bodies
        // received on the OLD channel and still queued when the lead
        // card's retune landed were being stamped with the target and
        // confirmed the hop ~10 ms after the order -- before the order's
        // RCF had even left the slotter. 0 = received mid-retune (or a
        // replay source): never equal to a real hop target, so it simply
        // fails to confirm, which is the safe direction.
        last_video_ch = m.rx_channel;
      }
    }
    // Re-read the clock: the drain above blocked up to 10 ms, and bodies
    // processed in it carry stamps newer than now_ms_u. Timeout/hold math
    // must run on a clock >= every stamp it has seen (the decoder and
    // reorder buffer also guard against stale clocks internally).
    const uint64_t drained_ms = mono_ms();
    if (drained_ms >= gap_update_ms + 1000) {
      gap_update_ms = drained_ms;
      for (int s = 0; s < 2; ++s) {
        gap_policy.update(s, agg.decoder().newest_seq(s),
                          agg.decoder().repair_window(s), drained_ms);
        fstream.set_gap_timeout(
            s, static_cast<uint64_t>(gap_policy.timeout_ms(s)));
      }
    }
#ifdef MABUR_LOSS_SIM
    if (loss_ctl.ok()) loss_ctl.poll(agg.loss_sim());
#endif
    // Compiled into every prod build, unlike loss_ctl above (cal_control.h).
    const auto cal_state_before_poll = cal_session.state();
    cal_ctl.poll(cal_session);
    // Loud, exactly once per accepted `maburcal start` (Idle/Done/Failed ->
    // AwaitAck): the operator's own terminal is streaming CalControl's
    // "CAL start -> ok started" reply already, but this is the one place
    // that knows where cal_log_dir actually resolved to -- see the cal_log
    // construction comment for why that is not always debug.dir().
    if (cal_state_before_poll != maburgs::CalSession::State::AwaitAck &&
        cal_session.state() == maburgs::CalSession::State::AwaitAck) {
      std::fprintf(stderr, "maburgs: calibration started -> %s/cal.log\n",
                   cal_log_dir.c_str());
    }

    // Session capability gate: the peer must be in an active SESSION (not
    // beaconing/pre-rendezvous) AND have advertised CAP_FRAME_WIRE in its
    // DiscAck before its video fragments are fed to the frame tail. A session
    // without the bit is a pre-frame-shm drone whose frag header this build
    // cannot parse: refuse its video loudly rather than render garbage. On any
    // change, drop FRAG-seq continuity and half-assembled frames — the new
    // session's seqs and frame_ids are unrelated to the old one's.
    const bool in_session = vrx.link_state() == maburgs::VrxState::SESSION;
    in_session_atomic.store(in_session, std::memory_order_relaxed);

    // ---- auto channel selection, per tick (spec 2026-09-13) ----
    plan.tick(now_ms, in_session);

    // ---- in-flight channel hop: verdict window (spec section 2) ----
    // Placed right after plan.tick() so a hop ordered below lands on
    // plan.op()/plan.hopping() this SAME tick -- the mechanical retune loop
    // and the RCF this tick's vrx.step() builds both read plan/vrx state
    // that follows, not precedes, this block.
    //
    // Gated on hop_active() (hop_burst_gate.h): SESSION and the boot scout
    // owning no card. On the falling edge the verdict engine is reset and
    // the cached VerdictOut cleared, so no trigger measured on a link that
    // was down can be acted on when it returns; on the rising edge the
    // per-card and recovered baselines are re-primed, so the first window
    // of a session measures the session, not the outage before it.
    const bool hop_active = maburgs::hop_active(in_session, scout_joined);
    if (hop_active != hop_was_active) {
      hop_was_active = hop_active;
      verdict.reset();
      last_verdict = maburgs::Verdict::Healthy;
      last_verdict_out = maburgs::VerdictOut{};
      std::fill(window_prev_ok.begin(), window_prev_ok.end(), false);
      recovered_prev_window = agg.decoder().stats(0).syms_recovered +
                              agg.decoder().stats(1).syms_recovered;
      last_window_ms = now_ms_u;
    }
    if (hop_active && now_ms_u - last_window_ms >= static_cast<uint64_t>(hcfg.window_ms)) {
      last_window_ms = now_ms_u;
      std::vector<maburgs::VerdictCardIn> vc(static_cast<size_t>(n_cards));
      for (int i = 0; i < n_cards; ++i) {
        auto& fe = *fronts[static_cast<size_t>(i)];
        const size_t si = static_cast<size_t>(i);
        const bool busy = dwell_busy.load() && dwell_card.load() == i;
        if (busy || !fe.ready()) { window_prev_ok[si] = false; continue; }
        const maburgs::ScoutEnergy e = fe.read_energy_scout();
        const maburgs::ScoutFrames f = fe.frames();
        const auto& t = agg.card(i);
        if (window_prev_ok[si]) {
          vc[si].valid = true;
          vc[si].fa = e.fa_ofdm;
          vc[si].cca = e.cca_ofdm;
          vc[si].foreign = static_cast<uint32_t>(f.foreign - window_prev[si].foreign);
          vc[si].crc_fail = static_cast<uint32_t>(t.crc_fail - window_prev_crc[si]);
          // Raw EMAs -> the dBm/dB the [hop.verdict] thresholds are in
          // (hop_verdict.h); feeding raw here left `weak` unreachable.
          vc[si].rssi_dbm = maburgs::rssi_raw_to_dbm(t.rssi_a_ema);
          vc[si].snr_db = maburgs::snr_raw_to_db(t.snr_ema);
          // Keep cards[i].energy on the sideport alive from this window --
          // Task 3 removed the 1 Hz A-record poll that used to feed it.
          energy_last[si] = maburgs::StatsEnergyIn{
              e.cca_ofdm, e.fa_ofdm, f.own - window_prev[si].own,
              f.foreign - window_prev[si].foreign, std::nullopt};
        }
        window_prev[si] = f;
        window_prev_crc[si] = t.crc_fail;
        window_prev_ok[si] = true;
      }
      maburgs::VerdictLinkIn vl;
      // The s1 (BASE) loss window, same family the ladder's own
      // pre_fec_loss reads (s1_loss_cur is the current-rung-scoped
      // sibling) -- sampled independently here since this block runs
      // before that window's .add() for THIS tick, so it reflects state as
      // of the end of the previous tick: one control-loop tick (~10-20 ms)
      // stale against a 150 ms-default verdict window, immaterial.
      const auto s1_hop_sample = s1_loss.sample(now_ms);
      vl.pre_fec_loss = s1_hop_sample.valid ? s1_hop_sample.loss : 0.0;
      const uint64_t recovered_now = agg.decoder().stats(0).syms_recovered +
                                     agg.decoder().stats(1).syms_recovered;
      vl.recovered = static_cast<uint32_t>(recovered_now - recovered_prev_window);
      recovered_prev_window = recovered_now;
      const auto vo = verdict.window(now_ms, vc, vl, vrx.ctl().rung());
      if (scan_log && (vo.v != maburgs::Verdict::Healthy || vo.v != last_verdict))
        scan_log->verdict(now_ms, vo, vc, vl);
      last_verdict = vo.v;
      last_verdict_out = vo;
      // Spec section 4: the rung store's EWMAs stop taking writes at the
      // first INTERFERED window of an episode, not at the order 300-450 ms
      // later. One deadline per episode, confirm_ms + the settle wide, and
      // only when the feature is enabled -- see hop_blank.h for why each
      // of those three gates is load-bearing. blank_store() keeps the
      // later of the deadlines it is given, so this never shortens
      // HopAction::Order's own window.
      if (const auto blank =
              maburgs::hop_store_blank_until(vo, hcfg.enable, hcfg.confirm_ms))
        vrx.blank_store(*blank);
    }

    // ---- in-flight channel hop: controller tick + actions (spec
    // section 4/5) ---- same hop_active gate as the window above: an
    // in-flight hop's own timers simply resume on the next active tick
    // (confirm_ms elapsed -> withdraw, the fail-safe outcome).
    if (hop_active) {
      maburgs::HopTick ht;
      ht.now_ms = now_ms;
      ht.verdict = last_verdict_out;
      ht.cur_op = plan.op();
      ht.n_cards = n_cards;
      ht.lead_card = n_cards >= 2 ? (sel.selected() == 0 ? 1 : 0) : -1;
      ht.best = ranker.best(now_ms, plan.op(), hopc.backed_off(now_ms));
      if (ht.best) {
        for (const auto& e : ranker.ranking(now_ms))
          if (e.ch == *ht.best) { ht.best_score = e.score; break; }
      }
      // lead_card (or the only card, one-card mode) confirms the hop by
      // landing a video body on the target channel -- last_video_ch is set
      // in the batch-drain loop above from RxBody::rx_channel, the channel
      // the producing card was actually tuned to when it lifted that frame
      // off the air (NOT cur_ch[m.card_id], where the card is tuned now),
      // matching the brief's "lead card received a video AU on hop_ch".
      ht.video_on_target = plan.hopping() && last_video_ch == plan.hop_target();
      ht.rcf_sent_since_order = static_cast<int>(rcf_sent_total - rcf_sent_at_order);
      // hop_burst_gate.h's hop_burst_due() (Task 15 fix round 1): "no hop
      // in flight" (Idle/Hold, not Ordered/Verifying) AND the trigger AND
      // the dwell_period_ms rate limit against last_burst_ms below -- see
      // that header for the four properties this gate has to get right,
      // now unit-tested directly instead of only inside this hardware-
      // touching body.
      if (maburgs::hop_burst_due(hopc.state(), last_verdict_out.trigger, now_ms,
                                 last_burst_ms, hcfg.dwell_period_ms)) {
        // Freshness burst (spec section 3): sweep every candidate once,
        // back to back, BEFORE a target is chosen -- so it must not
        // require ht.best to already hold one (dropped from this gate in
        // fix round 2; the plan's own snippet had it, the spec overrules).
        // BOTH card counts (spec: "the only card on a one-card GS since
        // the link is already impaired") -- a one-card GS never runs the
        // periodic scout thread (that stays two-card-only, see scout_loop
        // above), so this burst is the ONLY source of ranking data it will
        // ever have; without it (and without also accepting Hold above)
        // ht.best is permanently nullopt, the state machine can only ever
        // order home once and then hold forever, and it can never leave
        // Hold since leaving requires order(), which itself requires
        // best. Runs synchronously on the core thread (blocking it for
        // the whole sweep, a handful of candidates at a few ms each on
        // two cards -- see the report's threading notes and, for the
        // one-card case, the burst-duration note in fix round 2), so
        // inflight_mu is held for the duration to keep the scout thread's
        // own periodic dwell (when one is running, i.e. two-card) from
        // driving the same InflightScout/RadioFrontend at the same time.
        last_burst_ms = now_ms;
        const int burst_card = ht.lead_card >= 0 ? ht.lead_card : 0;
        std::lock_guard<std::mutex> ilk(inflight_mu);
        auto& fe = *fronts[static_cast<size_t>(burst_card)];
        inflight.set_radio(fe);
        std::vector<maburgs::ScoutDwell> recs;
        for (const auto& v : inflight.burst(plan.op(), recs)) ranker.add(v);
        for (const auto& d : recs)
          if (scan_log) scan_log->dwell(now_ms, burst_card, d);
        // Re-sync from live hardware, mirroring the periodic-dwell drain
        // below: a candidate dwell inside the burst may have failed its
        // return retune (kFlagRetuneFailed) and left the card off `op_`.
        cur_ch[static_cast<size_t>(burst_card)] = fe.channel();
        ht.best = ranker.best(now_ms, plan.op(), hopc.backed_off(now_ms));
        if (ht.best) {
          for (const auto& e : ranker.ranking(now_ms))
            if (e.ch == *ht.best) { ht.best_score = e.score; break; }
        }
      }
      const maburgs::HopAction act = hopc.tick(ht);
      // Shared with run_hop_inject_test() (Task 15 fix round 1) -- see
      // apply_hop_action()'s own comment. The three lines below are
      // run_radio()-only bookkeeping with no meaning to the test harness
      // (hopping_atomic is cross-thread state for the scout/inflight
      // threads; rcf_sent_at_order anchors the one-card RCF-repeat count
      // to THIS run's send counter), so they stay here, immediately after
      // the call, in the same relative order as before the extraction.
      apply_hop_action(act, now_ms, hcfg.confirm_ms, vrx, plan, verdict);
      switch (act.kind) {
        case maburgs::HopAction::Order:
          hopping_atomic.store(true);
          rcf_sent_at_order = rcf_sent_total;
          break;
        case maburgs::HopAction::Confirm:
        case maburgs::HopAction::Withdraw:
          hopping_atomic.store(false);
          break;
        case maburgs::HopAction::VerifyPass:
          // Handled in apply_hop_action() above (HopVerdict::reset()) --
          // nothing run_radio-specific to do.
        case maburgs::HopAction::OneCardRetune:
        case maburgs::HopAction::Hold:
        case maburgs::HopAction::None:
          break;
      }
      for (const auto& e : hopc.take_events()) {
        if (scan_log) scan_log->hop(e);
        std::fprintf(stderr, "maburgs hop: %s epoch %u target %u score %u +%.0f ms\n",
                     e.kind.c_str(), e.epoch, e.target, e.score, e.elapsed_ms);
        last_hop_event_ms = static_cast<uint64_t>(e.elapsed_ms >= 0 ? e.elapsed_ms : 0.0);
      }
    }

    // Proposal for the next DISC: the frozen op once the drone has answered,
    // the scout's current best before that, home with scanning off.
    vrx.set_proposal(plan.frozen() ? plan.op()
                                   : (scout ? scout->proposal() : cfg.radio.channel));
    // Consumed every tick regardless (an edge left unread would otherwise
    // sit stale until the next real ack -- take_ack_edge() clears it on
    // read), but only ACTED on outside a hop: ChannelPlan::on_ack() carries
    // no hopping_ guard of its own (Task 6's carried invariant), resting on
    // acks never arriving in-session mid-hop. That invariant is not
    // airtight -- a beacon (and so a fresh DiscAck) can only fire once the
    // rendezvous falls out of SESSION, which needs 1000 ms of video
    // silence, but a stalled one-card hop can run that long -- so this is
    // the enforcement rather than a bare trust in the invariant. An ack
    // this ignores while hopping is not lost: BEACONING keeps re-offering
    // DiscAcks every beacon_period_ms, so the next one lands the tick after
    // hopping() clears.
    if (vrx.take_ack_edge() && !plan.hopping()) {
      const uint8_t proposed = vrx.proposal();
      const bool first = !plan.frozen();
      plan.on_ack(now_ms, vrx.agreed_channel(), proposed);
      if (first && scout) {
        scout->freeze(plan.op());  // stops the scan for the process lifetime
        // "none" = the drone appeared before any channel reached
        // min_rounds (the DISC proposed home by default, not by
        // measurement). Computed outside the scan_log guard because the
        // hop ranker needs the same answer whether or not logging is on.
        const auto all = scout->ranking();
        bool any_ranked = false;
        if (cfg.radio.width == 40) {
          // At 40 MHz the entries are per HALF and pair_proposal only ranks
          // a pair once BOTH halves reach min_rounds; one half there (e.g.
          // the one-card home half, which also books home-window visits)
          // would call home-by-default a measured pick.
          any_ranked = maburgs::any_pair_ranked(all, cfg.radio.channel, scfg.candidates,
                                                scfg.min_rounds);
        } else {
          for (const auto& e : all)
            any_ranked = any_ranked || e.visits >= static_cast<uint32_t>(scfg.min_rounds);
        }
        // The real boot-time pick, for HopRanker's tie-break (spec
        // section 3). Only when the scan actually measured something: an
        // unranked "pick" is just home proposed by default, and feeding
        // that in would re-collapse the tiebreak.
        if (any_ranked) ranker.set_boot_pick(proposed);
        if (scan_log)
          scan_log->pick(now_ms,
                         any_ranked ? std::optional<uint8_t>(proposed) : std::nullopt,
                         scout->rounds(), all, scfg.min_rounds);
      }
      // Re-publish the proposal the instant the plan commits (Important
      // fix 3): set_proposal above ran BEFORE on_ack, so on the commit tick
      // it still holds the pre-ack proposal. On an ack_override -- the
      // drone agreeing to a channel that is not the one we proposed -- the
      // DISC built later in this same tick would otherwise beacon the STALE
      // proposal on the NEW channel, i.e. command the drone to move a
      // second time, to the channel it just declined.
      if (plan.frozen()) vrx.set_proposal(plan.op());
    }
    // Scout bookkeeping: drain the dwell records, then join the thread once
    // it has parked its card on the op channel (run() retunes before done()).
    if (scout) {
      if (scan_log)
        for (const auto& d : scout->take_dwells()) scan_log->dwell(now_ms, scout_card, d);
      else
        (void)scout->take_dwells();
      if (!scout_joined && scout->done()) {
        scout_thread.join();
        scout_joined = true;
        // The live atomic, not plan.op(): a second ack with a different
        // agreed channel between freeze and join would otherwise record a
        // channel the card is not on, and the loop below would never
        // correct it (it only retunes on a mismatch).
        cur_ch[static_cast<size_t>(scout_card)] =
            fronts[static_cast<size_t>(scout_card)]->channel();
      }
    }
    // In-flight scout thread: started once (idempotent, guarded by
    // inflight_started) as soon as the boot scout no longer owns any card
    // -- immediately, when radio.scan.enable is off and there was no boot
    // scout at all (scout_joined starts true). Two-card only: the thread's
    // own loop gates every cycle on n_cards >= 2, but there is no point
    // spinning it up on one card.
    if (!inflight_started && scout_joined && n_cards >= 2 &&
        (hcfg.enable || hcfg.scout_when_disabled)) {
      inflight_started = true;
      scout_run.store(true);
      scout_thread2 = std::thread(scout_loop);
    }
    // In-flight scout bookkeeping: drain completed dwells/visits. Re-sync
    // cur_ch[card] from the LIVE hardware channel() rather than trusting
    // the cache -- the scout thread retunes this card directly, bypassing
    // the mechanical retune loop below, and on a stranded return retune
    // (ScoutDwell::survey.flags & kFlagRetuneFailed, Task 10) the card can
    // be left parked on the candidate. Refreshing here is what lets the
    // mechanical loop (which only acts on a cur_ch/desired mismatch)
    // notice and pull it back next tick -- mirroring the boot scout's own
    // join-time resync just above. This is also why cur_ch must NOT be
    // written from the scout thread itself: it is single-writer (this
    // core loop) by construction, so no lock is needed around it.
    {
      std::vector<std::pair<int, maburgs::ScoutDwell>> drained;
      std::vector<maburgs::HopVisit> visits;
      {
        std::lock_guard<std::mutex> lk(dwell_mu);
        drained.swap(dwell_recs);
        visits.swap(dwell_visits);
      }
      size_t vi = 0;
      for (auto& rec : drained) {
        const int card = rec.first;
        cur_ch[static_cast<size_t>(card)] = fronts[static_cast<size_t>(card)]->channel();
        if (scan_log) scan_log->dwell(now_ms, card, rec.second);
        // Sideport dwell snapshot (Task 12, StatsCardIn::dwell). dwell_recs
        // and dwell_visits are pushed in lockstep by scout_loop -- one
        // ScoutDwell per completed dwell, one HopVisit iff that SAME
        // dwell's retune didn't fail (InflightScout::dwell() sets
        // kFlagRetuneFailed on exactly the two paths that produce no
        // visit, never otherwise). So `visits` is a strict, order-
        // preserving subsequence of `drained`'s successes: walking both in
        // lockstep and consuming one visit per non-failed record
        // attributes each score to the right card despite HopVisit itself
        // carrying no card field.
        const bool ok =
            !(rec.second.survey.flags & devourer::chanmig::kFlagRetuneFailed);
        maburgs::StatsDwellIn ds =
            dwell_stats[static_cast<size_t>(card)].value_or(maburgs::StatsDwellIn{});
        ++ds.visits;
        ds.cost_us = static_cast<uint32_t>(rec.second.to_us + rec.second.read_us +
                                           rec.second.back_us);
        if (ok && vi < visits.size()) ds.score = maburgs::HopRanker::score(visits[vi]);
        dwell_stats[static_cast<size_t>(card)] = ds;
        if (ok) ++vi;
      }
      for (const auto& v : visits) ranker.add(v);
    }
    // Every move that changes where the link lives -> M line + stderr.
    for (const auto& ev : plan.take_events()) {
      if (scan_log) scan_log->move(ev);
      std::fprintf(stderr, "maburgs channel: %s card %d %u -> %u\n",
                   maburgs::to_string(ev.reason), ev.card,
                   static_cast<unsigned>(ev.from), static_cast<unsigned>(ev.to));
    }
    // The mechanical per-card retune toward plan.desired(). The scout card is
    // untouchable until joined; a card the in-flight scout thread currently
    // has off on a candidate (dwell_busy) is untouchable too -- the scout
    // thread is mid-retune-sequence on it, and fe.retune() from both
    // threads at once on the same RadioFrontend is the one race this
    // feature must never allow (see the report's threading notes).
    for (int i = 0; i < n_cards; ++i) {
      const bool scouting = !scout_joined && i == scout_card;
      const bool inflight_dwelling = dwell_busy.load() && dwell_card.load() == i;
      auto& fe = *fronts[static_cast<size_t>(i)];
      if (scouting || inflight_dwelling || !fe.ready()) continue;
      const uint8_t want = plan.desired(i);
      if (cur_ch[static_cast<size_t>(i)] != want && fe.retune(want))
        cur_ch[static_cast<size_t>(i)] = want;
    }
    const bool fw = in_session && (vrx.peer_caps() & mabur::rc::CAP_FRAME_WIRE);
    // Told every tick (CalSession::set_peer): whether the link is up and
    // whether the peer's last DiscAck carried CAP_CALIBRATE. start() (via
    // CalControl, below the operator's `maburcal start`) is the only place
    // that reads these back.
    cal_session.set_peer(
        in_session, in_session && (vrx.peer_caps() & mabur::rc::CAP_CALIBRATE));
    if (fw != frame_wire) {
      frame_wire = fw;
      agg.decoder().reset_continuity();
      fstream.reset();
      lat_anchor.reset();  // new session's pts space is unrelated to the old one's
      // Drop any pre-reset samples too: without this, the anchor re-warms
      // (kWarmFrames) before the next flush, but the window itself still
      // holds up to kCap stale pre-reset samples that would silently mix
      // into that first post-reset flush -- worst right after a reconnect.
      lat_win.clear();
      std::fprintf(stderr, "maburgs: video tail -> %s\n",
                   fw ? "frame wire" : "off (no session)");
    }
    // Complain only about a peer we have actually heard a DiscAck from:
    // peer_caps() == 0 also reads as "no DiscAck yet", and the rendezvous
    // starts in SESSION, so gating on in_session alone printed this at every
    // startup — telling the operator to upgrade a maburd that was fine, seconds
    // before the tail came up anyway (caught on the rig 2026-07-25).
    if (vrx.peer_acked() && !fw && !refused_peer) {
      refused_peer = true;  // once per run: this cannot fix itself mid-session
      std::fprintf(stderr,
                   "maburgs: REFUSING video: peer session did not advertise "
                   "CAP_FRAME_WIRE (chip_caps=0x%04x). That drone predates the "
                   "frame wire format; upgrade maburd.\n",
                   vrx.peer_caps());
    }
    if (frame_wire) fstream.poll(drained_ms);
    if (au_on) au_bell.poll();

    // Transition boundaries for loss attribution + settle-blank of the two
    // residual decision windows: gs/src/transition_edge.h (extracted
    // 2026-09-05 so tests/test_transition_edge.cpp can pin what an edge
    // blanks). The util windows read the ArrivalTracker below and are no
    // longer reachable from the edge at all.
    edge.on_tick(vrx.cur_op(), agg.decoder(), s1_resid_cur, s3_resid_cur, now_ms);

    // Control step: post-FEC residual from the FEC decoder's own abandonment
    // counters — ONE formula for every consumer since 2026-09-02, see
    // gs/src/ladder_residual.cpp. Per-layer delivery is operator-facing only
    // — it rides the stats sideport below, never the RCF (RC_VERSION 3
    // dropped it; maburd never read it).
    std::array<uint8_t, 2> ld{};
    for (int s = 0; s < 2; ++s) {
      const auto lc =
          maburgs::residual_counts(agg.decoder(), s, /*cur=*/false);
      auto& w = layer_resid[static_cast<size_t>(s)];
      w.add(lc.expected, lc.arrived, now_ms);
      const auto ls = w.sample(now_ms);
      // Idle layer reads 100, matching the deleted window_delivery_pct (the
      // 2026-07 controller-wedge finding in docs/bench-validation.md turns
      // on that convention). Truncating like the old integer ratio did.
      const int pct =
          ls.valid ? static_cast<int>(100.0 * (1.0 - ls.loss)) : 100;
      ld[static_cast<size_t>(s)] =
          static_cast<uint8_t>(pct < 0 ? 0 : (pct > 100 ? 100 : pct));
    }
    const auto rc_tot =
        maburgs::residual_counts_pooled(agg.decoder(), /*cur=*/false);
    const auto rc_cur =
        maburgs::residual_counts_pooled(agg.decoder(), /*cur=*/true);
    pool_resid.add(rc_tot.expected, rc_tot.arrived, now_ms);
    pool_resid_cur.add(rc_cur.expected, rc_cur.arrived, now_ms);
    const auto pr = pool_resid.sample(now_ms);
    const auto prc = pool_resid_cur.sample(now_ms);
    std::optional<double> residual;
    if (pr.valid) residual = pr.loss;
    std::optional<double> residual_cur;
    if (prc.valid) residual_cur = prc.loss;
    // Zero completed packets while video frames still arrive = decode
    // collapse; the ladder's video_starved path forces the failsafe rung
    // then rather than trusting this window's (survivor-biased) loss sample.
    // Gate on having ever seen video so a pre-link idle window doesn't count
    // as starvation.
    //
    // packets_out is CUMULATIVE, so this diffs against a snapshot taken at
    // the same boundary the deleted reset_window() used (the controller's
    // last non-disc step) — the residual counters are cumulative too now, so
    // "expected == 0 this window" is no longer a thing the decoder can say.
    const uint64_t pkts_now = agg.decoder().stats(0).packets_out +
                              agg.decoder().stats(1).packets_out;
    const bool starved = (pkts_now == prev_pkts_out) && agg.last_video_us() != 0;

    // s1_* names are historical (predate the 2-stream flatten): this is the
    // BASE sid's (0) window, the critical always-decode layer that drives
    // the ladder's ordinary demote/promote decisions.
    const auto s1 = agg.decoder().stats(0);
    // Pre-FEC (util) accounting from the ArrivalTracker (spec 2026-09-05):
    // expected = seq advance past the settle line, arrived = heard, booked
    // at arrival -- no repair/completion lag, and a denominator that does
    // not depend on decoder progress after a re-key. The total feeds the
    // sideport/ctl-log gauge, the current-only side feeds block 5 (util).
    s1_loss.add(s1.arr_expected, s1.arr_arrived, now_ms);
    // Loss episodes (fec.log): drained every tick whether or not the log is
    // open, so the decoder's closed-episode queue never fills. Stamped with
    // the op and the sid's own commanded overhead as of this tick.
    {
      const auto& fop = vrx.cur_op();
      for (int sid = 0; sid < 2; ++sid)
        for (const auto& e : agg.decoder().take_episodes(sid))
          if (fec_log)
            fec_log->row(now_ms, sid, fop.mcs,
                         sid == 0 ? fop.overhead_base : fop.overhead_enh, e);
    }
    const auto s1_sample = s1_loss.sample(now_ms);
    s1_loss_cur.add(s1.arr_expected - s1.arr_expected_stale,
                    s1.arr_arrived - s1.arr_arrived_stale, now_ms);
    const auto s1_cur_sample = s1_loss_cur.sample(now_ms);
    // Both layers pooled for the exported gauge (sin.pre_fec_loss below):
    // the enh layer's erasures are as real on air as the base layer's, and
    // the OSD's LOSS row is the pilot's view of the downlink, not of the
    // ladder's input. Summed monotonic totals keep S1LossWindow's reset
    // detection intact (both layers re-key together).
    {
      const auto s2 = agg.decoder().stats(1);
      pre_loss_all.add((s1.arr_expected - s1.arr_expected_stale) +
                           (s2.arr_expected - s2.arr_expected_stale),
                       (s1.arr_arrived - s1.arr_arrived_stale) +
                           (s2.arr_arrived - s2.arr_arrived_stale),
                       now_ms);
    }
    const auto pre_all_sample = pre_loss_all.sample(now_ms);

    // Block 4's instant-demote input: BASE post-FEC loss from the FEC
    // decoder's own abandonment count, mirroring s3_resid_cur below. See
    // gs/src/ladder_residual.cpp for why this replaced the packet-level
    // delivery window on 2026-09-02.
    const auto s1_rc = maburgs::ladder_residual_counts(agg.decoder());
    s1_resid_cur.add(s1_rc.expected, s1_rc.arrived, now_ms);
    const auto s1_rcur_sample = s1_resid_cur.sample(now_ms);

    // s3 feedback for the probe-before-promote / s3-demote logic: pre-FEC
    // loss (same shape as s1's window), scored against the CURRENT rung's
    // budget. s3_* names are historical too: this is the ENH sid's (1)
    // window — the shed-able layer the probe candidate rides.
    const auto s3 = agg.decoder().stats(1);
    s3_loss.add(s3.arr_expected, s3.arr_arrived, now_ms);
    const auto s3_sample = s3_loss.sample(now_ms);

    const uint64_t s3_ab_cur = s3.syms_abandoned - s3.syms_abandoned_stale;
    const uint64_t s3_exp_cur = s3.syms_delivered + s3.syms_recovered + s3_ab_cur;
    s3_loss_cur.add(s3.arr_expected - s3.arr_expected_stale,
                    s3.arr_arrived - s3.arr_arrived_stale, now_ms);
    s3_resid_cur.add(s3_exp_cur, s3_exp_cur - s3_ab_cur, now_ms);
    const auto s3_cur_sample = s3_loss_cur.sample(now_ms);
    const auto s3_rcur_sample = s3_resid_cur.sample(now_ms);

    // Probe window (spec 2026-09-04 section 3.3): union block counters for
    // the profile currently commanded, windowed by the same machinery as the
    // s1/s3 windows above. Blanked on a commanded-profile change so it
    // refills only from bodies carrying the new profile (RCF lag + the
    // finalize window). Per-card siblings are diagnostics only -- the ladder
    // reads the union, because that is the path production video takes.
    probe_track.tick(now_ms);
    if (const uint8_t pc = vrx.probe_profile(); pc != probe_cmd_last) {
      probe_cmd_last = pc;
      probe_track.set_commanded(pc, now_ms);
      probe_loss.blank_until(now_ms + kProbeSwitchBlankMs);
      for (auto& w : probe_card_loss) w.blank_until(now_ms + kProbeSwitchBlankMs);
      // The probe body's own airtime is the floor of the slotter's learned
      // completion->probe offset (rcf_slot.h "Probe tail"): the burst
      // cannot end sooner than one probe body after the AU completes.
      int tail = 0;
      if (pc != mabur::rc::kNoProbeProfile) {
        mabur::rc::PhyMode pm; uint8_t pmcs, pbw;
        mabur::rc::decode_profile(pc, pm, pmcs, pbw);
        const auto spec = mabur::rc::ladder_from(pm, pmcs, pbw)[1];
        const double rate = mabur::rc::phy_rate_mbps(spec);
        const double us = rate > 0 ? mabur::probe::probe_body_len(probe_bpb, probe_block_payload) * 8.0 / rate : 0.0;
        tail = static_cast<int>(std::ceil(us / 1000.0));
        if (tail < 1) tail = 1;
      }
      rcf_slot.set_probe_tail_ms(tail);
    }
    const auto& pu = probe_track.union_counts();
    probe_loss.add(pu.expected_blocks, pu.arrived_blocks, now_ms);
    for (int i = 0; i < n_cards; ++i) {
      const auto& pcnt = probe_track.card_counts(i);
      probe_card_loss[static_cast<size_t>(i)].add(pcnt.expected_blocks,
                                                  pcnt.arrived_blocks, now_ms);
    }
    const auto probe_sample = probe_loss.sample(now_ms);
    // Drain every iteration even with no log open: ProbeTrack's bounded
    // structure is its pending ring, NOT the finalized list -- that is a
    // plain std::vector it appends to and only take_finalized() clears, so
    // skipping the drain would grow it without limit for the life of the
    // process.
    if (probe_log)
      for (const auto& f : probe_track.take_finalized())
        probe_log->row(f.t_ms, f.seq, f.profile & 0x0F, f.enh_fid, f.blocks_ok,
                       f.card_mask, f.snr_db[0], f.snr_db[1], f.evm_db[0],
                       f.evm_db[1], f.first_ms);
    else
      probe_track.take_finalized();

    // The strongest card that ACTUALLY RECEIVED s1-or-s3 this feedback
    // window supplies all three RF labels. Freshness is part of the argmax,
    // not a filter after it (select_label_card, rf_labels.h): a card whose
    // front-end wedged keeps a frozen-high EMA and would otherwise outrank
    // a live sibling forever. -1 = nothing measured this window, so all
    // three labels stay NaN -- inert for the fade trigger, null on the wire.
    for (int i = 0; i < n_cards; ++i) {
      const auto& ct = agg.card(i).rf_pool;
      label_card_inputs[static_cast<size_t>(i)] = maburgs::CardLabelInput{
          ct.has_ema, ct.frames, prev_pool_frames[static_cast<size_t>(i)],
          ct.snr_ema};
    }
    const int best_card = maburgs::select_label_card(label_card_inputs);
    // SNR (label + fade input) and EVM (label only) come from that ONE card,
    // never independently-best across cards, so the three are a coherent
    // single-card snapshot of the same radio at the same instant.
    double rf_snr_db = std::numeric_limits<double>::quiet_NaN();
    double rf_evm_db = std::numeric_limits<double>::quiet_NaN();
    double rf_rssi_dbm = std::numeric_limits<double>::quiet_NaN();
    if (best_card >= 0) {
      const auto& ct = agg.card(best_card).rf_pool;
      // Raw units are devourer's half-dB (snr_units.h); raw - 110 is the
      // exporter's own dBm conversion (stats_exporter.cpp rssi keys).
      rf_snr_db = ct.snr_ema * maburgs::kSnrRawToDb;
      if (ct.evm_has) rf_evm_db = ct.evm_ema * maburgs::kEvmRawToDb;
      rf_rssi_dbm = ct.rssi_ema - 110.0;
    }

    // Every demote input reads the CURRENT-rung (attributed) value; stale
    // transition debris can no longer fire any demote. Unconditional since
    // 2026-08-15. sample_valid / s3_valid / s3_expected_syms stay
    // total-based on purpose: they gate "was there traffic at all", and an
    // all-stale window must still count as feedback (a cur-based valid
    // would un-stamp last_feedback_ms_ and could walk into the blind-side
    // timeout during a long boundary).
    maburgs::LinkHealth health{
        s1_sample.valid,
        s1_cur_sample.valid ? s1_cur_sample.loss : 0.0,
        s1_rcur_sample.valid ? s1_rcur_sample.loss : 0.0,
        starved};
    health.s3_valid = s3_sample.valid;
    health.s3_pre_fec_loss = s3_cur_sample.valid ? s3_cur_sample.loss : 0.0;
    health.s3_residual_loss =
        s3_rcur_sample.valid ? s3_rcur_sample.loss : 0.0;
    health.s3_expected_syms = s3_loss.expected_in_window(now_ms);
    // Probe gate inputs (spec 2026-09-04 section 3.3). probe_rung is the rung
    // the sample was commanded at. The h.probe_rung == pr equality the
    // controller checks against is always true in practice -- both sides
    // read probe_rung() on the same tick -- and is kept only as a cheap
    // invariant check, not the staleness guard: the real guard against a
    // sample surviving a profile switch is the 150 ms
    // probe_loss.blank_until() set at the switch edge plus mark_transition's
    // streak reset.
    health.probe_valid = probe_sample.valid;
    health.probe_loss = probe_sample.valid ? probe_sample.loss : 0.0;
    health.probe_expected_syms = probe_loss.expected_in_window(now_ms);
    // Cumulative expected bodies for the body-count clean streak. Expected
    // books bpb blocks per AU (ProbeTrack), so this is exact.
    health.probe_bodies_total = pu.expected_blocks / static_cast<uint64_t>(probe_bpb);
    health.probe_rung = vrx.ctl().probe_rung();
    health.rf_snr_db = rf_snr_db;
    health.rf_evm_db = rf_evm_db;
    health.rf_rssi_dbm = rf_rssi_dbm;
    if (auto out = vrx.step(now_ms, health)) {
      if (!out->is_disc) {
        prev_pkts_out = pkts_now;  // window == RCF period
        // The RF staleness window and the loss window MUST share this
        // boundary: both are "since the last health the controller acted on".
        for (int i = 0; i < n_cards; ++i)
          prev_pool_frames[static_cast<size_t>(i)] = agg.card(i).rf_pool.frames;
      }
      std::vector<maburgs::CardSnapshot> snaps;
      for (int i = 0; i < n_cards; ++i) {
        const auto& t = agg.card(i);
        // A scouting card is off somewhere measuring a candidate: never a
        // transmit candidate, whatever its SNR history says.
        const bool scouting = !scout_joined && i == scout_card;
        snaps.push_back(maburgs::CardSnapshot{
            !scouting && fronts[static_cast<size_t>(i)]->alive(), t.snr_ema,
            t.rssi_b_ema, t.last_frame_us});
      }
      // Hold the switch decision while the in-flight scout has a card off
      // on a candidate: the dwelling card is never the TX card by
      // construction (the scout thread always picks the NON-tx_card_now
      // card), so it could only ever become a challenger here -- and
      // sel.update() is what would act on a challenger and switch onto it
      // mid-dwell, which is exactly what must not happen (spec section 6).
      // Simplest correct fix: skip the update entirely and keep the last
      // selection for this tick. The same hold covers a hop in flight
      // (tx_selection_frozen, hop_burst_gate.h): the lead card is on the
      // target and the RCF that carries the order must keep leaving on
      // the old channel until the drone has been seen there.
      const int tx = maburgs::tx_selection_frozen(dwell_busy.load(), plan.hopping())
                         ? sel.selected()
                         : sel.update(snaps, now_ms_u * 1000);
      tx_card_now.store(sel.selected(), std::memory_order_relaxed);
      // Which card(s) carry this frame. RCFs go to the TX selector's card,
      // as they always did. A DISC is rendezvous traffic and follows the
      // plan instead: both channels of a split, the home card while the
      // scout has the other one, nothing at all while a single card is off
      // measuring a candidate (spec section 6, GS side).
      std::vector<int> targets;
      if (!out->is_disc) {
        targets.push_back(tx);
      } else if (auto bc = plan.beacon_cards()) {
        targets = *bc;
      } else if (!scout_joined) {
        // Silent during every scout dwell (two cards: quiet(); one card:
        // outside the beacon phase).
        if (!scout->quiet() && (!one_card || scout->at_home())) targets.push_back(0);
      } else {
        targets.push_back(tx);
      }
      // link-rtt: build_rcf bumped seq_, so rcf_seq() IS this frame's seq;
      // captured now because the slotter may send it later. DISCs are
      // rendezvous traffic, not RCFs — the drone never ages against them,
      // so they are not matchable sends. The 1 Hz DISC keepalive is slotted
      // like any other send (it killed a PPDU per second when it bypassed).
      // Radio silence during a calibration sweep (design "Radio silence
      // during a sweep phase"): RcfSlotter hides sends in the drone's
      // inter-AU idle, but a sweep has no AUs at all -- rcf_slot.h says
      // plainly that with no recent AU everything passes through, which
      // would put RCF and the DISC keepalive (both ride this one path)
      // straight into the drone's back-to-back sweep transmissions.
      // radio_silent() is the predicate that actually knows a sweep is
      // running, so it gates the send itself rather than trusting the
      // slotter's AU-cadence guess to have degraded safely.
      for (size_t k = 0; k < targets.size(); ++k) {
        maburgs::SlotFrame sf{
            k + 1 == targets.size() ? std::move(out->frame) : out->frame,
            vrx.rcf_seq(), targets[k], !out->is_disc};
        if (!rcf_slot.offer(sf, drained_ms, false) &&
            !cal_session.radio_silent(drained_ms))
          send_control_frame(sf);
      }
    }
    // Slotted sends whose hold ended (an AU completed in this iteration's
    // drain, or the hold timed out). Same gate as above: a calibration
    // sweep leaves nothing for the slotter to hold against, so anything
    // still queued from before the sweep started must not go out either.
    for (const auto& f : rcf_slot.take_due(drained_ms))
      if (!cal_session.radio_silent(drained_ms)) send_control_frame(f);
    // Calibration uplink (T_CAL_CMD / T_CAL_RESULT): straight through
    // send_control_frame, bypassing rcf_slot entirely -- there is no video
    // for the slotter to hide a send behind during a sweep, and
    // radio_silent() already knows the drone's listen windows precisely
    // from the plan the GS itself sent, a tighter answer than the
    // slotter's AU-cadence guess. Gated the same way as every other
    // transmit above: nothing goes out while a sweep phase is running.
    if (!cal_session.radio_silent(drained_ms)) {
      if (auto cmd = cal_session.due_cmd(drained_ms)) {
        cal_pending_nonce = cmd->nonce;
        maburgs::SlotFrame cf{mabur::rc::pack_cal_cmd(*cmd), 0, sel.selected(),
                              false};
        cf.offered_ms = drained_ms;
        send_control_frame(cf);
      }
    }
    // T_CAL_RESULT is the one transmit NOT gated on radio_silent(), and
    // deliberately so: it must be repeated into the verify window until
    // the drone's first verify frame acks it (cal_session.h's due_result()
    // comment), and radio_silent() is true for that whole window. The
    // invariant still holds -- CalSession stops vending repeats the
    // instant a verify-phase frame arrives, and before that arrives the
    // drone has not applied and is not sweeping anything. Losing this one
    // frame to the 30-50%-lossy uplink otherwise ends the run with the
    // config untouched and the report claiming it was written.
    if (auto res = cal_session.due_result(drained_ms)) {
      maburgs::SlotFrame rf{mabur::rc::pack_cal_result(*res), 0,
                            sel.selected(), false};
      rf.offered_ms = drained_ms;
      send_control_frame(rf);
    }
    // ctl: rung transition line — load-bearing for post-mortems (Task 6
    // adds the sideport link.ctl block; this stderr line is independent of
    // it and persists in /tmp/maburgs.log even when no sideport consumer is
    // listening).
    if (const auto& e = vrx.ctl().last_event(); e.t_ms != last_ctl_event_ms) {
      last_ctl_event_ms = e.t_ms;
      std::fprintf(stderr, "ctl: rung %d->%d reason=%s u=%.2f pre=%.3f\n",
                   e.from, e.to, maburgs::to_string(e.reason), e.u,
                   health.pre_fec_loss);
      if (ctl_log)
        ctl_log->event(e.t_ms, e.from, e.to, maburgs::to_string(e.reason),
                        e.u, e.snr_db, e.evm_db);
      if (ctl_log) emit_rung_lines(now_ms);  // store state at the decision
    }
    // Probe gate EDGE records (ctllog 10): same t_ms-change detect pattern as
    // the rung-transition line above. Off edges are not logged -- the gate
    // leaving/entering Off just tracks whether a candidate rung exists at
    // all (top rung, feature disabled), which the S line's probe_rung column
    // already carries once per dwell sample. Also skip rung < 0: promoting
    // onto the top rung has no candidate rung to probe, and logging it would
    // read on flightreport as a phantom rung -1.
    if (const auto& pe = vrx.ctl().last_probe_edge(); pe.t_ms != last_probe_t_ms) {
      last_probe_t_ms = pe.t_ms;
      if (ctl_log && pe.state != maburgs::ProbeGateState::Off && pe.rung >= 0)
        ctl_log->probe(pe.t_ms, pe.rung, maburgs::to_string(pe.state), pe.snr_db,
                        pe.u, static_cast<int>(pe.prev_dur_ms), pe.evm_db);
    }
    if (const auto& n = vrx.ctl().last_penalty(); n.t_ms != last_penalty_t_ms) {
      last_penalty_t_ms = n.t_ms;
      if (ctl_log) ctl_log->penalty(n.t_ms, n.rung, n.k, n.until_ms);
    }

    // SIGUSR1 is consumed ONCE and shared: both blocks below honour it, so a
    // dump still produces an off-cadence S line AND a stderr line the way it
    // did when the two were one block.
    const bool dump_now = g_dump.exchange(false);

    // Adaptive-link log, on its OWN cadence (debug_log.ctl_period_ms, default
    // 1000). Split from the stderr block below on 2026-08-15: the ctl log is
    // the instrument the ladder is tuned from and wants to run as fast as
    // 50 ms, while the stderr line is human-readable and lands in /tmp, which
    // is tmpfs — running that at 20 Hz would fill RAM for no one's benefit.
    if (ctl_log &&
        (dump_now || now_ms_u - last_ctl_sample_ms >= static_cast<uint64_t>(
                                                          cfg.debug_log.ctl_period_ms))) {
      last_ctl_sample_ms = now_ms_u;
      const auto& c = vrx.ctl();
      // measured_rung(), not rung(): every other field in this row is a
      // window measurement, and a demote has already stepped the live rung
      // down by the time we get here (see LadderController::measured_rung()).
      // Probe columns: the gate's candidate rung, its scored utilization
      // (NaN unless the gate actually has a verdict) and the window's block
      // count, so a post-mortem can tell "clean probe" from "no probe data".
      const auto pg = vrx.ctl().probe_gate(now_ms);
      ctl_log->sample(now_ms, c.measured_rung(), c.util(), health.rf_snr_db,
                       residual.value_or(0.0), c.util3(),
                       health.s3_residual_loss, health.rf_evm_db,
                       residual_cur.value_or(0.0), c.fade_drssi(),
                       c.fade_dsnr(), health.rf_rssi_dbm, pg.rung,
                       (pg.state == maburgs::ProbeGateState::Clean ||
                        pg.state == maburgs::ProbeGateState::Lossy)
                           ? pg.u
                           : std::numeric_limits<double>::quiet_NaN(),
                       probe_loss.expected_in_window(now_ms));
      // R lines keep their own, much slower period — they are a store
      // snapshot, not a dwell sample, and must not follow the S cadence.
      if (now_ms - last_rung_log_ms >= cfg.debug_log.rung_period_s * 1000.0) {
        last_rung_log_ms = now_ms;
        emit_rung_lines(now_ms);
      }
    }

    // 1 Hz stats line / SIGUSR1 dump.
    if (dump_now || now_ms_u - last_stats_ms >= 1000) {
      last_stats_ms = now_ms_u;
      if (msp_sink) msp_sink->tick(now_ms_u);  // expire stale repair rows
      const auto& op = vrx.cur_op();
      std::fprintf(stderr,
                   "stats: state=%d tx_card=%d op=mcs%d/%d/ov%.2f "
                   "ring=%llu ring_drop=%llu q_drop=%llu",
                   static_cast<int>(vrx.link_state()), sel.selected(), op.mcs,
                   op.bw, op.overhead_base,
                   static_cast<unsigned long long>(au_ring.published()),
                   static_cast<unsigned long long>(au_ring.dropped_oversize()),
                   static_cast<unsigned long long>(queue.dropped()));
      for (int i = 0; i < n_cards; ++i) {
        const auto& t = agg.card(i);
        std::fprintf(stderr, " c%d[%s f=%llu cf=%llu snr=%.1f a=%.1f b=%.1f]",
                     i, fronts[static_cast<size_t>(i)]->alive() ? "up" : "DOWN",
                     static_cast<unsigned long long>(t.frames),
                     static_cast<unsigned long long>(t.crc_fail), t.snr_ema,
                     t.snr_a_ema, t.snr_b_ema);
      }
      for (int s = 0; s < 2; ++s) {
        const auto st = agg.decoder().stats(s);
        if (st.bodies == 0) continue;  // idle streams: keep the line short
        std::fprintf(stderr,
                     " s%d[p=%llu abn=%llu rec=%llu ra=%llu si=%llu st=%llu"
                     " bc=%llu sbf=%llu cor=%llu sal=%llu fl=%zu]",
                     s, static_cast<unsigned long long>(st.packets_out),
                     static_cast<unsigned long long>(st.syms_abandoned),
                     static_cast<unsigned long long>(st.syms_recovered),
                     static_cast<unsigned long long>(st.syms_recovered_arrived),
                     static_cast<unsigned long long>(st.symbols_in),
                     static_cast<unsigned long long>(st.symbols_stale),
                     static_cast<unsigned long long>(st.symbols_bad_cfg),
                     static_cast<unsigned long long>(st.subblocks_failed),
                     static_cast<unsigned long long>(st.bodies_corrupt),
                     static_cast<unsigned long long>(st.subblocks_salvaged),
                     st.rows_in_flight);
      }
      std::fprintf(stderr, " mis=%llu",
                   static_cast<unsigned long long>(agg.decoder().bodies_misrouted()));
      std::fprintf(stderr, " frames[clean/trunc/drop]=%llu/%llu/%llu badfrag=%llu stall=%llu",
                   static_cast<unsigned long long>(fstream.frames_clean()),
                   static_cast<unsigned long long>(fstream.frames_truncated()),
                   static_cast<unsigned long long>(fstream.frames_dropped()),
                   static_cast<unsigned long long>(fstream.bad_fragments()),
                   static_cast<unsigned long long>(fstream.stall_resets()));
      // Only while a one-card scout is actually costing us sends.
      if (scout_gated_sends)
        std::fprintf(stderr, " scoutgate=%llu",
                     static_cast<unsigned long long>(scout_gated_sends));
#ifdef MABUR_LOSS_SIM
      if (agg.loss_sim().enabled())
        std::fprintf(stderr, " LOSS-SIM[s0/s1/s2/s3/s4/s5]=%llu/%llu/%llu/%llu/%llu/%llu",
                     static_cast<unsigned long long>(agg.loss_sim().dropped(0)),
                     static_cast<unsigned long long>(agg.loss_sim().dropped(1)),
                     static_cast<unsigned long long>(agg.loss_sim().dropped(2)),
                     static_cast<unsigned long long>(agg.loss_sim().dropped(3)),
                     static_cast<unsigned long long>(agg.loss_sim().dropped(4)),
                     static_cast<unsigned long long>(agg.loss_sim().dropped(5)));
#endif
      std::fprintf(stderr, "\n");
    }

    if (stats) {
      maburgs::StatsInput sin;
      sin.vtx_id = cfg.link.vtx_id;
      // Live channel of the TX card, not the configured home (spec
      // section 7): with a pick committed they differ. Straight off the
      // front-end's atomic -- cur_ch is deliberately untracked for the
      // scout card while the scout owns it.
      sin.channel = fronts[static_cast<size_t>(sel.selected())]->channel();
      sin.home = cfg.radio.channel;
      // The scout, not the plan: the plan is still unfrozen after the
      // scout-card-death path abandons the scan, and with scan.enable
      // false there is no scout at all.
      sin.scan_state = (!scfg.enable || !scout)
                           ? "off"
                           : (scout->frozen() ? "frozen" : "scouting");
      sin.scan_rounds = scout ? scout->rounds() : 0;
      if (plan.frozen()) sin.scan_pick = plan.op();
      // In-flight channel hop snapshot (Task 12): straight off
      // HopController's own accessors + the latest HopVerdict output --
      // same no-controller-reference pattern as sin.ctl further down.
      sin.hop.enable = hcfg.enable;
      sin.hop.verdict = maburgs::to_string(last_verdict_out.v);
      sin.hop.evidence = last_verdict_out.evidence;
      if (last_verdict_out.ref_rung >= 0) sin.hop.ref_rung = last_verdict_out.ref_rung;
      sin.hop.epoch = hopc.epoch();
      sin.hop.state = hop_state_name(hopc.state());
      // hop_ch() is 0 sentinel ("never ordered a hop yet") until the first
      // order; once set it never reverts to 0 again (withdraw sets it to
      // the pre-attempt point, not to the sentinel), so this is null only
      // for a session where the hop feature has never fired.
      if (const uint8_t hc = hopc.hop_ch(); hc != 0) sin.hop.target = hc;
      sin.hop.hops = hopc.hops();
      sin.hop.holds = hopc.holds();
      sin.hop.last_ms = last_hop_event_ms;
      sin.in_session = in_session;
      sin.tx_card = sel.selected();
      sin.op = vrx.cur_op();
      for (int s = 0; s < 2; ++s)
        sin.gap_timeout_ms[s] = gap_policy.timeout_ms(s);
      sin.residual_loss = residual;
      sin.residual_cur = residual_cur;
      // The pooled base+enh window (pre_loss_all), exported unconditionally:
      // static-pin mode never fills sin.ctl below, and the OSD's pre-FEC
      // LOSS figure has to come from somewhere. Left empty on an invalid
      // window rather than defaulted to 0.0 the way LinkHealth does it -- a
      // controller needs a number every tick, a gauge does not, and "no
      // sample" must not render as a real zero-loss link. The ladder's own
      // base-only sample still goes out as link.ctl.pre_fec_loss.
      if (pre_all_sample.valid) sin.pre_fec_loss = pre_all_sample.loss;
      if (const double cms = agg.decoder().last_boundary_close_ms(0); cms >= 0)
        sin.attrib_close_ms = cms;
      for (int s = 0; s < 2; ++s)
        sin.layer_delivery_pct[static_cast<size_t>(s)] = ld[static_cast<size_t>(s)];
      for (int i = 0; i < n_cards; ++i) {
        const auto& t = agg.card(i);
        maburgs::StatsCardIn ci;
        ci.up = fronts[static_cast<size_t>(i)]->alive();
        ci.frames = t.frames;
        ci.crc_fail = t.crc_fail;
        ci.seq_expected = t.seq_expected;
        ci.seq_received = t.seq_received;
        ci.rx_bytes = t.rx_bytes;
        ci.last_frame_us = t.last_frame_us;
        ci.self_frames = t.self_frames;
        ci.foreign = fronts[static_cast<size_t>(i)]->foreign();
        ci.tx_frames = fronts[static_cast<size_t>(i)]->tx_frames();
        ci.tx_fail = fronts[static_cast<size_t>(i)]->tx_fail();
        ci.energy = energy_last[static_cast<size_t>(i)];  // last A sample
        ci.dwell = dwell_stats[static_cast<size_t>(i)];  // last in-flight scout dwell
        static_assert(maburgs::kNumStatsClasses == maburgs::kNumRfClasses,
                      "class arrays must stay in lockstep");
        for (int k = 0; k < maburgs::kNumStatsClasses; ++k) {
          auto& cls = ci.classes[static_cast<size_t>(k)];
          const auto& tcls = t.cls[static_cast<size_t>(k)];
          cls.frames = tcls.frames;
          cls.bytes = tcls.bytes;
          cls.has_ema = tcls.has_ema;
          cls.rssi_ema = tcls.rssi_ema;
          cls.rssi_a_ema = tcls.rssi_a_ema;
          cls.rssi_b_ema = tcls.rssi_b_ema;
          cls.snr_ema = tcls.snr_ema;
          cls.snr_a_ema = tcls.snr_a_ema;
          cls.snr_b_ema = tcls.snr_b_ema;
          cls.evm_ema = tcls.evm_ema;
          cls.evm_a_ema = tcls.evm_a_ema;
          cls.evm_b_ema = tcls.evm_b_ema;
          cls.evm_has = tcls.evm_has;
          cls.evm_a_has = tcls.evm_a_has;
          cls.evm_b_has = tcls.evm_b_has;
        }
        sin.cards.push_back(ci);
      }
      for (int s = 0; s < 2; ++s) {
        const auto st = agg.decoder().stats(s);
        auto& o = sin.streams[static_cast<size_t>(s)];
        o.bodies = st.bodies;
        o.subblocks_failed = st.subblocks_failed;
        o.bodies_corrupt = st.bodies_corrupt;
        o.subblocks_salvaged = st.subblocks_salvaged;
        o.arr_salvage_only = st.arr_salvage_only;
        o.syms_recovered = st.syms_recovered;
        o.syms_recovered_arrived = st.syms_recovered_arrived;
        o.syms_abandoned = st.syms_abandoned;
        o.syms_abandoned_stale = st.syms_abandoned_stale;
        o.symbols_in = st.symbols_in;
        o.symbols_stale = st.symbols_stale;
        o.symbols_bad_cfg = st.symbols_bad_cfg;
        o.rows_in_flight = st.rows_in_flight;
        o.arr_expected = st.arr_expected;
        o.arr_arrived = st.arr_arrived;
        o.arr_expected_stale = st.arr_expected_stale;
        o.arr_arrived_stale = st.arr_arrived_stale;
        o.arr_late = st.arr_late;
      }
      sin.frames_clean = fstream.frames_clean();
      sin.frames_truncated = fstream.frames_truncated();
      sin.frames_dropped = fstream.frames_dropped();
      sin.stall_resets = fstream.stall_resets();
      sin.ring_published = au_ring.published();
      sin.ring_dropped_oversize = au_ring.dropped_oversize();
      sin.ring_bytes = au_ring.bytes_published();
      sin.q_drop = queue.dropped();
      sin.telem = latest_telem.t;
      sin.telem_rx_ms = latest_telem.rx_ms;
      sin.rcf_slot = {rcf_slot.released_au(), rcf_slot.released_timeout(),
                      rcf_slot.passthru(), rcf_slot.released_probe(),
                      rcf_slot.tail_ub_ms()};
      // link-rtt block. floor via floor_us_from (pts_anchor.h), which owns
      // the 32-bit-seed vs 64-bit-MI-domain wrap rule.
      if (rtt_est.has_rtt()) {
        maburgs::StatsRttIn ri;
        ri.rtt_ms = rtt_est.rtt_ms();
        ri.rtt_min_ms = rtt_est.rtt_min_ms();
        ri.n = rtt_est.samples();
        if (rtt_est.has_offset()) {
          ri.pts_off_us = rtt_est.pts_off_us();
          if (lat_anchor.usable())
            ri.floor_ms = static_cast<double>(maburgs::floor_us_from(
                              lat_anchor.base_us(), rtt_est.pts_off_us())) /
                          1000.0;
        }
        sin.rtt = ri;
      }
      // lat_win.flush() is destructive (reads AND clears): only pay for it
      // on a poll that stats->due() says will actually emit. This loop
      // iterates roughly every 10ms (the drain() cadence above) while
      // interval_ms is >= 100ms, so an unconditional flush() here would
      // clear the window ~10x more often than it is ever read, reporting
      // only the last loop tick's handful of samples instead of a real
      // rolling window.
      if (lat_anchor.usable() && stats->due(drained_ms))
        sin.video_lat = lat_win.flush();
      // Ladder controller snapshot: absent in static-pin mode, where the
      // controller exists but is never ticked (see VrxController::ctl()).
      if (cfg.link.static_mcs < 0) {
        const auto& c = vrx.ctl();
        maburgs::StatsCtlIn ci;
        ci.rung_idx = c.rung();
        ci.rung_mcs = c.op().mcs;
        ci.rung_ov_base = c.op().overhead_base;
        ci.rung_ov_enh = c.op().overhead_enh;
        ci.util = c.util();
        ci.pre_fec_loss = c.pre_fec_loss();
        ci.budget = c.budget_base();
        ci.probation_ms_left = c.probation_ms_left(now_ms);
        for (const auto& p : c.penalized(now_ms)) ci.penalized.push_back(p);
        for (const auto& r : cfg.link.ladder_cfg.ladder)
          ci.ladder.emplace_back(r.mcs, r.overhead_base, r.overhead_enh);
        ci.down_util = cfg.link.ladder_cfg.down_util;
        ci.up_util = cfg.link.ladder_cfg.up_util;
        const auto& cnt = c.counters();
        ci.demotes_residual = cnt.demotes_residual;
        ci.demotes_util = cnt.demotes_util;
        ci.promotes = cnt.promotes;
        ci.probation_fails = cnt.probation_fails;
        ci.starved_drops = cnt.starved_drops;
        ci.timeout_drops = cnt.timeout_drops;
        const auto& e = c.last_event();
        ci.last_event_t_ms = e.t_ms;
        ci.last_event_from = e.from;
        ci.last_event_to = e.to;
        ci.last_event_reason = maburgs::to_string(e.reason);
        ci.last_event_u = e.u;
        ci.last_event_snr_db = e.snr_db;
        ci.last_event_evm_db = e.evm_db;
        ci.util3 = c.util3();
        ci.promotes_probed = cnt.promotes_probed;
        ci.probe_holds = cnt.probe_holds;
        ci.demotes_s3_residual = cnt.demotes_s3_residual;
        ci.demotes_s3_util = cnt.demotes_s3_util;
        ci.demotes_fade = cnt.demotes_fade;
        ci.fade_active = c.fade_active(now_ms);
        ci.fade_drssi = c.fade_drssi();
        ci.fade_dsnr = c.fade_dsnr();
        const maburgs::RungStore& rstore = c.rungs();
        for (std::size_t ri = 0; ri < rstore.size(); ++ri) {
          const maburgs::RungStat& rs = rstore.stat(static_cast<int>(ri));
          maburgs::StatsRungIn rg;
          rg.mcs = cfg.link.ladder_cfg.ladder[ri].mcs;
          rg.ov_base = cfg.link.ladder_cfg.ladder[ri].overhead_base;
          rg.ov_enh = cfg.link.ladder_cfg.ladder[ri].overhead_enh;
          rg.u = rs.u.v;
          rg.resid = rs.resid.v;
          rg.u3 = rs.u3.v;
          rg.resid3 = rs.s3_resid.v;
          rg.evm_db = rs.evm_db;
          rg.evm_sd_db = rs.evm_n
                              ? std::sqrt(rs.evm_var_db2)
                              : std::numeric_limits<double>::quiet_NaN();
          rg.n = rs.u.n;
          rg.probe_n = rs.probe_u.n;
          rg.age_s = rs.last_sample_ms < 0
                          ? -1.0
                          : (now_ms - rs.last_sample_ms) / 1000.0;
          rg.probe_age_s = rs.last_probe_ms < 0
                                ? -1.0
                                : (now_ms - rs.last_probe_ms) / 1000.0;
          rg.dwell_s = rstore.dwell_ms(static_cast<int>(ri), now_ms) / 1000.0;
          rg.visits = rs.visits;
          rg.exits_bad = rs.exits_bad;
          rg.probe_u = rs.probe_u.v;
          ci.rungs.push_back(rg);
        }
        sin.ctl = std::move(ci);
      }
      // Probe snapshot: OUTSIDE the ladder block on purpose, so static-pin
      // mode (link.probe.pin_mcs on the bench) still exports what the probe
      // stream is doing. In that mode the controller is never ticked, so
      // `state` stays "off" and `u`/`loss` export as JSON null (have_sample
      // needs a non-Off gate). `rung` is NOT -1 there: pinning does not
      // disable link.probe.enable, so probe_rung() off a frozen idx_ == 0
      // reports min(probe.rung_offset, top). The informative fields in pin
      // mode are `on`, `mcs`, `n`, `exp`, `rx`, `off_profile` and the
      // per-card rows -- cards[].loss has its own validity flag and does
      // not depend on the gate.
      {
        maburgs::StatsProbeIn pin;
        const uint8_t pc = vrx.probe_profile();
        pin.on = pc != mabur::rc::kNoProbeProfile;
        pin.mcs = pin.on ? (pc & 0x0F) : -1;
        const auto g = vrx.ctl().probe_gate(now_ms);
        pin.rung = g.rung;
        pin.state = maburgs::to_string(g.state);
        pin.have_sample =
            probe_sample.valid && g.state != maburgs::ProbeGateState::Off;
        pin.u = g.u;
        pin.loss = probe_sample.valid ? probe_sample.loss : 0.0;
        pin.streak_bodies = g.streak_bodies;
        pin.n = probe_loss.expected_in_window(now_ms);
        pin.exp = pu.expected_blocks;
        pin.rx = pu.bodies_rx;
        pin.off_profile = probe_track.off_profile();
        for (int i = 0; i < n_cards; ++i) {
          const auto cs = probe_card_loss[static_cast<size_t>(i)].sample(now_ms);
          pin.cards.push_back({cs.valid, cs.valid ? cs.loss : 0.0,
                               probe_track.card_counts(i).bodies_rx});
        }
        sin.probe = std::move(pin);
      }
      stats->poll(drained_ms, sin);
    }
  }
  // Shutdown: both scout threads must be gone before the cards they drive
  // are stopped and destroyed. The in-flight scout's loop wakes at most
  // dwell_period_ms after scout_run clears; join it first since it can
  // touch either card, then the boot scout (freeze() ends its loop at the
  // end of the current dwell).
  scout_run.store(false);
  if (inflight_started && scout_thread2.joinable()) scout_thread2.join();
  join_scout(plan.op());
  queue.close();
  for (auto& fe : fronts) fe->stop();
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = "/etc/maburgs.toml";
  std::string in_path, out_aus_path;
  bool dry_run = false;
#ifdef MABUR_LOSS_SIM
  int loss_sim_port = 0;
#endif
  maburgs::FrameFileSource::Options src_opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-c" && i + 1 < argc) config_path = argv[++i];
    else if (a == "--dry-run") dry_run = true;
    else if (a == "--in" && i + 1 < argc) in_path = argv[++i];
    else if (a == "--cards" && i + 1 < argc) src_opt.cards = std::atoi(argv[++i]);
    else if (a == "--drop-pct" && i + 1 < argc) src_opt.drop_pct = std::atoi(argv[++i]);
    else if (a == "--seed" && i + 1 < argc) src_opt.seed = static_cast<uint32_t>(std::atol(argv[++i]));
    else if (a == "--out-aus" && i + 1 < argc) out_aus_path = argv[++i];
#ifdef MABUR_LOSS_SIM
    else if (a == "--loss-sim") {
      loss_sim_port = 8302;
      if (i + 1 < argc && argv[i + 1][0] != '-') {
        const int p = std::atoi(argv[i + 1]);
        if (p > 0 && p < 65536) { loss_sim_port = p; ++i; }
      }
    }
#endif
    else if (a == "-h" || a == "--help") { usage(); return 0; }
    else { std::fprintf(stderr, "error: unknown arg %s\n", a.c_str()); usage(); return 2; }
  }

  if (!dry_run) {
    // real-radio mode: load config, then run. (Branches off BEFORE the
    // dry-run-only arg checks, exactly where the Plan-1 stub sat.)
    maburgs::Config cfg;
    std::vector<std::string> defaulted;
    try { cfg = maburgs::load_config(config_path, &defaulted); }
    catch (const std::exception& e) { std::fprintf(stderr, "error: %s\n", e.what()); return 2; }
    if (!defaulted.empty()) {
      std::fprintf(stderr, "config: %zu key(s) defaulted:\n", defaulted.size());
      for (const std::string& d : defaulted)
        std::fprintf(stderr, "  %s\n", d.c_str());
    }
#ifdef MABUR_LOSS_SIM
    return run_radio(cfg, loss_sim_port);
#else
    return run_radio(cfg);
#endif
  }

  // ---- dry-run path: MUST be byte-identical to Plan 1 ----
  if (in_path.empty()) { usage(); return 2; }
  if (src_opt.cards < 1) {
    std::fprintf(stderr, "error: --cards must be >= 1\n");
    usage();
    return 2;
  }

  maburgs::Config cfg;
  try {
    cfg = maburgs::load_config(config_path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }

  maburgs::FrameFileSource src(in_path, src_opt);
  if (!src.ok()) {
    std::fprintf(stderr, "error: cannot read %s\n", in_path.c_str());
    return 2;
  }

  const int n_cards = src_opt.cards;
  maburgs::Aggregator agg(cfg.uep_layers(),
                          static_cast<uint32_t>(cfg.fec.seq_horizon), n_cards,
                          static_cast<uint32_t>(cfg.link.arrival_guard_syms));
  AuFileOut file_out;
  if (!out_aus_path.empty() && !file_out.open(out_aus_path.c_str())) {
    std::fprintf(stderr, "error: cannot write %s\n", out_aus_path.c_str());
    return 2;
  }
  maburgs::AuRingWriter au_ring;
  maburgs::AuDoorbell au_bell;
  bool au_on = false;
  if (cfg.au_ring.enable) {
    const maburgs::AuRingGeom geom{
        static_cast<uint32_t>(cfg.au_ring.slot_kb) * 1024u,
        static_cast<uint32_t>(cfg.au_ring.slot_count)};
    au_on = au_ring.open(cfg.au_ring.path, geom);
    if (au_on && !au_bell.open(cfg.au_ring.socket, au_ring.geom()))
      std::fprintf(stderr, "warning: au_ring doorbell %s unusable\n",
                   cfg.au_ring.socket.c_str());
    if (!au_on)
      std::fprintf(stderr, "warning: au_ring %s unusable; disabled\n",
                   cfg.au_ring.path.c_str());
  }

  // Same video tail as run_radio (fragments -> FrameStream -> AU records),
  // so a replay exercises the real assembly rather than a dry-run-only
  // shortcut. --out-aus captures each reassembled AU as an LP record for
  // the e2e's NAL-exact comparison (tests/integration/verify_aus.py). No
  // session negotiation here: the input file IS the drone's own output.
  maburgs::FrameStream fstream(
      {static_cast<uint64_t>(cfg.video.frame_gap_timeout_ms),
       cfg.video.frame_lookahead},
      {[&](const mabur::framewire::FrameHdr& h, uint8_t sid) {
         if (au_on) au_ring.begin(h, sid);
         file_out.begin(h, sid);
       },
       [&](const uint8_t* d, size_t n) {
         if (au_on) au_ring.append(d, n);
         file_out.append(d, n);
       },
       [&](bool c, const maburgs::AuLatMeta& lat) {
         if (au_on) {
           const uint64_t rec = au_ring.finish(c, lat);
           if (rec != UINT64_MAX) au_bell.notify(rec);
         }
         file_out.finish(c);
       }});
  uint64_t replay_ms = 0;  // clock of the body being fed, for gap timeouts
  agg.set_frag_sink([&](const mabur::DecodedFrag& f) {
    fstream.push_fragment(f.stream_id, f.frag.data(), f.frag.size(), replay_ms);
  });
  uint64_t rc_frames = 0;
  agg.set_rc_sink([&](uint8_t, const std::vector<uint8_t>&, uint64_t) { ++rc_frames; });

  uint64_t last_ms = 0;
#ifdef MABUR_TEST
  // Captured for run_hop_inject_test()'s confirm step: any body out of this
  // fixture is genuine video (this IS the frame stream, not an RC frame),
  // so the first one seen is a fine template to replay on a different card.
  mabur::node::RxBody hop_confirm_template;
  bool hop_confirm_captured = false;
#endif
  while (auto m = src.next()) {
    replay_ms = m->mono_us / 1000;
    agg.on_rx_body(*m);
#ifdef MABUR_TEST
    if (!hop_confirm_captured) { hop_confirm_template = *m; hop_confirm_captured = true; }
#endif
    const uint64_t now_ms = m->mono_us / 1000;
    fstream.poll(now_ms);
    if (au_on) au_bell.poll();
    last_ms = now_ms;
  }
  // Let FrameStream time out whatever is still half-assembled (its gap timeout
  // is what turns an unrecoverable hole into a truncated frame).
  fstream.poll(last_ms + static_cast<uint64_t>(cfg.video.frame_gap_timeout_ms) +
               1);

  if (au_on)
    std::fprintf(stderr, "au_ring: published=%llu dropped_oversize=%llu\n",
                 static_cast<unsigned long long>(au_ring.published()),
                 static_cast<unsigned long long>(au_ring.dropped_oversize()));

  std::fprintf(stderr, "frames=%llu dropped=%llu malformed=%llu rc=%llu bad_card=%llu\n",
               static_cast<unsigned long long>(src.frames_read()),
               static_cast<unsigned long long>(src.dropped()),
               static_cast<unsigned long long>(src.malformed()),
               static_cast<unsigned long long>(rc_frames),
               static_cast<unsigned long long>(agg.bad_card_msgs()));
  for (int c = 0; c < n_cards; ++c) {
    const auto& t = agg.card(c);
    std::fprintf(stderr,
                 "card %d: frames=%llu crc_fail=%llu video=%llu seq %llu/%llu "
                 "rssiA=%.1f rssiB=%.1f snr=%.1f\n",
                 c, static_cast<unsigned long long>(t.frames),
                 static_cast<unsigned long long>(t.crc_fail),
                 static_cast<unsigned long long>(t.video_bodies),
                 static_cast<unsigned long long>(t.seq_received),
                 static_cast<unsigned long long>(t.seq_expected), t.rssi_a_ema,
                 t.rssi_b_ema, t.snr_ema);
  }
  for (int s = 0; s < 2; ++s) {
    const auto st = agg.decoder().stats(s);
    std::fprintf(stderr,
                 "stream %d: bodies=%llu corrupt=%llu sub_fail=%llu "
                 "salvaged=%llu salvage_only=%llu rec=%llu abn=%llu pkts=%llu "
                 "delivery=%d%%\n",
                 s, static_cast<unsigned long long>(st.bodies),
                 static_cast<unsigned long long>(st.bodies_corrupt),
                 static_cast<unsigned long long>(st.subblocks_failed),
                 static_cast<unsigned long long>(st.subblocks_salvaged),
                 static_cast<unsigned long long>(st.arr_salvage_only),
                 static_cast<unsigned long long>(st.syms_recovered),
                 static_cast<unsigned long long>(st.syms_abandoned),
                 static_cast<unsigned long long>(st.packets_out),
                 maburgs::delivery_pct(
                     maburgs::residual_counts(agg.decoder(), s, false)));
  }
  std::fprintf(stderr,
               "frames_out: clean=%llu truncated=%llu dropped=%llu bad_frag=%llu\n",
               static_cast<unsigned long long>(fstream.frames_clean()),
               static_cast<unsigned long long>(fstream.frames_truncated()),
               static_cast<unsigned long long>(fstream.frames_dropped()),
               static_cast<unsigned long long>(fstream.bad_fragments()));
  if (!out_aus_path.empty())
    std::fprintf(stderr, "aus_out=%llu (file)\n",
                 static_cast<unsigned long long>(file_out.written));
#ifdef MABUR_TEST
  if (std::getenv("MABUR_HOP_INJECT")) {
    if (!hop_confirm_captured) {
      std::fprintf(stderr, "hop-test: no bodies read from --in; nothing to confirm with\n");
      return 2;
    }
    const int rc = run_hop_inject_test(cfg, n_cards, agg, hop_confirm_template);
    if (rc != 0) return rc;
  }
#endif
  return 0;
}
