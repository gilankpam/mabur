#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "mtest.h"
#include "channel_core.h"
#include "fake_link_card.h"
#include "mabur/rc_proto.h"
#include "recording_sink.h"
#include "vrx_cfg.h"
using namespace maburgs;

static Config bundle() {
  return load_config(MABUR_SOURCE_DIR "/gs/bundle/maburgs.default.toml");
}

// A rig: N fake cards (USB first, then relays), the bundle config, a
// VrxController, a recording sink, an injected clock, no threads.
struct Rig {
  FakeClock clk;
  Config cfg = bundle();
  std::vector<std::unique_ptr<FakeCard>> cards;
  std::vector<LinkCard*> ptrs;
  RecordingSink sink;
  std::vector<uint8_t> stored;
  bool store_ok = true;
  std::unique_ptr<VrxController> vrx;
  std::unique_ptr<ChannelCore> core;
  Aggregator agg;
  // {may_send(0), may_send(1)} sampled from inside the scout's sleep
  // callback -- the synchronous equivalent of the core thread reading the
  // gate while the scout thread is mid-dwell on another thread in prod.
  std::vector<std::pair<bool, bool>> gate_obs;

  Rig(int n_usb, int n_relays, bool pinned = false, uint8_t start = 0)
      : agg(bundle().uep_layers(), 32, n_usb + n_relays, 0) {
    cfg.radio.channels = {40, 64, 112, 144};
    cfg.radio.width = 40;
    cfg.hop.enable = true;
    if (pinned) cfg.radio.pin = start ? start : 40;
    const uint8_t start_ch = start ? start : (pinned ? *cfg.radio.pin : 40);
    for (int i = 0; i < n_usb + n_relays; ++i) {
      auto c = std::make_unique<FakeCard>();
      c->clk = &clk; c->ch = start_ch; c->relay = i >= n_usb;
      // main.cpp: the boot scout card opens at 20, every other card at radio.width
      c->width_mhz = (i == n_usb - 1 && n_usb >= 1) ? 20 : cfg.radio.width;
      ptrs.push_back(c.get());
      cards.push_back(std::move(c));
    }
    vrx = std::make_unique<VrxController>(vrx_cfg_from(cfg, start_ch));
    ChannelCoreCfg cc;
    cc.radio = cfg.radio; cc.hop = cfg.hop; cc.key = cfg.link.key;
    cc.start_ch = start_ch; cc.n_usb = n_usb; cc.threaded = false;
    core = std::make_unique<ChannelCore>(
        cc, ptrs, *vrx, sink,
        [this](uint8_t ch) { stored.push_back(ch); return store_ok; },
        [this] { return clk.now_ms(); }, [this] { return clk.now_us(); },
        [this](int ms) {
          clk.sleep(ms);
          if (core) gate_obs.emplace_back(core->may_send(0), core->may_send(1));
        });
  }
  ChannelTickIn in(bool session, bool cal = false, int tx = 0) {
    ChannelTickIn i; i.now_ms = static_cast<double>(clk.ms); i.in_session = session;
    i.cal_running = cal; i.tx_card = tx; i.agg = &agg; return i;
  }
  ChannelTickOut tick(bool session = false, bool cal = false, int tx = 0, int step_ms = 10) {
    const auto out = core->tick(in(session, cal, tx));
    clk.ms += static_cast<uint64_t>(step_ms);
    return out;
  }
};

TEST(constructs_two_cards_auto_scouting) {
  Rig g(2, 0);
  const auto s = g.core->snapshot();
  CHECK(std::string(s.scan_state) == "scouting");
  CHECK(s.scan_rounds == 0);
  CHECK(!s.scan_pick.has_value());
  CHECK(std::string(s.hop.state) == "idle");
  CHECK(s.energy.size() == 2 && s.dwell.size() == 2);
  CHECK(g.core->op() == 40);
  CHECK(!g.core->hopping());
}

TEST(constructs_pinned_is_off_with_pick_latched) {
  Rig g(2, 0, /*pinned=*/true, 64);
  const auto s = g.core->snapshot();
  CHECK(std::string(s.scan_state) == "off");
  REQUIRE(s.scan_pick.has_value());
  CHECK(*s.scan_pick == 64);
  CHECK(!g.core->pick_open());
}

TEST(hop_state_name_covers_every_state) {
  CHECK(std::string(hop_state_name(HopState::Idle)) == "idle");
  CHECK(std::string(hop_state_name(HopState::Ordered)) == "ordered");
  CHECK(std::string(hop_state_name(HopState::Verifying)) == "verifying");
  CHECK(std::string(hop_state_name(HopState::Hold)) == "hold");
}

// main.cpp send_control_frame: drop when the scout owns a card AND (the
// frame is for the scout card while it is not beaconing, OR any card while
// the scout is in a quiet observe).
TEST(may_send_two_usb_drops_scout_card_unless_beaconing) {
  Rig g(2, 0);
  // scout card is card 1 (last scout-capable); search requested at start
  CHECK(g.core->may_send(0));          // link card always passes (not quiet yet)
  CHECK(!g.core->may_send(1));         // scout card, not beaconing
  CHECK(g.core->snapshot().scout_gated_sends == 1);   // the gate counts
  // beaconing_/quiet_ are set and cleared again within one synchronous
  // run_once() call, so polling may_send() from the test body after the call
  // returns can never observe either branch (see task-2-report.md). The
  // production core instead reads the gate from another thread while the
  // scout thread is mid-dwell; the synchronous equivalent is Rig's sleep
  // callback, which samples the gate (gate_obs) from inside the scout's own
  // sleep_ calls, while the atomics are still live.
  for (int i = 0; i < 6; ++i) g.core->run_scout_step();
  bool seen_burst_pass = false, seen_quiet_hold = false;
  for (const auto& o : g.gate_obs) {
    if (o.second) seen_burst_pass = true;      // scout card beaconing: gate passes it
    if (!o.first) seen_quiet_hold = true;       // quiet observe: even the link card is held
  }
  CHECK(seen_burst_pass);
  CHECK(seen_quiet_hold);
}

TEST(may_send_counts_gated_sends_only_through_note) {
  Rig g(1, 0);                          // one card: the sole card is the scout card
  CHECK(!g.core->may_send(0));          // prelude: silent
  CHECK(g.core->snapshot().scout_gated_sends == 1);
  CHECK(!g.core->may_send(0));
  CHECK(g.core->snapshot().scout_gated_sends == 2);
}

TEST(disc_targets_follow_scan_disc_targets_while_scout_owns) {
  Rig g(2, 1);                          // 2 USB + 1 relay
  g.cards[2]->is_ready = true;
  auto t = g.core->disc_targets(0);     // scout owns at start (search requested)
  // two USB: the link card (0) always; scout card (1) only while beaconing; relay when ready
  REQUIRE(!t.empty());
  CHECK(t[0] == 0);
  CHECK(std::find(t.begin(), t.end(), 2) != t.end());
  CHECK(std::find(t.begin(), t.end(), 1) == t.end());   // not beaconing yet
}

TEST(disc_for_card_retags_only_members) {
  Rig g(2, 0);
  mabur::rc::Disc d; d.vrx_nonce = 7; d.op_channel = 40; d.op_width = 40; d.seq = 1;
  const auto frame = mabur::rc::pack_disc(d, g.cfg.link.key);
  g.cards[1]->ch = 64;                                   // scout card on member 64
  const auto f1 = g.core->disc_for_card(frame, 1);
  const auto p1 = mabur::rc::parse_disc(f1.data(), f1.size());
  REQUIRE(p1.has_value());
  CHECK(p1->op_channel == 64);
  g.cards[1]->ch = 60;                                   // a 20 MHz half, not a member
  const auto f2 = g.core->disc_for_card(frame, 1);
  CHECK(f2 == frame);
}

TEST(session_opened_elsewhere_moves_op_there) {
  Rig g(2, 0);
  g.core->on_rc_body(64);               // the ack arrived on member 64
  mabur::rc::DiscAck ack; ack.vrx_nonce = g.vrx->rz_nonce(); ack.vtx_nonce = 1;
  ack.chip_caps = mabur::rc::CAP_FRAME_WIRE; ack.agreed_channel = 64; ack.seq = 1;
  g.core->on_session_opened(ack, 1000.0);
  CHECK(g.core->op() == 64);
  CHECK(g.vrx->proposal() == 64);
  CHECK(g.sink.has_line("drone found on 64 (op 40): the link forms there"));
  g.tick(true);
  CHECK(g.sink.has_move(MoveReason::LinkFound));
}

TEST(session_opened_with_key_mismatch_or_foreign_nonce_is_ignored) {
  Rig g(2, 0);
  g.core->on_rc_body(64);
  mabur::rc::DiscAck ack; ack.vrx_nonce = g.vrx->rz_nonce(); ack.vtx_nonce = 1;
  ack.agreed_channel = 64; ack.flags = mabur::rc::kAckKeyMismatch;
  g.core->on_session_opened(ack, 1000.0);
  CHECK(g.core->op() == 40);
  ack.flags = 0; ack.vrx_nonce = g.vrx->rz_nonce() + 1;
  g.core->on_session_opened(ack, 1000.0);
  CHECK(g.core->op() == 40);
  g.core->on_rc_body(60);               // not a member
  ack.vrx_nonce = g.vrx->rz_nonce();
  g.core->on_session_opened(ack, 1000.0);
  CHECK(g.core->op() == 40);
}

// Carried from Task 2 (task-2-brief.md Step 1): needed step_scout_inputs_ --
// the search request goes false once the plan is in session.
TEST(disc_targets_is_tx_when_scout_owns_nothing) {
  Rig g(2, 0, /*pinned=*/true, 40);
  g.tick(true, false, /*tx=*/1);  // linked: release_scout() false, search off
  g.tick(true, false, 1);
  const auto t = g.core->disc_targets(1);
  REQUIRE(t.size() == 1);
  CHECK(t[0] == 1);
}

// The one-card prelude (auto): silent one_card_ms, then AckPrelude commits
// the prelude ranking (no link) and the first op window runs. The link-edge
// continuation ("one-card linked") is Task 4's own test once step_move_edge_
// exists (step_move_edge_ is the only thing that can ever set link_edge_seen_).
TEST(one_card_prelude_commits_before_first_disc) {
  Rig g(1, 0);
  g.cards[0]->cca_per_ms_on[40] = 5;   // 40 busy, the rest clean
  // run the scout + core for one_card_ms + a round
  for (int i = 0; i < 800 && !g.sink.has_line("one-card prelude ranking picks"); ++i) g.tick();
  REQUIRE(g.sink.has_line("one-card prelude ranking picks"));
  CHECK(g.core->op() != 40);                       // committed off the busy channel
  CHECK(!g.stored.empty() && g.stored.back() == g.core->op());
  CHECK(g.vrx->proposal() == g.core->op());
  CHECK(g.sink.has_move(MoveReason::Commit));
  CHECK(std::string(g.core->snapshot().scan_state) == "scouting");   // pick still open
}

// Two cards, no link, auto: maturity commits the pick (K line) and freezes.
TEST(two_card_no_link_commits_at_maturity) {
  Rig g(2, 0);
  g.cards[1]->cca_per_ms_on[40] = 5; g.cards[1]->cca_per_ms_on[36] = 5;   // op pair busy
  for (int i = 0; i < 3000 && !g.sink.has_line("pick frozen on"); ++i) g.tick();
  REQUIRE(g.sink.has_line("pick frozen on"));
  CHECK(g.sink.has_line("(commit)"));
  CHECK(g.sink.has_line("maburgs channel: commit 40 -> "));
  CHECK(g.core->op() != 40);
  REQUIRE(!g.sink.picks.empty());
  CHECK(g.sink.picks.back().has_value());
  // every card retuned to op by the mechanical retune
  CHECK(g.cards[0]->ch == g.core->op());
}

TEST(store_failure_is_logged_not_fatal) {   // Review Focus 2
  Rig g(2, 0);
  g.store_ok = false;
  g.cards[1]->cca_per_ms_on[40] = 5; g.cards[1]->cca_per_ms_on[36] = 5;
  for (int i = 0; i < 3000 && !g.sink.has_line("pick frozen on"); ++i) g.tick();
  REQUIRE(g.sink.has_line("pick frozen on"));
  CHECK(g.sink.has_line("maburgs channel: could not write"));
  CHECK(g.core->op() != 40);
}

TEST(max_ms_freezes_unmeasured_in_place) {
  Rig g(2, 0);
  g.cards[1]->retune_ok = false;          // the scout never completes a dwell
  for (int i = 0; i < 4000 && !g.sink.has_line("pick frozen on"); ++i) g.tick();
  REQUIRE(g.sink.has_line("(max_ms)"));
  CHECK(g.core->op() == 40);
}

TEST(scout_card_death_while_working_freezes_pick) {
  Rig g(2, 0);
  g.tick();
  g.cards[1]->is_alive = false;
  g.core->on_card_died(1);
  CHECK(g.sink.has_line("scout card 1 died at"));
  g.tick();
  CHECK(g.sink.has_line("(scout card died)"));   // BootPick froze the pick
  CHECK(!g.core->pick_open());
}

// Pinned + linked: the scout has no work (search off, nothing to measure),
// so once it parks and gives up the card, the core's own width resync is
// the only thing left that can fix a card that reopened at 20 MHz.
TEST(reopened_scout_card_at_20_gets_width_resync) {
  Rig g(2, 0, /*pinned=*/true, 40);
  for (int i = 0; i < 10; ++i) g.tick(true, false, 0);   // linked: search off, the scout parks and owns nothing
  REQUIRE(g.cards[1]->width_mhz == 40);                   // parked at radio.width by the scout itself
  g.cards[1]->is_alive = false;
  g.core->on_card_died(1);
  // reopen as RadioFrontend would: InitWrite at the card's own cfg width (20 for the scout card)
  g.cards[1]->is_alive = true; g.cards[1]->ch = 40; g.cards[1]->width_mhz = 20;
  g.cards[1]->calls.clear();
  g.core->on_card_reopened(1);
  for (int i = 0; i < 50; ++i) g.tick(true, false, 0);
  bool resynced = false;
  for (auto& c : g.cards[1]->calls) if (c.rfind("set_width", 0) == 0) resynced = true;
  CHECK(resynced);                                        // the CORE's width resync, not the scout's park
  CHECK(g.cards[1]->width_mhz == 40);
}

TEST(mechanical_retune_skips_scout_card_and_follows_desired) {
  Rig g(2, 0);                                   // auto: the scout owns card 1 from the start
  g.tick(); g.tick();
  REQUIRE(g.core->scout_owns_card(1));
  // a found-elsewhere session moves op for every card: desired(0) and desired(1) become 64
  g.core->on_rc_body(64);
  mabur::rc::DiscAck ack; ack.vrx_nonce = g.vrx->rz_nonce(); ack.vtx_nonce = 1; ack.agreed_channel = 64; ack.seq = 1;
  g.core->on_session_opened(ack, static_cast<double>(g.clk.ms));
  g.cards[1]->calls.clear();
  // core->tick() directly (the brief's form); the check is specifically
  // for the CORE's mechanical "retune 64" on card 1 -- the scout's own
  // dwells retune through its own calls, which this tick does not count.
  g.core->tick(g.in(true));
  CHECK(g.cards[0]->ch == 64);                   // the TX card followed op via the core's mechanical retune
  bool core_retuned_scout = false;
  for (auto& c : g.cards[1]->calls) if (c == "retune 64") core_retuned_scout = true;
  CHECK(!core_retuned_scout);                    // the scout card is untouchable while the scout owns it
  CHECK(g.core->scout_owns_card(1));
}

// Helpers: bring the rig to SESSION on the start channel (synthetic ack, as
// run_hop_inject_test did), freeze the pick (pinned rigs are frozen from
// the start), and pump RCFs through vrx.step() so the core's note_sent
// sees them.
static void link_up(Rig& g) {
  mabur::rc::DiscAck ack; ack.vrx_nonce = g.vrx->rz_nonce(); ack.vtx_nonce = 1;
  ack.chip_caps = mabur::rc::CAP_FRAME_WIRE; ack.agreed_channel = g.core->op(); ack.seq = 1;
  const auto wire = mabur::rc::pack_disc_ack(ack);
  g.vrx->on_rc_frame(wire.data(), wire.size(), static_cast<double>(g.clk.ms));
  const LinkHealth healthy{true, 0.0, 0.0, false};
  for (int i = 0; i < 200 && g.vrx->link_state() != VrxState::SESSION; ++i) {
    g.clk.ms += 10;
    g.vrx->on_video(static_cast<double>(g.clk.ms));
    g.vrx->step(static_cast<double>(g.clk.ms), healthy);
  }
  REQUIRE(g.vrx->link_state() == VrxState::SESSION);
}
// One RCF built and "sent": returns the parsed Rcf.
static std::optional<mabur::rc::Rcf> pump_rcf(Rig& g) {
  const LinkHealth healthy{true, 0.0, 0.0, false};
  for (int i = 0; i < 20; ++i) {
    g.clk.ms += static_cast<uint64_t>(g.cfg.link.feedback_ms);
    g.vrx->on_video(static_cast<double>(g.clk.ms));
    auto out = g.vrx->step(static_cast<double>(g.clk.ms), healthy);
    g.core->tick(g.in(true));
    if (!out || out->is_disc) continue;
    g.core->note_sent(true, true);
    return mabur::rc::parse_rcf(out->frame.data(), out->frame.size());
  }
  return std::nullopt;
}
// An interfered verdict window: foreign >> foreign_pps on every usable
// card, AND impaired (HopVerdict::window() -- hop_verdict.cpp -- gates
// Interfered on `impaired`, which `contended`/`raised` alone never set;
// only pre_fec_loss/recovered/starved do). This rig's Aggregator never
// decodes a real body, so pre_fec_loss/recovered are permanently 0 --
// `own` is deliberately left unbumped (own delta 0 every window) so
// `starved` carries `impaired` instead. Runs enough windows for the
// trigger (persist 2).
static void interfere(Rig& g, int windows = 3) {
  for (int w = 0; w < windows; ++w) {
    for (auto& c : g.cards) { c->fr.foreign += 40; }
    for (int i = 0; i < g.cfg.hop.window_ms / 10 + 1; ++i) {
      g.vrx->on_video(static_cast<double>(g.clk.ms));
      g.core->tick(g.in(true));
      g.clk.ms += 10;
    }
  }
}

TEST(two_card_order_rcf_carries_hop_and_plan_leads_then_follows) {
  Rig g(2, 0, /*pinned=*/true, 40);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  // rank 64 as the best candidate: feed the ranker through the burst path
  // (the fake's energy is clean on 64, busy on 40) -- the burst runs when
  // the trigger fires; so first interfere, then the burst ranks, then order.
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  g.cards[0]->fr.foreign = 0; g.cards[1]->fr.foreign = 0;
  interfere(g, 4);
  REQUIRE(g.sink.has_hop("order"));
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value());
  CHECK(r->hop_ch != 0 && r->hop_ch != 40);
  CHECK(r->hop_epoch == 1);
  CHECK(g.sink.has_move(MoveReason::HopLead));
  CHECK(std::string(to_string(g.vrx->ctl().last_event().reason)) == "hop_restore");   // ctl hop_restore at the order
  // confirm: video on the target, received by the lead card
  g.core->note_video(r->hop_ch);
  g.tick(true);
  CHECK(g.sink.has_hop("lead_confirm"));
  CHECK(g.sink.has_move(MoveReason::HopFollow));
  CHECK(g.core->op() == r->hop_ch);
  // verify: healthy windows past verify_ms
  for (int i = 0; i < (g.cfg.hop.verify_ms / 10) + 30; ++i) { g.vrx->on_video(static_cast<double>(g.clk.ms)); g.tick(true); }
  CHECK(g.sink.has_hop("verify_pass"));
  CHECK(g.core->snapshot().hop.hops == 1);
}

TEST(stale_pre_hop_verdict_does_not_break_verify) {
  // After Confirm the cached VerdictOut (measured on the old channel) is
  // re-fed every tick until the next window: the controller must stay in
  // Verifying (C1).
  Rig g(2, 0, true, 40);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere(g, 4);
  REQUIRE(g.sink.has_hop("order"));
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value());
  g.core->note_video(r->hop_ch);
  g.tick(true);
  REQUIRE(g.sink.has_hop("lead_confirm"));
  g.tick(true);                              // same cached interfered verdict
  CHECK(std::string(g.core->snapshot().hop.state) == "verifying");
  CHECK(!g.sink.has_hop("withdraw"));
}

TEST(one_card_order_rides_repeats_then_retunes) {
  Rig g(1, 0, true, 40);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50;
  interfere(g, 4);
  REQUIRE(g.sink.has_hop("order"));
  CHECK(g.cards[0]->ch == 40);              // radio stays until one_card_repeats RCFs
  // interfere() only drives core->tick() (closing verdict windows), never
  // vrx.step() -- unlike run_radio(), where the SAME control-loop pass that
  // dispatches the Order also runs vrx.step() afterward and sends that
  // pass's RCF. Without this one send, the order's own control-tick
  // contributes nothing to rcf_sent_total_, and ordered_tick's
  // rcf_sent_since_order count (read before THIS call's own note_sent, like
  // every call below) would need one extra pump_rcf() call to reach
  // one_card_repeats -- pins note_sent()'s count to the Order's own pass,
  // not an extra one.
  pump_rcf(g);
  int sent = 0;
  std::optional<mabur::rc::Rcf> r;
  for (int i = 0; i < 10 && !g.sink.has_hop("one_card_retune"); ++i) { r = pump_rcf(g); ++sent; }
  REQUIRE(g.sink.has_hop("one_card_retune"));
  CHECK(sent == g.cfg.hop.one_card_repeats);
  REQUIRE(r.has_value());
  CHECK(g.cards[0]->ch == r->hop_ch);        // the sole radio moved (mechanical retune)
  CHECK(g.sink.has_move(MoveReason::HopOneCard));
  for (auto& e : g.sink.hops)
    if (e.kind == "one_card_retune") CHECK(e.elapsed_ms >= 200 && e.elapsed_ms <= 400);
  g.core->note_video(r->hop_ch);
  g.tick(true);
  CHECK(g.sink.has_hop("lead_confirm"));
  CHECK(g.sink.has_move(MoveReason::HopFollow));
  for (int i = 0; i < (g.cfg.hop.verify_ms / 10) + 30; ++i) { g.vrx->on_video(static_cast<double>(g.clk.ms)); g.tick(true); }
  CHECK(g.sink.has_hop("verify_pass"));
}

TEST(withdraw_on_no_video_restores_cards) {
  Rig g(2, 0, true, 40);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere(g, 4);
  REQUIRE(g.sink.has_hop("order"));
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value());
  for (int i = 0; i < (g.cfg.hop.confirm_extend_ms / 10) + 20; ++i) { g.vrx->on_video(static_cast<double>(g.clk.ms)); g.tick(true); }
  CHECK(g.sink.has_hop("withdraw"));
  CHECK(g.sink.has_move(MoveReason::HopWithdraw));
  CHECK(g.cards[1]->ch == 40);               // lead card back on op
  CHECK(g.core->op() == 40);
}

TEST(lead_card_dies_mid_order_withdraws_to_op) {   // Review Focus 1
  Rig g(2, 0, true, 40);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere(g, 4);
  REQUIRE(g.sink.has_hop("order"));
  g.cards[1]->is_ready = false;               // the lead vanishes
  g.tick(true);
  CHECK(std::string(g.core->snapshot().hop.state) == "ordered");   // latched lead, no re-pick
  for (int i = 0; i < (g.cfg.hop.confirm_extend_ms / 10) + 20; ++i) { g.vrx->on_video(static_cast<double>(g.clk.ms)); g.tick(true); }
  CHECK(g.sink.has_hop("withdraw"));
  CHECK(g.core->op() == 40);
  g.cards[1]->is_ready = true;
  g.tick(true);
  CHECK(g.cards[1]->ch == 40);                // retuned back once ready
}

TEST(session_loss_mid_order_withdraws) {
  Rig g(2, 0, true, 40);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere(g, 4);
  REQUIRE(g.sink.has_hop("order"));
  g.tick(false);                              // hop_active falling edge
  // on_session_lost() returns HopAction::Withdraw, but the EVENT it logs
  // (hop_controller.cpp) is kind "session_lost", not "withdraw" -- that
  // literal kind is only ever logged by withdraw() (the confirm_ms/
  // confirm_extend_ms timeout path), a different caller.
  CHECK(g.sink.has_hop("session_lost"));
  CHECK(!g.core->hopping());
}

TEST(cal_running_holds_relocation_and_move_edge) {   // Review Focus 3
  Rig g(2, 0, true, 40);
  g.core->on_rc_body(64);
  mabur::rc::DiscAck ack; ack.vrx_nonce = g.vrx->rz_nonce(); ack.vtx_nonce = 1; ack.agreed_channel = 64; ack.seq = 1;
  g.core->on_session_opened(ack, static_cast<double>(g.clk.ms));   // link formed on 64, want = 40
  link_up(g);
  CHECK(g.core->op() == 64);
  for (int i = 0; i < 20; ++i) g.tick(true, /*cal=*/true);
  CHECK(!g.sink.has_hop("order"));            // no relocate while calibrating
  CHECK(!g.sink.has_line("relocate 64 -> 40"));
  // The move edge during cal: a repeat DISC_ACK (same nonces) refreshes
  // agreed_channel() to a NON-member 60 without opening a new session, then
  // the edge fires. Held during cal => the step never reads agreed, so no
  // "not in our set" line and no plan move.
  const auto ack60 = [](Rig& r, const mabur::rc::DiscAck& base) {
    mabur::rc::DiscAck rep = base; rep.chip_caps = mabur::rc::CAP_FRAME_WIRE; rep.agreed_channel = 60; rep.seq = 2;
    const auto wire = mabur::rc::pack_disc_ack(rep);
    r.vrx->on_rc_frame(wire.data(), wire.size(), static_cast<double>(r.clk.ms));
    REQUIRE(r.vrx->agreed_channel() == 60);
    REQUIRE(r.vrx->link_state() == VrxState::SESSION);
  };
  ack60(g, ack);
  g.vrx->test_set_move_edge();
  for (int i = 0; i < 5; ++i) g.tick(true, /*cal=*/true);
  CHECK(!g.sink.has_line("drone acked 60, not in our set; ignored"));   // held, not acted on
  CHECK(!g.sink.has_move(MoveReason::AckOverride));
  CHECK(!g.sink.has_move(MoveReason::Commit));
  CHECK(!g.sink.has_line("relocate 64 -> 40"));
  // (Here the first post-cal tick also places the relocate, and
  // step_move_edge_ runs after step_controller_ with a `!plan_.hopping()`
  // guard, so this rig's released edge lands on a hopping tick and is
  // consumed without acting -- the old main.cpp order. The release itself
  // is pinned on rig h below, where no relocation is pending.)
  for (int i = 0; i < 20; ++i) g.tick(true, false);
  CHECK(g.sink.has_line("maburgs channel: relocate 64 -> 40 placed"));
  CHECK(g.sink.has_hop("relocate"));
  // Rig h: linked on op 40 = want, nothing to relocate. The edge armed
  // during cal is held (no line), then released after cal: the line once.
  Rig h(2, 0, true, 40);
  link_up(h);
  mabur::rc::DiscAck hack; hack.vrx_nonce = h.vrx->rz_nonce(); hack.vtx_nonce = 1; hack.seq = 1;
  ack60(h, hack);
  h.vrx->test_set_move_edge();
  for (int i = 0; i < 20; ++i) h.tick(true, /*cal=*/true);
  CHECK(!h.sink.has_line("drone acked 60, not in our set; ignored"));
  for (int i = 0; i < 20; ++i) h.tick(true, false);
  int acked60 = 0;
  for (const auto& l : h.sink.lines) if (l.find("drone acked 60, not in our set; ignored") != std::string::npos) ++acked60;
  CHECK(acked60 == 1);                         // the held edge released and acted on, exactly once
  CHECK(h.core->op() == 40);                   // a non-member ack moves nothing
}

TEST(relocate_lands_on_verify_pass_and_freezes_relocated) {
  // Pinned, not auto: BootPick's own "op unmeasured" guard (final review
  // I1, boot_pick.cpp) means an AUTO pick never relocates off a link found
  // before any scouting -- "a linked scout never measures op's own pair,
  // so a drone found at once leaves op with only its pre-link visits. A
  // working link is not moved on a one-sided comparison." That freezes
  // auto mode's pick in place on 64 forever (confirmed: 200+ ticks never
  // produce the relocate), which is BootPick's deliberate design, not a
  // gap this task's dispatch/controller code can or should route around.
  // Pinned mode skips the pick entirely (open_ false from construction)
  // and goes straight to the relocation gate this test actually exercises
  // -- same as cal_running_holds_relocation_and_move_edge, carried through
  // Confirm/Verify/VerifyPass.
  Rig g(2, 0, /*pinned=*/true, 40);
  g.core->on_rc_body(64);
  mabur::rc::DiscAck ack; ack.vrx_nonce = g.vrx->rz_nonce(); ack.vtx_nonce = 1; ack.agreed_channel = 64; ack.seq = 1;
  g.core->on_session_opened(ack, static_cast<double>(g.clk.ms));
  link_up(g);
  for (int i = 0; i < 200 && !g.sink.has_line("relocate 64 -> 40 placed"); ++i) g.tick(true);
  REQUIRE(g.sink.has_line("relocate 64 -> 40 placed"));
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value() && r->hop_ch == 40);
  g.core->note_video(40);
  g.tick(true);
  REQUIRE(g.sink.has_hop("lead_confirm"));
  for (int i = 0; i < (g.cfg.hop.verify_ms / 10) + 30; ++i) { g.vrx->on_video(static_cast<double>(g.clk.ms)); g.tick(true); }
  CHECK(g.sink.has_hop("verify_pass"));
  CHECK(g.core->op() == 40);
  CHECK(std::string(g.core->snapshot().scan_state) != "moving");
}

// Carried from Task 3 (continuation deferred to Task 4): the move edge
// (step_move_edge_) is what sets link_edge_seen_, which BootPick reads next
// tick as "one-card linked".
TEST(one_card_link_edge_freezes_one_card_linked) {
  Rig g(1, 0);
  g.cards[0]->cca_per_ms_on[40] = 5;
  for (int i = 0; i < 800 && !g.sink.has_line("one-card prelude ranking picks"); ++i) g.tick();
  REQUIRE(g.sink.has_line("one-card prelude ranking picks"));
  g.vrx->test_set_move_edge();   // the drone linked: the move edge fires once
  g.tick(true);
  g.tick(true);
  CHECK(g.sink.has_line("pick frozen on"));
  CHECK(g.sink.has_line("(one-card linked)"));
  CHECK(std::string(g.core->snapshot().scan_state) == "frozen");
  REQUIRE(g.core->snapshot().scan_pick.has_value());
  CHECK(*g.core->snapshot().scan_pick == g.core->op());
}

TEST(inflight_dwell_feeds_ranker_and_dwell_stats) {
  Rig g(2, 0, true, 40);                     // pinned: pick closed, scout idle after park
  link_up(g);
  for (int i = 0; i < 10; ++i) g.tick(true, false, 0);
  // in-flight step: card 1 (non-TX, scout-capable) dwells on the next candidate
  g.core->run_inflight_step();
  g.tick(true, false, 0);                    // drain
  CHECK(!g.core->dwell_busy());
  const auto s = g.core->snapshot();
  REQUIRE(s.dwell[1].has_value());
  CHECK(s.dwell[1]->visits == 1);
  REQUIRE(!g.sink.dwells.empty());
  CHECK(g.sink.dwells.back().first == 1);
  CHECK(g.cards[1]->ch == 40);               // back on op after the dwell
}

TEST(inflight_step_skips_when_not_in_session_or_hopping_or_one_card) {
  Rig g(1, 0, true, 40);
  link_up(g);
  for (int i = 0; i < 10; ++i) g.tick(true);
  g.core->run_inflight_step();
  g.tick(true);
  CHECK(!g.core->snapshot().dwell[0].has_value());   // one card: never dwells
  Rig h(2, 0, true, 40);
  for (int i = 0; i < 10; ++i) h.tick(false);
  h.core->run_inflight_step();
  h.tick(false);
  CHECK(!h.core->snapshot().dwell[1].has_value());   // no session: never dwells
}

TEST(tx_frozen_while_hopping) {
  Rig g(2, 0, true, 40);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere(g, 4);
  REQUIRE(g.sink.has_hop("order"));
  const auto out = g.core->tick(g.in(true));
  CHECK(out.tx_frozen);
  CHECK(g.core->tx_frozen());
}

TEST(shutdown_joins_threads_and_is_idempotent) {   // Review Focus 5
  // Real threads, real clock. Pinned + LINKED: the boot scout has no work and
  // parks, the pick is closed, so the in-flight thread starts and dwells on
  // the non-TX card every dwell_period_ms; shutdown() must join it.
  Config cfg = bundle();
  cfg.radio.channels = {40, 64}; cfg.radio.width = 40; cfg.radio.pin = 40;
  cfg.hop.enable = true;
  FakeCard a, b; a.ch = b.ch = 40; a.width_mhz = 40; b.width_mhz = 20;
  std::vector<LinkCard*> ptrs{&a, &b};
  VrxController vrx(vrx_cfg_from(cfg, 40));
  RecordingSink sink;
  Aggregator agg(cfg.uep_layers(), 32, 2, 0);
  ChannelCoreCfg cc; cc.radio = cfg.radio; cc.hop = cfg.hop; cc.key = cfg.link.key; cc.start_ch = 40; cc.n_usb = 2;
  auto now_ms = [] { return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()); };
  auto now_us = [] { return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()); };
  ChannelCore core(cc, ptrs, vrx, sink, [](uint8_t) { return true; }, now_ms, now_us,
                   [](int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); });
  ChannelTickIn in; in.agg = &agg; in.in_session = true; in.tx_card = 0;
  // Up to 3 s of linked ticks: a dwell record drained from the in-flight
  // thread proves that thread ran (dwell_period_ms is 333 in the bundle).
  bool dwelt = false;
  for (int i = 0; i < 300 && !dwelt; ++i) {
    in.now_ms = static_cast<double>(now_ms());
    core.tick(in);
    dwelt = core.snapshot().dwell[1].has_value();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  REQUIRE(dwelt);
  const uint32_t visits = core.snapshot().dwell[1]->visits;
  const uint64_t t0 = now_ms();
  core.shutdown();
  core.shutdown();                            // idempotent
  CHECK(now_ms() - t0 < 2000);                // joined both threads promptly
  std::this_thread::sleep_for(std::chrono::milliseconds(400));   // > dwell_period_ms
  in.now_ms = static_cast<double>(now_ms());
  const auto out = core.tick(in);                                // drains anything a live thread left
  CHECK(!out.dwell_busy);
  CHECK(core.snapshot().dwell[1]->visits == visits);             // no thread left dwelling
}

TEST(relay_only_roster_searches_without_measuring) {
  Rig g(0, 1);                                // one relay, auto
  const auto s = g.core->snapshot();
  CHECK(std::string(s.scan_state) == "off");  // nothing can measure
  CHECK(!g.core->pick_open());
  // the relay scouts: search bursts retune it across the set
  for (int i = 0; i < 40; ++i) g.tick();
  bool retuned_off_start = false;
  for (auto& c : g.cards[0]->calls) if (c == "retune 64" || c == "retune_width 64/20") retuned_off_start = true;
  CHECK(retuned_off_start);
  CHECK(g.sink.picks.empty());                // no K line ever: nothing measured
}

TEST(relay_only_roster_no_ready_relay_is_quiet) {   // Review Focus 4
  Rig g(0, 1);
  g.cards[0]->is_ready = false;
  for (int i = 0; i < 20; ++i) g.tick();
  CHECK(g.core->disc_targets(0).empty());
  CHECK(std::string(g.core->snapshot().scan_state) == "off");
  CHECK(g.cards[0]->calls.empty());           // scout never started (card not ready): nothing touched it
}

MTEST_MAIN
