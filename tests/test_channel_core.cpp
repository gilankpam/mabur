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
    cc.store_name = "the-store";
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
  const auto s = g.core->snapshot(0);
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
  const auto s = g.core->snapshot(0);
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
  CHECK(g.core->snapshot(0).scout_gated_sends == 1);   // the gate counts
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
  CHECK(g.core->snapshot(0).scout_gated_sends == 1);
  CHECK(!g.core->may_send(0));
  CHECK(g.core->snapshot(0).scout_gated_sends == 2);
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
  CHECK(std::string(g.core->snapshot(0).scan_state) == "scouting");   // pick still open
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
  CHECK(g.sink.has_line("maburgs channel: could not write the-store"));
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
static void interfere(Rig& g, int windows = 3, int tx = 0) {
  for (int w = 0; w < windows; ++w) {
    for (auto& c : g.cards) { c->fr.foreign += 40; }
    for (int i = 0; i < g.cfg.hop.window_ms / 10 + 1; ++i) {
      g.vrx->on_video(static_cast<double>(g.clk.ms));
      g.core->tick(g.in(true, false, tx));
      g.clk.ms += 10;
    }
  }
}

// Bring an AUTO rig to a frozen pick on 40 without a hop, so the reactive
// layer is live exactly as in flight. Two cards: no dwell ever completes
// (the scout card refuses to retune), so max_ms freezes op in place. One
// card: link-up freezes it ("one-card linked"). The hop tests used pinned
// rigs for this until 2026-10-04 (frozen from construction); pinned now
// means NO reactive hop.
static void freeze_auto(Rig& g) {
  if (g.cards.size() >= 2) {
    g.cards.back()->retune_ok = false;
    for (int i = 0; i < 4000 && g.core->pick_open(); ++i) g.tick();
    g.cards.back()->retune_ok = true;
  } else {
    link_up(g);
    for (int i = 0; i < 100 && g.core->pick_open(); ++i) g.tick(true);
  }
  REQUIRE(!g.core->pick_open());
  REQUIRE(g.core->op() == 40);
}

// Pinned = static. An interfered op produces a verdict (the OSD/sideport
// still say the pin is dirty) but no order, no escape, no exhausted hold:
// the controller is never fed a reactive trigger. Review Focus 1.
TEST(pinned_two_card_never_hops_on_interference) {
  Rig g(2, 0, /*pinned=*/true, 40);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  g.cards[0]->fr.foreign = 0; g.cards[1]->fr.foreign = 0;
  interfere(g, 6);
  CHECK(std::string(g.core->snapshot(0).hop.verdict) == "interfered");
  CHECK(g.sink.hops.empty());                 // no order/escape/hold of any kind
  CHECK(std::string(g.core->snapshot(0).hop.state) == "idle");
  CHECK(g.cards[0]->ch == 40 && g.cards[1]->ch == 40);
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value());
  CHECK(r->hop_ch == 0);                      // no order ever issued
  // and the in-flight scout never dwells, driven directly or not
  const size_t dwells = g.sink.dwells.size();
  g.core->run_inflight_step();
  g.tick(true);
  CHECK(g.sink.dwells.size() == dwells);
  CHECK(g.cards[1]->ch == 40);
}

// Same on one card (the web GS shape): the one-card path has no scout, so
// the only reactive source is the freshness burst + order. Review Focus 2.
TEST(pinned_one_card_never_hops_on_interference) {
  Rig g(1, 0, /*pinned=*/true, 40);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50;
  interfere(g, 6);
  CHECK(std::string(g.core->snapshot(0).hop.verdict) == "interfered");
  CHECK(g.sink.hops.empty());
  CHECK(g.cards[0]->ch == 40);
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value());
  CHECK(r->hop_ch == 0);
}

// Bench 2026-10-04 (row 9): the relocation onto pinned 40 landed, then one
// verify window read the pin interfered (the 36-48 router) -> verify_fail,
// "relocation to 40 did not land; staying on 40", holds 1. Pinned, the
// relocation has nowhere else to go: the confirm lands it, no verify.
TEST(pinned_relocation_is_not_failed_by_a_dirty_pin) {
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
  // the pin reads interfered through what would have been the verify window
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere(g, 8);
  CHECK(g.sink.has_hop("verify_pass"));
  CHECK(!g.sink.has_hop("verify_fail"));
  CHECK(!g.sink.has_line("did not land"));
  CHECK(g.core->snapshot(0).hop.holds == 0);
  CHECK(std::string(g.core->snapshot(0).hop.state) == "idle");
  CHECK(g.core->op() == 40 && g.cards[0]->ch == 40 && g.cards[1]->ch == 40);
  CHECK(std::string(g.core->snapshot(0).hop.verdict) == "interfered");   // still measured, still reported
}

TEST(two_card_order_rcf_carries_hop_and_plan_leads_then_follows) {
  Rig g(2, 0); freeze_auto(g);
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
  CHECK(g.core->snapshot(0).hop.hops == 1);
}

TEST(stale_pre_hop_verdict_does_not_break_verify) {
  // After Confirm the cached VerdictOut (measured on the old channel) is
  // re-fed every tick until the next window: the controller must stay in
  // Verifying (C1).
  Rig g(2, 0); freeze_auto(g);
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
  CHECK(std::string(g.core->snapshot(0).hop.state) == "verifying");
  CHECK(!g.sink.has_hop("withdraw"));
}

TEST(one_card_order_rides_repeats_then_retunes) {
  Rig g(1, 0); freeze_auto(g);               // links too
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
  Rig g(2, 0); freeze_auto(g);
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
  Rig g(2, 0); freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere(g, 4);
  REQUIRE(g.sink.has_hop("order"));
  g.cards[1]->is_ready = false;               // the lead vanishes
  g.tick(true);
  CHECK(std::string(g.core->snapshot(0).hop.state) == "ordered");   // latched lead, no re-pick
  for (int i = 0; i < (g.cfg.hop.confirm_extend_ms / 10) + 20; ++i) { g.vrx->on_video(static_cast<double>(g.clk.ms)); g.tick(true); }
  CHECK(g.sink.has_hop("withdraw"));
  CHECK(g.core->op() == 40);
  g.cards[1]->is_ready = true;
  g.tick(true);
  CHECK(g.cards[1]->ch == 40);                // retuned back once ready
}

TEST(session_loss_mid_order_withdraws) {
  Rig g(2, 0); freeze_auto(g);
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
  for (int i = 0; i < 20; ++i) g.tick(true, false);
  CHECK(g.sink.has_line("maburgs channel: relocate 64 -> 40 placed"));
  CHECK(g.sink.has_hop("relocate"));
  // Carried from plan 1: the edge released on the same tick as the relocate
  // was DROPPED by the `!plan_.hopping()` guard. Now it is held through the
  // hop and acted on once the hop resolves (here: confirm + verify_pass).
  {
    const auto r = pump_rcf(g);
    REQUIRE(r.has_value() && r->hop_ch == 40);
    g.core->note_video(40);
    g.tick(true);
    REQUIRE(g.sink.has_hop("lead_confirm"));
    for (int i = 0; i < (g.cfg.hop.verify_ms / 10) + 30; ++i) { g.vrx->on_video(static_cast<double>(g.clk.ms)); g.tick(true); }
    REQUIRE(g.sink.has_hop("verify_pass"));
    int acked60 = 0;
    for (const auto& l : g.sink.lines) if (l.find("drone acked 60, not in our set; ignored") != std::string::npos) ++acked60;
    CHECK(acked60 == 1);          // released once the hop resolved, acted on exactly once
    CHECK(g.core->op() == 40);    // a non-member ack still moves nothing
  }
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
  CHECK(std::string(g.core->snapshot(0).scan_state) != "moving");
}

// Carried from plan 1: the AUTO boot hop. The scout measures op's pair to
// min_rounds BEFORE the drone appears on 64 (a linked scout never visits
// op's own halves, so without that BootPick freezes "op unmeasured"), and
// the link forms while the later pairs are still short of min_rounds (else
// the unlinked pick would already have committed). At maturity the pick
// wants another pair -> WantPick -> relocate -> confirm -> verify_pass ->
// "pick frozen on <pick> (relocated)".
TEST(auto_want_pick_relocates_linked_drone_to_the_pick) {
  Rig g(2, 0);
  g.cards[1]->cca_per_ms_on[60] = 5;   // 64's pair is the busy one
  g.cards[1]->cca_per_ms_on[64] = 5;
  // The scout sweeps 36,40,60,64,108,112,140,144 each round: stop right
  // after 64's min_rounds-th visit, i.e. inside the last round.
  const auto visits = [&g](uint8_t ch) {
    int n = 0;
    for (const auto& d : g.sink.dwells) if (d.second.survey.def.primary == ch) ++n;
    return n;
  };
  const int mr = g.cfg.radio.scan.min_rounds;
  for (int i = 0; i < 3000 && visits(64) < mr; ++i) g.tick();
  REQUIRE(visits(60) >= mr && visits(64) >= mr);
  REQUIRE(visits(144) < mr);                // not mature yet
  REQUIRE(g.core->pick_open());             // ... so nothing committed unlinked
  g.core->on_rc_body(64);
  mabur::rc::DiscAck ack; ack.vrx_nonce = g.vrx->rz_nonce(); ack.vtx_nonce = 1; ack.agreed_channel = 64; ack.seq = 1;
  g.core->on_session_opened(ack, static_cast<double>(g.clk.ms));
  link_up(g);
  REQUIRE(g.core->op() == 64);
  for (int i = 0; i < 6000 && !g.sink.has_line("boot pick wants"); ++i) g.tick(true);
  REQUIRE(g.sink.has_line("boot pick wants"));
  REQUIRE(g.sink.has_line(", link on 64: relocating"));
  for (int i = 0; i < 200 && !g.sink.has_line("relocate 64 -> "); ++i) g.tick(true);
  REQUIRE(g.sink.has_line(" placed"));
  CHECK(std::string(g.core->snapshot(0).scan_state) == "moving");   // pick open, relocation placed
  const uint8_t target = g.core->hop_target();
  REQUIRE(target != 0 && target != 64);
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value() && r->hop_ch == target);
  g.core->note_video(target);
  g.tick(true);
  REQUIRE(g.sink.has_hop("lead_confirm"));
  for (int i = 0; i < (g.cfg.hop.verify_ms / 10) + 30; ++i) { g.vrx->on_video(static_cast<double>(g.clk.ms)); g.tick(true); }
  REQUIRE(g.sink.has_hop("verify_pass"));
  CHECK(g.core->op() == target);
  CHECK(g.sink.has_line("(relocated)"));
  CHECK(std::string(g.core->snapshot(0).scan_state) == "frozen");
  REQUIRE(g.core->snapshot(0).scan_pick.has_value());
  CHECK(*g.core->snapshot(0).scan_pick == target);
  CHECK(!g.stored.empty() && g.stored.back() == target);   // the store follows op
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
  CHECK(std::string(g.core->snapshot(0).scan_state) == "frozen");
  REQUIRE(g.core->snapshot(0).scan_pick.has_value());
  CHECK(*g.core->snapshot(0).scan_pick == g.core->op());
}

TEST(inflight_dwell_feeds_ranker_and_dwell_stats) {
  Rig g(2, 0); freeze_auto(g);               // auto, frozen: scout idle after park, reactive layer live
  link_up(g);
  for (int i = 0; i < 10; ++i) g.tick(true, false, 0);
  // in-flight step: card 1 (non-TX, scout-capable) dwells on the next candidate
  g.core->run_inflight_step();
  g.tick(true, false, 0);                    // drain
  CHECK(!g.core->dwell_busy());
  const auto s = g.core->snapshot(0);
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
  CHECK(!g.core->snapshot(0).dwell[0].has_value());   // one card: never dwells
  Rig h(2, 0, true, 40);
  for (int i = 0; i < 10; ++i) h.tick(false);
  h.core->run_inflight_step();
  h.tick(false);
  CHECK(!h.core->snapshot(0).dwell[1].has_value());   // no session: never dwells
  // pinned two-card, linked: never dwells either (Review Focus 4)
  Rig p(2, 0, /*pinned=*/true, 40);
  link_up(p);
  for (int i = 0; i < 10; ++i) p.tick(true);
  const size_t sweep = p.sink.dwells.size();   // the boot search sweep's own records
  p.core->run_inflight_step();
  p.tick(true);
  CHECK(p.sink.dwells.size() == sweep);
  CHECK(!p.core->snapshot(0).dwell[1].has_value());
}

TEST(tx_frozen_while_hopping) {
  Rig g(2, 0); freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere(g, 4);
  REQUIRE(g.sink.has_hop("order"));
  const auto out = g.core->tick(g.in(true));
  CHECK(out.tx_frozen);
  CHECK(g.core->tx_frozen());
}

TEST(snapshot_channel_is_the_given_tx_card) {
  Rig g(2, 0, true, 40);
  g.cards[1]->ch = 112;
  CHECK(g.core->snapshot(0).channel == 40);
  CHECK(g.core->snapshot(1).channel == 112);
}

TEST(shutdown_joins_threads_and_is_idempotent) {   // Review Focus 5
  // Real threads, real clock. Auto + LINKED, max_ms at its floor: the boot
  // scout thread measures for ~1 s, the pick freezes in place, the scout
  // parks, and the in-flight thread starts and dwells on the non-TX card
  // every dwell_period_ms; shutdown() must join both. (Pinned would start
  // no in-flight thread at all since 2026-10-04.)
  Config cfg = bundle();
  cfg.radio.channels = {40, 64}; cfg.radio.width = 40;
  cfg.radio.scan.max_ms = 1000;
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
  // Up to 5 s of linked ticks: a dwell record drained from the in-flight
  // thread proves that thread ran (dwell_period_ms is 333 in the bundle).
  bool dwelt = false;
  for (int i = 0; i < 500 && !dwelt; ++i) {
    in.now_ms = static_cast<double>(now_ms());
    core.tick(in);
    dwelt = core.snapshot(0).dwell[1].has_value();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  REQUIRE(dwelt);
  const uint32_t visits = core.snapshot(0).dwell[1]->visits;
  const uint64_t t0 = now_ms();
  core.shutdown();
  core.shutdown();                            // idempotent
  CHECK(now_ms() - t0 < 2000);                // joined both threads promptly
  std::this_thread::sleep_for(std::chrono::milliseconds(400));   // > dwell_period_ms
  in.now_ms = static_cast<double>(now_ms());
  const auto out = core.tick(in);                                // drains anything a live thread left
  CHECK(!out.dwell_busy);
  CHECK(core.snapshot(0).dwell[1]->visits == visits);             // no thread left dwelling
}

TEST(relay_only_roster_searches_without_measuring) {
  Rig g(0, 1);                                // one relay, auto
  const auto s = g.core->snapshot(0);
  CHECK(std::string(s.scan_state) == "off");  // nothing can measure
  CHECK(!g.core->pick_open());
  // the relay scouts: search bursts retune it across the set
  for (int i = 0; i < 40; ++i) g.tick();
  bool retuned_off_start = false;
  for (auto& c : g.cards[0]->calls) if (c == "retune 64" || c == "retune_width 64/20") retuned_off_start = true;
  CHECK(retuned_off_start);
  CHECK(g.sink.picks.empty());                // no K line ever: nothing measured
}

// Bench 2026-10-04: web GS on a CPE relay, auto channel set -- the drone
// linked, then sat in FAILSAFE. BootPick(false) never froze the scout, so
// pick_open() stayed true, scout_owns_() stayed true for life, and the gate
// dropped every RCF on the relay (the scout card) once search stopped
// beaconing. Pinned mode skips that clause, which is why it climbed.
TEST(relay_only_roster_in_session_lets_rcfs_through) {
  Rig g(0, 1);                                // one relay, auto
  for (int i = 0; i < 40; ++i) g.tick();      // searching
  for (int i = 0; i < 200; ++i) g.tick(/*session=*/true);
  CHECK(!g.core->scout_owns_card(0));
  CHECK(g.core->may_send(0));
}

TEST(relay_only_roster_no_ready_relay_is_quiet) {   // Review Focus 4
  Rig g(0, 1);
  g.cards[0]->is_ready = false;
  for (int i = 0; i < 20; ++i) g.tick();
  CHECK(g.core->disc_targets(0).empty());
  CHECK(std::string(g.core->snapshot(0).scan_state) == "off");
  CHECK(g.cards[0]->calls.empty());           // scout never started (card not ready): nothing touched it
}

// ---- relay sweep (spec 2026-10-05-cpe-relay-hop) ----
static SweepResult sweep_of(std::initializer_list<std::pair<uint8_t, uint16_t>> ch_busy) {
  SweepResult r;
  for (int pass = 0; pass < 2; ++pass)
    for (auto [ch, busy] : ch_busy) {
      SweepEntry e; e.ch = ch; e.pass = (uint8_t)pass; e.valid = true;
      e.active_ms = 20; e.busy_ms = busy; e.rx_ms = 0;
      r.entries.push_back(e);
    }
  return r;
}

// The whole relay-only path: interference -> one SCAN for set-minus-op ->
// the result ranks every candidate in ONE burst -> order on N RCFs -> the
// sole relay retunes (its TUNE). Blocked 64 (raw busy 100 %) is skipped.
TEST(relay_only_sweep_ranks_in_one_burst_then_hops) {
  Rig g(0, 1);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  interfere(g, 3);
  REQUIRE(g.cards[0]->sweeps.size() == 1);
  CHECK((g.cards[0]->sweeps[0] == std::vector<uint8_t>{64, 112, 144}));   // set minus op 40
  CHECK(g.core->sweep_pending());
  CHECK(!g.sink.has_hop("order"));                 // waits for the result, no hold_exhausted either
  CHECK(!g.sink.has_hop("hold_exhausted"));
  g.cards[0]->pending_result = sweep_of({{64, 20}, {112, 1}, {144, 4}});
  interfere(g, 3);                                 // persist 2-of-3 again once the card is usable
  CHECK(!g.core->sweep_pending());
  REQUIRE(g.sink.has_hop("order"));
  pump_rcf(g);
  std::optional<mabur::rc::Rcf> r;
  for (int i = 0; i < 10 && !g.sink.has_hop("one_card_retune"); ++i) r = pump_rcf(g);
  REQUIRE(g.sink.has_hop("one_card_retune"));
  REQUIRE(r.has_value());
  CHECK(r->hop_ch == 112);                         // least busy unblocked; 64 is blocked (100 %)
  CHECK(g.cards[0]->ch == 112);
  size_t relay_dwells = 0;
  for (auto& [card, d] : g.sink.dwells) if (d.rx_valid && card == 0) ++relay_dwells;
  CHECK(relay_dwells == 6);                        // 3 channels x 2 passes logged as D records
}

TEST(relay_only_sweep_timeout_allows_next_burst) {   // Review Focus 3
  Rig g(0, 1);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  interfere(g, 3);
  REQUIRE(g.cards[0]->sweeps.size() == 1);
  g.cards[0]->sweep_in_flight = false;             // the relay never answers
  // Checked AT the timeout, not 1100 ms on: the verdict's 2-of-3 window
  // still holds the pre-sweep interfered windows (a no-valid-card window
  // never pushes), so the first usable window after the timeout re-triggers
  // and -- relay_burst_period_ms having passed -- sends the next SCAN at once.
  for (int i = 0; i < 60; ++i) g.tick(true);       // 600 ms on (the SCAN left ~220 ms earlier): still waiting
  CHECK(g.core->sweep_pending());
  CHECK(g.core->sweep_timeouts() == 0);
  for (int i = 0; i < 50 && g.core->sweep_timeouts() == 0; ++i) g.tick(true);   // crosses 1000 ms
  CHECK(!g.core->sweep_pending());
  CHECK(g.core->sweep_timeouts() == 1);
  CHECK(g.core->snapshot(0).hop.sweep_timeouts == 1);   // -> sideport hop.sweep_timeouts
  interfere(g, 3);
  CHECK(g.cards[0]->sweeps.size() == 2);           // relay_burst_period_ms (1000) has passed
}

// Fix round 1: a lost SCAN / SCAN_RESULT. RemoteCard::sweeping() stays true
// until a matching result, the next start_scan or a reopen -- so the fake
// keeps sweep_in_flight set throughout. The verdict must not skip the relay
// on the card's own flag past the core's timeout, or a one-card roster
// reads Unknown forever, never triggers, never re-scans: detection dead.
TEST(relay_lost_scan_result_does_not_blind_the_verdict) {
  Rig g(0, 1);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  interfere(g, 3);
  REQUIRE(g.cards[0]->sweeps.size() == 1);
  for (int i = 0; i < 150 && g.core->sweep_timeouts() == 0; ++i) g.tick(true);
  REQUIRE(g.core->sweep_timeouts() == 1);
  CHECK(g.cards[0]->sweeping());                   // the relay still thinks a scan is pending
  interfere(g, 3);
  CHECK(g.cards[0]->sweeps.size() == 2);           // interfered verdicts again -> the next SCAN
}

// Fix round 1: a sweep pending across a session drop. On reconnect an
// expired sweep is a timeout; a result that sat on the card meanwhile is
// taken and discarded, never ranked with the reconnect's timestamp.
TEST(relay_sweep_expired_across_session_drop_discards_late_result) {
  Rig g(0, 1);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  interfere(g, 3);
  REQUIRE(g.cards[0]->sweeps.size() == 1);
  REQUIRE(g.core->sweep_pending());
  const size_t dwells0 = g.sink.dwells.size();
  for (int i = 0; i < 120; ++i) g.tick(false);     // session down > 1 s
  CHECK(g.core->sweep_timeouts() == 0);            // nothing polled while hop-inactive
  g.cards[0]->pending_result = sweep_of({{64, 20}, {112, 1}, {144, 4}});
  g.tick(true);                                    // session back
  CHECK(!g.core->sweep_pending());
  CHECK(g.core->sweep_timeouts() == 1);
  CHECK(!g.cards[0]->pending_result.has_value());  // taken from the card...
  CHECK(!g.cards[0]->sweeping());
  CHECK(g.sink.dwells.size() == dwells0);          // ...and discarded: no D records, no visits
}

// The relay burst paces on hop.relay_burst_period_ms (1000), not the USB
// burst's dwell_period_ms: an empty result lands at once, and a burst 400 ms
// after the first SCAN must not happen.
TEST(relay_burst_waits_relay_burst_period_ms) {
  Rig g(0, 1);
  freeze_auto(g);
  REQUIRE(g.cfg.hop.dwell_period_ms < 400);
  REQUIRE(g.cfg.hop.relay_burst_period_ms > 600);
  for (int i = 0; i < 5; ++i) g.tick(true);
  interfere(g, 3);
  REQUIRE(g.cards[0]->sweeps.size() == 1);
  g.cards[0]->sweep_in_flight = false;
  g.cards[0]->pending_result = SweepResult{};      // an empty answer: nothing ranked
  for (int i = 0; i < 40; ++i) g.tick(true);
  CHECK(!g.core->sweep_pending());
  interfere(g, 1);                                 // the trigger is still high
  CHECK(g.cards[0]->sweeps.size() == 1);
}

// Rebuild a relay-only rig's VrxController + core around another channel
// set (start 40, n_usb 0).
static void rebuild_with_set(Rig& g, std::vector<uint8_t> set) {
  g.cfg.radio.channels = std::move(set);
  g.core.reset();
  g.vrx = std::make_unique<VrxController>(vrx_cfg_from(g.cfg, 40));
  ChannelCoreCfg cc; cc.radio = g.cfg.radio; cc.hop = g.cfg.hop; cc.key = g.cfg.link.key;
  cc.start_ch = 40; cc.n_usb = 0; cc.threaded = false; cc.store_name = "s";
  g.core = std::make_unique<ChannelCore>(cc, g.ptrs, *g.vrx, g.sink,
      [](uint8_t) { return true; }, [&g] { return g.clk.now_ms(); }, [&g] { return g.clk.now_us(); },
      [&g](int ms) { g.clk.sleep(ms); });
}

TEST(relay_only_single_member_set_sends_no_scan) {   // Review Focus 4
  Rig g(0, 1);
  rebuild_with_set(g, {40});
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  interfere(g, 4);
  CHECK(g.cards[0]->sweeps.empty());
  CHECK(!g.core->sweep_pending());
}

TEST(relay_survey_window_feeds_blocked) {
  Rig g(0, 1);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->survey_win = SurveyWindow{true, 97.0, 2.0};   // analog-VTX shape: busy, not decodable
  interfere(g, 2);
  REQUIRE(g.sink.last_cards.size() == 1);
  CHECK(g.sink.last_cards[0].busy_valid);
  CHECK(std::abs(g.sink.last_cards[0].nhm_busy_pct - 97.0) < 1e-9);
  CHECK(std::abs(g.sink.last_cards[0].own_air_pct - 2.0) < 1e-9);
  CHECK(g.core->snapshot(0).hop.evidence & kEvBlocked);
}

TEST(relay_plus_usb_tx_usb_lets_the_relay_sweep) {
  Rig g(1, 1);
  // freeze_auto's >=2-card branch blocks the LAST card (the relay), but here
  // the boot scout is card 0 (the one USB card): block it too, so no dwell
  // ever completes and max_ms freezes op in place on 40.
  g.cards[0]->retune_ok = false;
  freeze_auto(g);
  g.cards[0]->retune_ok = true;
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true, false, /*tx=*/0);
  REQUIRE(g.cards[0]->ch == 40);
  g.cards[0]->calls.clear();                       // the refused boot-scan retunes above are not the hop's
  g.cards[0]->cca_per_ms_on[40] = 50;
  interfere(g, 3);                                 // tx_card 0 (USB) by default in g.in()
  CHECK(g.cards[1]->sweeps.size() == 1);           // the relay spare swept
  bool usb_retuned_off_op = false;
  for (auto& c : g.cards[0]->calls) if (c == "retune 64" || c == "retune 112" || c == "retune 144") usb_retuned_off_op = true;
  CHECK(!usb_retuned_off_op);                      // the USB TX card never left the link
}

// Final review item 4: the sweep timeout is derived from the request --
// max(1000, passes * n * (observe_ms + 40) + 300). An 8-member set sweeps 7
// channels x 2 passes x 60 ms + 300 = 1140 ms: a pending SCAN must NOT time
// out at 1000 ms.
TEST(relay_sweep_timeout_scales_with_the_request) {
  Rig g(0, 1);
  rebuild_with_set(g, {40, 36, 44, 48, 64, 112, 144, 149});
  link_up(g);
  for (int i = 0; i < 100 && g.core->pick_open(); ++i) g.tick(true);
  REQUIRE(!g.core->pick_open());
  for (int i = 0; i < 5; ++i) g.tick(true);
  uint64_t sent_ms = 0;
  for (int w = 0; w < 4 && g.cards[0]->sweeps.empty(); ++w) {
    for (auto& c : g.cards) c->fr.foreign += 40;
    for (int i = 0; i < g.cfg.hop.window_ms / 10 + 1 && g.cards[0]->sweeps.empty(); ++i) {
      g.vrx->on_video(static_cast<double>(g.clk.ms));
      sent_ms = g.clk.ms;
      g.core->tick(g.in(true));
      g.clk.ms += 10;
    }
  }
  REQUIRE(g.cards[0]->sweeps.size() == 1);
  REQUIRE(g.cards[0]->sweeps[0].size() == 7);
  g.cards[0]->sweep_in_flight = false;             // never answers
  while (g.clk.ms < sent_ms + 1100) g.tick(true);
  CHECK(g.core->sweep_pending());                  // the fixed 1000 ms would have expired it
  CHECK(g.core->sweep_timeouts() == 0);
  while (g.clk.ms < sent_ms + 1150) g.tick(true);
  CHECK(!g.core->sweep_pending());
  CHECK(g.core->sweep_timeouts() == 1);
}

// Final review item 1: relay + USB with the relay down (booting / not owned)
// and the USB card transmitting. The dead relay must not take the burst:
// the USB TX card bursts (pick rule 3), no SCAN is attempted.
TEST(relay_plus_usb_relay_not_ready_usb_tx_bursts) {
  Rig g(1, 1);
  g.cards[0]->retune_ok = false;
  freeze_auto(g);
  g.cards[0]->retune_ok = true;
  link_up(g);
  g.cards[1]->is_ready = false;                    // the relay is down
  for (int i = 0; i < 5; ++i) g.tick(true, false, /*tx=*/0);
  const size_t d0 = g.sink.dwells.size();
  g.cards[0]->cca_per_ms_on[40] = 50;
  interfere(g, 3, /*tx=*/0);
  CHECK(g.cards[1]->sweeps.empty());
  CHECK(!g.core->sweep_pending());
  size_t usb_dwells = 0;
  for (size_t i = d0; i < g.sink.dwells.size(); ++i) if (g.sink.dwells[i].first == 0) ++usb_dwells;
  CHECK(usb_dwells > 0);                           // the USB TX card burst
}

// Final review item 2: relay + USB, the relay transmits, the USB card is
// the spare: the USB card bursts (pick rule 1), no SCAN.
TEST(relay_plus_usb_usb_spare_bursts) {
  Rig g(1, 1);
  g.cards[0]->retune_ok = false;
  freeze_auto(g);
  g.cards[0]->retune_ok = true;
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true, false, /*tx=*/1);
  const size_t d0 = g.sink.dwells.size();
  g.cards[0]->cca_per_ms_on[40] = 50;
  interfere(g, 3, /*tx=*/1);
  CHECK(g.cards[1]->sweeps.empty());
  size_t usb_dwells = 0;
  for (size_t i = d0; i < g.sink.dwells.size(); ++i) if (g.sink.dwells[i].first == 0) ++usb_dwells;
  CHECK(usb_dwells > 0);
}

// Final review item 3: while the relay sweeps, the TX selector must not
// move TX onto it -- the sweep freezes the TX selection like a dwell.
TEST(tx_frozen_while_relay_sweeps) {
  Rig g(1, 1);
  g.cards[0]->retune_ok = false;
  freeze_auto(g);
  g.cards[0]->retune_ok = true;
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true, false, 0);
  g.cards[0]->cca_per_ms_on[40] = 50;
  interfere(g, 3);
  REQUIRE(g.core->sweep_pending());
  REQUIRE(!g.core->hopping());
  REQUIRE(!g.core->dwell_busy());
  const auto out = g.core->tick(g.in(true));
  CHECK(out.tx_frozen);
  CHECK(g.core->tx_frozen());
  g.cards[1]->pending_result = SweepResult{};      // the result lands: frozen no more
  const auto out2 = g.core->tick(g.in(true));
  CHECK(!g.core->sweep_pending());
  CHECK(!out2.tx_frozen);
}

MTEST_MAIN
