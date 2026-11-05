#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "mtest.h"
#include "cal_picker.h"
#include "channel_core.h"
#include "fake_link_card.h"
#include "hop_blank.h"
#include "mabur/rc_proto.h"
#include "nhm_busy.h"
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
  // The width the rig was built at (ChannelCoreCfg::width_mhz is no longer
  // read back off cfg.radio, which no longer carries it); rebuild_with_set
  // reuses it unchanged.
  uint8_t width = 40;
  // {may_send(0), may_send(1)} sampled from inside the scout's sleep
  // callback -- the synchronous equivalent of the core thread reading the
  // gate while the scout thread is mid-dwell on another thread in prod.
  std::vector<std::pair<bool, bool>> gate_obs;

  // Called after every injected sleep (the scout's and the in-flight
  // dwell's): a test acts from inside a dwell through it.
  std::function<void(int)> on_sleep;

  Rig(int n_usb, int n_relays, bool pinned = false, uint8_t start = 0)
      : Rig(n_usb, n_relays, pinned, start, {40, 64, 112, 144}, 40) {}
  // The set and the width go into the config before the core is built.
  Rig(int n_usb, int n_relays, bool pinned, uint8_t start, std::vector<uint8_t> channels,
      uint8_t width)
      : agg(bundle().uep_layers(), 32, n_usb + n_relays, 0) {
    this->width = width;
    cfg.radio.channels = std::move(channels);
    // A 40 MHz rig flies the five-rung 40 MHz ladder; the hop-restore tests
    // index into it (rung 2 = MCS 2). A 20 MHz rig flies the same rungs at 20.
    cfg.link.ladder_cfg.ladder = {{0, 0.5, 0.25, width}, {1, 0.5, 0.25, width},
                                  {2, 0.5, 0.25, width}, {3, 0.5, 0.25, width},
                                  {4, 0.5, 0.25, width}};
    if (pinned) cfg.radio.pin = start ? start : 40;
    const uint8_t start_ch = start ? start : (pinned ? *cfg.radio.pin : 40);
    for (int i = 0; i < n_usb + n_relays; ++i) {
      auto c = std::make_unique<FakeCard>();
      c->clk = &clk; c->ch = start_ch; c->relay = i >= n_usb;
      c->width_mhz = width;   // main.cpp: every card, the boot scout's included
      ptrs.push_back(c.get());
      cards.push_back(std::move(c));
    }
    vrx = std::make_unique<VrxController>(vrx_cfg_from(cfg, start_ch));
    ChannelCoreCfg cc;
    cc.radio = cfg.radio; cc.hop = cfg.hop; cc.key = cfg.link.key;
    cc.width_mhz = width;
    cc.start_ch = start_ch; cc.n_usb = n_usb; cc.threaded = false;
    cc.store_name = "the-store";
    core = std::make_unique<ChannelCore>(
        cc, ptrs, *vrx, sink,
        [this](uint8_t ch) { stored.push_back(ch); return store_ok; },
        [this] { return clk.now_ms(); }, [this] { return clk.now_us(); },
        [this](int ms) {
          clk.sleep(ms);
          if (core) gate_obs.emplace_back(core->may_send(0), core->may_send(1));
          if (on_sleep) on_sleep(ms);
        });
  }
  std::vector<uint16_t> seq_next;   // decode_one(): per-card next mac_seq
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
  // returns can never observe either branch. The
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
  g.cards[1]->ch = 60;                                   // not a member
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

// Needs step_scout_inputs_ -- the search request goes false once the plan
// is in session.
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
// continuation ("one-card linked") has its own test,
// one_card_link_edge_freezes_one_card_linked (step_move_edge_ is the only
// thing that can ever set link_edge_seen_).
TEST(one_card_prelude_commits_before_first_disc) {
  Rig g(1, 0);
  g.cards[0]->nhm_busy_pct_on[40] = 100;   // 40 blocked, the rest unread
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
  g.cards[1]->nhm_busy_pct_on[40] = 100;   // op's pair blocked
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

TEST(store_failure_is_logged_not_fatal) {
  Rig g(2, 0);
  g.store_ok = false;
  g.cards[1]->nhm_busy_pct_on[40] = 100;   // op's pair blocked
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
// the only thing that can fix a card that reopened at the wrong width.
TEST(reopened_scout_card_at_20_gets_width_resync) {
  Rig g(2, 0, /*pinned=*/true, 40);
  for (int i = 0; i < 10; ++i) g.tick(true, false, 0);   // linked: search off, the scout parks and owns nothing
  REQUIRE(g.cards[1]->width_mhz == 40);                   // opened at the derived receiver width, never switched
  g.cards[1]->is_alive = false;
  g.core->on_card_died(1);
  // a revive that comes up at the wrong width
  g.cards[1]->is_alive = true; g.cards[1]->ch = 40; g.cards[1]->width_mhz = 20;
  g.cards[1]->calls.clear();
  g.core->on_card_reopened(1);
  for (int i = 0; i < 50; ++i) g.tick(true, false, 0);
  bool resynced = false;
  for (auto& c : g.cards[1]->calls) if (c.rfind("set_width", 0) == 0) resynced = true;
  CHECK(resynced);                                        // the CORE's width resync: the scout never sets a width
  CHECK(g.cards[1]->width_mhz == 40);
}

// Pinned + linked: no scout work, so the flip retunes both cards at once.
TEST(set_width_retunes_every_ready_card_before_returning) {
  Rig g(2, 0, /*pinned=*/true, 40);
  for (int i = 0; i < 10; ++i) g.tick(true, false, 0);
  CHECK(g.core->width_mhz() == 40);
  CHECK(!g.core->width_refusal(20));
  for (auto& c : g.cards) c->calls.clear();
  g.core->set_width(20);
  CHECK(g.core->width_mhz() == 20);
  for (auto& c : g.cards) {
    bool set = false;
    for (auto& s : c->calls) if (s.rfind("set_width", 0) == 0) set = true;
    CHECK(set);
    CHECK(c->width_mhz == 20);
  }
  CHECK(g.sink.has_line("maburgs link: width 40 -> 20 MHz (ladder set)"));
  for (auto& c : g.cards) c->calls.clear();
  g.core->set_width(20);                                     // same width: nothing happens
  for (auto& c : g.cards) CHECK(c->calls.empty());
  CHECK(g.core->width_refusal(20) == std::nullopt);
}

TEST(set_width_orders_relay_cards_too) {
  Rig g(1, 1, /*pinned=*/true, 40);
  for (int i = 0; i < 10; ++i) g.tick(true, false, 0);
  g.core->set_width(20);
  CHECK(g.cards[1]->width_mhz == 20);                        // the relay, ordered like a local card
}

TEST(width_refusal_while_the_boot_scan_owns_a_card) {
  Rig g(2, 0);                                               // auto: the scout owns card 1
  g.tick(false, false, 0);
  CHECK(g.core->pick_open());
  auto r = g.core->width_refusal(20);
  REQUIRE(r.has_value());
  CHECK(*r == "ladder refused: scan in flight");
}

TEST(width_refusal_names_a_non_pair_member_on_a_flip_to_40) {
  // A 20 MHz rig on a set 165 cannot fly at 40: 165 has no HT40 pair.
  Rig g(2, 0, /*pinned=*/true, 40, {40, 165}, 20);
  for (int i = 0; i < 10; ++i) g.tick(true, false, 0);
  REQUIRE(g.core->width_mhz() == 20);
  auto r = g.core->width_refusal(40);
  REQUIRE(r.has_value());
  CHECK(*r == "ladder refused: channel set is not 40 MHz pair-valid (member 165)");
  CHECK(!g.core->width_refusal(20));                         // staying at 20 is no flip
}

TEST(set_width_on_a_down_card_lands_on_revive) {
  Rig g(2, 0, /*pinned=*/true, 40);
  for (int i = 0; i < 10; ++i) g.tick(true, false, 0);
  g.cards[1]->is_alive = false; g.cards[1]->is_ready = false; g.core->on_card_died(1);
  g.core->set_width(20);
  CHECK(g.cards[0]->width_mhz == 20);
  CHECK(g.cards[1]->width_mhz == 40);                        // down: not touched by the flip
  g.cards[1]->is_alive = true; g.cards[1]->is_ready = true;
  g.cards[1]->width_mhz = 40; g.cards[1]->calls.clear();
  g.core->on_card_reopened(1);
  for (int i = 0; i < 50; ++i) g.tick(true, false, 0);
  CHECK(g.cards[1]->width_mhz == 20);                        // the per-tick resync, not the flip
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
  // core->tick() directly; the check is specifically
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
// The op's air held by a foreign transmitter for the lifetime of this
// object: NHM reads 100 % busy on every USB card, armed at the sampler's
// op-window period (nhm_period_4us(window_ms - 10)) so a scout dwell -- which
// arms its own period -- reads nothing and the candidates stay clean; a
// relay's SURVEY reads 97 % busy. A card the test already gave NHM air of
// its own (nhm_any_period) keeps it. The previous state comes back on
// destruction: the interference lives on op for exactly that span, and a
// channel landed on later reads whatever the test set.
struct OpJam {
  Rig& g;
  struct Saved { bool arm_ok; NhmBusyRead read; SurveyWindow survey; };
  std::vector<Saved> saved;
  explicit OpJam(Rig& rig) : g(rig) {
    NhmBusyRead full; full.valid = true; full.period = nhm_period_4us(g.cfg.hop.window_ms - 10);
    full.buckets[11] = 100;                        // every sample above busy_dbm
    for (auto& c : g.cards) {
      saved.push_back({c->nhm_arm_ok, c->nhm_read, c->survey_win});
      if (c->relay) { c->survey_win = SurveyWindow{true, 97.0, 2.0}; continue; }
      if (c->nhm_any_period) continue;
      c->nhm_arm_ok = true; c->nhm_read = full;
    }
  }
  ~OpJam() {
    for (size_t i = 0; i < g.cards.size(); ++i) {
      if (g.cards[i]->nhm_any_period) continue;
      g.cards[i]->nhm_arm_ok = saved[i].arm_ok; g.cards[i]->nhm_read = saved[i].read;
      g.cards[i]->survey_win = saved[i].survey;
    }
  }
};
// Detection in this rig is NHM occupancy only (the detect route needs a
// loss window, and pre_fec_loss/recovered stay 0 here): `interfered` needs
// the op blocked for hop.verdict.occupied_windows consecutive windows with
// a busy reading. The windows a blocked op needs on top of the old
// three-window trigger shape.
static int occ_lead(const Rig& g) { return g.cfg.hop.verdict.occupied_windows - 1; }
// One decoded frame per card, walking its seq. A jammed window in flight
// still decodes some of the drone's frames; the window fill gives a card
// that decoded nothing no busy reading at all (its own airtime cannot be
// told from what the air carried), so every jammed window below decodes.
// The body misroutes (not SBI, not RC): no FEC loss, no recovered symbols.
static mabur::node::RxBody junk_body(uint8_t card, uint16_t seq);
static void decode_one(Rig& g) {
  if (g.seq_next.size() < g.cards.size()) g.seq_next.resize(g.cards.size(), 100);
  for (size_t i = 0; i < g.cards.size(); ++i)
    g.agg.on_rx_body(junk_body(static_cast<uint8_t>(i), g.seq_next[i]++));
}

// Interfered verdict windows: the op jammed (OpJam: every NHM sample above
// busy_dbm on every usable card), foreign frames added alongside as
// context only -- the verdict does not read them. This rig's Aggregator never
// decodes a real body, so pre_fec_loss/recovered stay 0: the only route to
// `impaired` here is occupancy. The first window arms NHM (no reading yet);
// the occupied_windows-th blocked window after it is the first `impaired`
// window (and also the first `interfered`, since occupancy is both at
// once), the next one the trigger (persist 2): occ_lead(g) + 3 windows.
static void interfere(Rig& g, int windows, int tx = 0) {
  OpJam jam(g);
  for (int w = 0; w < windows; ++w) {
    for (auto& c : g.cards) { c->fr.foreign += 40; }
    for (int i = 0; i < g.cfg.hop.window_ms / 10 + 1; ++i) {
      g.vrx->on_video(static_cast<double>(g.clk.ms));
      decode_one(g);
      g.core->tick(g.in(true, false, tx));
      g.clk.ms += 10;
    }
  }
}

// Interfered windows until the controller orders, bounded. The trigger
// lands occupied_windows in, and the order then waits on a burst ranking the
// candidates (two fresh visits each), so a fixed window count would pin the
// test to how the trigger falls against the burst schedule.
static void interfere_until_order(Rig& g, int tx = 0) {
  for (int w = 0; w < occ_lead(g) + 12 && !g.sink.has_hop("order"); ++w) interfere(g, 1, tx);
  REQUIRE(g.sink.has_hop("order"));
}

// Bring an AUTO rig to a frozen pick on 40 without a hop, so the reactive
// layer is live exactly as in flight. Two cards: no dwell ever completes
// (the scout card refuses to retune), so max_ms freezes op in place. One
// card: link-up freezes it ("one-card linked"). Not a pinned rig: pinned
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

// The hop's restore rung is frozen at the onset as an index into the
// ladder of that moment. A live ladder swap inside the frozen episode
// translates it by {mcs, bw}, so the hop restores the pre-onset operating
// point and not whatever the new ladder holds at the old index. Revert =
// replace_ladder without the remap: the order restores index 2 of the new
// ladder, MCS 4.
static void onset_then_swap(Rig& g, std::vector<Rung> new_rungs) {
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.vrx->restore_rung(2, static_cast<double>(g.clk.ms));   // mcs 0..4 @40: index 2 = MCS 2
  REQUIRE(g.vrx->cur_op().mcs == 2);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  g.cards[0]->fr.foreign = 0; g.cards[1]->fr.foreign = 0;
  // Onset: the ref_rung snapshot freezes on the first `impaired` window,
  // which here (no loss in this rig) is the first occupied one, occ_lead(g)
  // + 2 windows into the jam. The ladder itself never demotes in this rig
  // (no real decode to drive it), so the rung is still 2 at that window.
  interfere(g, occ_lead(g) + 2);
  REQUIRE(*g.core->snapshot(0).hop.ref_rung == 2);
  REQUIRE(!g.sink.has_hop("order"));
  LadderCfg nl = g.vrx->ladder();
  nl.ladder = std::move(new_rungs);
  g.core->replace_ladder(nl, static_cast<double>(g.clk.ms));
}
TEST(hop_restore_survives_ladder_swap) {
  Rig g(2, 0); freeze_auto(g);
  onset_then_swap(g, {{2, 0.5, 0.25, 40}, {3, 0.5, 0.25, 40}, {4, 0.5, 0.25, 40}});
  REQUIRE(g.vrx->cur_op().mcs == 2);                        // the swap itself keeps MCS 2
  CHECK(*g.core->snapshot(0).hop.ref_rung == 0);
  interfere_until_order(g);                                 // interference goes on: hop ordered
  CHECK(g.vrx->cur_op().mcs == 2);                          // the pre-onset MCS
}
// A pre-onset rung the new ladder lacks restores rung 0, as the swap
// itself falls back -- never a clamp onto the top rung of a shorter ladder.
TEST(hop_restore_after_swap_dropping_the_ref_rung_lands_on_rung_0) {
  Rig g(2, 0); freeze_auto(g);
  onset_then_swap(g, {{0, 0.5, 0.25, 40}, {4, 0.5, 0.25, 40}});
  REQUIRE(g.vrx->cur_op().mcs == 0);
  CHECK(*g.core->snapshot(0).hop.ref_rung == 0);
  interfere_until_order(g);
  CHECK(g.vrx->cur_op().mcs == 0);
}

// Pinned = static. An interfered op produces a verdict (the OSD/sideport
// still say the pin is dirty) but no order, no escape, no exhausted hold:
// the controller is never fed a reactive trigger.
TEST(pinned_two_card_never_hops_on_interference) {
  Rig g(2, 0, /*pinned=*/true, 40);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  g.cards[0]->fr.foreign = 0; g.cards[1]->fr.foreign = 0;
  interfere(g, occ_lead(g) + 6);
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
// the only reactive source is the freshness burst + order.
TEST(pinned_one_card_never_hops_on_interference) {
  Rig g(1, 0, /*pinned=*/true, 40);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50;
  interfere(g, occ_lead(g) + 6);
  CHECK(std::string(g.core->snapshot(0).hop.verdict) == "interfered");
  CHECK(g.sink.hops.empty());
  CHECK(g.cards[0]->ch == 40);
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value());
  CHECK(r->hop_ch == 0);
}

// Bench (row 9): the relocation onto pinned 40 landed, then one
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
  interfere(g, occ_lead(g) + 8);
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
  interfere_until_order(g);
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

// The occupied count carries across a hop: a reactive hop lands with it
// already at occupied_windows, so a target that reads blocked fails verify
// on its FIRST blocked reading there (~300 ms: the settle, then one window)
// instead of rebuilding the count over occupied_windows readings.
// Revert (verdict_.reset() on HopAction::Confirm in apply_hop_action_): the
// count rebuilds and verify_fail comes ~900 ms after landing -- past the
// bound below, and with occupied_windows one higher, past verify_ms.
TEST(reactive_hop_onto_a_blocked_target_fails_verify_on_its_first_blocked_reading) {
  Rig g(2, 0); freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere_until_order(g);
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value());
  REQUIRE(r->hop_ch != 0 && r->hop_ch != 40);
  OpJam jam(g);                              // the air is blocked wherever the sampler arms
  g.core->note_video(r->hop_ch);
  g.tick(true);
  REQUIRE(g.sink.has_hop("lead_confirm"));
  const uint64_t landed = g.clk.ms;
  for (int i = 0; i < g.cfg.hop.verify_ms / 10 + 30 && !g.sink.has_hop("verify_fail") &&
                  !g.sink.has_hop("verify_pass"); ++i) {
    g.vrx->on_video(static_cast<double>(g.clk.ms));
    decode_one(g);
    g.tick(true);
  }
  CHECK(g.sink.has_hop("verify_fail"));
  CHECK(!g.sink.has_hop("verify_pass"));
  // The settle, the NHM re-arm on the target and one blocked window: well
  // inside the occupied_windows x window_ms a rebuilt count would need.
  CHECK(g.clk.ms - landed < kHopSettleBlankMs + 4 * g.cfg.hop.window_ms);
}

// Unlike blocked_streak_, the detect ring must NOT carry: landing on a
// clean target with HopVerdict::landed() uncalled would mix the old
// channel's ring entries into the target's first reading. This rig's
// Aggregator never decodes a real body (see interfere()'s comment above),
// so loss_impaired -- and with it kEvDetected -- can never fire here
// regardless of the ring's contents, which means this test cannot by
// itself FAIL on a reverted landed() call (checked locally: it passes
// either way). It still exercises the Confirm flow onto a clean target
// end to end; the actual pin on the ring-clearing behaviour is
// HopVerdict's own landed_clears_the_detect_ring_and_keeps_the_blocked_streak
// unit test, backed by a code-review check of the Confirm case below.
TEST(confirm_clears_the_detect_ring) {
  Rig g(2, 0); freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere_until_order(g);
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value());
  REQUIRE(r->hop_ch != 0 && r->hop_ch != 40);
  g.core->note_video(r->hop_ch);
  g.tick(true);
  REQUIRE(g.sink.has_hop("lead_confirm"));
  // the target's first window: low-foreign, no busy jam riding it
  for (int i = 0; i < g.cfg.hop.window_ms / 10 + 2; ++i) {
    g.vrx->on_video(static_cast<double>(g.clk.ms));
    g.tick(true);
  }
  const std::string v = g.core->snapshot(0).hop.verdict;
  CHECK((v == "healthy" || v == "unknown"));
}

// Onset -> order on two cards, bounded: the trigger needs occupied_windows
// blocked windows plus the persist, and the order then waits on at most one
// more freshness burst (burst_period_ms) to rank the candidates (1350 ms
// against a 1533 ms bound at the bundle defaults). Revert (`occupied` at
// blocked_streak_ >= occupied_windows + 2 in hop_verdict.cpp): the order
// lands at 1650 ms, past the bound.
TEST(two_card_onset_to_order_is_occupancy_plus_persist_plus_one_burst) {
  Rig g(2, 0); freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  const uint64_t onset = g.clk.ms;
  interfere_until_order(g);
  double order_ms = -1;
  for (const auto& h : g.sink.hops) if (h.kind == "order") { order_ms = h.t_ms; break; }
  REQUIRE(order_ms >= 0);
  const auto& v = g.cfg.hop.verdict;
  // + 1 window: the first jammed window only arms NHM.
  const double bound = (v.occupied_windows + g.cfg.hop.persist + 1) * g.cfg.hop.window_ms +
                       g.cfg.hop.burst_period_ms;
  CHECK(order_ms - static_cast<double>(onset) <= bound);
  CHECK(order_ms - static_cast<double>(onset) >= v.occupied_windows * g.cfg.hop.window_ms);
}

TEST(stale_pre_hop_verdict_does_not_break_verify) {
  // After Confirm the cached VerdictOut (measured on the old channel) is
  // re-fed every tick until the next window: the controller must stay in
  // Verifying (C1).
  Rig g(2, 0); freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere_until_order(g);
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
  interfere_until_order(g);
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
  // Upper bound widened from 400: dwell_observe_ms (20 ms, up from 5 ms)
  // lengthens the one-card ranking burst itself (3 candidates observed per
  // burst), pushing the order a little later and phase-shifting it against
  // vrx's own RCF/DISC cadence during the repeats that follow.
  for (auto& e : g.sink.hops)
    if (e.kind == "one_card_retune") CHECK(e.elapsed_ms >= 200 && e.elapsed_ms <= 600);
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
  interfere_until_order(g);
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value());
  for (int i = 0; i < (g.cfg.hop.confirm_extend_ms / 10) + 20; ++i) { g.vrx->on_video(static_cast<double>(g.clk.ms)); g.tick(true); }
  CHECK(g.sink.has_hop("withdraw"));
  CHECK(g.sink.has_move(MoveReason::HopWithdraw));
  CHECK(g.cards[1]->ch == 40);               // lead card back on op
  CHECK(g.core->op() == 40);
}

TEST(lead_card_dies_mid_order_withdraws_to_op) {
  Rig g(2, 0); freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere_until_order(g);
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
  interfere_until_order(g);
  g.tick(false);                              // hop_active falling edge
  // on_session_lost() returns HopAction::Withdraw, but the EVENT it logs
  // (hop_controller.cpp) is kind "session_lost", not "withdraw" -- that
  // literal kind is only ever logged by withdraw() (the confirm_ms/
  // confirm_extend_ms timeout path), a different caller.
  CHECK(g.sink.has_hop("session_lost"));
  CHECK(!g.core->hopping());
}

TEST(cal_running_holds_relocation_and_move_edge) {
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
  // The edge released on the same tick as the relocate must not be dropped
  // by the `!plan_.hopping()` guard: it is held through the hop and acted
  // on once the hop resolves (here: confirm + verify_pass).
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
  // Pinned, not auto: BootPick's own "op unmeasured" guard (boot_pick.cpp)
  // means an AUTO pick never relocates off a link found before any
  // scouting -- "a linked scout never measures op's own pair,
  // so a drone found at once leaves op with only its pre-link visits. A
  // working link is not moved on a one-sided comparison." That freezes
  // auto mode's pick in place on 64 forever (confirmed: 200+ ticks never
  // produce the relocate), which is BootPick's deliberate design, not a
  // gap the dispatch/controller code can or should route around.
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

// The AUTO boot hop. The scout measures op's pair to
// min_rounds BEFORE the drone appears on 64 (a linked scout never visits
// op's own pair, so without that BootPick freezes "op unmeasured"), and
// the link forms while the later pairs are still short of min_rounds (else
// the unlinked pick would already have committed). At maturity the pick
// wants another pair -> WantPick -> relocate -> confirm -> verify_pass ->
// "pick frozen on <pick> (relocated)".
TEST(auto_want_pick_relocates_linked_drone_to_the_pick) {
  Rig g(2, 0);
  g.cards[1]->nhm_busy_pct_on[64] = 100;   // 64's pair is blocked
  // The scout sweeps 40,64,112,144 each round: stop right after 64's
  // min_rounds-th visit, i.e. inside the last round.
  const auto visits = [&g](uint8_t ch) {
    int n = 0;
    for (const auto& d : g.sink.dwells) if (d.second.survey.def.primary == ch) ++n;
    return n;
  };
  const int mr = g.cfg.radio.scan.min_rounds;
  for (int i = 0; i < 3000 && visits(64) < mr; ++i) g.tick();
  REQUIRE(visits(64) >= mr);
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

// The move edge (step_move_edge_) is what sets link_edge_seen_, which BootPick reads next
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

// A flip while the in-flight scout has a card off on a dwell. In prod the
// dwell runs on the in-flight thread under inflight_mu_ and the flip on the
// core thread; here a second thread runs the dwell and parks inside its
// observe sleep until the flip's card walk has passed the dwelling card
// (card 0: TX is card 1) and reached card 1. The dwelling card is skipped,
// not marked tried, so the per-tick resync retunes it once the dwell ends;
// card 1 is retuned before set_width returns.
TEST(set_width_retunes_a_busy_card_on_a_later_tick) {
  Rig g(2, 0); freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 10; ++i) g.tick(true, false, /*tx=*/1);
  REQUIRE(g.cards[0]->width_mhz == 40 && g.cards[1]->width_mhz == 40);
  std::atomic<bool> in_dwell{false}, release{false}, armed{false}, released_in_time{false};
  const auto wait_for = [](const std::atomic<bool>& f) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!f.load() && std::chrono::steady_clock::now() < end) std::this_thread::yield();
    return f.load();
  };
  g.on_sleep = [&](int) {
    if (in_dwell.load()) return;
    in_dwell.store(true);
    released_in_time.store(wait_for(release));
  };
  g.cards[1]->on_ready = [&] { if (armed.load()) release.store(true); };
  std::thread dweller([&] { g.core->run_inflight_step(); });
  REQUIRE(wait_for(in_dwell));
  CHECK(g.core->dwell_busy());
  CHECK(!g.core->width_refusal(20));         // a dwell in flight is no refusal
  armed.store(true);
  g.core->set_width(20);
  armed.store(false);
  dweller.join();
  g.on_sleep = nullptr;
  g.cards[1]->on_ready = nullptr;
  CHECK(released_in_time.load());            // the walk reached card 1 while card 0 still dwelt
  CHECK(g.cards[1]->width_mhz == 20);        // idle: retuned before set_width returned
  CHECK(g.cards[0]->width_mhz == 40);        // busy: skipped now
  for (int i = 0; i < 3; ++i) g.tick(true, false, 1);
  CHECK(g.cards[0]->width_mhz == 20);        // picked up once idle
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
  // pinned two-card, linked: never dwells either
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
  interfere_until_order(g);
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

TEST(shutdown_joins_threads_and_is_idempotent) {
  // Real threads, real clock. Auto + LINKED, max_ms at its floor: the boot
  // scout thread measures for ~1 s, the pick freezes in place, the scout
  // parks, and the in-flight thread starts and dwells on the non-TX card
  // every dwell_period_ms; shutdown() must join both. (Pinned would start
  // no in-flight thread at all.) dwell_period_ms is overridden to 200 ms
  // (below the bundle's 1000 ms default) so the post-shutdown sleep below
  // comfortably outlasts one period -- a thread left running would dwell
  // again well inside it.
  Config cfg = bundle();
  cfg.radio.channels = {40, 64};
  cfg.radio.scan.max_ms = 1000;
  cfg.hop.dwell_period_ms = 200;
  FakeCard a, b; a.ch = b.ch = 40; a.width_mhz = 40; b.width_mhz = 40;
  std::vector<LinkCard*> ptrs{&a, &b};
  VrxController vrx(vrx_cfg_from(cfg, 40));
  RecordingSink sink;
  Aggregator agg(cfg.uep_layers(), 32, 2, 0);
  ChannelCoreCfg cc; cc.radio = cfg.radio; cc.hop = cfg.hop; cc.key = cfg.link.key; cc.start_ch = 40; cc.n_usb = 2;
  cc.width_mhz = 40;
  auto now_ms = [] { return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()); };
  auto now_us = [] { return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()); };
  ChannelCore core(cc, ptrs, vrx, sink, [](uint8_t) { return true; }, now_ms, now_us,
                   [](int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); });
  ChannelTickIn in; in.agg = &agg; in.in_session = true; in.tx_card = 0;
  // Up to 5 s of linked ticks: a dwell record drained from the in-flight
  // thread proves that thread ran (dwell_period_ms is 200 here, see above).
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

// Real threads, real clock. Two USB cards, the op occupied on both (100 %
// NHM busy), so the occupancy trigger goes live with no video at all. The
// spare card's freshness burst is posted to the in-flight thread: its dwell
// records arrive (dwell_period_ms is 60 s, so no periodic dwell can be the
// source), and no core tick ever waits for it -- one burst is three 20 ms
// observes, so a tick that ran it inline would take >= 60 ms.
// Revert (burst inline on the core thread): max_tick_ms reads >= 60.
TEST(threaded_two_card_burst_runs_off_the_core_thread) {
  Config cfg = bundle();
  cfg.radio.channels = {40, 64, 112, 144};
  cfg.radio.scan.max_ms = 1000;
  cfg.hop.dwell_period_ms = 60000;
  // The op window's arm period (the sampler's), so op reads 100 % busy.
  NhmBusyRead full; full.valid = true; full.period = nhm_period_4us(cfg.hop.window_ms - 10);
  full.buckets[11] = 100;
  FakeCard a, b;
  for (FakeCard* c : {&a, &b}) {
    c->ch = 40; c->width_mhz = 40; c->nhm_arm_ok = true; c->nhm_read = full;
  }
  std::vector<LinkCard*> ptrs{&a, &b};
  VrxController vrx(vrx_cfg_from(cfg, 40));
  RecordingSink sink;
  Aggregator agg(cfg.uep_layers(), 32, 2, 0);
  ChannelCoreCfg cc; cc.radio = cfg.radio; cc.hop = cfg.hop; cc.key = cfg.link.key; cc.start_ch = 40; cc.n_usb = 2;
  cc.width_mhz = 40;
  auto now_ms = [] { return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()); };
  auto now_us = [] { return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()); };
  ChannelCore core(cc, ptrs, vrx, sink, [](uint8_t) { return true; }, now_ms, now_us,
                   [](int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); });
  ChannelTickIn in; in.agg = &agg; in.in_session = true; in.tx_card = 0;
  auto burst_recs = [&] {
    size_t n = 0;
    for (const auto& d : sink.dwells) if (d.first == 1 && d.second.in_session) ++n;
    return n;
  };
  uint64_t max_tick_ms = 0;
  bool armed = false;   // boot pick closed: from here on the core may burst
  uint16_t seq = 100;   // every tick decodes a frame per card: a window with none has no busy reading
  for (int i = 0; i < 800 && burst_recs() < 3; ++i) {
    agg.on_rx_body(junk_body(0, seq));
    agg.on_rx_body(junk_body(1, seq));
    ++seq;
    in.now_ms = static_cast<double>(now_ms());
    const uint64_t t0 = now_ms();
    core.tick(in);
    if (armed) max_tick_ms = std::max(max_tick_ms, now_ms() - t0);
    armed = armed || !core.pick_open();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  core.shutdown();
  REQUIRE(burst_recs() >= 3);           // one burst: every candidate but op
  CHECK(max_tick_ms < 40);
}

TEST(relay_only_roster_searches_without_measuring) {
  Rig g(0, 1);                                // one relay, auto
  const auto s = g.core->snapshot(0);
  CHECK(std::string(s.scan_state) == "off");  // nothing can measure
  CHECK(!g.core->pick_open());
  // the relay scouts: search bursts retune it across the set
  for (int i = 0; i < 40; ++i) g.tick();
  bool retuned_off_start = false;
  for (auto& c : g.cards[0]->calls) if (c == "retune 64") retuned_off_start = true;
  CHECK(retuned_off_start);
  CHECK(g.sink.picks.empty());                // no K line ever: nothing measured
}

// Bench: web GS on a CPE relay, auto channel set -- the drone
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

TEST(relay_only_roster_no_ready_relay_is_quiet) {
  Rig g(0, 1);
  g.cards[0]->is_ready = false;
  for (int i = 0; i < 20; ++i) g.tick();
  CHECK(g.core->disc_targets(0).empty());
  CHECK(std::string(g.core->snapshot(0).scan_state) == "off");
  CHECK(g.cards[0]->calls.empty());           // scout never started (card not ready): nothing touched it
}

// ---- relay sweep (CPE relay hop) ----
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
  interfere(g, occ_lead(g) + 3);
  REQUIRE(g.cards[0]->sweeps.size() == 1);
  CHECK((g.cards[0]->sweeps[0] == std::vector<uint8_t>{64, 112, 144}));   // set minus op 40
  CHECK(g.core->sweep_pending());
  CHECK(!g.sink.has_hop("order"));                 // waits for the result, no hold_exhausted either
  CHECK(!g.sink.has_hop("hold_exhausted"));
  g.cards[0]->pending_result = sweep_of({{64, 20}, {112, 1}, {144, 4}});
  interfere(g, occ_lead(g) + 3);                                 // persist 2-of-3 again once the card is usable
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

TEST(relay_only_sweep_timeout_allows_next_burst) {
  Rig g(0, 1);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  interfere(g, occ_lead(g) + 3);
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
  // The blocked streak held through the windows without a busy reading, so
  // the op is still occupied: no new lead-in before the trigger.
  interfere(g, 3);
  CHECK(g.cards[0]->sweeps.size() == 2);           // relay_burst_period_ms (1000) has passed
}

// A lost SCAN / SCAN_RESULT. RemoteCard::sweeping() stays true
// until a matching result, the next start_scan or a reopen -- so the fake
// keeps sweep_in_flight set throughout. The verdict must not skip the relay
// on the card's own flag past the core's timeout, or a one-card roster
// reads Unknown forever, never triggers, never re-scans: detection dead.
TEST(relay_lost_scan_result_does_not_blind_the_verdict) {
  Rig g(0, 1);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  interfere(g, occ_lead(g) + 3);
  REQUIRE(g.cards[0]->sweeps.size() == 1);
  for (int i = 0; i < 150 && g.core->sweep_timeouts() == 0; ++i) g.tick(true);
  REQUIRE(g.core->sweep_timeouts() == 1);
  CHECK(g.cards[0]->sweeping());                   // the relay still thinks a scan is pending
  interfere(g, 3);                                 // still occupied: the streak held
  CHECK(g.cards[0]->sweeps.size() == 2);           // interfered verdicts again -> the next SCAN
}

// A sweep pending across a session drop. On reconnect an
// expired sweep is a timeout; a result that sat on the card meanwhile is
// taken and discarded, never ranked with the reconnect's timestamp.
TEST(relay_sweep_expired_across_session_drop_discards_late_result) {
  Rig g(0, 1);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  interfere(g, occ_lead(g) + 3);
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
// burst's hop.burst_period_ms: an empty result lands at once, and a burst
// 400 ms after the first SCAN must not happen.
TEST(relay_burst_waits_relay_burst_period_ms) {
  Rig g(0, 1);
  freeze_auto(g);
  REQUIRE(g.cfg.hop.burst_period_ms < 400);
  REQUIRE(g.cfg.hop.relay_burst_period_ms > 600);
  for (int i = 0; i < 5; ++i) g.tick(true);
  interfere(g, occ_lead(g) + 3);
  REQUIRE(g.cards[0]->sweeps.size() == 1);
  g.cards[0]->sweep_in_flight = false;
  g.cards[0]->pending_result = SweepResult{};      // an empty answer: nothing ranked
  for (int i = 0; i < 40; ++i) g.tick(true);
  CHECK(!g.core->sweep_pending());
  interfere(g, 1);                                 // the trigger is still high
  CHECK(g.cards[0]->sweeps.size() == 1);
}

// Rebuild a rig's VrxController + core around another channel set (start
// 40), e.g. after mutating g.cfg.hop post-construction (ChannelCoreCfg
// captures hop by value at construction time). n_usb defaults to 0 for the
// relay-only callers below.
static void rebuild_with_set(Rig& g, std::vector<uint8_t> set, int n_usb = 0) {
  g.cfg.radio.channels = std::move(set);
  g.core.reset();
  g.vrx = std::make_unique<VrxController>(vrx_cfg_from(g.cfg, 40));
  ChannelCoreCfg cc; cc.radio = g.cfg.radio; cc.hop = g.cfg.hop; cc.key = g.cfg.link.key;
  cc.width_mhz = g.width;
  cc.start_ch = 40; cc.n_usb = n_usb; cc.threaded = false; cc.store_name = "s";
  g.core = std::make_unique<ChannelCore>(cc, g.ptrs, *g.vrx, g.sink,
      [](uint8_t) { return true; }, [&g] { return g.clk.now_ms(); }, [&g] { return g.clk.now_us(); },
      [&g](int ms) { g.clk.sleep(ms); });
}

// Mirror of relay_burst_waits_relay_burst_period_ms for a USB burst card:
// the burst paces on hop.burst_period_ms, not on the periodic dwell
// thread's hop.dwell_period_ms.
TEST(usb_burst_paces_on_burst_period_not_dwell_period) {
  Rig g(2, 0);                       // two USB cards; the spare bursts
  g.cfg.hop.dwell_period_ms = 5000;  // periodic dwell far slower than the burst
  g.cfg.hop.burst_period_ms = 333;
  rebuild_with_set(g, g.cfg.radio.channels, 2);   // ChannelCoreCfg captured hop by value at construction
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  const size_t d0 = g.sink.dwells.size();
  interfere(g, occ_lead(g) + 3);                   // trigger live -> burst 1 (sess=1 D records)
  const size_t d1 = g.sink.dwells.size();
  REQUIRE(d1 > d0);
  interfere(g, occ_lead(g) + 3);                   // > 333 ms later, trigger still live -> burst 2
  CHECK(g.sink.dwells.size() > d1);
}

// Interfered windows for `windows` x window_ms, ticking the core directly;
// returns the clock of every tick on which a burst ran (new D records).
static std::vector<uint64_t> interfere_bursts(Rig& g, int windows) {
  OpJam jam(g);
  std::vector<uint64_t> bursts;
  size_t seen = g.sink.dwells.size();
  for (int w = 0; w < windows; ++w) {
    for (auto& c : g.cards) c->fr.foreign += 40;
    for (int i = 0; i < g.cfg.hop.window_ms / 10 + 1; ++i) {
      const uint64_t t = g.clk.ms;
      g.vrx->on_video(static_cast<double>(t));
      decode_one(g);
      g.core->tick(g.in(true));
      if (g.sink.dwells.size() > seen) { bursts.push_back(t); seen = g.sink.dwells.size(); }
      g.clk.ms += 10;
    }
  }
  return bursts;
}

// Every candidate blocked (100 % NHM busy on the burst card's dwells) and
// the op interfered, one card: the cold ranker holds after burst 1 (one
// visit each, nothing ranked), burst 2 comes burst_period_ms later and
// ranks them all -- blocked -- so the controller stays in hold_exhausted
// with the trigger live. The one card's burst is its only ranking source,
// so it keeps bursting, but each burst takes the sole radio off op for
// every candidate: it paces on dwell_period_ms, no burst within
// burst_period_ms, one once dwell_period_ms has passed.
TEST(one_card_burst_in_a_fully_ranked_all_blocked_hold_paces_on_dwell_period) {
  Rig g(1, 0);
  g.cfg.hop.dwell_period_ms = 1000;
  g.cfg.hop.burst_period_ms = 333;
  rebuild_with_set(g, g.cfg.radio.channels, 1);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  NhmBusyRead full; full.valid = true; full.period = nhm_period_4us(g.cfg.hop.dwell_observe_ms);
  full.buckets[11] = 100;                          // every sample above busy_dbm: 100 % busy
  g.cards[0]->nhm_arm_ok = true; g.cards[0]->nhm_read = full;
  g.cards[0]->nhm_any_period = true;               // op reads it too: the op is occupied
  const auto bursts = interfere_bursts(g, occ_lead(g) + 22);     // ~3.5 s past the lead-in
  CHECK(g.sink.has_hop("hold_exhausted"));
  CHECK(!g.sink.has_hop("order")); CHECK(!g.sink.has_hop("escape"));
  CHECK(std::string(g.core->snapshot(0).hop.state) == "hold");
  REQUIRE(bursts.size() >= 4);
  CHECK(bursts[1] - bursts[0] < 1000);             // burst 2 ranks the set: burst_period_ms
  for (size_t i = 2; i < bursts.size(); ++i) {
    CHECK(bursts[i] - bursts[i - 1] >= 1000);      // fully ranked Hold: dwell_period_ms
    CHECK(bursts[i] - bursts[i - 1] < 1000 + 333);
  }
}

// The same all-blocked Hold on two cards: the spare card's bursts stop
// once burst 2 has ranked every candidate -- from there the periodic dwell
// thread keeps the ranking on that card, and a burst would only stall the
// core loop. Revert (burst regardless of ranking_complete): bursts 3.. come
// back at dwell_period_ms.
TEST(two_card_spare_burst_stops_once_every_candidate_is_ranked) {
  Rig g(2, 0);
  g.cfg.hop.dwell_period_ms = 1000;
  g.cfg.hop.burst_period_ms = 333;
  rebuild_with_set(g, g.cfg.radio.channels, 2);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  NhmBusyRead full; full.valid = true; full.period = nhm_period_4us(g.cfg.hop.dwell_observe_ms);
  full.buckets[11] = 100;
  g.cards[1]->nhm_arm_ok = true; g.cards[1]->nhm_read = full;
  g.cards[1]->nhm_any_period = true;               // between dwells card 1 reads op busy too
  const auto bursts = interfere_bursts(g, occ_lead(g) + 22);     // ~3.5 s past the lead-in
  CHECK(g.sink.has_hop("hold_exhausted"));
  CHECK(std::string(g.core->snapshot(0).hop.state) == "hold");
  CHECK(bursts.size() == 2);                       // burst 1, burst 2 (ranks), then none
}

// The same Hold with one candidate that never gets a visit (its retune
// fails, so it stays unranked): the next burst is still what could rank it,
// so Hold keeps burst_period_ms. max_hops_per_min = 0 holds (hold_cap) on
// the first triggered tick.
TEST(usb_burst_in_a_hold_with_an_unranked_candidate_paces_on_burst_period) {
  Rig g(2, 0);
  g.cfg.hop.dwell_period_ms = 1000;
  g.cfg.hop.burst_period_ms = 333;
  g.cfg.hop.max_hops_per_min = 0;
  rebuild_with_set(g, g.cfg.radio.channels, 2);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[1]->refuse_ch = 144;
  const auto bursts = interfere_bursts(g, occ_lead(g) + 16);     // ~2.6 s past the lead-in
  CHECK(std::string(g.core->snapshot(0).hop.state) == "hold");
  REQUIRE(bursts.size() >= 5);
  for (size_t i = 1; i < bursts.size(); ++i) CHECK(bursts[i] - bursts[i - 1] < 1000);
}

TEST(relay_only_single_member_set_sends_no_scan) {
  Rig g(0, 1);
  rebuild_with_set(g, {40});
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  interfere(g, occ_lead(g) + 4);
  CHECK(g.cards[0]->sweeps.empty());
  CHECK(!g.core->sweep_pending());
}

TEST(relay_survey_window_feeds_blocked) {
  Rig g(0, 1);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->survey_win = SurveyWindow{true, 97.0, 2.0};   // analog-VTX shape: busy, not decodable
  interfere(g, occ_lead(g) + 2);
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
  interfere(g, occ_lead(g) + 3);                                 // tx_card 0 (USB) by default in g.in()
  CHECK(g.cards[1]->sweeps.size() == 1);           // the relay spare swept
  bool usb_retuned_off_op = false;
  for (auto& c : g.cards[0]->calls) if (c == "retune 64" || c == "retune 112" || c == "retune 144") usb_retuned_off_op = true;
  CHECK(!usb_retuned_off_op);                      // the USB TX card never left the link
}

// The sweep timeout is derived from the request --
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
  {
    OpJam jam(g);
    for (int w = 0; w < occ_lead(g) + 4 && g.cards[0]->sweeps.empty(); ++w) {
      for (auto& c : g.cards) c->fr.foreign += 40;
      for (int i = 0; i < g.cfg.hop.window_ms / 10 + 1 && g.cards[0]->sweeps.empty(); ++i) {
        g.vrx->on_video(static_cast<double>(g.clk.ms));
        decode_one(g);
        sent_ms = g.clk.ms;
        g.core->tick(g.in(true));
        g.clk.ms += 10;
      }
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

// Relay + USB with the relay down (booting / not owned)
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
  interfere(g, occ_lead(g) + 3, /*tx=*/0);
  CHECK(g.cards[1]->sweeps.empty());
  CHECK(!g.core->sweep_pending());
  size_t usb_dwells = 0;
  for (size_t i = d0; i < g.sink.dwells.size(); ++i) if (g.sink.dwells[i].first == 0) ++usb_dwells;
  CHECK(usb_dwells > 0);                           // the USB TX card burst
}

// Relay + USB, the relay transmits, the USB card is
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
  interfere(g, occ_lead(g) + 3, /*tx=*/1);
  CHECK(g.cards[1]->sweeps.empty());
  size_t usb_dwells = 0;
  for (size_t i = d0; i < g.sink.dwells.size(); ++i) if (g.sink.dwells[i].first == 0) ++usb_dwells;
  CHECK(usb_dwells > 0);
}

// While the relay sweeps, the TX selector must not
// move TX onto it -- the sweep freezes the TX selection like a dwell.
TEST(tx_frozen_while_relay_sweeps) {
  Rig g(1, 1);
  g.cards[0]->retune_ok = false;
  freeze_auto(g);
  g.cards[0]->retune_ok = true;
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true, false, 0);
  g.cards[0]->cca_per_ms_on[40] = 50;
  interfere(g, occ_lead(g) + 3);
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

// The sampler stamps a record with its own time; a record the core takes a
// tick later must still read own_air over the sampled span, or the airtime
// percentage shrinks by the consume delay on every late sample.
TEST(sampled_record_consumed_next_tick_uses_sample_span) {
  Rig g(1, 0, /*pinned=*/true, 40);
  link_up(g);
  // past the hop-active edge, with at least one window consumed
  for (int i = 0; i < 2 * (g.cfg.hop.window_ms / 10) + 2; ++i) g.tick(true);
  // a record at a known time T1 (the seam restarts the cadence from T1)
  g.core->run_sampler_step();
  g.clk.ms += 10;
  g.core->tick(g.in(true));                // consumed at T1 + 10: window_prev_ms_[0] = T1
  for (int i = 0; i < 13; ++i) g.tick(true);   // now T1 + 140, no window due before T1 + 150
  g.cards[0]->fr.own_air_us += 70000;      // 50 % of the 140 ms since T1
  g.cards[0]->fr.own += 10;
  g.core->run_sampler_step();              // sample at T1 + 140
  g.clk.ms += 30;
  g.cards[0]->fr.own_air_us += 5000;       // airtime after the sample: must not count
  g.core->tick(g.in(true));                // consumed at T1 + 170; nothing else is due
  const auto e = g.core->snapshot(0).energy[0];
  REQUIRE(e.has_value());
  REQUIRE(e->own_air_pct.has_value());
  // 70000 us over the 140 ms sample span (T1 -> T1 + 140) = 50 %. The two
  // consume delays differ (10 ms, 30 ms), so a span measured between the
  // consumes (T1 + 10 -> T1 + 170, 160 ms) reads 43.75 % instead.
  CHECK(*e->own_air_pct > 49.0 && *e->own_air_pct < 51.0);
}

// Own airtime is what the drone transmitted, not what the card decoded. A
// card fading at mid range decodes a fraction of our frames while every one
// of them still fills NHM busy, so an own-airtime figure rebuilt from decoded
// frames alone books the undecoded rest as foreign -- a fade then reads as
// interference and confirms itself with its own loss. The window fill scales
// the decoded airtime by the card's sequence delivery (expected / received
// over the window, from the drone's hardware seq walk), which counts every
// frame the drone sent whether or not this card heard it.
// Revert (own_used = own_pct unscaled): card 0 reads 40 %.
static mabur::node::RxBody junk_body(uint8_t card, uint16_t seq) {
  mabur::node::RxBody m;
  m.card_id = card; m.mac_seq = seq; m.crc_ok = true;
  m.rssi[0] = 38; m.rssi[1] = 40; m.snr[0] = 10; m.snr[1] = 25;
  m.mono_us = 1000u * seq;
  m.body.assign(20, 0);   // not SBI, not RC: misroutes, still walks the seq
  return m;
}

// The weak gate reads the card's best receive chain, not chain A. Chain A
// under weak_rssi_dbm with chain B well above it is a card still hearing
// the drone fine on one antenna, so a jammed op reads interfered, not
// fade. Revert (rssi_a_ema at the window fill): weak trips on chain A and
// the window is a fade.
TEST(weak_gate_reads_the_best_chain_not_chain_a) {
  Rig g(2, 0, /*pinned=*/true, 40);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  for (uint8_t card = 0; card < 2; ++card) {
    auto m = junk_body(card, 1);
    m.phy_valid = true;
    m.rssi[0] = 25; m.rssi[1] = 40;   // chain A -85 dBm (under the wall), chain B -70 dBm
    m.snr[0] = 10; m.snr[1] = 10;     // 5 dB on both: under weak_snr_db, so RSSI alone decides
    g.agg.on_rx_body(m);
  }
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  g.cards[0]->fr.foreign = 0; g.cards[1]->fr.foreign = 0;
  interfere(g, occ_lead(g) + 6);
  CHECK(std::string(g.core->snapshot(0).hop.verdict) == "interfered");
  CHECK((g.core->snapshot(0).hop.evidence & kEvWeak) == 0);
}
// One sampled window on a one-card pinned rig: `before` runs between the
// T1 record and its consume (the previous window), `during` inside the
// 140 ms window that the second record closes. Returns the window's energy.
static std::optional<StatsEnergyIn> one_window(Rig& g, const std::function<void()>& before,
                                               const std::function<void()>& during) {
  link_up(g);
  for (int i = 0; i < 2 * (g.cfg.hop.window_ms / 10) + 2; ++i) g.tick(true);
  before();
  g.core->run_sampler_step();
  g.clk.ms += 10;
  g.core->tick(g.in(true));                // consumed at T1 + 10
  for (int i = 0; i < 13; ++i) g.tick(true);   // T1 + 140
  during();
  g.core->run_sampler_step();              // sample at T1 + 140
  g.clk.ms += 10;
  g.core->tick(g.in(true));
  return g.core->snapshot(0).energy[0];
}
TEST(own_air_scales_by_the_cards_seq_delivery) {
  Rig g(1, 0, /*pinned=*/true, 40);
  const auto e = one_window(g,
      [&] { for (uint16_t s = 0; s < 10; ++s) g.agg.on_rx_body(junk_body(0, s)); },
      [&] {
        // seq 20..30 after 9: 11 decoded of 21 sent in this window
        for (uint16_t s = 20; s <= 30; ++s) g.agg.on_rx_body(junk_body(0, s));
        g.cards[0]->fr.own_air_us += 56000;   // 40 % of the 140 ms span, decoded frames only
        g.cards[0]->fr.own += 11;
      });
  REQUIRE(e.has_value());
  REQUIRE(e->own_air_pct.has_value());
  // 40 % x 21/11 = 76.4 %
  CHECK(*e->own_air_pct > 75.0 && *e->own_air_pct < 78.0);
}
TEST(own_air_scaled_is_capped_at_100) {
  Rig g(1, 0, /*pinned=*/true, 40);
  const auto e = one_window(g,
      [&] { for (uint16_t s = 0; s < 10; ++s) g.agg.on_rx_body(junk_body(0, s)); },
      [&] {
        for (uint16_t s = 20; s <= 30; ++s) g.agg.on_rx_body(junk_body(0, s));
        g.cards[0]->fr.own_air_us += 84000;   // 60 % decoded x 21/11 = 114 %
        g.cards[0]->fr.own += 11;
      });
  REQUIRE(e.has_value());
  REQUIRE(e->own_air_pct.has_value());
  CHECK(*e->own_air_pct > 99.9 && *e->own_air_pct < 100.1);
}

// Own airtime never reads under the airtime the card decoded. A frame with
// a seq ahead of the drone's (another counter that slipped into the walk)
// leaves every later drone frame "behind" the mark: received climbs while
// expected stands, and a delivery ratio under 1 zeroed the card's own
// airtime while it decoded fine, so its whole busy reading booked as
// foreign. Revert (delivery = expected / received unfloored): 0 %.
TEST(own_air_never_reads_under_the_decoded_airtime) {
  Rig g(1, 0, /*pinned=*/true, 40);
  const auto e = one_window(g,
      [&] {
        for (uint16_t s = 0; s < 10; ++s) g.agg.on_rx_body(junk_body(0, s));
        g.agg.on_rx_body(junk_body(0, 1500));   // the mark jumps ahead
      },
      [&] {
        for (uint16_t s = 10; s <= 30; ++s) g.agg.on_rx_body(junk_body(0, s));   // all "behind"
        g.cards[0]->fr.own_air_us += 56000;   // 40 % of the 140 ms span
        g.cards[0]->fr.own += 21;
      });
  REQUIRE(e.has_value());
  REQUIRE(e->own_air_pct.has_value());
  CHECK(*e->own_air_pct > 39.0 && *e->own_air_pct < 41.0);
}

// A window that decoded nothing has no delivery ratio (the seq walk only
// advances on a decoded frame), so its own airtime cannot be told from
// what the air carried: a null at mid range leaves our frames above
// busy_dbm with none of them decoded, and the RSSI EMA frozen above the
// weak wall. That card gives the verdict no busy reading for the window
// (the blocked streak holds instead of advancing) and the sideport shows
// busy_pct null. Revert (busy_valid from the NHM read alone): the card
// reads 80 % foreign off its own video.
TEST(no_busy_reading_when_the_window_decoded_nothing) {
  Rig g(1, 0, /*pinned=*/true, 40);
  g.cards[0]->nhm_busy_pct_on[40] = 80;
  const auto e = one_window(g,
      [&] { for (uint16_t s = 0; s < 10; ++s) g.agg.on_rx_body(junk_body(0, s)); },
      [&] { g.cards[0]->fr.own_air_us += 14000; });   // 10 % of the span, no frame decoded
  REQUIRE(e.has_value());
  CHECK(!e->busy_pct.has_value());
}

// ---- calibration channel pick (CalPicker / ChannelCore) ----

// One linked tick with the TX card (card 0) on op hearing its own video and
// `op_foreign` foreign frames: own frames feed the sideport's own-frame
// delta (StatsEnergyIn::own), foreign frames are what op's live reading
// scores, and a decoded frame per card gives op's window a busy reading.
// Card 1 is the scout card.
static void live_tick(Rig& g, uint32_t op_foreign = 0) {
  g.cards[0]->fr.own += 5;
  g.cards[0]->fr.foreign += op_foreign;
  g.vrx->on_video(static_cast<double>(g.clk.ms));
  decode_one(g);
  g.tick(true);
}

static const HopEvent* find_hop(const RecordingSink& s, const char* kind) {
  for (const auto& e : s.hops) if (e.kind == kind) return &e;
  return nullptr;
}

// Auto, two cards, linked, the boot pick long closed: one measure round
// dwells once on every pair outside op's own, reads op from the
// verdict's live window, emits one K record and names the best member.
// Op (40) and 64's pair read NHM busy: 112 is the clean member, well past
// the pick margin.
TEST(cal_measure_round_ranks_the_set_once) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  g.cards[0]->nhm_arm_ok = true;
  g.cards[0]->nhm_any_period = true;
  g.cards[0]->nhm_read.valid = true;
  g.cards[0]->nhm_read.buckets[11] = 25;   // op's NHM busy: 25 % (below blocked_pct)
  g.cards[0]->nhm_read.buckets[0] = 75;
  g.cards[1]->nhm_busy_pct_on[64] = 25;
  g.cards[1]->nhm_busy_pct_on[112] = 5;    // the clean member
  g.cards[1]->nhm_busy_pct_on[144] = 25;
  for (int i = 0; i < 30; ++i) live_tick(g, 200);
  ChannelCorePicker p(*g.core);
  REQUIRE(p.mode() == CalPicker::Mode::Auto);
  CHECK(p.op() == 40);
  const size_t picks0 = g.sink.picks.size(), dwells0 = g.sink.dwells.size();
  p.begin_measure(g.clk.ms);
  CHECK(!p.measured_best().has_value());
  for (int i = 0; i < 50 && !p.measured_best(); ++i) live_tick(g, 200);
  REQUIRE(p.measured_best().has_value());
  CHECK(*p.measured_best() == 112);
  CHECK(g.sink.picks.size() == picks0 + 1);
  REQUIRE(g.sink.picks.back().has_value());
  CHECK(*g.sink.picks.back() == 112);
  std::vector<uint8_t> chs;
  for (size_t i = dwells0; i < g.sink.dwells.size(); ++i) {
    CHECK(g.sink.dwells[i].first == 1);
    chs.push_back(g.sink.dwells[i].second.survey.def.primary);
  }
  std::sort(chs.begin(), chs.end());
  CHECK((chs == std::vector<uint8_t>{64, 112, 144}));   // one dwell per pair; op's (40) never dwelt on
  CHECK(g.core->op() == 40);                                          // measuring moves nothing
  for (int i = 0; i < 3; ++i) live_tick(g, 200);
  CHECK(!g.core->scout_owns_card(1));                                 // parked again ...
  CHECK(g.cards[1]->ch == 40 && g.cards[1]->width_mhz == 40);         // ... on op at the link width
  const size_t dwells1 = g.sink.dwells.size();
  for (int i = 0; i < 20; ++i) live_tick(g, 200);
  CHECK(g.sink.dwells.size() == dwells1);                             // one round, not a loop
  CHECK(g.sink.picks.size() == picks0 + 1);
  CHECK(*p.measured_best() == 112);
}

// The same round over a quiet-ish op: a candidate must beat op by
// pick_margin_pct busy points, so a close-but-not-clear op stays the
// target.
TEST(cal_measure_round_keeps_a_clean_op) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  g.cards[0]->nhm_arm_ok = true;
  g.cards[0]->nhm_any_period = true;
  g.cards[0]->nhm_read.valid = true;
  g.cards[0]->nhm_read.buckets[11] = 15;   // op's NHM busy: 15 %
  g.cards[0]->nhm_read.buckets[0] = 85;
  g.cards[1]->nhm_busy_pct_on[64] = 40;
  g.cards[1]->nhm_busy_pct_on[112] = 10;   // 5 points short of the margin (10)
  g.cards[1]->nhm_busy_pct_on[144] = 40;
  for (int i = 0; i < 30; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  p.begin_measure(g.clk.ms);
  for (int i = 0; i < 50 && !p.measured_best(); ++i) live_tick(g);
  REQUIRE(p.measured_best().has_value());
  CHECK(*p.measured_best() == 40);
}

// The boot pick's relocate, tagged cal: H relocate with reason "cal", the
// order rides the RCF, the confirm lands it (no verify window), the store
// follows op. One relocate at a time; none while a hop is in flight.
TEST(cal_relocate_orders_a_move_and_settles_on_confirm) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  CHECK(!p.relocate_settled().has_value());
  CHECK(!p.order_relocate(40));                   // already there: nothing to order
  CHECK(!p.order_relocate(36));                   // not a member
  REQUIRE(p.order_relocate(112));
  CHECK(!p.order_relocate(144));                  // one relocate at a time
  for (int i = 0; i < 20 && !g.sink.has_line("relocate 40 -> 112 placed"); ++i) live_tick(g);
  REQUIRE(g.sink.has_line("maburgs channel: relocate 40 -> 112 placed (cal)"));
  const HopEvent* h = find_hop(g.sink, "relocate");
  REQUIRE(h != nullptr);
  CHECK(h->target == 112);
  CHECK(h->reason == "cal");
  CHECK(!p.relocate_settled().has_value());
  CHECK(!p.order_relocate(144));                  // a hop in flight
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value() && r->hop_ch == 112);
  g.core->note_video(112);
  g.tick(true);
  REQUIRE(g.sink.has_hop("lead_confirm"));
  for (int i = 0; i < 3 && !p.relocate_settled(); ++i) g.tick(true);
  REQUIRE(p.relocate_settled().has_value());
  CHECK(*p.relocate_settled());
  CHECK(g.sink.has_hop("verify_pass"));
  CHECK(std::string(g.core->snapshot(0).hop.state) == "idle");
  CHECK(g.core->op() == 112);
  CHECK(p.op() == 112);
  CHECK(!g.stored.empty() && g.stored.back() == 112);   // the remembered channel follows
  for (const auto& e : g.sink.hops)
    if (e.kind != "relocate") CHECK(e.reason.empty());   // only the order carries the reason
}

// The drone never shows up on the target: the order withdraws at
// confirm_ms, the relocate settles false and the link stays on op.
TEST(cal_relocate_settles_false_when_the_drone_does_not_follow) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  REQUIRE(p.order_relocate(112));
  for (int i = 0; i < 20 && !g.sink.has_line("relocate 40 -> 112 placed"); ++i) live_tick(g);
  REQUIRE(g.sink.has_line("relocate 40 -> 112 placed (cal)"));
  for (int i = 0; i < g.cfg.hop.confirm_ms / 10 + 20 && !p.relocate_settled(); ++i) live_tick(g);
  REQUIRE(p.relocate_settled().has_value());
  CHECK(!*p.relocate_settled());
  CHECK(g.sink.has_hop("withdraw"));
  CHECK(g.core->op() == 40);
  for (int i = 0; i < 20; ++i) live_tick(g);
  CHECK(g.sink.has_line("relocation to 112 did not land; staying on 40"));   // BootPick accepts op
  CHECK(g.cards[0]->ch == 40 && g.cards[1]->ch == 40);
  const auto placed = std::count_if(g.sink.lines.begin(), g.sink.lines.end(), [](const std::string& l) {
    return l.find("relocate 40 -> 112 placed") != std::string::npos;
  });
  CHECK(placed == 1);                               // no retry behind the run's back
}

// Pinned and one-card GSs cannot measure: no round runs, op is the answer
// at once.
TEST(pinned_core_has_no_measure) {
  Rig g(2, 0, /*pinned=*/true, 64);
  link_up(g);
  for (int i = 0; i < 10; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  CHECK(p.mode() == CalPicker::Mode::Pinned);
  const size_t dwells0 = g.sink.dwells.size(), picks0 = g.sink.picks.size();
  p.begin_measure(g.clk.ms);
  REQUIRE(p.measured_best().has_value());
  CHECK(*p.measured_best() == p.op());
  CHECK(p.op() == 64);
  for (int i = 0; i < 20; ++i) live_tick(g);
  CHECK(g.sink.dwells.size() == dwells0);
  CHECK(g.sink.picks.size() == picks0);
  CHECK(!g.core->scout_owns_card(1));

  Rig h(1, 0);
  ChannelCorePicker q(*h.core);
  CHECK(q.mode() == CalPicker::Mode::OneCard);
  q.begin_measure(h.clk.ms);
  REQUIRE(q.measured_best().has_value());
  CHECK(*q.measured_best() == q.op());
}

// ---- the calibration round's pick rule (cal_round_pick.h) ----
static std::vector<CalReading> readings_with(std::initializer_list<std::pair<uint8_t, uint32_t>> fr,
                                             std::optional<double> busy = std::nullopt) {
  std::vector<CalReading> v;
  for (auto [ch, f] : fr) v.push_back(CalReading{ch, f, busy.has_value(), busy.value_or(0.0)});
  return v;
}
static const std::vector<uint8_t> kSet{40, 64, 112, 144};

TEST(cal_pick_busy_margin_decides_among_read_candidates) {
  // 112 reads busy 15 %, the others more; op 30 %: a real gap, but short
  // of the pick margin (20 points).
  const std::vector<CalReading> h = {
      {64, 0, true, 40.0}, {112, 0, true, 15.0}, {144, 0, true, 40.0}};
  CHECK(cal_round_pick(h, CalReading{40, 0, true, 30.0}, 40, kSet, 20, 50.0).best == 40);
  // 20 points better than op: it moves.
  CHECK(cal_round_pick(h, CalReading{40, 0, true, 35.0}, 40, kSet, 20, 50.0).best == 112);
}

TEST(cal_pick_busy_tie_stays_on_op) {
  // 64 ties op on busy %, even though op's frame count (foreign) reads far
  // worse: foreign only orders the candidates among themselves, never the
  // move itself.
  const std::vector<CalReading> h = {
      {64, 0, true, 30.0}, {112, 0, true, 30.0}, {144, 0, true, 50.0}};
  CHECK(cal_round_pick(h, CalReading{40, 300, true, 30.0}, 40, kSet, 20, 50.0).best == 40);
}

TEST(cal_pick_clearly_blocked_op_moves) {
  const auto h = readings_with({{64, 0}, {112, 0}, {144, 0}}, 5.0);
  CHECK(cal_round_pick(h, CalReading{40, 0, true, 90.0}, 40, kSet, 20, 50.0).best == 64);
}

TEST(cal_pick_unblocked_beats_blocked_op_at_once) {
  // Op just blocked (55 %), every candidate just under (45 %, unblocked):
  // a better tier moves at once, no margin needed.
  const auto h = readings_with({{64, 0}, {112, 0}, {144, 0}}, 45.0);
  CHECK(cal_round_pick(h, CalReading{40, 0, true, 55.0}, 40, kSet, 20, 50.0).best == 64);
  // A blocked candidate never replaces an unblocked op, however busy op's
  // frame count reads.
  const auto b = readings_with({{64, 0}, {112, 0}, {144, 0}}, 60.0);
  CHECK(cal_round_pick(b, CalReading{40, 300, true, 40.0}, 40, kSet, 20, 50.0).best == 40);
}

TEST(cal_pick_rows_are_one_per_member_plus_op) {
  const auto h = readings_with({{64, 0}, {112, 0}, {144, 0}}, 5.0);
  const auto out = cal_round_pick(h, CalReading{40, 0, true, 5.0}, 40, kSet, 20, 50.0);
  CHECK(out.rows.size() == 4);
}

TEST(cal_pick_excludes_a_member_never_read) {
  // 144 never read: it is not ranked, even though it is in the set.
  const std::vector<CalReading> h = {{64, 0, true, 10.0}, {112, 0, true, 40.0}};
  CHECK(cal_round_pick(h, CalReading{40, 0, true, 50.0}, 40, kSet, 20, 50.0).best == 64);
  // Op unread: stay, whatever the candidates read.
  CHECK(cal_round_pick(h, std::nullopt, 40, kSet, 20, 50.0).best == 40);
  // K rows: every member read plus op's, one row each, one visit each.
  const auto out = cal_round_pick(h, CalReading{40, 0, true, 50.0}, 40, kSet, 20, 50.0);
  CHECK(out.rows.size() == 3);
  for (const auto& r : out.rows) CHECK(r.visits == 1);
}

TEST(cal_pick_a_candidate_without_a_busy_reading_never_outranks_one_with) {
  // 112 has no NHM busy reading: scored on foreign alone it would read 0
  // and beat 144 (5 frames + 10 %) on a term the others pay and it does
  // not. While any candidate has a busy reading, one without is not
  // eligible.
  const std::vector<CalReading> h = {
      {64, 10, true, 20.0}, {112, 0, false, 0.0}, {144, 5, true, 10.0}};
  CHECK(cal_round_pick(h, CalReading{40, 100, true, 30.0}, 40, kSet, 20, 50.0).best == 144);
  // With no busy reading anywhere, foreign still orders the candidates
  // (the K rows), but the move itself needs a busy reading on both sides:
  // stay.
  const auto f = readings_with({{64, 50}, {112, 0}, {144, 5}});
  CHECK(cal_round_pick(f, CalReading{40, 100}, 40, kSet, 20, 50.0).best == 40);
}

// The best candidate is the one with the lowest busy %, not the lowest
// foreign + busy % score: the move is decided on busy %, so the candidate
// it is decided for must be ranked on it. 112 reads 5 % with 30 foreign
// frames (score 35), 64 reads 20 % with 2 (score 22); op 30 %: 112 clears
// the 20-point margin, 64 would not.
// Revert (best by score): 64 is best, short of the margin, and op stays.
TEST(cal_pick_best_ranks_on_busy_before_score) {
  const std::vector<CalReading> h = {{64, 2, true, 20.0}, {112, 30, true, 5.0}};
  CHECK(cal_round_pick(h, CalReading{40, 0, true, 30.0}, 40, kSet, 20, 50.0).best == 112);
  // Under the margin the pick still stays on op.
  CHECK(cal_round_pick(h, CalReading{40, 0, true, 20.0}, 40, kSet, 20, 50.0).best == 40);
}

TEST(cal_pick_width_20_compares_members) {
  const std::vector<uint8_t> set{36, 40, 44};
  const std::vector<CalReading> h = {{40, 0, true, 40.0}, {44, 0, true, 5.0}};
  CHECK(cal_round_pick(h, CalReading{36, 0, true, 50.0}, 36, set, 20, 50.0).best == 44);
}

// ---- the round's lifecycle ----

// Start a round on a linked, frozen two-card auto rig: it is running once
// the scout card is owned.
static void start_round(Rig& g, ChannelCorePicker& p) {
  p.begin_measure(g.clk.ms);
  for (int i = 0; i < 5 && !g.sink.has_line("calibration measure round on card"); ++i) live_tick(g);
  REQUIRE(g.sink.has_line("calibration measure round on card 1"));
  live_tick(g);
  REQUIRE(g.core->scout_owns_card(1));
}

TEST(cal_cancel_releases_the_card_and_withdraws_an_unplaced_relocate) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 10; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  start_round(g, p);
  p.cancel();
  for (int i = 0; i < 3; ++i) live_tick(g);
  CHECK(!g.core->scout_owns_card(1));
  CHECK(g.cards[1]->ch == 40 && g.cards[1]->width_mhz == 40);
  CHECK(!p.measured_best().has_value());
  const size_t dwells = g.sink.dwells.size(), picks = g.sink.picks.size();
  for (int i = 0; i < 20; ++i) live_tick(g);
  CHECK(g.sink.dwells.size() == dwells);
  CHECK(g.sink.picks.size() == picks);
  // A relocate accepted but not yet placed is withdrawn with it.
  REQUIRE(p.order_relocate(112));
  p.cancel();
  for (int i = 0; i < 20; ++i) live_tick(g);
  CHECK(!g.sink.has_line("relocate 40 -> 112 placed"));
  CHECK(!g.sink.has_hop("relocate"));
  CHECK(g.core->op() == 40);
  CHECK(!p.relocate_settled().has_value());
}

TEST(cal_waiting_round_is_dropped_once_a_run_calibrates) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 10; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  p.begin_measure(g.clk.ms);
  g.tick(true, /*cal=*/true);
  CHECK(g.sink.has_line("calibration measure round dropped"));
  const size_t dwells = g.sink.dwells.size();
  for (int i = 0; i < 30; ++i) live_tick(g);
  CHECK(!g.sink.has_line("calibration measure round on card"));
  CHECK(!g.core->scout_owns_card(1));
  CHECK(g.sink.dwells.size() == dwells);
  CHECK(!p.measured_best().has_value());
}

TEST(cal_round_never_starts_without_a_running_scout) {
  Rig g(2, 0);
  g.cards[1]->is_ready = false;              // the scout card never comes up: no scout thread
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 10; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  p.begin_measure(g.clk.ms);
  for (int i = 0; i < 50; ++i) live_tick(g);
  CHECK(!g.sink.has_line("calibration measure round on card"));
  CHECK(!g.core->scout_owns_card(1));
  CHECK(!p.measured_best().has_value());
  p.cancel();
}

TEST(cal_running_round_ends_at_its_bound_with_op) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 10; ++i) live_tick(g, 200);
  ChannelCorePicker p(*g.core);
  start_round(g, p);
  CHECK(!p.measured_best().has_value());
  g.clk.ms += 10000;                         // the scout made no progress for 10 s (a stalled step)
  live_tick(g, 200);
  REQUIRE(p.measured_best().has_value());
  CHECK(*p.measured_best() == 40);
  CHECK(g.sink.has_line("calibration measure round overran"));
  for (int i = 0; i < 3; ++i) live_tick(g, 200);
  CHECK(!g.core->scout_owns_card(1));
  CHECK(g.cards[1]->ch == 40 && g.cards[1]->width_mhz == 40);
}

// ---- relocate edges ----
static void cal_move(Rig& g, ChannelCorePicker& p, uint8_t ch) {
  REQUIRE(p.order_relocate(ch));
  for (int i = 0; i < 20 && !g.core->hopping(); ++i) live_tick(g);
  REQUIRE(g.core->hopping());
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value() && r->hop_ch == ch);
  g.core->note_video(ch);
  for (int i = 0; i < 5 && !p.relocate_settled(); ++i) g.tick(true);
  REQUIRE(p.relocate_settled().has_value() && *p.relocate_settled());
  for (int i = 0; i < 3; ++i) live_tick(g);
  REQUIRE(g.core->op() == ch);
}

TEST(cal_relocate_refused_at_the_hop_cap) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  REQUIRE(g.cfg.hop.max_hops_per_min == 4);
  cal_move(g, p, 112);
  cal_move(g, p, 144);
  cal_move(g, p, 64);
  cal_move(g, p, 40);
  CHECK(!p.order_relocate(112));
  CHECK(g.sink.has_line("calibration relocate to 112 refused: hop cap"));
  CHECK(g.core->op() == 40);
}

TEST(cal_relocate_never_placed_settles_false) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  REQUIRE(p.order_relocate(112));
  g.tick(true, /*cal=*/true);                // the link stops being hop-active before the order leaves
  REQUIRE(p.relocate_settled().has_value());
  CHECK(!*p.relocate_settled());
  CHECK(g.sink.has_line("calibration relocate to 112 never placed"));
  for (int i = 0; i < 20; ++i) live_tick(g);
  CHECK(!g.sink.has_hop("relocate"));        // want followed op: nothing is placed later
  CHECK(g.core->op() == 40);
}

TEST(cal_relocate_session_lost_in_flight_settles_false) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  REQUIRE(p.order_relocate(112));
  for (int i = 0; i < 20 && !g.sink.has_line("relocate 40 -> 112 placed"); ++i) live_tick(g);
  REQUIRE(g.sink.has_line("relocate 40 -> 112 placed (cal)"));
  g.tick(false);                             // session lost before the confirm
  REQUIRE(p.relocate_settled().has_value());
  CHECK(!*p.relocate_settled());
  CHECK(g.sink.has_hop("session_lost"));
  CHECK(g.core->op() == 40);
}

// ---- live pin / auto (set_pin, set_auto) ----

// Linked, auto, pick closed: the pin is a relocate tagged pin, landed by the
// confirm (no verify window), and from then on the GS is static -- an
// interfered pin is reported, never fled.
TEST(set_pin_while_linked_relocates_and_freezes_the_reactive_layer) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  CHECK(!g.core->pinned());
  CHECK(!g.core->snapshot(0).pinned);
  CHECK(!g.core->set_pin(112).has_value());
  CHECK(g.core->pinned());
  CHECK(g.core->snapshot(0).pinned);
  CHECK(g.sink.has_line("maburgs channel: pinned to 112, link on 40: relocating"));
  for (int i = 0; i < 20 && !g.sink.has_line("relocate 40 -> 112 placed"); ++i) live_tick(g);
  REQUIRE(g.sink.has_line("maburgs channel: relocate 40 -> 112 placed (pin)"));
  const HopEvent* h = find_hop(g.sink, "relocate");
  REQUIRE(h != nullptr);
  CHECK(h->target == 112);
  CHECK(h->reason == "pin");
  CHECK(g.core->hop_target() == 112 || g.core->op() == 112 || g.core->snapshot(0).hop.state != std::string("idle"));
  CHECK(g.core->set_pin(144) == std::optional<std::string>("relocate in flight"));
  const auto r = pump_rcf(g);
  REQUIRE(r.has_value() && r->hop_ch == 112);
  g.core->note_video(112);
  g.tick(true);
  REQUIRE(g.sink.has_hop("lead_confirm"));
  for (int i = 0; i < 3 && !g.sink.has_hop("verify_pass"); ++i) g.tick(true);
  CHECK(g.sink.has_hop("verify_pass"));         // the confirm landed it: no verify window
  CHECK(!g.sink.has_line("did not land"));
  CHECK(g.core->op() == 112);
  CHECK(std::string(g.core->snapshot(0).scan_state) == "off");
  REQUIRE(g.core->snapshot(0).scan_pick.has_value());
  CHECK(*g.core->snapshot(0).scan_pick == 112);
  CHECK(!g.stored.empty() && g.stored.back() == 112);
  for (int i = 0; i < 5; ++i) live_tick(g);
  CHECK(g.cards[0]->ch == 112 && g.cards[1]->ch == 112);
  // an interfered pin: the verdict reads it, the controller never moves
  const size_t hops0 = g.sink.hops.size();
  g.cards[0]->cca_per_ms_on[112] = 50; g.cards[1]->cca_per_ms_on[112] = 50;
  g.cards[0]->fr.foreign = 0; g.cards[1]->fr.foreign = 0;
  interfere(g, occ_lead(g) + 6);
  CHECK(std::string(g.core->snapshot(0).hop.verdict) == "interfered");
  CHECK(g.sink.hops.size() == hops0);          // no order/escape/hold of any kind
  CHECK(!g.core->hopping());
  CHECK(g.core->op() == 112);
  const size_t dwells = g.sink.dwells.size();
  g.core->run_inflight_step();                 // the in-flight scout is off too
  g.tick(true);
  CHECK(g.sink.dwells.size() == dwells);
}

// Linked with the boot pick still open (the scout measuring beside the
// link): the pin closes the pick, the scout parks, then the relocate goes.
TEST(set_pin_while_linked_with_the_pick_open_closes_it) {
  Rig g(2, 0);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  REQUIRE(g.core->pick_open());
  CHECK(!g.core->set_pin(112).has_value());
  CHECK(!g.core->pick_open());
  CHECK(std::string(g.core->snapshot(0).scan_state) == "off");
  for (int i = 0; i < 50 && !g.sink.has_line("relocate 40 -> 112 placed"); ++i) g.tick(true);
  REQUIRE(g.sink.has_line("relocate 40 -> 112 placed (pin)"));
  CHECK(!g.core->scout_owns_card(1));
  CHECK(g.sink.picks.empty());                  // no boot pick was ever taken
}

TEST(set_pin_while_unlinked_moves_op_and_waits_there) {
  Rig g(2, 0);
  g.cards[1]->cca_per_ms_on[112] = 5; g.cards[1]->cca_per_ms_on[108] = 5;   // auto would never pick 112
  g.tick();
  CHECK(!g.core->set_pin(112).has_value());
  CHECK(g.core->op() == 112);
  CHECK(g.core->pinned());
  CHECK(!g.core->pick_open());
  CHECK(std::string(g.core->snapshot(0).scan_state) == "off");
  g.tick();                                     // the core's per-card retune runs in tick()
  CHECK(g.cards[0]->ch == 112);   // cards follow op
  CHECK(g.sink.has_move(MoveReason::Commit));
  CHECK(!g.stored.empty() && g.stored.back() == 112);
  CHECK(g.vrx->proposal() == 112);
  // the scout keeps searching the set but never measures or picks
  const size_t dwells0 = g.sink.dwells.size();
  for (int i = 0; i < 3000; ++i) g.tick();
  CHECK(g.core->op() == 112);
  CHECK(g.sink.picks.empty());
  CHECK(!g.sink.has_line("pick frozen on"));
  REQUIRE(g.sink.dwells.size() > dwells0 + 4);  // a full search round at least
  for (size_t i = dwells0 + 1; i < g.sink.dwells.size(); ++i) {   // past one dwell already under way
    const uint8_t c = g.sink.dwells[i].second.survey.def.primary;
    CHECK(c == 40 || c == 64 || c == 112 || c == 144);   // members only, as a pinned boot searches
    CHECK(g.sink.dwells[i].second.survey.observe_ms == 0);   // no observe
  }
}

TEST(set_pin_refused_for_non_member) {
  Rig g(2, 0);
  CHECK(g.core->set_pin(100) == std::optional<std::string>("not a member"));
  CHECK(!g.core->pinned());
  CHECK(g.core->pick_open());
  CHECK(g.core->op() == 40);
}

TEST(set_pin_refused_while_hopping) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere_until_order(g);
  REQUIRE(g.core->hopping());
  const size_t lines0 = g.sink.lines.size();
  CHECK(g.core->set_pin(112) == std::optional<std::string>("hop in flight"));
  CHECK(!g.core->pinned());
  CHECK(!g.core->snapshot(0).pinned);
  CHECK(g.sink.lines.size() == lines0);        // nothing logged, nothing ordered
}

TEST(set_pin_refused_while_calibrating) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  g.tick(true, /*cal=*/true);
  CHECK(g.core->set_pin(112) == std::optional<std::string>("calibration running"));
  CHECK(!g.core->pinned());
}

TEST(set_pin_refused_at_the_hop_cap) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  REQUIRE(g.cfg.hop.max_hops_per_min == 4);
  cal_move(g, p, 112);
  cal_move(g, p, 144);
  cal_move(g, p, 64);
  cal_move(g, p, 40);
  CHECK(g.core->set_pin(112) == std::optional<std::string>("hop cap"));
  CHECK(!g.core->pinned());
  CHECK(!g.core->set_pin(40).has_value());     // pinning where the link is needs no hop
  CHECK(g.core->pinned());
}

TEST(set_pin_to_op_is_a_noop) {
  Rig g(2, 0, true, 40);
  const size_t lines0 = g.sink.lines.size();
  CHECK(!g.core->set_pin(40).has_value());
  CHECK(g.core->op() == 40);
  CHECK(g.core->pinned());
  CHECK(g.sink.lines.size() == lines0);
  g.tick();
  CHECK(g.sink.moves.empty());
  CHECK(g.stored.empty());
}

TEST(set_auto_from_pinned_rearms_the_hop_from_op_without_a_relocate) {
  Rig g(2, 0, true, 40);
  link_up(g);
  for (int i = 0; i < 10; ++i) g.tick(true);
  const size_t sweep = g.sink.dwells.size();
  g.core->run_inflight_step();
  g.tick(true);
  REQUIRE(g.sink.dwells.size() == sweep);       // pinned: the in-flight step skips
  g.core->set_auto();
  CHECK(!g.core->pinned());
  CHECK(!g.core->snapshot(0).pinned);
  CHECK(g.core->op() == 40);
  CHECK(!g.core->hopping());
  g.tick(true);
  // the in-flight scout is now armed: run_inflight_step does work instead of skipping
  CHECK(!g.core->pick_open());                  // linked: no pick reopens
  CHECK(std::string(g.core->snapshot(0).scan_state) == "frozen");
  g.core->run_inflight_step();
  g.tick(true);
  CHECK(g.sink.dwells.size() == sweep + 1);
  REQUIRE(g.core->snapshot(0).dwell[1].has_value());
  CHECK(g.core->snapshot(0).dwell[1]->visits == 1);
  CHECK(g.cards[1]->ch == 40);                  // back on op after the dwell
  CHECK(!g.sink.has_hop("relocate"));
  // and an interfered op is fled again
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere_until_order(g);
  CHECK(g.sink.has_hop("order"));
}

TEST(set_auto_while_unlinked_reopens_the_boot_pick) {
  Rig g(2, 0, true, 40);
  g.tick();
  g.clk.ms += 60000;                            // long past radio.scan.max_ms since start
  g.tick();
  g.core->set_auto();
  CHECK(g.core->pick_open());
  CHECK(std::string(g.core->snapshot(0).scan_state) == "scouting");
  CHECK(!g.core->snapshot(0).scan_pick.has_value());
  g.tick();
  CHECK(g.core->pick_open());                   // max_ms counts from the reopen
  // the scout measures again (one 40 MHz dwell per pair, on its primary)
  // and the pick commits off the blocked op pair
  g.cards[1]->nhm_busy_pct_on[40] = 100;
  for (int i = 0; i < 3000 && !g.sink.has_line("pick frozen on"); ++i) g.tick();
  REQUIRE(g.sink.has_line("pick frozen on"));
  CHECK(g.sink.has_line("(commit)"));
  CHECK(g.core->op() != 40);
  bool observed_40 = false;
  for (const auto& d : g.sink.dwells) {
    if (d.second.survey.def.primary == 40) observed_40 = true;
    CHECK(d.second.survey.def.primary != 36);   // never a secondary
    CHECK(d.second.survey.def.width == CHANNEL_WIDTH_40);
  }
  CHECK(observed_40);
}

// A pin's relocate cut by link loss is a relocate like any other: the GS
// accepts the channel the link is on and stays pinned there.
TEST(set_pin_relocate_lost_with_the_session_stays_on_op) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  REQUIRE(!g.core->set_pin(112).has_value());
  for (int i = 0; i < 20 && !g.sink.has_line("relocate 40 -> 112 placed"); ++i) live_tick(g);
  REQUIRE(g.sink.has_line("relocate 40 -> 112 placed (pin)"));
  g.tick(false);                                // session lost before the confirm
  CHECK(g.sink.has_hop("session_lost"));
  for (int i = 0; i < 5; ++i) g.tick(false);
  CHECK(g.core->op() == 40);
  CHECK(g.core->pinned());
  CHECK(g.sink.has_line("relocation to 112 did not land; staying on 40"));
}

// Back to auto before the pin's relocate left: it is withdrawn, the link
// stays where it is (auto places no relocate of its own).
TEST(set_auto_withdraws_a_pin_relocate_not_yet_placed) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  REQUIRE(!g.core->set_pin(112).has_value());
  g.core->set_auto();
  CHECK(!g.core->pinned());
  for (int i = 0; i < 50; ++i) live_tick(g);
  CHECK(!g.sink.has_hop("relocate"));
  CHECK(g.core->op() == 40);
}

// One card (the web GS shape) takes a live pin the same way: the boot
// pick's relocate, tagged pin, on the one-card order path.
TEST(set_pin_one_card_linked_relocates) {
  Rig g(1, 0);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  CHECK(!g.core->set_pin(112).has_value());
  for (int i = 0; i < 50 && !g.sink.has_line("relocate 40 -> 112 placed"); ++i) live_tick(g);
  CHECK(g.sink.has_line("relocate 40 -> 112 placed (pin)"));
  CHECK(g.core->snapshot(0).pinned);
}

// One card (the web GS shape), booted pinned, back to auto with no link:
// the reopened pick runs the silent prelude again and commits its ranking
// off a busy op, as a one-card boot does.
TEST(set_auto_while_unlinked_one_card_reruns_the_prelude_and_commits) {
  Rig g(1, 0, true, 40);
  for (int i = 0; i < 10; ++i) g.tick();
  g.clk.ms += 60000;                            // long past max_ms and one_card_ms since start
  g.tick();
  g.cards[0]->nhm_busy_pct_on[40] = 100;         // 40 blocked, the rest unread
  g.core->set_auto();
  CHECK(g.core->pick_open());
  for (int i = 0; i < 800 && !g.sink.has_line("one-card prelude ranking picks"); ++i) g.tick();
  REQUIRE(g.sink.has_line("one-card prelude ranking picks"));
  CHECK(g.core->op() != 40);                    // committed off the busy channel
  CHECK(g.sink.has_move(MoveReason::Commit));
  CHECK(!g.stored.empty() && g.stored.back() == g.core->op());
  CHECK(g.core->pick_open());                   // the one-card pick stays open after the prelude
}

// ---- pin_refusal: set_pin's answer, asked without placing anything ----

// pin_refusal(ch) answers what set_pin(ch) would, and changes nothing; the
// set_pin that follows must then agree. Called last in a test whose
// expected answer is a refusal (a refused set_pin changes nothing either).
static void pin_refusal_agrees(Rig& g, uint8_t ch, const std::optional<std::string>& want) {
  const bool pinned0 = g.core->pinned();
  const uint8_t op0 = g.core->op();
  const size_t lines0 = g.sink.lines.size(), hops0 = g.sink.hops.size();
  CHECK(g.core->pin_refusal(ch) == want);
  CHECK(g.core->pinned() == pinned0);
  CHECK(g.core->op() == op0);
  CHECK(g.sink.lines.size() == lines0);
  CHECK(g.sink.hops.size() == hops0);
  CHECK(g.core->set_pin(ch) == want);
}

TEST(pin_refusal_agrees_with_set_pin_for_a_non_member_and_a_free_member) {
  Rig g(2, 0);
  pin_refusal_agrees(g, 100, std::string("not a member"));
  pin_refusal_agrees(g, 112, std::nullopt);
}

// The double-tap: a second pin while the first pin's relocate is still
// out. plan_.hopping() is not what refuses it.
TEST(pin_refusal_agrees_with_set_pin_while_a_relocate_is_in_flight) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  REQUIRE(!g.core->set_pin(112).has_value());
  for (int i = 0; i < 20 && !g.sink.has_line("relocate 40 -> 112 placed"); ++i) live_tick(g);
  REQUIRE(g.sink.has_line("relocate 40 -> 112 placed (pin)"));
  pin_refusal_agrees(g, 144, std::string("relocate in flight"));
  pin_refusal_agrees(g, 112, std::string("relocate in flight"));
}

// One card: the pin's relocate is out on the RCFs while the plan does not
// call it hopping yet (the sole radio stays on op until the repeats).
TEST(pin_refusal_agrees_with_set_pin_while_a_one_card_relocate_is_in_flight) {
  Rig g(1, 0);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  REQUIRE(!g.core->set_pin(112).has_value());
  for (int i = 0; i < 50 && !g.sink.has_line("relocate 40 -> 112 placed"); ++i) live_tick(g);
  REQUIRE(g.sink.has_line("relocate 40 -> 112 placed (pin)"));
  REQUIRE(!g.core->hopping());
  pin_refusal_agrees(g, 144, std::string("relocate in flight"));
}

TEST(pin_refusal_agrees_with_set_pin_while_a_two_card_hop_is_in_flight) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50; g.cards[1]->cca_per_ms_on[40] = 50;
  interfere_until_order(g);
  REQUIRE(g.core->hopping());
  pin_refusal_agrees(g, 112, std::string("hop in flight"));
}

// One card: the reactive hop's Order rides the RCFs before the radio
// retunes, and the plan does not call it hopping until then.
TEST(pin_refusal_agrees_with_set_pin_while_a_one_card_reactive_hop_is_ordered) {
  Rig g(1, 0);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick(true);
  g.cards[0]->cca_per_ms_on[40] = 50;
  interfere_until_order(g);
  REQUIRE(!g.sink.has_hop("one_card_retune"));
  REQUIRE(!g.core->hopping());
  REQUIRE(!g.core->sweep_pending());
  pin_refusal_agrees(g, 112, std::string("hop in flight"));
}

TEST(pin_refusal_agrees_with_set_pin_at_the_hop_cap) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  cal_move(g, p, 112);
  cal_move(g, p, 144);
  cal_move(g, p, 64);
  cal_move(g, p, 40);
  REQUIRE(!g.core->hopping());
  pin_refusal_agrees(g, 112, std::string("hop cap"));
  pin_refusal_agrees(g, 40, std::nullopt);      // pinning where the link is needs no hop
}

TEST(pin_refusal_agrees_with_set_pin_while_a_relay_sweeps) {
  Rig g(1, 1);
  g.cards[0]->retune_ok = false;
  freeze_auto(g);
  g.cards[0]->retune_ok = true;
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true, false, 0);
  g.cards[0]->cca_per_ms_on[40] = 50;
  interfere(g, occ_lead(g) + 3);
  REQUIRE(g.core->sweep_pending());
  REQUIRE(!g.core->hopping());
  pin_refusal_agrees(g, 112, std::string("sweep in flight"));
}

TEST(pin_refusal_agrees_with_set_pin_while_a_calibration_run_is_live) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 5; ++i) live_tick(g);
  g.tick(true, /*cal=*/true);
  pin_refusal_agrees(g, 112, std::string("calibration running"));
}

// A run's channel pick before the run itself reports running: its measure
// round, then its relocate.
TEST(pin_refusal_agrees_with_set_pin_while_a_calibration_picks_its_channel) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 10; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  p.begin_measure(g.clk.ms);                    // the round waits for its tick
  pin_refusal_agrees(g, 112, std::string("calibration running"));
  start_round(g, p);                            // and now runs
  pin_refusal_agrees(g, 112, std::string("calibration running"));
  p.cancel();
  for (int i = 0; i < 3; ++i) live_tick(g);
  REQUIRE(p.order_relocate(112));               // the run's relocate, not yet placed
  pin_refusal_agrees(g, 144, std::string("calibration running"));
}

// ---- the width flip while the scout searches or a sweep runs ----

static bool called_set_width(const FakeCard& c) {
  for (const auto& s : c.calls) if (s.rfind("set_width", 0) == 0) return true;
  return false;
}

// Auto, the pick frozen long ago, the drone never found: the scout card is
// out searching and stays out for as long as the GS is unlinked. A flip is
// taken: the link card is retuned on the spot, the searching scout card is
// left alone until the search lets go of it, then retuned to the new width.
TEST(width_flip_while_unlinked_after_the_pick_froze_retunes_the_free_cards) {
  Rig g(2, 0);
  freeze_auto(g);
  for (int i = 0; i < 5; ++i) g.tick();
  REQUIRE(g.core->scout_owns_card(1));            // the unlinked search
  CHECK(g.core->width_refusal(20) == std::nullopt);
  for (auto& c : g.cards) c->calls.clear();
  g.core->set_width(20);
  CHECK(g.cards[0]->width_mhz == 20);              // retuned before set_width returned
  CHECK(g.cards[1]->width_mhz == 40);              // the search's card: not touched
  CHECK(!called_set_width(*g.cards[1]));
  for (int i = 0; i < 20; ++i) g.tick();
  CHECK(g.cards[1]->width_mhz == 40);              // still searching: still left alone
  // Dwells taken meanwhile are as wide as the card they ran on.
  bool saw_scout_dwell = false;
  for (const auto& d : g.sink.dwells)
    if (d.first == 1 && !(d.second.survey.flags & devourer::chanmig::kFlagRetuneFailed)) {
      saw_scout_dwell = true;
      CHECK(d.second.survey.def.width == CHANNEL_WIDTH_40);
    }
  CHECK(saw_scout_dwell);
  link_up(g);
  for (int i = 0; i < 10; ++i) g.tick(true);
  CHECK(!g.core->scout_owns_card(1));
  CHECK(g.cards[1]->width_mhz == 20);              // released: the resync takes it
  CHECK(called_set_width(*g.cards[1]));
  // The next search (link lost for radio.scan.search_after_ms) dwells at
  // the width the card now holds.
  g.sink.dwells.clear();
  for (int i = 0; i < g.cfg.radio.scan.search_after_ms / 10 + 60; ++i) g.tick();
  REQUIRE(g.core->scout_owns_card(1));
  bool saw = false;
  for (const auto& d : g.sink.dwells)
    if (d.first == 1) { saw = true; CHECK(d.second.survey.def.width == CHANNEL_WIDTH_20); }
  CHECK(saw);
  CHECK(g.cards[1]->width_mhz == 20);
}

// Pinned and unlinked: the scout only searches (no pick to hold open), so
// a flip is taken at once.
TEST(width_flip_pinned_unlinked_is_accepted) {
  Rig g(2, 0, /*pinned=*/true, 40);
  for (int i = 0; i < 5; ++i) g.tick();
  REQUIRE(g.core->scout_owns_card(1));
  CHECK(g.core->width_refusal(20) == std::nullopt);
  g.core->set_width(20);
  CHECK(g.cards[0]->width_mhz == 20);
  CHECK(g.cards[1]->width_mhz == 40);
}

// Pinned, linked, then the link lost for longer than search_after_ms: the
// search is back on the scout card, and a flip is still taken.
TEST(width_flip_after_the_link_is_lost_is_accepted) {
  Rig g(2, 0, /*pinned=*/true, 40);
  link_up(g);
  for (int i = 0; i < 10; ++i) g.tick(true);
  for (int i = 0; i < g.cfg.radio.scan.search_after_ms / 10 + 20; ++i) g.tick();
  REQUIRE(g.core->scout_owns_card(1));
  CHECK(g.core->width_refusal(20) == std::nullopt);
}

// While the boot pick is open no card's width is fixed up -- not even a
// card that is not the scout's -- and a flip is refused.
TEST(no_width_fixup_while_the_boot_pick_is_open) {
  Rig g(2, 0);
  g.cards[1]->retune_ok = false;                   // no dwell completes: only max_ms freezes the pick
  g.tick();
  REQUIRE(g.core->pick_open());
  g.cards[0]->width_mhz = 20;                      // a revive at the wrong width
  g.cards[0]->calls.clear();
  g.core->on_card_reopened(0);
  for (int i = 0; i < 20; ++i) g.tick();
  REQUIRE(g.core->pick_open());
  CHECK(!called_set_width(*g.cards[0]));
  CHECK(g.core->width_refusal(20) == std::optional<std::string>("ladder refused: scan in flight"));
  for (int i = 0; i < 4000 && g.core->pick_open(); ++i) g.tick();
  REQUIRE(!g.core->pick_open());
  g.tick();
  CHECK(g.cards[0]->width_mhz == 40);              // fixed once the pick froze
}

// A calibration measure round owns the scout card: refused as a scan.
TEST(width_refusal_while_a_measure_round_runs) {
  Rig g(2, 0);
  freeze_auto(g);
  link_up(g);
  for (int i = 0; i < 10; ++i) live_tick(g);
  ChannelCorePicker p(*g.core);
  start_round(g, p);
  CHECK(g.core->width_refusal(20) == std::optional<std::string>("ladder refused: scan in flight"));
}

// Past the scan check, width_refusal gives the busy refusals pin_refusal
// gives: a calibration run, a hop in flight.
TEST(width_refusal_falls_through_to_the_busy_refusals) {
  {
    Rig g(2, 0);
    freeze_auto(g);
    link_up(g);
    for (int i = 0; i < 5; ++i) live_tick(g);
    g.tick(true, /*cal=*/true);
    CHECK(g.core->width_refusal(20) == std::optional<std::string>("calibration running"));
    CHECK(g.core->width_refusal(40) == std::nullopt);   // no flip, no refusal
  }
  {
    Rig g(1, 0);
    freeze_auto(g);
    for (int i = 0; i < 5; ++i) g.tick(true);
    g.cards[0]->cca_per_ms_on[40] = 50;
    interfere_until_order(g);
    CHECK(g.core->width_refusal(20) == std::optional<std::string>("hop in flight"));
  }
}

// A relay sweep holds a card off op: no retune under it. The pin's own
// refusal for the same state is unchanged.
TEST(width_refusal_while_a_relay_sweeps) {
  Rig g(1, 1);
  g.cards[0]->retune_ok = false;
  freeze_auto(g);
  g.cards[0]->retune_ok = true;
  link_up(g);
  for (int i = 0; i < 5; ++i) g.tick(true, false, 0);
  g.cards[0]->cca_per_ms_on[40] = 50;
  interfere(g, occ_lead(g) + 3);
  REQUIRE(g.core->sweep_pending());
  REQUIRE(!g.core->hopping());
  CHECK(g.core->width_refusal(20) == std::optional<std::string>("sweep in flight"));
  CHECK(g.core->pin_refusal(40) == std::nullopt);      // pinning where the link is: unchanged
}

// The hop ranker ranks at the live width: at 40 a dwell's NHM busy reading
// (the whole pair) orders ahead of the frame terms (the primary only).
TEST(set_width_moves_the_hop_ranker_to_the_new_width) {
  Rig g(2, 0, /*pinned=*/true, 40, {40, 64, 112, 144}, 20);
  for (int i = 0; i < 10; ++i) g.tick(true, false, 0);
  CHECK(g.core->hop_ranker().width_mhz() == 20);
  g.core->set_width(40);
  CHECK(g.core->hop_ranker().width_mhz() == 40);
  g.core->set_width(20);
  CHECK(g.core->hop_ranker().width_mhz() == 20);
}

MTEST_MAIN
