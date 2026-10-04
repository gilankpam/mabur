#include <algorithm>
#include <memory>
#include <string>
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

MTEST_MAIN
