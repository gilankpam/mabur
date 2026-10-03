#include <string>
#include <utility>
#include <vector>
#include "mtest.h"
#include "channel_scout.h"
using namespace maburgs;

// Fake radio + fake clock: sleep() advances `now`, every call is recorded.
struct FakeRadio : ScoutRadio {
  int64_t now = 0;
  std::vector<std::string> calls;
  uint8_t ch = 0;
  uint8_t width = 20;
  uint32_t cca_per_ms_on[256] = {};  // busy rate per channel
  ScoutFrames fr;
  int64_t last_read = 0;
  bool retune(uint8_t c) override { ch = c; calls.push_back("retune " + std::to_string(c)); return true; }
  bool retune_width(uint8_t c, uint8_t w) override {
    ch = c; width = w;
    calls.push_back("retune_width " + std::to_string(c) + "/" + std::to_string(w));
    return true;
  }
  ScoutEnergy read_energy(bool with_nhm) override {
    calls.push_back(with_nhm ? "read+nhm" : "read");
    ScoutEnergy e; e.fa_valid = true;
    e.cca_ofdm = static_cast<uint32_t>((now - last_read) * cca_per_ms_on[ch]);
    last_read = now;
    if (with_nhm) { e.floor_valid = true; e.floor_dbm = -95; }
    return e;
  }
  ScoutEnergy read_energy_scout() override { return ScoutEnergy{}; }
  ScoutFrames frames() const override { return fr; }
};

static ScoutCfg two(bool measure = true) {
  ScoutCfg c; c.channels = {136, 144}; c.measure = measure; c.dwell_ms = 250; c.settle_ms = 30;
  c.min_rounds = 2; c.search_ms = 100; c.op_window_ms = 300; c.beacon_period_ms = 20;
  c.pick_margin = 20; c.link_width_mhz = 40;
  return c;
}

// (state, ms) per sleep: B = beaconing, Q = quiet, W = working.
struct Rig {
  FakeRadio r;
  ChannelScout s;
  std::vector<std::pair<std::string, int>> phases;
  explicit Rig(ScoutCfg c)
      : s(c, r, [this] { return r.now; }, [this](int ms) {
          phases.push_back({std::string(s.beaconing() ? "B" : "") + (s.quiet() ? "Q" : "") +
                                (s.working() ? "W" : ""),
                            ms});
          r.now += ms;
        }) {}
};

TEST(idle_until_search_or_measure_then_first_dwell_switches_to_20) {
  Rig g(two(false));                       // pinned: search only
  g.s.set_op(136); g.s.set_search(false);
  CHECK(!g.s.run_once()); CHECK(!g.s.working());
  CHECK(g.r.calls.empty());                // never worked: nothing to park
  g.s.set_search(true);
  CHECK(g.s.run_once()); CHECK(g.s.working());
  REQUIRE(!g.r.calls.empty());
  CHECK(g.r.calls[0] == "retune_width 136/20" && g.r.width == 20);
  g.s.run_once();
  CHECK(g.r.calls.back() == "retune 144");  // no second width switch
}

TEST(pinned_dwell_is_retune_settle_burst_only_on_members) {
  Rig g(two(false)); g.s.set_op(136); g.s.set_search(true);
  g.s.run_once();
  // settle(30,W) burst(100,BW) gap(20,W); no read at all
  REQUIRE(g.phases.size() == 3);
  CHECK(g.phases[0].first == "W" && g.phases[0].second == 30);
  CHECK(g.phases[1].first == "BW" && g.phases[1].second == 100);
  CHECK(g.phases[2].first == "W" && g.phases[2].second == 20);
  g.phases.clear(); g.s.run_once();
  CHECK(g.r.ch == 144);                    // members only: 132/140 never dwelt on
  g.s.run_once(); g.s.run_once();
  for (auto& c : g.r.calls) {
    CHECK(c.rfind("read", 0) != 0);        // pinned never reads energy
    CHECK(c.find("132") == std::string::npos && c.find("140") == std::string::npos);
  }
  CHECK(g.s.rounds() == 2);                // a full pass = both members
}

TEST(auto_unlinked_dwell_is_burst_then_quiet_observe_on_primaries_observe_only_on_secondaries) {
  Rig g(two()); g.s.set_op(136); g.s.set_search(true);
  g.s.run_once();                          // first half of 136's pair = 132 (not a member): no burst
  // settle(W) observe(QW 250)
  REQUIRE(g.phases.size() == 2);
  CHECK(g.phases[0].first == "W" && g.phases[0].second == 30);
  CHECK(g.phases[1].first == "QW" && g.phases[1].second == 250);
  CHECK(g.r.ch == 132);
  g.phases.clear(); g.s.run_once();        // 136: member -> burst, gap, observe
  REQUIRE(g.phases.size() == 4);
  CHECK(g.phases[1].first == "BW" && g.phases[1].second == 100);
  CHECK(g.phases[2].first == "W" && g.phases[2].second == 20);
  CHECK(g.phases[3].first == "QW" && g.phases[3].second == 250);
  CHECK(!g.s.quiet() && !g.s.beaconing());  // released after the dwell
  auto d = g.s.take_dwells();
  REQUIRE(d.size() == 2);
  CHECK(d[1].survey.def.primary == 136 && d[1].survey.observe_ms == 250);
  // dwell-set order is what ranking() exposes
  auto k = g.s.ranking();
  REQUIRE(k.size() == 4);
  CHECK(k[0].ch == 132 && k[1].ch == 136 && k[2].ch == 140 && k[3].ch == 144);
}

TEST(auto_linked_dwell_has_no_burst_is_never_quiet_and_subtracts_the_tx_leak) {
  ScoutCfg c = two(); c.leak_per_frame = 2.0;
  FakeRadio r;
  r.cca_per_ms_on[144] = 1;                // 250 cca per observe on 144
  std::vector<std::string> states;         // beaconing/quiet flags at every sleep
  uint64_t tx = 0;
  ChannelScout* sp = nullptr;
  // The TX card "sends" 60 frames during every observe: bump the cumulative
  // counter the core would feed in, from inside the 250 ms sleep.
  ChannelScout s(c, r, [&] { return r.now; }, [&](int ms) {
    states.push_back(std::string(sp->beaconing() ? "B" : "") + (sp->quiet() ? "Q" : ""));
    if (ms == 250) { tx += 60; sp->set_tx_frames(tx); }
    r.now += ms;
  });
  sp = &s;
  s.set_op(136); s.set_search(false);      // linked, pick open
  s.set_tx_frames(0);
  for (int i = 0; i < 4; ++i) s.run_once();           // 132 136 140 144
  REQUIRE(!states.empty());
  for (auto& st : states) CHECK(st.empty());          // never beaconing, never quiet
  bool seen = false;
  for (auto& e : s.ranking())
    if (e.ch == 144) { seen = true; CHECK(e.worst_busy == 250 - 120); }  // 250 cca - 2.0*60
  CHECK(seen);
}

TEST(a_tx_counter_that_goes_back_is_no_leak) {
  FakeRadio r;
  r.cca_per_ms_on[144] = 1;                // 250 cca per observe on 144
  ChannelScout* sp = nullptr;
  ChannelScout s(two(), r, [&] { return r.now; }, [&](int ms) {
    if (ms == 250) sp->set_tx_frames(0);   // reset below the pre-observe 1000
    r.now += ms;
  });
  sp = &s;
  s.set_op(136); s.set_search(false);
  for (int i = 0; i < 4; ++i) { s.set_tx_frames(1000); s.run_once(); }
  bool seen = false;
  for (auto& e : s.ranking())
    if (e.ch == 144) { seen = true; CHECK(e.worst_busy == 250); }
  CHECK(seen);
}

TEST(two_card_proposal_matures_after_min_rounds_and_moves_only_past_the_margin) {
  Rig g(two()); g.s.set_op(136); g.s.set_search(false);
  for (int i = 0; i < 4; ++i) g.s.run_once();
  CHECK(!g.s.mature()); CHECK(g.s.proposal() == 136);
  CHECK(g.s.pick_ranking().empty());
  for (int i = 0; i < 4; ++i) g.s.run_once();
  CHECK(g.s.mature()); CHECK(g.s.rounds() == 2);
  CHECK(g.s.proposal() == 136);            // tie: stay
  Rig h(two()); h.s.set_op(136); h.s.set_search(false);
  h.r.cca_per_ms_on[132] = 1;              // 136's pair worse half = 250/visit; 144 clean
  for (int i = 0; i < 8; ++i) h.s.run_once();
  CHECK(h.s.proposal() == 144);
  auto pr = h.s.pick_ranking();
  REQUIRE(pr.size() == 2);
  CHECK(pr[0] == 144 && pr[1] == 136);
  h.s.set_op(144);                         // the margin is against op: on 144, stay
  CHECK(h.s.proposal() == 144);
}

// publish_() reads op under the lock. The cross-thread interleaving itself
// (scout loads op, core set_op()s + publishes, scout overwrites) cannot be
// staged with the single-threaded fake; this pins the observable contract:
// an op change that lands during the observe is what the post-dwell
// publish proposes against.
TEST(op_change_during_the_observe_is_what_the_dwell_publishes_against) {
  FakeRadio r;                             // all halves clean: a tie stays on op
  ChannelScout* sp = nullptr;
  int observes = 0;
  ChannelScout s(two(), r, [&] { return r.now; }, [&](int ms) {
    if (ms == 250 && ++observes == 8) sp->set_op(144);   // the last dwell (144)
    r.now += ms;
  });
  sp = &s;
  s.set_op(144); s.set_search(false);
  for (int i = 0; i < 7; ++i) s.run_once();
  s.set_op(136);
  CHECK(s.proposal() == 136);              // nothing mature: op
  s.run_once();                            // op flips to 144 mid-observe
  CHECK(s.mature());
  CHECK(s.proposal() == 144);              // the tie is judged against the new op
}

TEST(two_card_width_20_ranks_the_members_and_proposes_past_the_margin) {
  ScoutCfg c = two(); c.link_width_mhz = 20; Rig g(c);
  g.s.set_op(136); g.s.set_search(false);
  g.r.cca_per_ms_on[136] = 1;
  for (int i = 0; i < 4; ++i) g.s.run_once();
  CHECK(g.s.mature()); CHECK(g.s.rounds() == 2);
  CHECK(g.s.proposal() == 144);
  auto pr = g.s.pick_ranking();
  REQUIRE(pr.size() == 2);
  CHECK(pr[0] == 144 && pr[1] == 136);
  g.s.freeze();
  CHECK(!g.s.run_once());
  CHECK(g.r.calls.back() == "retune 136");  // park at 20 on op
}

TEST(freeze_stops_measuring_and_parks_at_link_width_on_op) {
  Rig g(two()); g.s.set_op(144); g.s.set_search(false);
  g.s.run_once();
  CHECK(g.s.pick_open() && g.s.working());
  g.s.freeze();
  CHECK(!g.s.pick_open());
  CHECK(!g.s.run_once());                  // nothing to do: parked
  CHECK(!g.s.working() && !g.s.quiet() && !g.s.beaconing());
  CHECK(g.r.calls.back() == "retune_width 144/40");
  const size_t n = g.r.calls.size();
  CHECK(!g.s.run_once());                  // parks once
  CHECK(g.r.calls.size() == n);
  g.s.set_search(true);                    // a later loss: search resumes, measurement does not
  g.phases.clear(); g.s.run_once(); g.s.run_once();
  for (auto& p : g.phases) CHECK(p.first.find('Q') == std::string::npos);
  for (size_t i = n; i < g.r.calls.size(); ++i) CHECK(g.r.calls[i].rfind("read", 0) != 0);
  CHECK(g.r.calls[n].rfind("retune_width ", 0) == 0 && g.r.width == 20);
  CHECK(g.s.working());
}

TEST(one_card_prelude_is_silent_then_op_windows_alternate_with_dwells) {
  ScoutCfg c = two(); c.one_card = true; c.one_card_ms = 1000; Rig g(c);
  g.s.set_op(136); g.s.set_search(true);
  CHECK(!g.s.prelude_done());
  while (g.r.now < 1000) g.s.run_once();   // silent dwells: never beaconing
  REQUIRE(!g.phases.empty());
  for (auto& p : g.phases) CHECK(p.first.find('B') == std::string::npos);
  bool observed = false;                   // prelude dwells observe
  for (auto& c : g.r.calls) observed = observed || c == "read+nhm";
  CHECK(observed);
  CHECK(!g.s.prelude_done());
  g.s.run_once();
  CHECK(g.s.prelude_done());
  // proposal available on the deadline ranking (1 visit per half suffices)
  CHECK(g.s.mature());
  CHECK(g.s.proposal() == 136 || g.s.proposal() == 144);
  g.s.ack_prelude(136);                     // the core committed (here: op unchanged)
  // op window: retune to op, beacon 300, gap 20, then a dwell with a burst
  g.phases.clear();
  const size_t n_calls = g.r.calls.size();
  g.s.run_once();
  REQUIRE(n_calls < g.r.calls.size());
  CHECK(g.r.calls[n_calls] == "retune 136");   // the window tunes the card to op first
  REQUIRE(g.phases.size() >= 2);
  CHECK(g.phases[0].first == "BW" && g.phases[0].second == 300);
  CHECK(g.phases[1].first == "W" && g.phases[1].second == 20);
  // The dwell after the window bursts on a member; the half set alternates
  // primary/secondary, so one of the next two steps carries it.
  g.s.run_once();
  bool burst = false;
  for (auto& p : g.phases) burst = burst || (p.first == "BW" && p.second == 100);
  CHECK(burst);
  // the prelude never re-arms
  g.s.set_search(false); g.s.run_once(); g.s.set_search(true);
  CHECK(g.s.prelude_done());
}

TEST(one_card_no_disc_window_on_the_old_op_after_the_prelude) {
  ScoutCfg c = two(); c.one_card = true; c.one_card_ms = 1000; Rig g(c);
  g.s.set_op(136); g.s.set_search(true);
  while (!g.s.prelude_done()) g.s.run_once();
  // Until the core commits, no op window at all: no beaconing, no tune.
  g.phases.clear();
  const size_t n_calls = g.r.calls.size();
  for (int i = 0; i < 5; ++i) CHECK(!g.s.run_once());
  CHECK(g.r.calls.size() == n_calls);
  for (auto& p : g.phases) CHECK(p.first.find('B') == std::string::npos);
  CHECK(!g.s.beaconing());
  CHECK(g.s.working());                     // still owns the card meanwhile
  // A stray set_op of the old op does not release it either.
  g.s.set_op(136);
  CHECK(!g.s.run_once());
  CHECK(g.r.calls.size() == n_calls);
  // The core commits 144: the first window tunes to 144 and beacons there.
  g.s.ack_prelude(144);
  g.phases.clear();
  CHECK(g.s.run_once());
  REQUIRE(g.r.calls.size() > n_calls);
  CHECK(g.r.calls[n_calls] == "retune 144");
  REQUIRE(!g.phases.empty());
  CHECK(g.phases[0].first == "BW" && g.phases[0].second == 300);
  for (size_t i = n_calls; i < g.r.calls.size(); ++i) CHECK(g.r.calls[i] != "retune 136");
}

TEST(one_card_frozen_before_ack_does_not_wait) {
  ScoutCfg c = two(); c.one_card = true; c.one_card_ms = 1000; Rig g(c);
  g.s.set_op(136); g.s.set_search(true);
  while (!g.s.prelude_done()) g.s.run_once();
  g.s.freeze();                             // pick closed (e.g. max_ms): search only
  g.phases.clear();
  CHECK(g.s.run_once());
  REQUIRE(!g.phases.empty());
  CHECK(g.phases[0].first == "BW" && g.phases[0].second == 300);
}

TEST(one_card_pinned_has_no_prelude) {
  ScoutCfg c = two(false); c.one_card = true; Rig g(c);
  g.s.set_op(136); g.s.set_search(true);
  CHECK(g.s.prelude_done());
  g.s.run_once();
  REQUIRE(!g.phases.empty());
  CHECK(g.phases[0].first == "BW" && g.phases[0].second == 300);
}

TEST(run_exits_on_stop_and_parks) {
  Rig g(two()); g.s.set_op(136); g.s.set_search(true);
  g.s.run_once();
  CHECK(g.s.working());
  g.s.stop();
  g.s.run();
  CHECK(g.s.done() && !g.s.working());
  CHECK(g.r.calls.back() == "retune_width 136/40");
}
MTEST_MAIN
