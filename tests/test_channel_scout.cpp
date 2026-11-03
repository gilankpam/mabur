#include <algorithm>
#include <iterator>
#include <string>
#include <utility>
#include <vector>
#include "mtest.h"
#include "channel_scout.h"
#include "mabur/ht40.h"
#include "nhm_busy.h"
using namespace maburgs;

// Fake radio + fake clock: sleep() advances `now`, every call is recorded.
// The radio has no width switch: the card stays at whatever width it was
// opened with, so every tune is a plain retune.
struct FakeRadio : ScoutRadio {
  int64_t now = 0;
  std::vector<std::string> calls;
  uint8_t ch = 0;
  uint32_t cca_per_ms_on[256] = {};  // busy rate per channel
  int busy_pct_on[256];              // NHM busy % per channel; -1 (default): no NHM window
  uint16_t armed = 0;
  ScoutFrames fr;
  int64_t last_read = 0;
  FakeRadio() { std::fill(std::begin(busy_pct_on), std::end(busy_pct_on), -1); }
  bool retune(uint8_t c) override { ch = c; calls.push_back("retune " + std::to_string(c)); return true; }
  bool arm_nhm_busy(uint16_t p) override { armed = p; return busy_pct_on[ch] >= 0; }
  NhmBusyRead read_nhm_busy() override {
    NhmBusyRead r; r.valid = true; r.period = armed;
    r.buckets[0] = static_cast<uint8_t>(100 - busy_pct_on[ch]);   // below every edge
    r.buckets[11] = static_cast<uint8_t>(busy_pct_on[ch]);        // above every edge
    return r;
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

TEST(idle_until_search_or_measure_then_dwells_retune_the_card_as_opened) {
  Rig g(two(false));                       // pinned: search only
  g.s.set_op(136); g.s.set_search(false);
  CHECK(!g.s.run_once()); CHECK(!g.s.working());
  CHECK(g.r.calls.empty());                // never worked: nothing to park
  g.s.set_search(true);
  CHECK(g.s.run_once()); CHECK(g.s.working());
  REQUIRE(!g.r.calls.empty());
  CHECK(g.r.calls[0] == "retune 136");
  g.s.run_once();
  CHECK(g.r.calls.back() == "retune 144");
}

// Width 40: the card is opened at 40 and stays there. One dwell per pair,
// on its primary, in config order; the dwell is recorded at 40. Revert
// (dwell both halves): 132/140/36 are tuned and ranked.
TEST(at_40_the_dwell_set_is_the_primaries_and_the_card_never_switches_width) {
  ScoutCfg c = two(); c.channels = {136, 144, 40}; Rig g(c);
  g.s.set_op(136); g.s.set_search(true);
  for (int i = 0; i < 3; ++i) g.s.run_once();
  CHECK(g.s.rounds() == 1);                // one full pass = one dwell per pair
  std::vector<std::string> tunes;
  for (const auto& s : g.r.calls)
    if (s.rfind("retune", 0) == 0) tunes.push_back(s);
  CHECK((tunes == std::vector<std::string>{"retune 136", "retune 144", "retune 40"}));
  const auto d = g.s.take_dwells();
  REQUIRE(d.size() == 3);
  for (const auto& x : d) {
    CHECK(x.survey.def.width == CHANNEL_WIDTH_40);
    CHECK(x.survey.def.offset == mabur::ht40_offset(x.survey.def.primary));   // as FastRetune keeps it
  }
  const auto k = g.s.ranking();
  REQUIRE(k.size() == 3);
  CHECK(k[0].ch == 136 && k[1].ch == 144 && k[2].ch == 40);
}

TEST(park_at_40_is_a_plain_retune_to_op) {
  Rig g(two()); g.s.set_op(136); g.s.set_search(true);
  g.s.run_once(); g.s.run_once();          // 136 144: the card sits on 144
  g.s.set_search(false); g.s.freeze();
  CHECK(!g.s.run_once());
  CHECK(g.r.calls.back() == "retune 136");
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

TEST(auto_unlinked_dwell_is_burst_then_quiet_observe_on_every_member) {
  Rig g(two()); g.s.set_op(136); g.s.set_search(true);
  g.s.run_once();                          // 136: settle(W) burst(BW) gap(W) observe(QW)
  REQUIRE(g.phases.size() == 4);
  CHECK(g.phases[0].first == "W" && g.phases[0].second == 30);
  CHECK(g.phases[1].first == "BW" && g.phases[1].second == 100);
  CHECK(g.phases[2].first == "W" && g.phases[2].second == 20);
  CHECK(g.phases[3].first == "QW" && g.phases[3].second == 250);
  CHECK(g.r.ch == 136);
  CHECK(!g.s.quiet() && !g.s.beaconing());  // released after the dwell
  g.phases.clear(); g.s.run_once();        // 144: the same
  REQUIRE(g.phases.size() == 4);
  CHECK(g.phases[1].first == "BW" && g.phases[3].first == "QW");
  auto d = g.s.take_dwells();
  REQUIRE(d.size() == 2);
  CHECK(d[0].survey.def.primary == 136 && d[0].survey.observe_ms == 250);
  CHECK(d[1].survey.def.primary == 144);
  // dwell-set order is what ranking() exposes
  auto k = g.s.ranking();
  REQUIRE(k.size() == 2);
  CHECK(k[0].ch == 136 && k[1].ch == 144);
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
  for (int i = 0; i < 2; ++i) s.run_once();           // 144 twice: op 136 is skipped
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
  for (int i = 0; i < 2; ++i) { s.set_tx_frames(1000); s.run_once(); }
  bool seen = false;
  for (auto& e : s.ranking())
    if (e.ch == 144) { seen = true; CHECK(e.worst_busy == 250); }
  CHECK(seen);
}

// Unlinked (searching): op is measured like every other channel. A linked
// scout skips op's own pair (tests below).
TEST(two_card_proposal_matures_after_min_rounds_and_moves_only_past_the_margin) {
  Rig g(two()); g.s.set_op(136); g.s.set_search(true);
  for (int i = 0; i < 2; ++i) g.s.run_once();
  CHECK(!g.s.mature()); CHECK(g.s.proposal() == 136);
  CHECK(g.s.pick_ranking().empty());
  for (int i = 0; i < 2; ++i) g.s.run_once();
  CHECK(g.s.mature()); CHECK(g.s.rounds() == 2);
  CHECK(g.s.proposal() == 136);            // tie: stay
  Rig h(two()); h.s.set_op(136); h.s.set_search(true);
  h.r.cca_per_ms_on[136] = 1;              // 136's pair = 250/visit; 144 clean
  for (int i = 0; i < 4; ++i) h.s.run_once();
  CHECK(h.s.proposal() == 144);
  auto pr = h.s.pick_ranking();
  REQUIRE(pr.size() == 2);
  CHECK(pr[0] == 144 && pr[1] == 136);
  h.s.set_op(144);                         // the margin is against op: on 144, stay
  CHECK(h.s.proposal() == 144);
}

// Width 40 ranks NHM busy first: a pair with fewer frames but more busy
// airtime loses, and op keeps the pick until a candidate beats its busy by
// pick_margin_pct points. Revert (frame score first): 136 (0 events) is
// proposed over 144; revert (pick_margin_pct not passed in): 12 points
// against a margin of 15 still moves.
TEST(at_40_proposal_ranks_busy_first_and_holds_op_by_pick_margin_pct) {
  ScoutCfg c = two(); c.pick_margin_pct = 15;
  Rig g(c); g.s.set_op(136); g.s.set_search(true);
  g.r.busy_pct_on[136] = 30;
  g.r.busy_pct_on[144] = 18;
  g.r.cca_per_ms_on[144] = 1;              // 144 has the worse frame score
  for (int i = 0; i < 4; ++i) g.s.run_once();
  REQUIRE(g.s.mature());
  const auto pr = g.s.pick_ranking();
  REQUIRE(pr.size() == 2);
  CHECK(pr[0] == 144 && pr[1] == 136);     // busy first
  CHECK(g.s.proposal() == 136);            // 30 - 18 = 12 < 15: op holds
  Rig h(c); h.s.set_op(136); h.s.set_search(true);
  h.r.busy_pct_on[136] = 40;
  h.r.busy_pct_on[144] = 18;
  h.r.cca_per_ms_on[144] = 1;
  for (int i = 0; i < 4; ++i) h.s.run_once();
  CHECK(h.s.proposal() == 144);            // 22 >= 15
}

// publish_() reads op under the lock. The cross-thread interleaving itself
// (scout loads op, core set_op()s + publishes, scout overwrites) cannot be
// staged with the single-threaded fake; this pins the observable contract:
// an op change that lands during the observe is what the post-dwell
// publish proposes against.
TEST(op_change_during_the_observe_is_what_the_dwell_publishes_against) {
  FakeRadio r;                             // all members clean: a tie stays on op
  ChannelScout* sp = nullptr;
  int observes = 0;
  ChannelScout s(two(), r, [&] { return r.now; }, [&](int ms) {
    if (ms == 250 && ++observes == 4) sp->set_op(144);   // the last dwell (144)
    r.now += ms;
  });
  sp = &s;
  s.set_op(144); s.set_search(true);
  for (int i = 0; i < 3; ++i) s.run_once();
  s.set_op(136);
  CHECK(s.proposal() == 136);              // nothing mature: op
  s.run_once();                            // op flips to 144 mid-observe
  CHECK(s.mature());
  CHECK(s.proposal() == 144);              // the tie is judged against the new op
}

TEST(two_card_width_20_ranks_the_members_and_proposes_past_the_margin) {
  ScoutCfg c = two(); c.link_width_mhz = 20; Rig g(c);
  g.s.set_op(136); g.s.set_search(true);
  g.r.cca_per_ms_on[136] = 1;
  for (int i = 0; i < 4; ++i) g.s.run_once();
  CHECK(g.s.mature()); CHECK(g.s.rounds() == 2);
  CHECK(g.s.proposal() == 144);
  auto pr = g.s.pick_ranking();
  REQUIRE(pr.size() == 2);
  CHECK(pr[0] == 144 && pr[1] == 136);
  g.s.set_search(false); g.s.freeze();
  CHECK(!g.s.run_once());
  CHECK(g.r.calls.back() == "retune 136");  // park at 20 on op
}

// Linked (search off, pick open), op's own pair carries the drone's video
// -- in NHM's airtime and in the frame counts -- so the scout never tunes
// to op and op keeps only its pre-link visits. Revert (no skip in
// step_dwell_): 136 is dwelt on and ranked against the link's own video.
TEST(linked_dwells_never_tune_to_op) {
  Rig g(two()); g.s.set_op(136); g.s.set_search(false);
  for (int i = 0; i < 4; ++i) CHECK(g.s.run_once());
  for (auto& c : g.r.calls)
    CHECK(c.find("132") == std::string::npos && c.find("136") == std::string::npos);
  for (auto& e : g.s.ranking())
    CHECK(e.ch == 136 ? e.visits == 0 : e.visits == 4);
  CHECK(g.s.rounds() == 4);                // skipped bins still advance the round-robin
}

TEST(linked_width_20_skips_only_op) {
  ScoutCfg c = two(); c.link_width_mhz = 20; Rig g(c);
  g.s.set_op(136); g.s.set_search(false);
  for (int i = 0; i < 3; ++i) g.s.run_once();
  for (auto& call : g.r.calls) CHECK(call.find("136") == std::string::npos);
  for (auto& e : g.s.ranking()) CHECK(e.ch == 136 ? e.visits == 0 : e.visits == 3);
}

// mature() ignores op once linked; op_ranked() says whether op's pre-link
// visits were enough. Revert (mature() requires op): a linked scout never
// matures.
TEST(linked_mature_ignores_op_and_op_ranked_reports_it) {
  Rig g(two()); g.s.set_op(136); g.s.set_search(true);
  g.s.run_once();                          // 136: one pre-link visit
  g.s.set_search(false);                   // linked
  for (int i = 0; i < 2; ++i) g.s.run_once();   // 144 x2
  CHECK(g.s.mature());
  CHECK(!g.s.op_ranked());                 // 1 < min_rounds 2: op unmeasured
  Rig h(two()); h.s.set_op(136); h.s.set_search(true);
  for (int i = 0; i < 3; ++i) h.s.run_once();   // 136 144 136
  h.s.set_search(false);
  CHECK(!h.s.mature());                    // 144 has 1 visit
  h.s.run_once();                          // 144
  CHECK(h.s.mature() && h.s.op_ranked());
}

// Unlinked, op is visited like any other channel, so maturity waits for its
// last visit too. Revert (mature() ignores op while searching): mature()
// fires one dwell early with op's pair a visit short.
TEST(unlinked_mature_waits_for_op) {
  Rig g(two()); g.s.set_op(144); g.s.set_search(true);   // op's pair is visited last
  for (int i = 0; i < 3; ++i) g.s.run_once();
  CHECK(!g.s.op_ranked());
  CHECK(!g.s.mature());
  g.s.run_once();
  CHECK(g.s.op_ranked() && g.s.mature());
}

TEST(freeze_stops_measuring_and_parks_on_op) {
  Rig g(two()); g.s.set_op(144); g.s.set_search(false);
  g.s.run_once();
  CHECK(g.s.pick_open() && g.s.working());
  g.s.freeze();
  CHECK(!g.s.pick_open());
  CHECK(!g.s.run_once());                  // nothing to do: parked
  CHECK(!g.s.working() && !g.s.quiet() && !g.s.beaconing());
  CHECK(g.r.calls.back() == "retune 144");
  const size_t n = g.r.calls.size();
  CHECK(!g.s.run_once());                  // parks once
  CHECK(g.r.calls.size() == n);
  g.s.set_search(true);                    // a later loss: search resumes, measurement does not
  g.phases.clear(); g.s.run_once(); g.s.run_once();
  for (auto& p : g.phases) CHECK(p.first.find('Q') == std::string::npos);
  for (size_t i = n; i < g.r.calls.size(); ++i) CHECK(g.r.calls[i].rfind("read", 0) != 0);
  CHECK(g.r.calls[n] == "retune 144");     // the round-robin's next member, a plain retune
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
  // proposal available on the deadline ranking (1 visit per pair suffices)
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
  // The dwell after the window bursts on its member (every dwell is on one).
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
  int windows = 0;                          // the step's one op window is 144's
  for (auto& p : g.phases) windows += p.first == "BW" && p.second == 300;
  CHECK(windows == 1);
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
  CHECK(g.r.calls.back() == "retune 136");
}
// set_measure: a search-only (pinned) scout dwells on the members without
// reading; measuring, it observes the same members on a fresh ranker; off
// again, no reads and the ranks are dropped.
TEST(set_measure_switches_the_observe_and_rebuilds_the_ranker) {
  Rig g(two(false)); g.s.set_op(136); g.s.set_search(true);
  for (int i = 0; i < 4; ++i) g.s.run_once();
  for (auto& c : g.r.calls) CHECK(c.rfind("read", 0) != 0);
  g.s.set_measure(true);
  g.s.set_measure(true);                   // unchanged: a no-op
  g.r.calls.clear();
  for (int i = 0; i < 4; ++i) g.s.run_once();
  bool read = false;
  for (auto& c : g.r.calls) {
    CHECK(c != "retune 132" && c != "retune 140");
    if (c.rfind("read", 0) == 0) read = true;
  }
  CHECK(read);
  CHECK(g.s.mature());                     // the members rank on the rebuilt ranker
  g.s.set_measure(false);
  CHECK(!g.s.mature());                    // a fresh ranker, nothing ranked yet
  g.r.calls.clear();
  for (int i = 0; i < 4; ++i) g.s.run_once();
  for (auto& c : g.r.calls) {
    CHECK(c != "retune 132" && c != "retune 140");
    CHECK(c.rfind("read", 0) != 0);
  }
}

// unfreeze: the pick opens again over a fresh ranker, and measuring resumes.
TEST(unfreeze_reopens_the_pick_on_a_fresh_ranking) {
  Rig g(two()); g.s.set_op(136); g.s.set_search(true);
  for (int i = 0; i < 8; ++i) g.s.run_once();
  REQUIRE(g.s.mature());
  g.s.freeze();
  CHECK(!g.s.pick_open());
  g.s.unfreeze();
  CHECK(g.s.pick_open());
  CHECK(!g.s.mature());                    // ranks from before the freeze are gone
  for (auto& e : g.s.ranking()) CHECK(e.visits == 0);
  for (int i = 0; i < 8; ++i) g.s.run_once();
  CHECK(g.s.mature());
}

// A one-card scout that booted search-only (pinned), measuring again and
// reopened: the silent prelude runs once more for a full one_card_ms from
// the reopen (not from the scout's first step), then waits for the ack.
TEST(one_card_unfreeze_reruns_the_prelude_from_the_reopen) {
  ScoutCfg c = two(false); c.one_card = true; c.one_card_ms = 1000; Rig g(c);
  g.s.set_op(136); g.s.set_search(true);
  for (int i = 0; i < 10; ++i) g.s.run_once();   // op windows: well past one_card_ms
  REQUIRE(g.r.now > 2000);
  g.s.freeze();
  g.s.set_measure(true);
  g.s.unfreeze();
  CHECK(!g.s.prelude_done());
  const int64_t t_reopen = g.r.now;
  g.phases.clear();
  for (int i = 0; i < 200 && g.r.now < t_reopen + 1000; ++i) g.s.run_once();   // bounded: a stalled scout stops the clock
  REQUIRE(!g.phases.empty());
  for (auto& p : g.phases) CHECK(p.first.find('B') == std::string::npos);   // silent
  CHECK(!g.s.prelude_done());
  g.s.run_once();
  CHECK(g.s.prelude_done());
  CHECK(g.s.mature());                      // the fresh ranking, 1 visit per pair
  g.phases.clear();
  CHECK(!g.s.run_once());                   // no op window before the core's ack
  for (auto& p : g.phases) CHECK(p.first.find('B') == std::string::npos);
}

MTEST_MAIN
