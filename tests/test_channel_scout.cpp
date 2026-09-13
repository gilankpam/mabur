#include <string>
#include <vector>
#include "mtest.h"
#include "channel_scout.h"
using namespace maburgs;

// Fake radio + fake clock: sleep() advances time, so a dwell "takes" exactly
// settle+dwell ms and every call is recorded in order.
struct FakeRadio : ScoutRadio {
  int64_t now = 0;
  std::vector<std::string> calls;
  uint8_t ch = 0;
  uint32_t cca_per_ms_on[256] = {};  // busy rate per channel
  ScoutFrames fr;
  int64_t last_read = 0;
  bool retune(uint8_t c) override { ch = c; calls.push_back("retune " + std::to_string(c)); return true; }
  ScoutEnergy read_energy(bool with_nhm) override {
    calls.push_back(std::string(with_nhm ? "read+nhm" : "read"));
    ScoutEnergy e; e.fa_valid = true;
    e.cca_ofdm = static_cast<uint32_t>((now - last_read) * cca_per_ms_on[ch]);
    last_read = now;
    if (with_nhm) { e.floor_valid = true; e.floor_dbm = -95; }
    return e;
  }
  ScoutFrames frames() const override { return fr; }
};

static ScoutCfg cfg2(bool one_card = false) {
  ScoutCfg c; c.home = 136; c.candidates = {149, 161}; c.dwell_ms = 250; c.settle_ms = 30;
  c.min_rounds = 2; c.home_window_ms = 300; c.beacon_period_ms = 20; c.one_card = one_card;
  return c;
}

TEST(two_card_dwell_sequence_and_discard_read) {
  FakeRadio r;
  ChannelScout s(cfg2(), r, [&] { return r.now; }, [&](int ms) { r.now += ms; });
  CHECK(s.run_once());
  // First dwell of a round: beacon window (quiet false) -> gap -> retune
  // home (plan order) -> settle -> discard read -> silent observe -> read.
  REQUIRE(r.calls.size() == 3);
  CHECK(r.calls[0] == "retune 136");
  CHECK(r.calls[1] == "read");
  CHECK(r.calls[2] == "read+nhm");
  CHECK(r.now == 300 + 20 + 30 + 250);      // window + gap + settle + dwell
  CHECK(s.quiet());                         // dwells stay silent until the next window
  auto d = s.take_dwells();
  REQUIRE(d.size() == 1);
  CHECK(d[0].survey.def.primary == 136);
  CHECK(d[0].survey.observe_ms == 250);
  CHECK(d[0].floor_valid && d[0].floor_dbm == -95);
  CHECK(s.take_dwells().empty());
  // Later dwells of the round: no window.
  CHECK(s.run_once());
  CHECK(r.calls[3] == "retune 149");
  CHECK(r.now == 600 + 30 + 250);
}

TEST(two_card_beacon_window_opens_once_per_round) {
  FakeRadio r;
  std::vector<std::pair<int, bool>> sleeps;   // (ms, quiet at that sleep)
  ChannelScout s(cfg2(), r, [&] { return r.now; }, [&](int ms) {
    sleeps.push_back({ms, s.quiet()});
    r.now += ms;
  });
  for (int i = 0; i < 3; ++i) s.run_once();   // one full round: 136, 149, 161
  CHECK(s.rounds() == 1);
  // window(300, not quiet) gap(20, quiet) then 3 x [settle, observe] all quiet
  REQUIRE(sleeps.size() == 8);
  CHECK(sleeps[0].first == 300 && !sleeps[0].second);
  CHECK(sleeps[1].first == 20 && sleeps[1].second);
  for (size_t i = 2; i < 8; ++i) CHECK(sleeps[i].second);
  sleeps.clear();
  s.run_once();                               // next round opens a new window
  REQUIRE(sleeps.size() == 4);
  CHECK(sleeps[0].first == 300 && !sleeps[0].second);
  s.freeze(149);
  s.run();
  CHECK(!s.quiet());                          // released for the core once done
}

TEST(round_covers_home_and_candidates_then_proposes_best) {
  FakeRadio r;
  r.cca_per_ms_on[136] = 2; r.cca_per_ms_on[149] = 0; r.cca_per_ms_on[161] = 40;
  ChannelScout s(cfg2(), r, [&] { return r.now; }, [&](int ms) { r.now += ms; });
  for (int i = 0; i < 3; ++i) s.run_once();
  CHECK(s.rounds() == 1);
  CHECK(s.proposal() == 136);               // min_rounds 2 not met -> home
  for (int i = 0; i < 3; ++i) s.run_once();
  CHECK(s.rounds() == 2);
  CHECK(s.proposal() == 149);
  auto k = s.ranking();
  REQUIRE(k.size() == 3);
  CHECK(k[0].ch == 136 && k[1].ch == 149 && k[2].ch == 161);
  CHECK(k[2].worst_busy == 40u * 250u);
}

TEST(freeze_stops_run_and_retunes_to_target) {
  FakeRadio r;
  ChannelScout s(cfg2(), r, [&] { return r.now; }, [&](int ms) { r.now += ms; });
  s.freeze(149);
  CHECK(s.frozen());
  CHECK(!s.run_once());
  s.run();
  CHECK(s.done());
  CHECK(r.ch == 149);
  CHECK(s.proposal() == 149);               // frozen proposal is the target
}

TEST(one_card_home_is_beacon_phase_then_silent_measurement) {
  FakeRadio r;
  std::vector<std::pair<int, bool>> sleeps;   // (ms, at_home at that sleep)
  ChannelScout s(cfg2(true), r, [&] { return r.now; }, [&](int ms) {
    sleeps.push_back({ms, s.at_home()});
    r.now += ms;
  });
  CHECK(!s.at_home());
  CHECK(s.run_once());
  // home cycle: retune home, settle, BEACON phase (at_home), quiet gap,
  // discard read, silent observe of dwell_ms, read; then one candidate.
  REQUIRE(r.calls.size() == 6);
  CHECK(r.calls[0] == "retune 136");
  CHECK(r.calls[1] == "read");
  CHECK(r.calls[2] == "read+nhm");
  CHECK(r.calls[3] == "retune 149");
  CHECK(r.calls[4] == "read");
  CHECK(r.calls[5] == "read+nhm");
  REQUIRE(sleeps.size() == 6);
  CHECK(sleeps[0].first == 30 && !sleeps[0].second);   // settle
  CHECK(sleeps[1].first == 300 && sleeps[1].second);   // beacon phase
  CHECK(sleeps[2].first == 20 && !sleeps[2].second);   // quiet gap
  CHECK(sleeps[3].first == 250 && !sleeps[3].second);  // silent observe
  CHECK(sleeps[4].first == 30 && !sleeps[4].second);   // candidate settle
  CHECK(sleeps[5].first == 250 && !sleeps[5].second);  // candidate observe
  CHECK(!s.at_home());
  CHECK(!s.quiet());                                    // one card never uses quiet()
  CHECK(r.now == 30 + 300 + 20 + 250 + 30 + 250);
  auto d = s.take_dwells();
  REQUIRE(d.size() == 2);
  CHECK(d[0].survey.def.primary == 136);
  CHECK(d[0].survey.observe_ms == 250);                 // same window as a candidate
  CHECK(d[1].survey.def.primary == 149);
  CHECK(d[1].survey.observe_ms == 250);
}

TEST(one_card_home_counts_as_visits) {
  FakeRadio r;
  r.cca_per_ms_on[136] = 9;
  ChannelScout s(cfg2(true), r, [&] { return r.now; }, [&](int ms) { r.now += ms; });
  s.run_once(); s.run_once();   // 2 cycles = 2 home visits, 149 + 161 once each
  CHECK(s.proposal() == 136);   // only home has 2 visits (busy but the only ranked one)
  s.run_once(); s.run_once();
  CHECK(s.proposal() == 149);
}
MTEST_MAIN
