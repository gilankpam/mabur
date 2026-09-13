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
  // retune -> settle -> discard read (no nhm) -> observe -> real read (nhm)
  REQUIRE(r.calls.size() == 3);
  CHECK(r.calls[0] == "retune 136");        // scheduler visits plan order: home first
  CHECK(r.calls[1] == "read");
  CHECK(r.calls[2] == "read+nhm");
  CHECK(r.now == 280);                      // settle 30 + dwell 250
  auto d = s.take_dwells();
  REQUIRE(d.size() == 1);
  CHECK(d[0].survey.def.primary == 136);
  CHECK(d[0].survey.observe_ms == 250);
  CHECK(d[0].floor_valid && d[0].floor_dbm == -95);
  CHECK(s.take_dwells().empty());
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

TEST(one_card_interleaves_home_window_and_dwell_with_quiet_gap) {
  FakeRadio r;
  bool saw_home_true = false;
  ChannelScout s(cfg2(true), r, [&] { return r.now; }, [&](int ms) {
    if (s.at_home()) saw_home_true = true;
    r.now += ms;
  });
  CHECK(!s.at_home());
  CHECK(s.run_once());
  // home window first: retune home, settle, discard, at_home for
  // (home_window - beacon_period), quiet beacon_period, read; then one
  // candidate dwell.
  REQUIRE(r.calls.size() == 6);
  CHECK(r.calls[0] == "retune 136");
  CHECK(r.calls[1] == "read");
  CHECK(r.calls[2] == "read+nhm");
  CHECK(r.calls[3] == "retune 149");
  CHECK(r.calls[4] == "read");
  CHECK(r.calls[5] == "read+nhm");
  CHECK(saw_home_true);
  CHECK(!s.at_home());
  CHECK(r.now == 30 + 300 + 30 + 250);
  auto d = s.take_dwells();
  REQUIRE(d.size() == 2);
  CHECK(d[0].survey.def.primary == 136);
  CHECK(d[0].survey.observe_ms == 300);
  CHECK(d[1].survey.def.primary == 149);
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
