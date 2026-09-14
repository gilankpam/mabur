#include <string>
#include <vector>
#include "inflight_scout.h"
#include "mtest.h"
using namespace maburgs;

// Fake radio: own/foreign are CUMULATIVE hardware counters, exactly like the
// real chip's -- dwell() reads frames() twice (before and after the observe
// sleep) and takes the DELTA, same as ChannelScout::dwell does for
// dvr_frames. So the fake must actually advance its counters somewhere
// between those two reads, or every delta is trivially 0 no matter what the
// counter's absolute value is set to beforehand. Here the injected SleepFn
// (which stands in for the observe window itself) bumps `foreign` by 2 each
// time it's called -- modelling a neighbour's frames arriving while the
// dwell listens. That makes the delta -- what visit.foreign and
// HopRanker's 4x-weighted "foreign" score term actually consume -- a real,
// non-zero number instead of a fixture artifact.
struct FakeRadio : ScoutRadio {
  std::vector<std::string> log; uint8_t ch = 136; uint32_t fa_next = 0; uint64_t own = 0, foreign = 0;
  bool retune(uint8_t c) override { log.push_back("retune " + std::to_string(c)); ch = c; return true; }
  ScoutEnergy read_energy(bool) override { log.push_back("read_full"); return {}; }
  ScoutEnergy read_energy_scout() override { log.push_back("read_scout"); ScoutEnergy e; e.fa_valid = true; e.fa_ofdm = fa_next; e.cca_ofdm = fa_next; return e; }
  ScoutFrames frames() const override { return ScoutFrames{own, foreign}; }
};
static InflightScoutCfg cfg() { InflightScoutCfg c; c.candidates = {120, 149, 165}; return c; }

TEST(dwell_sequence_and_record) {
  FakeRadio r; int64_t t = 0;
  InflightScout s(cfg(), r, [&] { return t; }, [&](int ms) { t += ms * 1000; r.foreign += 2; });
  r.fa_next = 13;
  ScoutDwell d; HopVisit v;
  CHECK(s.dwell(149, 136, d, v));
  REQUIRE(r.log.size() == 4);
  CHECK(r.log[0] == "retune 149" && r.log[1] == "read_scout" && r.log[2] == "read_scout" && r.log[3] == "retune 136");
  CHECK(r.ch == 136);
  CHECK(d.in_session && d.survey.observe_ms == 5 && d.survey.fa_ofdm == 13);
  CHECK(v.ch == 149 && v.fa == 13 && v.foreign == 2);
}
TEST(round_robin_and_burst) {
  FakeRadio r; int64_t t = 0;
  InflightScout s(cfg(), r, [&] { return t; }, [&](int ms) { t += ms * 1000; r.foreign += 2; });
  CHECK(s.next_candidate() == 120 && s.next_candidate() == 149 && s.next_candidate() == 165 && s.next_candidate() == 120);
  std::vector<ScoutDwell> recs;
  auto visits = s.burst(136, recs);
  CHECK(visits.size() == 3 && recs.size() == 3 && r.ch == 136);
}
TEST(failed_retune_returns_false_and_leaves_card_on_back) {
  struct Bad : FakeRadio { bool retune(uint8_t c) override { log.push_back("retune"); return c != 149; } } r;
  int64_t t = 0; InflightScout s(cfg(), r, [&] { return t; }, [&](int ms) { t += ms * 1000; r.foreign += 2; });
  ScoutDwell d; HopVisit v;
  CHECK(!s.dwell(149, 136, d, v));
  CHECK(d.survey.flags & devourer::chanmig::kFlagRetuneFailed);
}
MTEST_MAIN
