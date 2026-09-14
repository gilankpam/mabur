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
  // Pre-seed foreign to a NONZERO value before dwell() runs, so f0 != 0 and
  // an absolute-count implementation (foreign_delta = f1.foreign, dropping
  // the f0 subtraction) genuinely diverges from the correct delta -- with
  // foreign starting at 0 (the field's default), the two are numerically
  // identical and this assertion can't tell them apart. Round 1 fix: a
  // mutation to the absolute form was confirmed to still pass 3/3 before
  // this seed was added.
  r.foreign = 100;
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
  // Bad tracks `ch` the way the base FakeRadio does -- set on a successful
  // retune, left untouched on a failed one -- so this test can actually
  // check the safety invariant it's named for: the card lands on `back`,
  // not stranded on the candidate. Without this the test only checked the
  // return value and the flag, and would still pass if the recovery
  // retune(back) call were deleted entirely.
  struct Bad : FakeRadio {
    bool retune(uint8_t c) override {
      log.push_back("retune " + std::to_string(c));
      if (c == 149) return false;   // the candidate retune fails; ch does NOT move
      ch = c;
      return true;
    }
  } r;
  // FakeRadio::ch defaults to 136, which is also this test's `back` -- if
  // left at that default, a CHECK(r.ch == 136) below would pass trivially
  // even with the recovery retune(back) call deleted entirely. Seed it to
  // something else first so the assertion can actually tell "recovered
  // onto back" apart from "was never touched".
  r.ch = 0;
  int64_t t = 0; InflightScout s(cfg(), r, [&] { return t; }, [&](int ms) { t += ms * 1000; r.foreign += 2; });
  ScoutDwell d; HopVisit v;
  CHECK(!s.dwell(149, 136, d, v));
  CHECK(d.survey.flags & devourer::chanmig::kFlagRetuneFailed);
  CHECK(r.ch == 136);   // recovered onto `back`, never left parked on the candidate
}
// Round 1 fix #3: the return-to-`back` retune's own result must be checked
// too -- a stranded scout card (parked on the candidate, mistaken for
// having returned) is exactly the failure this module exists to prevent,
// since live video is on the OTHER card. A fake whose SECOND retune call
// (the return-to-back one; the first, to the candidate, succeeds) fails,
// and -- matching RadioFrontend::retune's real contract, where a failed
// call never reaches FastRetune -- leaves `ch` wherever the last
// successful retune put it, i.e. still on the candidate.
TEST(failed_return_retune_flags_and_refuses_the_visit) {
  struct SecondFails : FakeRadio {
    int retunes = 0;
    bool retune(uint8_t c) override {
      log.push_back("retune " + std::to_string(c));
      ++retunes;
      if (retunes == 2) return false;   // return-to-back retune fails; ch does NOT move
      ch = c;
      return true;
    }
  } r;
  int64_t t = 0;
  InflightScout s(cfg(), r, [&] { return t; }, [&](int ms) { t += ms * 1000; r.foreign += 2; });
  r.fa_next = 13;
  ScoutDwell d; HopVisit v;
  CHECK(!s.dwell(149, 136, d, v));
  CHECK(d.survey.flags & devourer::chanmig::kFlagRetuneFailed);
  // The observation itself still happened and is still recorded --
  // it's the visit (and the bogus confidence that the card came home)
  // that must not be booked.
  CHECK(d.survey.fa_ofdm == 13);
  CHECK(r.ch == 149);   // still parked on the candidate, not `back`
}
// Round 1 fix #2: main.cpp (Task 11) picks the dwell card per cycle and
// must be able to repoint an already-constructed InflightScout at a
// different card's control plane -- a reference can't be reseated, hence
// the pointer member + set_radio() setter. Confirms the NEXT dwell after a
// set_radio() call drives the new radio, not the one passed to the
// constructor.
TEST(set_radio_repoints_the_next_dwell) {
  FakeRadio r1, r2; int64_t t = 0;
  r1.ch = 200; r2.ch = 210;   // distinct starting channels so a mix-up is visible
  InflightScout s(cfg(), r1, [&] { return t; }, [&](int ms) { t += ms * 1000; });
  s.set_radio(r2);
  ScoutDwell d; HopVisit v;
  CHECK(s.dwell(149, 136, d, v));
  CHECK(r1.log.empty());             // r1 was never touched after the reseat
  REQUIRE(r2.log.size() == 4);
  CHECK(r2.log[0] == "retune 149" && r2.log[3] == "retune 136");
  CHECK(r2.ch == 136);
}
MTEST_MAIN
