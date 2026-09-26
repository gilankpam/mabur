#include <cmath>
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
static InflightScoutCfg cfg() { InflightScoutCfg c; c.candidates = {120, 149, 165}; c.home = 136; return c; }

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
  CHECK(!v.busy_valid);
}
TEST(dwell_arms_and_reads_busy_for_the_observe_span) {
  struct Nhm : FakeRadio {
    uint16_t armed = 0;
    bool arm_nhm_busy(uint16_t p) override { log.push_back("arm"); armed = p; return true; }
    NhmBusyRead read_nhm_busy() override {
      log.push_back("nhm"); NhmBusyRead r; r.valid = true; r.period = armed;
      r.buckets[0] = 55; r.buckets[11] = 200; return r;
    }
  } r;
  int64_t t = 0;
  InflightScout s(cfg(), r, [&] { return t; }, [&](int ms) { t += ms * 1000; });
  ScoutDwell d; HopVisit v;
  REQUIRE(s.dwell(149, 136, d, v));
  // retune, discard read, ARM, observe, NHM read, FA read, retune back
  CHECK((r.log == std::vector<std::string>{"retune 149", "read_scout", "arm", "nhm", "read_scout", "retune 136"}));
  CHECK(v.busy_valid && std::fabs(v.busy_pct - 100.0 * 200 / 255) < 1e-9);
  CHECK(d.busy_valid);
}
TEST(round_robin_and_burst) {
  FakeRadio r; int64_t t = 0;
  InflightScout s(cfg(), r, [&] { return t; }, [&](int ms) { t += ms * 1000; r.foreign += 2; });
  // Sitting on home: the rotation is the three candidates, home skipped.
  CHECK(s.next_candidate(136) == 120 && s.next_candidate(136) == 149 && s.next_candidate(136) == 165 &&
        s.next_candidate(136) == 120);
  std::vector<ScoutDwell> recs;
  auto visits = s.burst(136, recs);
  CHECK(visits.size() == 3 && recs.size() == 3 && r.ch == 136);
}
// The in-flight ranker is the scout's only source of visits (boot visits
// never reach it), so a channel the rotation never dwells on can never be
// ranked. Home used to be exactly that: after a hop off home it was
// reachable only as the blind "nothing ranked" fallback. Revert = the
// rotation over cfg.candidates alone (home never returned).
TEST(rotation_visits_home_once_off_it) {
  FakeRadio r; int64_t t = 0;
  InflightScout s(cfg(), r, [&] { return t; }, [&](int) {});
  std::vector<uint8_t> got;
  for (int i = 0; i < 8; ++i) got.push_back(*s.next_candidate(149));
  // On 149: home appended after the candidates, 149 itself skipped.
  CHECK((got == std::vector<uint8_t>{120, 165, 136, 120, 165, 136, 120, 165}));
}
// A dwell on the channel the card already sits on is a no-op retune whose
// visit best() then excludes -- a wasted slot. Revert = no skip.
TEST(rotation_never_returns_the_skipped_channel) {
  FakeRadio r; int64_t t = 0;
  InflightScout s(cfg(), r, [&] { return t; }, [&](int) {});
  for (uint8_t skip : {uint8_t{120}, uint8_t{149}, uint8_t{165}, uint8_t{136}})
    for (int i = 0; i < 9; ++i) CHECK(*s.next_candidate(skip) != skip);
}
// Home listed among the candidates too: one entry, not two.
TEST(home_listed_as_candidate_is_not_doubled) {
  FakeRadio r; int64_t t = 0;
  InflightScoutCfg c; c.candidates = {144, 136, 64}; c.home = 136;
  InflightScout s(c, r, [&] { return t; }, [&](int) {});
  std::vector<uint8_t> got;
  for (int i = 0; i < 6; ++i) got.push_back(*s.next_candidate(64));
  CHECK((got == std::vector<uint8_t>{144, 136, 144, 136, 144, 136}));
}
// Nothing but the op channel to dwell on (no candidates: the config
// default) -> nullopt, not an index into an empty list. Revert = the old
// `% candidates.size()` (division by zero with no candidates).
TEST(nothing_to_dwell_on_is_nullopt) {
  FakeRadio r; int64_t t = 0;
  InflightScoutCfg c; c.home = 136;
  InflightScout s(c, r, [&] { return t; }, [&](int) {});
  CHECK(!s.next_candidate(136).has_value());
  CHECK(*s.next_candidate(144) == 136);   // off home, home is the one channel left
  std::vector<ScoutDwell> recs;
  CHECK(s.burst(136, recs).empty() && recs.empty() && r.log.empty());
}
// The freshness burst sweeps the same set: home included, `back` skipped.
TEST(burst_covers_home_and_skips_back) {
  FakeRadio r; int64_t t = 0;
  InflightScout s(cfg(), r, [&] { return t; }, [&](int ms) { t += ms * 1000; r.foreign += 2; });
  r.ch = 149;
  std::vector<ScoutDwell> recs;
  auto visits = s.burst(149, recs);
  REQUIRE(visits.size() == 3 && recs.size() == 3);
  CHECK(visits[0].ch == 120 && visits[1].ch == 165 && visits[2].ch == 136);
  CHECK(r.ch == 149);
}
// End to end with the ranker: after a hop to 149, the rotation alone gets
// home ranked, and a clean home beats dirtier candidates in best().
TEST(home_becomes_a_ranked_hop_target_after_a_hop) {
  struct Busy : FakeRadio {
    ScoutEnergy read_energy_scout() override {
      ScoutEnergy e; e.fa_valid = true; e.fa_ofdm = e.cca_ofdm = ch == 136 ? 1 : 50; return e;
    }
  } r;
  r.ch = 149;
  int64_t t = 0;
  InflightScout s(cfg(), r, [&] { return t; }, [&](int ms) { t += ms * 1000; });
  HopCfg hc;
  HopRanker ranker(hc, {120, 149, 165}, 136, 0);
  for (int i = 0; i < 6; ++i) {
    ScoutDwell d; HopVisit v;
    if (s.dwell(*s.next_candidate(149), 149, d, v)) ranker.add(v);
  }
  auto best = ranker.best(t / 1000.0, 149, {});
  REQUIRE(best.has_value());
  CHECK(*best == 136);
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
// Fix round 1 item 1: pin the CONTRACT stated on dwell()'s header docstring
// (inflight_scout.h) -- main.cpp's sideport dwell/score attribution (Task
// 12) depends on it and has no test of its own that would catch a
// violation, since dwell_stats/scout_loop live in main.cpp and aren't
// unit-testable without refactoring it. Asserted here instead, at the
// level where the contract actually lives: false <=> flagged + no visit;
// true <=> visit populated with visit.ch == the dwelled channel.
TEST(dwell_return_value_flag_and_visit_population_stay_in_lockstep) {
  // Failure path 1: the retune TO the candidate fails.
  {
    struct Bad : FakeRadio {
      bool retune(uint8_t c) override {
        log.push_back("retune " + std::to_string(c));
        if (c == 149) return false;
        ch = c;
        return true;
      }
    } r;
    int64_t t = 0;
    InflightScout s(cfg(), r, [&] { return t; }, [&](int ms) { t += ms * 1000; });
    ScoutDwell d; HopVisit v;  // v default-constructed: ch == 0
    CHECK(!s.dwell(149, 136, d, v));
    CHECK(d.survey.flags & devourer::chanmig::kFlagRetuneFailed);
    CHECK(v.ch == 0);   // untouched -- dwell() never wrote to it
  }
  // Failure path 2: the retune BACK to `back` fails (candidate retune, the
  // observation, all succeed).
  {
    struct SecondFails : FakeRadio {
      int retunes = 0;
      bool retune(uint8_t c) override {
        log.push_back("retune " + std::to_string(c));
        ++retunes;
        if (retunes == 2) return false;
        ch = c;
        return true;
      }
    } r;
    int64_t t = 0;
    InflightScout s(cfg(), r, [&] { return t; }, [&](int ms) { t += ms * 1000; });
    ScoutDwell d; HopVisit v;
    CHECK(!s.dwell(149, 136, d, v));
    CHECK(d.survey.flags & devourer::chanmig::kFlagRetuneFailed);
    CHECK(v.ch == 0);
  }
  // Success path: both retunes succeed.
  {
    FakeRadio r;
    int64_t t = 0;
    InflightScout s(cfg(), r, [&] { return t; }, [&](int ms) { t += ms * 1000; r.foreign += 2; });
    ScoutDwell d; HopVisit v;
    CHECK(s.dwell(149, 136, d, v));
    CHECK(!(d.survey.flags & devourer::chanmig::kFlagRetuneFailed));
    CHECK(v.ch == 149);   // populated, matches the dwelled channel
  }
}
TEST(dwell_record_carries_the_tuned_width_and_pair_offset) {
  FakeRadio r;
  int64_t t = 0;
  InflightScout s(InflightScoutCfg{5, 333, {144}, 40}, r,
                  [&] { return t; }, [&](int ms) { t += ms * 1000; });
  ScoutDwell d; HopVisit v;
  CHECK(s.dwell(144, 136, d, v));
  CHECK(d.survey.def.width == CHANNEL_WIDTH_40);
  CHECK(d.survey.def.offset == 2);   // 140+144: primary is the upper half
  InflightScout s20(InflightScoutCfg{5, 333, {149}, 20}, r,
                    [&] { return t; }, [&](int ms) { t += ms * 1000; });
  ScoutDwell d20; HopVisit v20;
  CHECK(s20.dwell(149, 136, d20, v20));
  CHECK(d20.survey.def.width == CHANNEL_WIDTH_20);
  CHECK(d20.survey.def.offset == 0);
}

MTEST_MAIN
