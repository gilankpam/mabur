#pragma once
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "channel_scout.h"
#include "hop_ranker.h"
#include "scout_radio.h"

namespace maburgs {

struct InflightScoutCfg {
  int observe_ms = 5;
  int period_ms = 333;
  std::vector<uint8_t> candidates;
  uint8_t width_mhz = 20;  // radio.width: the dwell keeps the card's tuning (FastRetune), recorded here
  uint8_t home = 0;        // radio.channel: dwelt on like a candidate whenever the link is off it
  int busy_dbm = -83;      // HopCfg::verdict.busy_dbm: NHM bucket edge for the dwell's busy-airtime read
};

// The ~10 ms mid-flight dwell (spec 2026-09-14-inflight-channel-hop §3): a
// single, timed visit to one candidate channel on the SPARE card, back to
// `back` (the card's normal channel) before returning, so the live video --
// which never left the other card -- sees no interruption. Every retune,
// discard read, sleep and real read is called through the injected
// ScoutRadio/clock/sleep, exactly like ChannelScout, so the whole sequence
// runs instantly and deterministically under test.
//
// A failed retune to the candidate MUST NOT strand the card there: the
// scout card is the link's only fallback diversity, so dwell() always
// ATTEMPTS to leave the card on `back` before returning, success or
// failure -- and checks that return retune's own result too. If it fails,
// the card's true position is no longer certain (live video is on the
// OTHER card, so a stranded scout card means the aircraft is down to one
// radio with no diversity); dwell() flags the record and returns false
// rather than booking a visit around an unknown card position. Recovery is
// NOT this module's job -- Task 11's mechanical retune loop (driven off
// `plan.desired(card)` every tick) is the real reconciliation path.
//
// Pure apart from the injected radio: no threads, no clock of its own.
// main.cpp (Task 11) owns the thread that calls dwell()/burst() on a
// period_ms cadence, and picks the dwell card per cycle via set_radio().
class InflightScout {
 public:
  using NowUsFn = std::function<int64_t()>;
  using SleepFn = std::function<void(int)>;

  InflightScout(InflightScoutCfg cfg, ScoutRadio& radio, NowUsFn now_us, SleepFn sleep_ms);

  // Repoints the scout at a different card's control plane -- the thread
  // picks the dwell card each cycle, and a reference can't be reseated.
  // Takes a reference (never null) so neither entry point can hand this a
  // dangling/absent radio.
  void set_radio(ScoutRadio& radio);

  // One dwell on `ch`, returning to `back`. Fills d.survey (fa/cca/frames/
  // observe_ms), d.in_session=true, and the three step timings regardless
  // of outcome. Also arms an NHM busy-airtime window for the observe span
  // and reads it back (spec 2026-09-25-nhm-airtime §6), filling
  // d.busy_valid/d.busy_pct and, on success, visit.busy_valid/busy_pct --
  // left at their false/0 defaults on radios without NHM support.
  //
  // CONTRACT (relied on by gs/src/main.cpp's sideport dwell attribution,
  // Task 12 -- HopVisit carries no card field, so main.cpp pairs each
  // drained ScoutDwell against dwell_visits purely off this return-value/
  // flag/visit relationship, in order, across the two vectors):
  //   - Returns false, with d.survey.flags carrying kFlagRetuneFailed and
  //     `visit` left UNPOPULATED (not to be read), if EITHER the retune to
  //     `ch` failed (the card is left on `back`) OR the observation
  //     completed but the return retune to `back` itself failed (the
  //     card's position is then unknown -- it may still be parked on
  //     `ch`). These are the only two failure paths; any future one MUST
  //     also set kFlagRetuneFailed and refuse to populate `visit`.
  //   - Returns true implies `visit` is fully populated, with
  //     visit.ch == ch, and d.survey.flags does NOT carry
  //     kFlagRetuneFailed.
  // Breaking either half of this pairing (return value disagreeing with
  // the flag, or with whether `visit` was actually filled) silently
  // corrupts main.cpp's per-card score/visit attribution without failing
  // any test at this call site -- see tests/test_inflight_scout.cpp's
  // dwell_return_value_flag_and_visit_population_stay_in_lockstep group.
  bool dwell(uint8_t ch, uint8_t back, ScoutDwell& d, HopVisit& visit);

  // Round-robin over the dwell set -- cfg_.candidates plus home, in
  // HopRanker's order (config order, home appended when not listed) --
  // skipping `skip`, the channel the card already sits on (a dwell there is
  // a no-op retune whose visit HopRanker::best() excludes anyway). Home is
  // in the set because these dwells are HopRanker's only source of visits:
  // without them home is never ranked in flight and a hop off it is one-way
  // (reachable only as the blind "nothing ranked" fallback). nullopt when
  // `skip` is the only channel in the set.
  std::optional<uint8_t> next_candidate(uint8_t skip);

  // Every channel of the dwell set except `back` once, back to back, each
  // returning to `back` in between. Appends one ScoutDwell per dwell to
  // `records` (in set order) and returns the matching HopVisits (fewer
  // than records.size() if any dwell failed).
  std::vector<HopVisit> burst(uint8_t back, std::vector<ScoutDwell>& records);

 private:
  InflightScoutCfg cfg_;
  std::vector<uint8_t> set_;   // cfg_.candidates, home appended if not listed
  ScoutRadio* radio_;
  NowUsFn now_us_;
  SleepFn sleep_ms_;
  size_t next_idx_ = 0;
  uint64_t seq_ = 0;
};

}  // namespace maburgs
