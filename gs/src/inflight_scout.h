#pragma once
#include <cstdint>
#include <functional>
#include <vector>

#include "channel_scout.h"
#include "hop_ranker.h"
#include "scout_radio.h"

namespace maburgs {

struct InflightScoutCfg {
  int observe_ms = 5;
  int period_ms = 333;
  std::vector<uint8_t> candidates;
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
// leaves the card on `back` before returning, success or failure.
//
// Pure apart from the injected radio: no threads, no clock of its own.
// main.cpp (Task 11) owns the thread that calls dwell()/burst() on a
// period_ms cadence.
class InflightScout {
 public:
  using NowUsFn = std::function<int64_t()>;
  using SleepFn = std::function<void(int)>;

  InflightScout(InflightScoutCfg cfg, ScoutRadio& radio, NowUsFn now_us, SleepFn sleep_ms);

  // One dwell on `ch`, returning to `back`. Fills d.survey (fa/cca/frames/
  // observe_ms), d.in_session=true, the three step timings, and a HopVisit.
  // Returns false if the retune to `ch` failed -- the card is left on
  // `back` in that case too.
  bool dwell(uint8_t ch, uint8_t back, ScoutDwell& d, HopVisit& visit);

  // Round-robin over cfg_.candidates.
  uint8_t next_candidate();

  // Every candidate once, back to back, each returning to `back` in
  // between. Appends one ScoutDwell per candidate to `records` (in
  // candidate order) and returns the matching HopVisits.
  std::vector<HopVisit> burst(uint8_t back, std::vector<ScoutDwell>& records);

 private:
  InflightScoutCfg cfg_;
  ScoutRadio& radio_;
  NowUsFn now_us_;
  SleepFn sleep_ms_;
  size_t next_idx_ = 0;
  uint64_t seq_ = 0;
};

}  // namespace maburgs
