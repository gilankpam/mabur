#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

#include "chanmig/ScanPlan.h"
#include "chanmig/SurveyRecord.h"
#include "channel_ranker.h"
#include "scout_radio.h"

namespace maburgs {

struct ScoutCfg {
  uint8_t home = 149;
  std::vector<uint8_t> candidates;
  int dwell_ms = 250;
  int settle_ms = 30;
  int min_rounds = 3;
  int home_window_ms = 300;
  int beacon_period_ms = 20;
  uint32_t home_margin = 0;
  bool one_card = false;
  uint8_t link_width_mhz = 20;  // radio.width: 40 = dwell every half at 20, pick a pair, park at 40
};

struct ScoutDwell {
  devourer::chanmig::SurveyDwell survey;
  bool floor_valid = false;
  int8_t floor_dbm = 0;
  // In-session (Task 10) provenance: whether this dwell ran mid-flight, and
  // the step timings in microseconds. Boot-time dwells leave these at their
  // defaults (0/0/0/0) so scan.log's D record stays valid without a hop.
  bool in_session = false;
  int64_t to_us = 0, read_us = 0, back_us = 0;
  bool busy_valid = false;
  double busy_pct = 0;  // NHM busy % over the observe span (scanlog 4 D)
};

// Boot-time scout (spec 2026-09-13-auto-channel-select §4). Owns one card's
// control plane from construction until done(). Two cards: the scheduler
// walks candidates ∪ {home}. One card: each cycle is a home window (during
// which at_home() is true and the core may beacon) followed by one
// scheduler dwell over the candidates.
//
// Every dwell is measured IN SILENCE (bench 2026-09-13: the beaconing
// card's TX leaks into the adjacent scout on every channel -- decoded and
// subtracted as own frames on home, counted as undecodable energy on a
// candidate -- and a single card's TX leaks into its own receiver). Two
// cards: quiet() is true during every dwell; once per scheduler round the
// scout opens a beacon window of home_window_ms (quiet() false) so the
// drone can still be discovered mid-scan. One card: each cycle is a beacon
// phase (at_home()), a quiet gap, a silent observe of dwell_ms on home with
// its own discard read, then one silent candidate dwell.
// Time comes from the injected clock so tests run instantly.
class ChannelScout {
 public:
  using NowFn = std::function<int64_t()>;
  using SleepFn = std::function<void(int)>;
  ChannelScout(ScoutCfg cfg, ScoutRadio& radio, NowFn now_ms, SleepFn sleep_ms);

  void run();
  bool run_once();
  uint8_t proposal() const { return proposal_.load(std::memory_order_acquire); }
  void freeze(uint8_t target);
  bool frozen() const { return frozen_.load(std::memory_order_acquire); }
  bool done() const { return done_.load(std::memory_order_acquire); }
  bool at_home() const { return at_home_.load(std::memory_order_acquire); }
  // Two cards: a silent dwell is in progress, the core must not beacon.
  bool quiet() const { return quiet_.load(std::memory_order_acquire); }
  uint64_t rounds() const { return rounds_.load(std::memory_order_acquire); }
  std::vector<RankEntry> ranking() const;
  std::vector<ScoutDwell> take_dwells();

 private:
  enum class Kind { Silent, HomeOneCard };
  // Retune, settle, [one-card home: beacon phase + gap,] discard read,
  // silent observe of dwell_ms, real read; records the dwell + sample.
  bool dwell(uint8_t ch, Kind kind, uint64_t round);
  void publish_();

  ScoutCfg cfg_;
  ScoutRadio& radio_;
  NowFn now_;
  SleepFn sleep_;
  devourer::chanmig::ScanScheduler sched_;
  mutable std::mutex mu_;          // ranker_, dwells_
  ChannelRanker ranker_;
  std::vector<ScoutDwell> dwells_;
  uint64_t seq_ = 0;
  std::atomic<uint8_t> proposal_;
  std::atomic<bool> frozen_{false}, done_{false}, at_home_{false}, quiet_{false};
  std::atomic<uint8_t> target_{0};
  std::atomic<uint64_t> rounds_{0};
  bool beacon_due_ = true;   // two cards: open a beacon window before the next dwell
};

}  // namespace maburgs
