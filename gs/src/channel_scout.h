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
  bool one_card = false;
};

struct ScoutDwell {
  devourer::chanmig::SurveyDwell survey;
  bool floor_valid = false;
  int8_t floor_dbm = 0;
};

// Boot-time scout (spec 2026-09-13-auto-channel-select §4). Owns one card's
// control plane from construction until done(). Two cards: the scheduler
// walks candidates ∪ {home}. One card: each cycle is a home window (during
// which at_home() is true and the core may beacon) followed by one
// scheduler dwell over the candidates; home is measured by the window.
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
  uint64_t rounds() const { return rounds_.load(std::memory_order_acquire); }
  std::vector<RankEntry> ranking() const;
  std::vector<ScoutDwell> take_dwells();

 private:
  // Retune, settle, discard read, observe `observe_ms`, real read; records
  // the dwell + ranker sample. `home_window` = the one-card home window
  // (at_home() true for observe - beacon_period, then a quiet gap).
  bool dwell(uint8_t ch, int observe_ms, bool home_window, uint64_t round);
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
  std::atomic<bool> frozen_{false}, done_{false}, at_home_{false};
  std::atomic<uint8_t> target_{0};
  std::atomic<uint64_t> rounds_{0};
};

}  // namespace maburgs
