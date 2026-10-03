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
  std::vector<uint8_t> channels;   // the set, config order (members are primaries at width 40)
  bool measure = true;             // auto: observe + rank; false (pinned): search only
  int dwell_ms = 250;
  int settle_ms = 30;
  int min_rounds = 3;
  int search_ms = 100;             // DISC burst on a member during a search dwell
  int op_window_ms = 300;          // one card: beacon window on op between dwells
  int beacon_period_ms = 20;
  int one_card_ms = 5000;          // one card + measure: silent prelude before the first DISC
  uint32_t pick_margin = 20;
  bool one_card = false;
  uint8_t link_width_mhz = 20;     // radio.width: 40 = dwell every half at 20, pick a pair, park at 40
  int busy_dbm = -83;
  double blocked_pct = 1e9;
  double leak_per_frame = 1.0;     // RankSample::leak per TX-card frame sent during an observe
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

// The GS's long-lived search + measure scheduler. Owns the spare card
// whenever it has work: has_work = search || (measure && !frozen).
//
//  - search (link down): sweep the set, bursting DISC (beaconing() true for
//    search_ms) on every member dwell so the drone can be FOUND wherever
//    it is. Bursts happen only when search && the dwell channel is a member.
//  - measure (auto mode, pick open): observe every dwell channel (both 20
//    MHz halves of each member's pair at width 40) and rank it. An observe
//    while unlinked is quiet(): the TX card holds its DISC (bench
//    2026-09-13: a beaconing card's TX leaks into the adjacent scout).
//    Linked, the TX card carries video and is never held; its leak is
//    subtracted instead: RankSample::leak = round(leak_per_frame x frames
//    the TX card sent during the observe), fed in via set_tx_frames().
//  - no work (linked and pinned, or frozen): park on op at link_width_mhz
//    (once), working() false. freeze() closes the pick for good.
//
// One card: with measure on, a silent prelude of one_card_ms (observe-only
// dwells, no DISC anywhere) runs first and never re-arms; afterwards every
// step is an op window (beaconing() on op for op_window_ms) then one dwell.
// Time comes from the injected clock so tests run instantly. Every input
// and output is an atomic; ranker_/dwells_ are under mu_.
class ChannelScout {
 public:
  using NowFn = std::function<int64_t()>;
  using SleepFn = std::function<void(int)>;
  ChannelScout(ScoutCfg cfg, ScoutRadio& radio, NowFn now_ms, SleepFn sleep_ms);

  void run();       // thread body: loops run_once() until stop(); parks the card on exit
  bool run_once();  // one scheduler step; false = nothing to do this step (idle, parked)

  // core -> scout (any thread)
  void set_op(uint8_t ch);
  void set_search(bool on) { search_.store(on, std::memory_order_release); }
  void set_tx_frames(uint64_t cumulative) { tx_frames_.store(cumulative, std::memory_order_release); }
  void freeze() { frozen_.store(true, std::memory_order_release); }  // pick closed: measuring stops
  void stop() { stop_.store(true, std::memory_order_release); }

  // scout -> core
  bool working() const { return working_.load(std::memory_order_acquire); }
  bool beaconing() const { return beaconing_.load(std::memory_order_acquire); }
  bool quiet() const { return quiet_.load(std::memory_order_acquire); }
  bool pick_open() const { return !frozen_.load(std::memory_order_acquire); }
  bool prelude_done() const { return prelude_done_.load(std::memory_order_acquire); }
  bool mature() const;
  uint8_t proposal() const { return proposal_.load(std::memory_order_acquire); }
  std::vector<uint8_t> pick_ranking() const;
  uint64_t rounds() const { return rounds_.load(std::memory_order_acquire); }
  std::vector<RankEntry> ranking() const;
  std::vector<ScoutDwell> take_dwells();
  bool done() const { return done_.load(std::memory_order_acquire); }

 private:
  bool has_work_() const;
  bool measuring_() const;
  bool member_(uint8_t ch) const;
  int eff_min_rounds_() const;
  void park_();
  bool tune_(uint8_t ch);
  bool step_dwell_(bool burst_ok, bool observe);
  // Retune, settle, [burst + gap,] [discard read, observe dwell_ms, read];
  // records the dwell (+ the sample when observed).
  bool dwell(uint8_t ch, uint64_t round, bool burst, bool observe);
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
  std::atomic<uint8_t> op_;
  std::atomic<uint64_t> tx_frames_{0};
  std::atomic<bool> search_{false}, frozen_{false}, stop_{false};
  std::atomic<bool> working_{false}, beaconing_{false}, quiet_{false}, prelude_done_{true},
      done_{false};
  std::atomic<uint64_t> rounds_{0};
  bool need_width_ = true;   // run thread only: the next tune switches the card to 20 MHz
  int64_t t0_ = -1;          // run thread only: first run_once() (one-card prelude start)
};

}  // namespace maburgs
