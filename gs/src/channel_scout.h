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
  bool measure = true;             // auto: observe + rank; false (pinned): search only. Boot value: set_measure() switches it live
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
  // In-session provenance: whether this dwell ran mid-flight, and
  // the step timings in microseconds. Boot-time dwells leave these at their
  // defaults (0/0/0/0) so scan.log's D record stays valid without a hop.
  bool in_session = false;
  int64_t to_us = 0, read_us = 0, back_us = 0;
  bool busy_valid = false;
  double busy_pct = 0;  // NHM busy % over the observe span (scanlog 4 D)
  bool rx_valid = false;
  double rx_pct = 0;  // relay sweep: rx % of the observe (scanlog 6 D)
};

// The GS's long-lived search + measure scheduler. Owns the spare card
// whenever it has work: has_work = search || (measure && !frozen).
//
//  - search (link down): sweep the set, bursting DISC (beaconing() true for
//    search_ms) on every member dwell so the drone can be FOUND wherever
//    it is. Bursts happen only when search && the dwell channel is a member.
//  - measure (auto mode, pick open): observe every dwell channel (both 20
//    MHz halves of each member's pair at width 40) and rank it. An observe
//    while unlinked is quiet(): the TX card holds its DISC (bench: a
//    beaconing card's TX leaks into the adjacent scout).
//    Linked, the TX card carries video and is never held; its leak is
//    subtracted instead: RankSample::leak = round(leak_per_frame x frames
//    the TX card sent during the observe), fed in via set_tx_frames().
//    With the search off (linked), op's own halves are skipped: they carry
//    the drone's video.
//  - no work (linked and pinned, or frozen): park on op at link_width_mhz
//    (once), working() false. freeze() closes the pick for good.
//  - one round (a calibration run's channel pick, after the freeze):
//    begin_round() measures again exactly once -- a fresh ranker, every
//    dwell channel visited once except op's own halves (skipped as above
//    while the search is off), then no work again and it parks. The core
//    takes the round's samples (round_samples) and makes the pick.
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
  // The pick open again (the GS went back to auto with no link): a fresh
  // ranker, so ranks taken before the freeze -- possibly long ago -- never
  // decide the new pick.
  void unfreeze();
  // Measuring on or off (auto / pinned). The dwell set follows it at width
  // 40 (both halves of every pair vs the members alone), so the ranker is
  // rebuilt here and the scheduler by the run thread on its next step (it
  // is the scheduler's only user). Unchanged: a no-op.
  void set_measure(bool on);
  void stop() { stop_.store(true, std::memory_order_release); }
  // One card + measure: the core has committed the prelude pick (op = the
  // committed channel); the first op window may run. Until then, with the
  // pick still open, the scout idles after prelude_done() (no DISC window).
  void ack_prelude(uint8_t op);
  // One measure round now, the pick already frozen and the scout parked
  // (the core checks both). Resets the ranker: the round ranks only what
  // it measures. round_open() reads true from this call until the scout
  // has parked again, so the core never touches the card in between.
  void begin_round();
  // Ends a round early (the run gave up on it, or it overran its bound): no
  // more measuring; the run thread parks the card on its next step.
  // force_idle: the run thread is not running (it never started), so
  // nothing will park -- release the card at once.
  void cancel_round(bool force_idle);

  // scout -> core
  bool working() const { return working_.load(std::memory_order_acquire); }
  bool beaconing() const { return beaconing_.load(std::memory_order_acquire); }
  bool quiet() const { return quiet_.load(std::memory_order_acquire); }
  bool pick_open() const { return !frozen_.load(std::memory_order_acquire); }
  bool prelude_done() const { return prelude_done_.load(std::memory_order_acquire); }
  // mature()/proposal()/pick_ranking() use the effective min_rounds: one
  // card, prelude done and rounds < min_rounds -> 1, else min_rounds. When
  // that flips from 1 back to min_rounds they can step back once (mature()
  // true -> false, proposal() -> op): the core must latch its decision.
  // After freeze() proposal() is stale: it holds its last value even if op
  // changes.
  // mature(): every member except op ranked (op too while searching: then
  // it is visited like any other). op_ranked(): op's pair (width 40) / op
  // itself (width 20) has the effective min_rounds. While the search is off
  // and the pick open, op's halves are never dwelt on (they carry
  // the drone's own video), so op keeps only its pre-link visits
  // and may well be unranked at maturity -- the core freezes in place then.
  bool mature() const;
  bool op_ranked() const;
  uint8_t proposal() const { return proposal_.load(std::memory_order_acquire); }
  std::vector<uint8_t> pick_ranking() const;
  uint64_t rounds() const { return rounds_.load(std::memory_order_acquire); }
  std::vector<RankEntry> ranking() const;
  std::vector<ScoutDwell> take_dwells();
  bool done() const { return done_.load(std::memory_order_acquire); }
  bool round_measuring() const { return round_.load(std::memory_order_acquire) == kRoundMeasuring; }
  bool round_open() const { return round_.load(std::memory_order_acquire) != kRoundIdle; }
  // Every observed sample of the current (or last) round, in dwell order.
  std::vector<RankSample> round_samples() const;

 private:
  bool has_work_() const;
  bool measuring_() const;
  bool member_(uint8_t ch) const;
  int eff_min_rounds_() const;
  void park_();
  bool tune_(uint8_t ch);
  bool step_dwell_(bool burst_ok, bool observe);
  bool is_op_half_(uint8_t ch) const;
  bool skip_op_half_(uint8_t ch) const;
  bool op_ranked_locked_(int mr) const;   // mu_ held
  void note_round_dwell_();               // run thread: a round dwell finished
  void end_round_measuring_();
  // Retune, settle, [burst + gap,] [discard read, observe dwell_ms, read];
  // records the dwell (+ the sample when observed).
  bool dwell(uint8_t ch, uint64_t round, bool burst, bool observe);
  void publish_();

  ScoutCfg cfg_;                   // cfg_.measure is the boot value only: measure_ is live
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
  std::atomic<bool> measure_{true};
  std::atomic<bool> resched_{false};   // set_measure() changed the dwell set: rebuild sched_
  std::atomic<bool> working_{false}, beaconing_{false}, quiet_{false}, prelude_done_{true},
      prelude_ack_{true}, done_{false};
  std::atomic<uint64_t> rounds_{0};
  // The one-round state: measuring -> measured (no work, not yet parked)
  // -> idle once parked. One atomic, moved by compare-exchange, so a
  // begin_round() racing the run thread's park can never be lost.
  static constexpr int kRoundIdle = 0, kRoundMeasuring = 1, kRoundMeasured = 2;
  std::atomic<int> round_{kRoundIdle};
  int round_dwells_ = 0;     // under mu_: dwells taken this round (bounds a dead half)
  std::vector<RankSample> round_samples_;   // under mu_
  bool need_width_ = true;   // run thread only: the next tune switches the card to 20 MHz
  int64_t t0_ = -1;          // run thread only: first run_once() (one-card prelude start)
};

}  // namespace maburgs
