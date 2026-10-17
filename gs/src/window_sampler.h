#pragma once
// WindowSampler: the channel-health window's card reads, off the core
// thread. Every hop.window_ms it samples each card -- survey rebase, NHM
// busy read, FA/CCA delta, frame counters, NHM re-arm -- and hands the core
// one WindowRecord through a one-slot mailbox. The reads are synchronous
// USB register I/O that shares devourer's per-device lock with the card's
// own coex tick (10-26 ms every 2 s); the core thread must never wait on
// that, because a frame body waits in BodyQueue for as long as it does.
// Threaded: its own thread (start/stop). Stepped (tests, the web GS
// without core_threads): step() from tick(), same tick as the consume.
// Owns the per-card NhmWindowTracker: an NHM reading counts only if this
// sampler armed it, on the same channel, with no scout dwell and no core
// retune since (the two generation counters the core bumps).
// Owns the per-card device-use lock (hold_card): one pass holds a card's
// lock across all of that card's reads, and the card lifecycle's
// stop/reopen, the core's dwell burst and the in-flight scout's dwell take
// the same lock, so the reads never land on a card being torn down or
// tuned away and back. The core's single retunes and width changes do not
// take it: the tune generation already voids an arm they span.
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "link_card.h"
#include "nhm_window.h"
#include "sweep_types.h"

namespace maburgs {

struct CardSample {
  bool valid = false;                   // ready, not blocked, on the op channel
  std::optional<double> nhm_busy_pct;   // empty = no NHM evidence this window
  ScoutEnergy energy;                   // read_energy_scout(): fa/cca since the previous read
  ScoutFrames frames;                   // counters at the sample instant
  uint8_t channel = 0;
  SurveyWindow survey;                  // relay cards; invalid on USB cards
};

struct WindowRecord {
  uint64_t t_ms = 0;                    // sample instant, the core's ms clock
  std::vector<CardSample> cards;
  uint32_t epoch = 0;                   // reset generation this record was sampled under
};

struct WindowSamplerCfg {
  int window_ms = 150;
  uint16_t nhm_period_4us = 0;          // the arm period (nhm_period_4us(window_ms - 10))
  int busy_dbm = -83;                   // radio.scan.busy.busy_dbm
};

// Facts the core thread owns, mirrored as atomics it writes and the
// sampler only reads. card_blocked = the boot scout owns the card or a
// relay sweep is on it; a dwell in progress is dwell_busy/dwell_card.
struct WindowGates {
  const std::atomic<bool>& in_session;
  const std::atomic<bool>& cal_running;
  const std::atomic<uint8_t>& op;
  const std::atomic<bool>& dwell_busy;
  const std::atomic<int>& dwell_card;
  const std::vector<std::atomic<uint32_t>>& dwell_gen;
  const std::vector<std::atomic<uint32_t>>& tune_gen;
  const std::vector<std::atomic<bool>>& card_blocked;
};

class WindowSampler {
 public:
  using NowMsFn = std::function<uint64_t()>;
  using SleepFn = std::function<void(int)>;

  WindowSampler(std::vector<LinkCard*> cards, WindowSamplerCfg cfg, WindowGates gates);
  ~WindowSampler();   // stop()
  WindowSampler(const WindowSampler&) = delete;
  WindowSampler& operator=(const WindowSampler&) = delete;

  bool active() const;                    // in_session && !cal_running
  WindowRecord sample_once(uint64_t now_ms);
  // Hop-inactive edge: drop every arm and the mailbox, bump the reset epoch
  // so a pass already in flight can never republish its pre-edge record
  // once this returns, and (stepped mode) restart the cadence so the first
  // window after the edge is a full window_ms away. The thread keeps its
  // own phase; its first record after the edge only primes the core's
  // per-card baselines.
  void reset(uint64_t now_ms);
  // Stepped mode: one cadence check; publishes when a window is due and active.
  void step(uint64_t now_ms);
  // Sample and publish now, whatever the cadence, and restart it from now
  // (ChannelCore's test seam).
  void step_now(uint64_t now_ms);
  // Threaded mode: one thread, sleep_ms(window_ms) between passes.
  void start(NowMsFn now_ms, SleepFn sleep_ms);
  // Idempotent; joins. Returns after up to one sleep_ms(window_ms) plus a
  // pass, so callers order stop() before tearing the cards down.
  void stop();

  // Newest wins; a replaced record is counted as dropped and its FA/CCA
  // delta folded into the newer one, but a record sampled under an epoch
  // reset() has since moved past is discarded silently.
  void publish(WindowRecord r);
  std::optional<WindowRecord> take();     // core thread
  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

  // Card i's device-use lock, held for as long as the returned lock lives.
  // Waits for at most one in-flight pass on that card. Never call stop()
  // while holding it: the pass stop() joins may be waiting on it.
  std::unique_lock<std::mutex> hold_card(size_t i);

 private:
  std::vector<LinkCard*> cards_;
  WindowSamplerCfg cfg_;
  WindowGates gates_;
  std::vector<std::mutex> card_mu_;          // per-card device-use lock (hold_card)
  std::vector<NhmWindowTracker> trackers_;   // sampling thread only (after reset_req_)
  std::atomic<bool> reset_req_{false};
  std::atomic<uint32_t> epoch_{0};           // bumped by reset(); pins publish() to its pass
  uint64_t last_ms_ = 0;                     // stepped cadence (core thread)
  std::mutex mu_;
  std::optional<WindowRecord> box_;
  std::atomic<uint64_t> dropped_{0};
  std::atomic<bool> run_{false};
  std::thread th_;
};

}  // namespace maburgs
