#pragma once
// ChannelCore: maburgs's channel/hop wiring as one unit -- the channel plan,
// the boot scout + BootPick, the hop verdict/ranker/controller and the
// in-flight scout, plus the per-card retune/width bookkeeping they share.
// Six seams: lifecycle (on_card_died/on_card_reopened/shutdown), the rc sink
// (on_rc_body/on_session_opened), the drain hooks (note_video/note_au_end),
// tick(), the send path (disc_targets/may_send/disc_for_card/note_tx_card/
// note_sent) and snapshot(). Per tick: drain -> tick() -> send path.
// Threading: the core thread calls everything public; the scout thread and
// the in-flight thread are owned (started, joined) here; threaded = false
// runs without threads for tests. tx_frozen()/dwell_busy() are live reads
// because the send path runs after tick() and a dwell may start in between.
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "aggregator.h"
#include "boot_pick.h"
#include "channel_plan.h"
#include "channel_scout.h"
#include "config.h"
#include "hop_burst_gate.h"
#include "hop_controller.h"
#include "hop_ranker.h"
#include "hop_verdict.h"
#include "inflight_scout.h"
#include "link_card.h"
#include "mabur/link_key.h"
#include "mabur/rc_proto.h"
#include "nhm_window.h"
#include "s1_loss.h"
#include "stats_exporter.h"
#include "vrx_controller.h"

namespace maburgs {

// hop.state on the sideport: HopState has no to_string() of its own.
inline const char* hop_state_name(HopState s) {
  switch (s) {
    case HopState::Idle:      return "idle";
    case HopState::Ordered:   return "ordered";
    case HopState::Verifying: return "verifying";
    case HopState::Hold:      return "hold";
  }
  return "idle";
}

struct ChannelCoreCfg {
  RadioCfg radio;              // channels, pin, width, scan knobs
  HopCfg hop;                  // [hop] + [hop.verdict]
  mabur::LinkKey key;          // disc_for_channel re-tags per channel
  uint8_t start_ch = 0;        // the pin, else the remembered member, else channels[0]
  int n_usb = 0;               // USB cards come first in the roster
  double leak_per_frame = 1.0; // bench row 7 pins this (docs/channel-select.md)
  // false = no threads: tick() runs one scout step itself; the in-flight
  // body runs only through run_inflight_step() (tests; deterministic with
  // an injected clock).
  bool threaded = true;
};

// Where the records and the stderr-style lines go. maburgs: ScanLog +
// stderr. Web: the page log. Record shapes are the scan.log ones.
class ChannelSink {
 public:
  virtual ~ChannelSink() = default;
  virtual void caps(double t_ms, int card, const CardCaps& c) = 0;
  virtual void dwell(double t_ms, int card, const ScoutDwell& d) = 0;
  virtual void pick(double t_ms, std::optional<uint8_t> picked, uint64_t rounds,
                    const std::vector<RankEntry>& all, int min_rounds) = 0;
  virtual void move(const MoveEvent& e) = 0;
  virtual void verdict(double t_ms, const VerdictOut& o, const std::vector<VerdictCardIn>& cards,
                       const VerdictLinkIn& link) = 0;
  virtual void hop(const HopEvent& e) = 0;
  virtual void log(const std::string& line) = 0;   // one stderr line, no trailing newline
};

struct ChannelTickIn {
  double now_ms = 0;
  bool in_session = false;     // VrxState::SESSION
  bool cal_running = false;    // CalSession::running()
  int tx_card = 0;             // TxSelector::selected()
  const Aggregator* agg = nullptr;   // per-card crc_fail/rssi_a_ema/snr_ema, decoder stats
};

struct ChannelTickOut {
  bool tx_frozen = false;      // keep sel.selected() this tick (tx_selection_frozen)
  bool dwell_busy = false;     // cal_cmd_clear input
};

struct ChannelSnapshot {
  int channel = 0;                        // the TX card's live channel
  const char* scan_state = "off";         // off | scouting | moving | frozen
  uint64_t scan_rounds = 0;
  std::optional<int> scan_pick;
  StatsHopIn hop;
  std::vector<std::optional<StatsEnergyIn>> energy;   // per card
  std::vector<std::optional<StatsDwellIn>> dwell;     // per card
  uint64_t scout_gated_sends = 0;
};

class ChannelCore {
 public:
  using NowMsFn = std::function<uint64_t()>;
  using NowUsFn = std::function<uint64_t()>;
  using SleepFn = std::function<void(int)>;
  using StoreFn = std::function<bool(uint8_t)>;   // remembered channel; false = could not write

  ChannelCore(ChannelCoreCfg cfg, std::vector<LinkCard*> cards, VrxController& vrx,
              ChannelSink& sink, StoreFn store, NowMsFn now_ms, NowUsFn now_us, SleepFn sleep_ms);
  ~ChannelCore();   // shutdown()

  // ---- lifecycle (core thread) ----
  void on_card_died(int card);       // the card reads !alive(): scout-card handling
  void on_card_reopened(int card);   // open_and_start() succeeded: resync cur_ch/width_tried
  void shutdown();                   // join both threads; idempotent

  // ---- receive hooks (core thread, in the drain loop) ----
  void on_rc_body(uint8_t rx_ch);                        // before agg.on_rx_body()
  // The DISC_ACK that opened the session (BEACONING/… -> SESSION inside this
  // on_rc_frame call): the link forms where it was heard (link_found).
  void on_session_opened(const mabur::rc::DiscAck& ack, double now_ms);
  bool is_link_video(uint8_t rx_ch) const { return plan_.is_link_video(rx_ch); }
  void note_video(uint8_t rx_ch) { last_video_ch_ = rx_ch; }
  void note_au_end() { au_seq_.fetch_add(1, std::memory_order_relaxed); }

  // ---- the per-tick block (where main.cpp's "auto channel set, per tick" began) ----
  ChannelTickOut tick(const ChannelTickIn& in);

  // ---- send path (core thread, after tick()) ----
  std::vector<int> disc_targets(int tx) const;   // scan_disc_targets while the scout owns a card, else {tx}
  bool may_send(int card) const;                 // the scout gate; false = drop (counted)
  std::vector<uint8_t> disc_for_card(const std::vector<uint8_t>& frame, int card) const;
  void note_sent(bool sent_ok, bool is_rcf);
  // The send path's TX choice, stored the instant it is made (old main.cpp
  // stored tx_card_now right after sel.update()): the in-flight thread
  // picks its dwell card against the LIVE TX card, not last tick's.
  void note_tx_card(int tx) { tx_card_now_.store(tx, std::memory_order_relaxed); }

  // ---- state ----
  ChannelSnapshot snapshot() const;
  uint8_t op() const { return plan_.op(); }
  int scout_card() const { return scout_card_; }   // the boot scout card (-1 = none); opens at 20 MHz
  bool hopping() const { return plan_.hopping(); }
  uint8_t hop_target() const { return plan_.hop_target(); }
  // Live reads of the cross-thread state at call time (not the end-of-tick
  // ChannelTickOut copy): the send path runs after tick(), and a dwell the
  // scout thread starts in between must still freeze TX / hold the cal cmd.
  bool tx_frozen() const { return tx_selection_frozen(dwell_busy_.load(), plan_.hopping()); }
  bool dwell_busy() const { return dwell_busy_.load(); }
  uint64_t scout_gated_sends() const { return scout_gated_sends_; }
  bool pick_open() const { return boot_pick_.open(); }
  bool scout_working() const { return scout_ && scout_->working(); }       // card reopen gate
  bool scout_owns_card(int card) const { return scout_owns_() && card == scout_card_; }  // TX snapshots

  // ---- test seam (threaded = false) ----
  void run_scout_step();      // one ChannelScout::run_once()
  void run_inflight_step();   // one body of the in-flight loop (no sleep)

 private:
  // ---- construction-time ----
  ChannelCoreCfg cfg_;
  std::vector<LinkCard*> cards_;
  VrxController& vrx_;
  ChannelSink& sink_;
  StoreFn store_;
  NowMsFn now_ms_;
  NowUsFn now_us_;
  SleepFn sleep_;
  int n_cards_ = 0;
  bool pinned_ = false;
  std::vector<bool> can_scout_;
  std::vector<bool> snr_ok_;
  int scout_card_ = -1;        // pick_boot_scout(can_scout_)
  bool one_card_ = false;      // n_usb == 1 (scout mode); the hop's one-card path is lead<0
  bool relay_search_only_ = false;   // no card can measure: the last relay searches (Task 6)
  uint8_t saved_op_ = 0;       // last value handed to store_
  uint64_t gs_start_ms_ = 0;

  // ---- units ----
  ChannelPlan plan_;
  std::unique_ptr<ChannelScout> scout_;
  BootPick boot_pick_;
  HopVerdict verdict_;
  HopRanker ranker_;
  HopController hopc_;
  std::unique_ptr<InflightScout> inflight_;
  S1LossWindow s1_hop_loss_;

  // ---- per-card bookkeeping (core thread) ----
  std::vector<uint8_t> cur_ch_;
  std::vector<bool> width_tried_;
  std::vector<std::optional<StatsEnergyIn>> energy_last_;
  std::vector<std::optional<StatsDwellIn>> dwell_stats_;
  std::vector<ScoutFrames> window_prev_;
  std::vector<uint64_t> window_prev_crc_;
  std::vector<bool> window_prev_ok_;
  std::vector<uint64_t> window_prev_ms_;
  std::vector<NhmWindowTracker> nhm_win_;
  uint16_t nhm_op_period_ = 0;

  // ---- scout thread state ----
  std::thread scout_thread_;
  bool scout_started_ = false;
  bool scout_was_working_ = false;
  bool scout_card_down_ = false;
  bool scout_search_req_ = false;
  bool scout_owns_() const;

  // ---- boot pick / relocation ----
  bool link_edge_seen_ = false;
  bool scout_card_died_seen_ = false;
  std::optional<uint8_t> pending_relocate_;
  std::optional<uint8_t> frozen_pick_;
  BootPickIn last_bi_;

  // ---- hop state ----
  int hop_lead_latched_ = -1;
  uint64_t last_window_ms_ = 0;
  uint64_t recovered_prev_window_ = 0;
  uint64_t au_seq_prev_ = 0;
  bool hop_was_active_ = false;
  CalMoveEdgeHold cal_move_hold_;
  Verdict last_verdict_ = Verdict::Healthy;
  VerdictOut last_verdict_out_;
  std::optional<uint64_t> last_hop_event_ms_;
  double last_burst_ms_ = -1e18;
  uint64_t rcf_sent_total_ = 0, rcf_sent_at_order_ = 0, ctrl_sent_total_ = 0;
  mutable uint64_t scout_gated_sends_ = 0;   // bumped inside the const gate query
  uint8_t last_video_ch_ = 0;
  uint8_t rc_body_rx_ch_ = 0;

  // ---- cross-thread ----
  std::atomic<bool> dwell_busy_{false};
  std::atomic<int> dwell_card_{-1};
  std::atomic<bool> scout_run_{false};
  std::atomic<bool> in_session_atomic_{false};
  std::atomic<bool> cal_running_atomic_{false};
  std::atomic<bool> hopping_atomic_{false};
  std::atomic<int> tx_card_now_{0};
  std::atomic<bool> scout_working_atomic_{false};
  std::vector<std::atomic<uint32_t>> dwell_gen_;
  std::atomic<uint64_t> au_seq_{0};
  std::mutex dwell_mu_;
  std::vector<std::pair<int, ScoutDwell>> dwell_recs_;
  std::vector<HopVisit> dwell_visits_;
  std::mutex inflight_mu_;
  bool inflight_started_ = false;
  std::thread scout_thread2_;
  bool shut_ = false;
  // the tick's current inputs, for helpers
  double now_ms_cur_ = 0;
  uint64_t now_ms_u_cur_ = 0;

  // ---- steps (Task 3/4/5) ----
  void freeze_pick_(double t, const char* why);
  void dispatch_hop_action_(const HopAction& act, bool relocate_tick, double now_ms);
  void apply_hop_action_(const HopAction& act, double now_ms);
  void step_scout_inputs_();
  void step_boot_pick_(const ChannelTickIn& in);
  void step_store_();
  void step_hop_edge_and_window_(const ChannelTickIn& in);
  void step_controller_(const ChannelTickIn& in);
  void step_move_edge_(const ChannelTickIn& in);
  void step_drains_();
  void step_width_resync_();
  void step_mechanical_retune_();
  void inflight_body_();
  void scout_loop_();
  std::string logf_(const char* fmt, ...);
};

}  // namespace maburgs
