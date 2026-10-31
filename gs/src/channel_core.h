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
//
// A calibration run's channel pick (CalPicker, cal_picker.h) rides the same
// units, no parallel path. begin_cal_measure() arms one measure round of
// the boot scout (ChannelScout::begin_round): once the boot pick has closed
// and nothing else holds the scout card, a hop or the controller, the
// scout measures every 20 MHz half of the set (every member at width 20)
// once on a fresh ranker -- the same dwell_ms observes, settle and send
// gating as the boot pick -- skipping op's own halves, which carry the
// drone's video. Op is read from the verdict window instead
// (the worst valid card on op: decoded foreign frames scaled to dwell_ms,
// NHM busy minus our own airtime). The pick compares only those two terms
// on both sides, with pick_margin against op in every tier
// (cal_round_pick.h); it is logged as a K record. The round starts only
// with the scout thread up and its card alive, is dropped when a run is
// already calibrating, and ends at a wall-clock bound (twice a full
// round's dwell time) with op as the answer; cancel_cal_measure() releases
// it at any point. order_cal_relocate() moves want and re-arms BootPick's
// relocation, so the move is the boot pick's own relocate order (H
// relocate, reason "cal"), landed by the confirm (no verify window: the run
// calibrates wherever it lands) and resolved through BootPick (a failure
// accepts op). The store follows op as for any hop. Pinned or one-card:
// nothing is measured and op is the answer. Both run before the session
// sets cal_running, which freezes channel changes for the run.
//
// The pin changes live (set_pin / set_auto; the caller has already written
// radio.channel). Pinning: linked, the move is the boot pick's own relocate
// order (H relocate, reason "pin"), landed by the confirm with no verify
// window, exactly as a boot-time pin relocates a drone found elsewhere; a
// relocate that fails or is cut by link loss leaves the link where it is,
// still pinned (the boot pin's rule). Unlinked, op moves at once (M commit)
// and the cards follow on the next tick. Either way the pick closes, the
// scout goes back to search-only and the reactive hop stops. Back to auto:
// linked, only the reactive hop re-arms from op (no pick, no relocate);
// unlinked, the boot pick reopens over a fresh ranking from op, its max_ms
// counted from the reopen.
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
#include "cal_round_pick.h"
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
#include "relay_sweep_map.h"
#include "s1_loss.h"
#include "stats_exporter.h"
#include "window_sampler.h"
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
  // Name of the remembered-channel store for the "could not write" line:
  // maburgs passes mabur::kGsChannelFile, the web GS "CHANNEL line".
  std::string store_name;
};

// Where the records and the stderr-style lines go. maburgs: ScanLog +
// stderr. Web: the page log. Record shapes are the scan.log ones. Six
// records: dwell, pick, move, verdict, hop, log. Card caps are not one:
// the core never emits them (maburgs logs caps to scan.log itself).
class ChannelSink {
 public:
  virtual ~ChannelSink() = default;
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
  bool pinned = false;                    // ChannelCore::pinned(): the live pin, not the boot one
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
  ChannelSnapshot snapshot(int tx_card) const;   // channel = cards[tx_card]->channel()
  uint8_t op() const { return plan_.op(); }
  int scout_card() const { return scout_card_; }   // the boot scout card (-1 = none); opens at 20 MHz
  bool hopping() const { return plan_.hopping(); }
  uint8_t hop_target() const { return plan_.hop_target(); }
  // Live reads of the cross-thread state at call time (not the end-of-tick
  // ChannelTickOut copy): the send path runs after tick(), and a dwell the
  // scout thread starts in between must still freeze TX / hold the cal cmd.
  // A relay sweep in flight freezes it too (the selector must not move TX
  // onto a card that is off op for up to the sweep timeout).
  bool tx_frozen() const { return tx_selection_frozen(dwell_busy_.load() || sweep_.on, plan_.hopping()); }
  bool dwell_busy() const { return dwell_busy_.load(); }
  uint64_t scout_gated_sends() const { return scout_gated_sends_; }
  bool pick_open() const { return boot_pick_.open(); }
  // Relay sweep seams: a SCAN is in flight / timed out.
  bool sweep_pending() const { return sweep_.on; }
  uint64_t sweep_timeouts() const { return sweep_timeouts_; }
  bool scout_working() const { return scout_ && scout_->working(); }       // card reopen gate
  bool scout_owns_card(int card) const { return scout_owns_() && card == scout_card_; }  // TX snapshots
  uint64_t sampler_dropped() const { return sampler_ ? sampler_->dropped() : 0; }
  // The card's device-use lock (WindowSampler::hold_card): the card
  // lifecycle holds it across stop() + open_and_start() so no window read
  // or dwell is on the device while it is torn down. Not across shutdown().
  std::unique_lock<std::mutex> hold_card(int card) {
    return sampler_->hold_card(static_cast<size_t>(card));
  }

  // ---- calibration channel pick (CalPicker; core thread) ----
  bool pinned() const { return pinned_; }
  // Auto mode with a spare card that can measure (two or more USB cards).
  bool cal_can_measure() const;
  // Arms one measure round (restarts nothing while one is pending or
  // running). Not measurable: a no-op, cal_measure_best() is op.
  void begin_cal_measure(double now_ms);
  // The run is done with the pick (timeout, abort, failure): drop a pending
  // round, end a running one (the scout parks), and stop tracking a cal
  // relocate -- one not yet placed is withdrawn by want following op; one
  // already placed proceeds or withdraws like any relocate.
  void cancel_cal_measure();
  // The round's best member; nullopt while it waits or runs, op when the
  // GS cannot measure.
  std::optional<uint8_t> cal_measure_best();
  // The boot pick's relocate toward ch, reason cal. false = refused: not a
  // member, already op, link not live, a hop/relocation/sweep in flight or
  // the pick still open, the scout on its card, the hop rate limit.
  bool order_cal_relocate(uint8_t ch);
  // true once the confirm landed it, false when withdrawn, held at the
  // cap, lost with the session or never placed; nullopt pending / none.
  std::optional<bool> cal_relocate_settled();

  // ---- live pin (core thread) ----
  // Pin the link to ch. nullopt = done (pinning where a pinned link already
  // is changes nothing); otherwise why not, and nothing changed: "not a
  // member", "calibration running" (a run, its measure round or its
  // relocate), "relocate in flight", "hop in flight", and when a linked
  // move has to be ordered "sweep in flight" / "hop cap". One-card and
  // relay-only GSs take a pin like any other: the relocate is the one their
  // boot pin uses.
  std::optional<std::string> set_pin(uint8_t ch);
  // What set_pin(ch) would refuse with, asked without changing anything:
  // the live-config listener checks it before it touches the config file,
  // so a pin the core would refuse is never published first.
  std::optional<std::string> pin_refusal(uint8_t ch) const;
  // Back to auto. Never refused: nothing moves on the spot.
  void set_auto();

  // ---- test seam (threaded = false) ----
  void run_scout_step();      // one ChannelScout::run_once()
  void run_inflight_step();   // one body of the in-flight loop (no sleep)
  void run_sampler_step();    // WindowSampler::step_now() at the tick's time: sample, publish, restart the cadence

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
  // !pinned_: the in-flight hop + its scout and burst run only in auto mode.
  // Atomic: set_pin/set_auto flip it on the core thread while the in-flight
  // thread reads it.
  std::atomic<bool> reactive_{true};
  std::vector<bool> can_scout_;
  std::vector<bool> can_sweep_;   // LinkCard::can_sweep(): a relay that takes a v4 SCAN
  std::vector<bool> snr_ok_;
  int scout_card_ = -1;        // pick_boot_scout(can_scout_)
  bool one_card_ = false;      // n_usb == 1 (scout mode); the hop's one-card path is lead<0
  bool relay_search_only_ = false;   // no card can measure: the last relay searches
  uint8_t saved_op_ = 0;       // last value handed to store_
  uint64_t pick_start_ms_ = 0;  // GS start, or set_auto's reopen of the pick: radio.scan.max_ms counts from here

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
  uint64_t recovered_prev_window_ = 0;
  uint64_t au_seq_prev_ = 0;
  bool hop_was_active_ = false;
  CalMoveEdgeHold cal_move_hold_;
  Verdict last_verdict_ = Verdict::Healthy;
  VerdictOut last_verdict_out_;
  std::optional<uint64_t> last_hop_event_ms_;
  double last_burst_ms_ = -1e18;
  // ---- relay sweep ----
  // One SCAN in flight at most; it ends on its SCAN_RESULT, on the timeout,
  // or on the card dying. A reopened relay drops a pending scan without a
  // result, so !sweeping() never means "result ready".
  // The timeout is derived from the request when the SCAN leaves:
  // max(1000, passes * n * (observe_ms + 40) + 300) -- 40 ms per channel
  // covers the relay's retune + settle, 300 ms the round trip and slack. A
  // 4-member set (3 swept) sits on the 1000 ms floor; 8 members -> 1140 ms.
  struct PendingSweep { bool on = false; int card = -1; double sent_ms = 0; double timeout_ms = 1000; } sweep_;
  uint64_t sweep_round_ = 0, sweep_timeouts_ = 0;
  static constexpr double kSweepTimeoutMinMs = 1000, kSweepPerChannelMs = 40, kSweepSlackMs = 300;
  static constexpr uint8_t kSweepPasses = 2, kSweepObserveMs = 20;
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
  // Bumped after every core-thread retune / width change of a card
  // (step_mechanical_retune_, step_width_resync_) and on both sides of a
  // dwell burst (step_controller_): pins an NHM arm to the tune the card
  // had when it was armed.
  std::vector<std::atomic<uint32_t>> tune_gen_;
  // The sampler's view of the op channel and of "the core owns this card
  // right now" (boot scout on it, relay sweep on it), written each tick.
  std::atomic<uint8_t> op_atomic_{0};
  std::vector<std::atomic<bool>> card_blocked_;
  std::unique_ptr<WindowSampler> sampler_;
  bool sampler_started_ = false;
  std::atomic<uint64_t> au_seq_{0};
  std::mutex dwell_mu_;
  std::vector<std::pair<int, ScoutDwell>> dwell_recs_;
  std::vector<HopVisit> dwell_visits_;
  std::mutex inflight_mu_;
  bool inflight_started_ = false;
  std::thread scout_thread2_;
  bool shut_ = false;
  // ---- calibration channel pick ----
  struct CalMeasure {
    enum S { Off, Waiting, Running, Done } s = Off;
    double t0 = 0;       // round start (op's reading must be newer)
    double bound_ms = 0; // Running: ends the round here, op the answer
    uint8_t best = 0;
  } cal_meas_;
  struct CalReloc {
    enum S { Off, Waiting, InFlight, Settled } s = Off;
    uint8_t ch = 0;
    double t0 = 0;
    bool confirmed = false;
    bool ok = false;
  } cal_reloc_;
  bool cal_reloc_tick_ = false;      // this tick's Relocate is the calibration's
  // A live pin's relocate, until the controller takes or refuses it: only
  // the order's H record and log line carry the reason; after that it is
  // the boot pick's relocation like any other.
  std::optional<uint8_t> pin_reloc_;
  bool pin_reloc_tick_ = false;      // this tick's Relocate is the pin's
  // The latest verdict window's reading of op: the worst valid card on op,
  // foreign frames scaled to radio.scan.dwell_ms.
  std::optional<CalReading> op_live_;
  double op_live_ms_ = -1;
  bool in_session_cur_ = false, cal_running_cur_ = false;
  // the tick's current inputs, for helpers
  double now_ms_cur_ = 0;
  uint64_t now_ms_u_cur_ = 0;

  // ---- steps ----
  void freeze_pick_(double t, const char* why);
  void dispatch_hop_action_(const HopAction& act, bool relocate_tick, double now_ms);
  void apply_hop_action_(const HopAction& act, double now_ms);
  void step_scout_inputs_();
  void step_boot_pick_(const ChannelTickIn& in);
  void step_store_();
  void step_gates_();
  void step_hop_edge_and_window_(const ChannelTickIn& in);
  void step_controller_(const ChannelTickIn& in);
  void step_move_edge_(const ChannelTickIn& in);
  void step_drains_();
  void step_width_resync_();
  void step_mechanical_retune_();
  void poll_sweep_(double now_ms);
  void step_cal_(const ChannelTickIn& in);
  void finish_cal_measure_(double now_ms);
  void settle_cal_reloc_(bool ok, const char* why);
  std::optional<std::string> move_refusal_(uint8_t ch) const;
  std::optional<std::string> relocate_refusal_(uint8_t ch) const;
  void inflight_body_();
  void scout_loop_();
  std::string logf_(const char* fmt, ...);
};

}  // namespace maburgs
