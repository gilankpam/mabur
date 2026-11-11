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
// scout measures every member of the set (one dwell per pair at width 40)
// once on a fresh ranker -- the same dwell_ms observes, settle and send
// gating as the boot pick -- skipping op's own pair, which carries the
// drone's video. Op is read from the verdict window instead
// (the worst valid card on op: decoded foreign frames scaled to dwell_ms,
// NHM busy minus our own airtime). The pick compares only those two terms
// on both sides, with pick_margin_pct against op's busy % once both are
// in the same tier (cal_round_pick.h); it is logged as a K record. The
// round starts only with the scout thread up and its card alive, is
// dropped when a run is already calibrating, and ends at a wall-clock
// bound (twice a full
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
//
// The observe lease (observe(ms), the 8402 `observe` verb): while the
// goggles' channel picker is open the player keeps a deadline renewed and
// the core measures the set for it -- every member but op, at the link
// width, feeding scan.members and never the hop ranker. On a two-card GS
// that is scout rounds back to back (the calibration round's begin_round,
// under the same start conditions plus no relocate due). A pinned scout is
// frozen and has measuring turned on for the lease, off again once the
// lease has lapsed and the round is idle. A lease that runs out lets its
// open round finish; observe(0), set_pin, set_auto and begin_cal_measure
// end the lease and cancel its open round first. On a one-USB-card GS it
// is the in-flight burst on that card, once per hop.lease_sweep_period_ms,
// run synchronously on the core thread (a relay carrying TX included) and
// only while no hop or relocate is in flight or due and no relay sweep or
// posted burst is out. The burst takes the sole card off op for ~75 ms, so the hop verdict
// skips every sampler window from the burst's start to one window_ms past
// its end: those windows re-seed the per-card baselines instead of being
// classified, and the verdict's loss windows are blanked to the same
// instant, so the gap reads as neither interference nor loss.
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

// Which measurer an observe lease engages (ChannelCore::lease_state).
enum class LeaseState { None, TwoCard, OneCard, Relay, Search };
// The `observe` verb's reply word: "none" "two_card" "one_card" "relay" "search".
inline const char* lease_state_name(LeaseState s) {
  switch (s) {
    case LeaseState::None:    return "none";
    case LeaseState::TwoCard: return "two_card";
    case LeaseState::OneCard: return "one_card";
    case LeaseState::Relay:   return "relay";
    case LeaseState::Search:  return "search";
  }
  return "none";
}

struct ChannelCoreCfg {
  RadioCfg radio;              // channels, pin, scan knobs (the width is width_mhz)
  HopCfg hop;                  // [hop] + [hop.verdict]
  mabur::LinkKey key;          // disc_for_channel re-tags per channel
  // The receiver width every card is opened at. The live value is
  // ChannelCore::width_mhz(); set_width moves it, the hop ranker and the
  // in-flight scout with it, and the boot scout once its card is retuned.
  uint8_t width_mhz = 20;
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
  const Aggregator* agg = nullptr;   // per-card crc_fail/rssi_ema/snr_ema, decoder stats
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
  // Every channel-set member's row (the goggles Link > Channel row),
  // config order: built fresh each snapshot() call from member_figs_/
  // op_fig_ -- see ChannelCore::snapshot.
  std::vector<StatsScanMemberIn> members;
  // The live pin's relocate and its sequence counter, derived from
  // pin_reloc_/pin_seq_ (ChannelCore::snapshot): null while the pin needs
  // no move or none is tracked yet.
  std::optional<StatsRelocateIn> relocate;
  uint64_t relocate_seq = 0;
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
  int scout_card() const { return scout_card_; }   // the boot scout card (-1 = none); opens at the receiver width
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
  // ---- live receiver width (core thread) ----
  // The width the cards are held at (the per-tick width resync's target).
  uint8_t width_mhz() const { return width_mhz_; }
  // What set_width(want) would be refused with, asked without changing
  // anything. nullopt when want is the current width; otherwise, first
  // match: "ladder refused: scan in flight" (the boot pick is open or a
  // calibration measure round runs, so the width resync is closed),
  // "ladder refused: channel set is not 40 MHz pair-valid (member <n>)" on
  // a flip to 40, the calibration / relocate / hop refusals pin_refusal
  // gives, then "sweep in flight" (a relay sweep holds its card off op).
  // An unlinked search on the scout card is no refusal, nor is an in-flight
  // dwell: that card is retuned once it is back.
  std::optional<std::string> width_refusal(uint8_t want) const;
  // Flip the receiver width. A no-op at the current width. Otherwise every
  // card's one-shot width resync is re-armed and run once on the spot, so
  // each ready card that is neither off on an in-flight dwell nor held by
  // the unlinked search is retuned before this returns; a dwelling,
  // searching or down card is taken by the per-tick resync once it is free
  // (or reopened). The hop ranker ranks at the new width from here, the
  // in-flight scout records its dwells at it, and the boot scout follows
  // once its card is retuned. The caller has checked width_refusal first.
  void set_width(uint8_t want);
  // ---- live ladder swap (core thread) ----
  // VrxController::replace_ladder, plus the hop verdict's frozen restore
  // rung translated by {mcs, bw} into the new ladder (rung 0 when it has no
  // such rung, as the swap itself falls back): the rung is an index, and
  // the same index in another ladder is another operating point.
  void replace_ladder(const LadderCfg& ladder, double now_ms);

  // ---- the observe lease (core thread) ----
  // Keep measuring for ms more (clamped to [500, 5000]) and return
  // lease_state() as of now. ms <= 0 releases the lease at once (its open
  // round is cancelled; the scout parks on its next step) and returns None.
  LeaseState observe(int ms);
  bool observing() const;   // a lease is held: now is before its deadline
  // The measurer a lease engages, ignoring whether one is held; first
  // match: calibration owns the scout (a run, its measure round Waiting/
  // Running, its relocate Waiting/InFlight) -> None; the boot pick open ->
  // None; unlinked -> Search while the scout searches (not a relay-only
  // GS), else None; linked: two or more USB cards -> TwoCard, one USB card
  // -> OneCard, relays only with a ready sweep-capable one -> Relay; else
  // None. TwoCard runs scout rounds and OneCard the in-flight burst;
  // Relay and Search are reported to the player and measure nothing extra.
  LeaseState lease_state() const;
  // The whole `observe <rest>` verb: "ok observe <state>", or
  // "err usage: observe <ms>" unless rest is a bare non-negative integer.
  std::string observe_verb(const std::string& rest);

  // ---- test seam (threaded = false) ----
  void run_scout_step();      // one ChannelScout::run_once()
  void run_inflight_step();   // one body of the in-flight loop (no sleep)
  void run_sampler_step();    // WindowSampler::step_now() at the tick's time: sample, publish, restart the cadence
  const HopRanker& hop_ranker() const { return ranker_; }

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
  // The verdict's own pre-FEC loss windows, one per video layer (base =
  // decoder stream 0, enhancement = stream 1); the verdict reads the worse
  // (hop_verdict_link_loss).
  S1LossWindow s1_hop_loss_;
  S1LossWindow enh_hop_loss_;

  // ---- per-card bookkeeping (core thread) ----
  std::vector<uint8_t> cur_ch_;
  uint8_t width_mhz_ = 20;     // the live receiver width (set_width); cfg_.width_mhz at start
  std::vector<bool> width_tried_;
  std::vector<std::optional<StatsEnergyIn>> energy_last_;
  std::vector<std::optional<StatsDwellIn>> dwell_stats_;
  // Newest NHM busy reading landed on each channel-set member (index
  // matches cfg_.radio.channels) from any dwell -- boot scout, a
  // calibration measure round, the in-flight scout or a relay sweep.
  // nullopt = never measured; overwritten by emit_dwell_, never merged, so
  // the table always shows the most recent reading, not an average. Op's
  // own figure lives separately in op_fig_: a linked op is read from the
  // verdict window instead, never dwelt on.
  struct MemberFig { double busy_pct = 0; uint64_t t_ms = 0; };
  std::vector<std::optional<MemberFig>> member_figs_;
  std::vector<ScoutFrames> window_prev_;
  std::vector<uint64_t> window_prev_crc_;
  // The card's seq walk at the previous sample (CardTrack::seq_expected /
  // seq_received): the window's delivery ratio scales its own airtime.
  std::vector<uint64_t> window_prev_seq_exp_, window_prev_seq_rcv_;
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
  // In-flight thread: how often its period wait checks for a posted burst;
  // and how long the core waits for a posted burst before giving it up.
  static constexpr int kScoutPollMs = 5;
  static constexpr double kBurstLostMs = 2000;
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
  bool burst_done_ = false;   // dwell_mu_: the posted burst's results are in dwell_recs_/dwell_visits_
  // A spare card's freshness burst, posted by the core thread and run by the
  // in-flight thread (threaded, two cards): the card and the op to return to.
  // -1 = nothing posted. The core holds the hop trigger back from the post
  // until step_drains_ has ranked the results (burst_inflight_).
  std::atomic<int> burst_req_card_{-1};
  std::atomic<uint8_t> burst_req_op_{0};
  bool burst_inflight_ = false;   // core thread
  double burst_posted_ms_ = 0;    // core thread
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
  // A live pin's relocate: the channel row's own view of it
  // (scan.relocate/scan.relocate_seq, ChannelCore::snapshot derives them
  // from this -- no parallel field). Off outside a linked pin move; Waiting
  // until BootPick hands the order to the controller; InFlight from Order to
  // Confirm/VerifyPass/Withdraw; Settled holds the outcome (ok/why) for the
  // rest of the pin's life, until set_pin or set_auto move it again.
  struct PinReloc {
    enum S { Off, Waiting, InFlight, Settled } s = Off;
    uint8_t ch = 0;
    bool confirmed = false, ok = false;
    std::string why;
  } pin_reloc_;
  // scan.relocate_seq: bumped by every set_pin call that is not refused
  // (the no-op onto the channel already pinned to included) and by every
  // set_auto call, so a watcher can tell a still-pending outcome from a
  // fresh relocate that landed on the channel it was already watching.
  uint64_t pin_seq_ = 0;
  bool pin_reloc_tick_ = false;      // this tick's Relocate is the pin's
  // ---- the observe lease ----
  // The lease's deadline on the now_ms_ clock; 0 = none held.
  double lease_until_ = 0;
  // A scout round the lease began is open (cleared once round_open() reads
  // false). The lease keeps its own flag, never cal_meas_: pin_refusal
  // answers "calibration running" while cal_meas_ is live.
  bool lease_round_ = false;
  // The lease turned the pinned scout's measuring on; it goes off once the
  // lease has lapsed and no round is open (set_measure(false) on a round
  // still Measuring would leave round_open() true for good).
  bool lease_forced_measure_ = false;
  // The engaged state last logged ("observe lease <ms> (<state>)").
  LeaseState lease_logged_ = LeaseState::None;
  // Drops the lease's deadline, logging the lapse once; an open round is
  // left to finish.
  void lapse_lease_();
  // lapse_lease_, and the open lease round is cancelled (the scout parks
  // on its next step): a release, a pin, auto, a calibration pick.
  void end_lease_();
  void step_lease_(const ChannelTickIn& in);
  // The one-card lease: one in-flight burst on the sole USB card when
  // lease_sweep_period_ms has passed since the last one and nothing else
  // holds the card, the controller or a burst.
  void step_lease_one_card_(const ChannelTickIn& in);
  // The last one-card lease burst's start (tick time).
  double last_lease_burst_ms_ = -1e18;
  // Sampler records stamped in (from, until] are masked from the hop
  // verdict: from = the burst's start, until = its end plus window_ms (the
  // window that straddles the burst's end). until < from = none yet.
  double lease_mask_from_ms_ = 0, lease_mask_until_ms_ = -1;
  // The latest verdict window's reading of op: the worst valid card on op,
  // foreign frames scaled to radio.scan.dwell_ms.
  std::optional<CalReading> op_live_;
  double op_live_ms_ = -1;
  // scan.members[]'s own figure for op while linked: the MIN across every
  // card with a busy reading this window of (busy_pct - own airtime),
  // floored at 0 -- the best-case reading, unlike op_live_'s worst-case
  // (the calibration pick wants the worst card; the channel row wants
  // what a hop would actually buy). Stamped with the window's sample
  // time; reset to nullopt on every hop_active edge (step_hop_edge_and_window_).
  std::optional<std::pair<double, uint64_t>> op_fig_;
  bool in_session_cur_ = false, cal_running_cur_ = false;
  // the tick's current inputs, for helpers
  double now_ms_cur_ = 0;
  uint64_t now_ms_u_cur_ = 0;

  // ---- steps ----
  // Every dwell record's one entrypoint to the sink: logs it (sink_.dwell)
  // then, when it is usable (busy_valid and the retune did not fail), lands
  // its busy reading in member_figs_ against the member it was dwelt on.
  void emit_dwell_(double now_ms, int card, const ScoutDwell& d);
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
  // The live pin's own relocate, mirroring step_cal_'s confirmed-then-Idle
  // check: a session lost between the Confirm and its VerifyPass leaves
  // HopController idle without ever handing back the VerifyPass action.
  void step_pin_reloc_(const ChannelTickIn& in);
  void settle_pin_reloc_(bool ok, const char* why);
  std::optional<std::string> move_refusal_(uint8_t ch) const;
  std::optional<std::string> link_moving_refusal_() const;
  std::optional<std::string> busy_refusal_() const;
  std::optional<std::string> relocate_refusal_(uint8_t ch) const;
  // The boot pick is open or a measure round runs: the width resync is
  // closed and a width flip is refused.
  bool scan_holds_width_() const;
  void inflight_body_();
  void scout_loop_();
  // One freshness burst on `card`, back to `op`; caller holds inflight_mu_.
  std::vector<HopVisit> burst_on_card_(int card, uint8_t op, std::vector<ScoutDwell>& recs);
  void run_posted_burst_();
  void start_inflight_if_due_();
  std::string logf_(const char* fmt, ...);
};

}  // namespace maburgs
