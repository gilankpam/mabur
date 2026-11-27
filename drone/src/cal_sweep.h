#pragma once
// Drone-side calibration sweep state machine (docs/calibration.md).
//
// Runs the sweeps of a GS-commanded run one SweepCmd -- a (kind, width) --
// at a time: a Capacity sweep runs one production-sized blast per MCS and
// reports what the owner's CapacityCtl measured; a Wall sweep drives the
// owner's WallCtl, which measures the walls on the drone itself, and
// reports what it found.
//
// Three constraints shape this class:
//
//   1. It owns no thread. RadioTx is single-caller by contract
//      (drone/src/radio_tx.h) and the TX writer thread
//      (drone/src/maburd/tx_writer.cpp) is its sole legal caller. pump()
//      is meant to be called FROM that thread once the TxQueue is
//      quiesced; this class never spawns anything of its own. A Capacity
//      point's blast() call is allowed to block pump() for the whole
//      dwell -- that thread is the one legal driver of the device, so
//      there is no other thread for it to block.
//   2. Every non-Idle state carries a deadline. Losing the link mid-run is
//      the expected case, not an error -- a capacity point deliberately
//      saturates the air, and the GS is deliberately silent for most of a
//      run. pump() checks the run-wide deadline before
//      anything else, so the drone always finds its way back to Idle with
//      power restored, GS or no GS.
//   3. Idempotent by (nonce, seq), monotonically. The uplink loses
//      30-50% of frames, so the GS repeats a SweepCmd into the drone's
//      listen window, and those repeats can arrive out of order. A seq at
//      or behind the one already accepted -- a repeat of the sweep
//      running now, or a late duplicate of an EARLIER sweep arriving
//      after the run has moved on -- must be ignored or re-acked, not
//      restarted: restarting would reset the point cursor (or reopen a
//      measurement) and throw away partial progress.
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "mabur/cal_proto.h"
#include "mabur/cal_wire.h"
#include "mabur/profile.h"

namespace mabur {

struct CalSweepCfg {
  // Absolute ceiling on one calibration run (every kind/width sweep of one
  // nonce combined), from the first accepted SweepCmd. The backstop for "the GS
  // never speaks again" -- see constraint 2.
  uint32_t hard_cap_ms = 180000;
  // How long to wait for the next run event -- the next SweepCmd or an
  // on_apply -- before giving up on the run and restoring power on its
  // own. Re-armed fresh at each of those events.
  uint32_t await_next_ms = 15000;
  // radio.ldpc: whether Capacity frames fly LDPC. STBC is
  // always on (an RTL8821AU GS ground card decodes STBC but not every
  // chip's HT-LDPC at every rate, which is exactly what calibration is
  // measuring for -- so the sweep rides whatever the operator has chosen
  // for the link, not a fixed "most robust" choice of its own).
  bool ldpc = true;
  // Capacity frame body size -- the production A-MPDU payload
  // (13 + bpb*(16 + symbol_size) from [fec]) -- so a Capacity point
  // measures throughput at the size real video frames actually fly at.
  size_t capacity_frame_len = 1405;
};

// One Capacity point's result: how many frames the blast offered and what
// fraction of them the device blocked on (not "lost" -- Capacity measures
// sender-side backpressure, not delivery; the GS's own receiver-side count
// is the loss number).
struct BlastResult {
  uint32_t offered = 0;
  uint8_t blocked_pct = 0;
};

// Runs one SweepCmd at a time -- Capacity or Wall -- and reports the
// result back, then hands off a matching ApplyCmd (once one arrives) for
// the owner to apply. Not thread-safe; on_cmd/on_apply/on_abort/pump are
// all meant to be called from the TX writer thread (see the file header).
//
// The chip's TXAGC anchor is read once per run (read_anchor_idx(), below)
// and stays drone-internal: it is what close_session() parks the override
// at, and what a Capacity sweep restores the override to on acceptance.
class CalSweep {
 public:
  enum class State { Idle, Sweeping, Applying };

  // Test/production seam for the three knobs this class needs from the
  // drone's real power path (drone/src/power_plan.h in production): the
  // flat TXAGC override, zeroing the per-rate diff table so the anchor
  // reads clean, and reading that anchor back.
  struct PowerCtl {
    virtual ~PowerCtl() = default;
    virtual bool set_index_override(int idx) = 0;
    virtual bool zero_rate_diffs() = 0;
    // The chip's TXAGC reference for the current channel with the diff
    // table zeroed and no override live; -1 if the chip cannot report it.
    virtual int read_anchor_idx() = 0;
  };

  // Test/production seam for a Capacity point. Owned by the TX writer
  // thread, same as PowerCtl -- set_point programs the production ladder
  // and A-MPDU mode for (mcs, bw) (devourer's SetAmpduMode is a live
  // switch), and blast sends the point's frame as fast as the device
  // will take it for the dwell, BLOCKING the calling pump() for that
  // whole duration by design: this thread is the device's sole driver,
  // so there is nothing else for it to yield to.
  struct CapacityCtl {
    virtual ~CapacityCtl() = default;
    virtual bool set_point(uint8_t mcs, uint8_t bw) = 0;
    virtual BlastResult blast(const uint8_t* frame, size_t len, uint32_t dwell_ms) = 0;
  };

  // Test/production seam for a Wall sweep, owned by the TX writer thread
  // like the other two. pump() calls start() once the start delay has
  // passed, then step() once per pump until it returns false, then
  // finish(); any path that abandons a started measurement calls cancel().
  struct WallCtl {
    virtual ~WallCtl() = default;
    // Accepted, or why not. Writes nothing to the device.
    virtual cal::AckResult check(uint8_t channel, uint8_t bw) = 0;
    // Takes the radio to the channel and width and opens the measurement.
    virtual void start(uint8_t channel, uint8_t bw, uint64_t now_ms) = 0;
    virtual bool step(uint64_t now_ms) = 0;  // false once the measurement has finished
    // Puts the radio back, stores a valid result, and returns the wall fields of the report.
    virtual cal::SweepDone finish() = 0;
    // Puts the radio back and stores nothing. Safe when nothing is open.
    virtual void cancel() = 0;
  };

  explicit CalSweep(CalSweepCfg cfg) : cfg_(cfg) {}

  // Accepts a sweep command (Capacity or Wall). A brand-new nonce starts
  // a fresh run (state -> Sweeping, the chip's anchor captured right here,
  // synchronously), UNLESS that nonce is the one this engine just closed
  // (hard cap, abort, or the await timeout) -- a straggler or a GS resend
  // of an already-closed nonce is ignored outright, with no ack, so a
  // closed run can never reopen and hard_cap_ms keeps bounding the run it
  // was armed for. A seq strictly BEHIND the one already accepted for this
  // nonce -- a late duplicate of an earlier sweep -- is ignored outright
  // with no ack. An EXACT repeat of the seq already running re-arms the
  // same Accepted ack every time (the GS resends its SweepCmd into a
  // 30-50%-lossy uplink, and a single lost ack must not cost the whole
  // sweep) without touching the sweep: no second wall.check(), no restart.
  // The re-armed ack's start_in_ms reports whatever is left of the
  // original 200 ms start delay, clamped to 0 once the sweep is under way.
  // An exact repeat of a sweep that has already FINISHED (its SweepDone
  // still repeating) gets no ack: it re-queues that SweepDone for
  // take_done(), keeps the repeat schedule, and changes nothing else.
  //
  // A Wall command past those filters is checked (wall.check()) before
  // anything else: a refusal answers with that result and changes nothing
  // -- no run opens, an open run and its running sweep go on untouched,
  // and nothing is owed back to the power path.
  //
  // A new run zeroes the per-rate diff table before its one anchor read,
  // whatever the kind (the chip reports reference + diff while a custom
  // table is live). A chip that cannot report its anchor (read_anchor_idx()
  // < 0), or a table that would not zero, refuses the command with
  // AckResult::AnchorUnreadable rather than dropping it silently, and arms
  // take_refused(). A seq strictly AHEAD of the one already accepted
  // (matching nonce) starts a new sweep from this command's own fields,
  // reusing the run's anchor, cancelling a Wall measurement still open,
  // and dropping any ApplyCmd still queued from an earlier sweep (it is
  // superseded). An accepted Capacity sweep restores the TXAGC override
  // to the anchor; the owner then programs this width's production plan
  // and its CapacityCtl clears the override per point, since a live
  // override flattens the per-rate table. Any accepted sweep does not
  // start until 200 ms after acceptance (see pump()): the ack has to have
  // a chance to reach the GS first.
  void on_cmd(const cal::SweepCmd& c, uint64_t now_ms, PowerCtl& pwr, WallCtl& wall);

  // Accepts an ApplyCmd for the run currently in flight. Ignored if there
  // is no open run, the nonce doesn't match, a sweep is actively
  // Sweeping (accepting one now would silently abandon that sweep's
  // progress), or this exact (nonce, seq) has already been handed out by
  // take_pending_apply() -- the GS resends APPLY every 200 ms until it
  // hears back APPLIED, and a repeat landing after the owner has already
  // applied it must not re-queue anything. Otherwise queues it for take_pending_apply()
  // and moves to Applying; the run stays open (close_session is not
  // called) so a silent GS still times out through the normal
  // await/hard-cap path -- checked in Applying too, not just the shared
  // hard cap.
  void on_apply(const cal::ApplyCmd& a, uint64_t now_ms);

  // Cancels the run identified by `a.nonce` immediately -- a started Wall
  // measurement first (wall.cancel()), then the same close_session every
  // other exit path uses, so power is restored and the run is torn down
  // synchronously rather than waiting on a timeout. Ignored if there is no
  // open run or the nonce doesn't match.
  void on_abort(const cal::Abort& a, uint64_t now_ms, PowerCtl& pwr, WallCtl& wall);

  // True while a run (any sweep, or the Idle gap between sweeps/apply
  // while awaiting the next event) is open -- i.e. before it has
  // timed out, been aborted, or fully drained. Distinct from state(),
  // which reports the transmit sub-state rather than whether a run is in
  // flight at all.
  bool active() const { return has_session_; }

  // The open run's nonce; meaningful only while active().
  uint32_t run_nonce() const { return nonce_; }

  // True iff on_cmd(c) would start a sweep (a new run or a later seq of
  // the open one) rather than re-ack or ignore it -- before the anchor
  // read and the wall check, which can still refuse it. The owner programs
  // what a starting sweep needs from the device (aggregation off, a
  // capacity width's power table) only for a command this accepts, so a
  // GS resend landing mid-sweep never touches the device under the sweep
  // running.
  bool starts_sweep(const cal::SweepCmd& c) const;

  // Advances the sweep by one unit of work: entering or running one
  // Capacity point, starting or stepping a Wall measurement, or ending a
  // sweep/run. Checks the run-wide deadline first (constraint 2) so a
  // silent GS can never leave the drone mid-sweep; a started Wall
  // measurement is cancelled before the run closes there. Once a Wall
  // sweep's start delay has passed, the first pump only calls
  // wall.start(); every later one calls wall.step(), and the pump that
  // sees it return false reports wall.finish() with nonce and seq filled
  // in. Must be called from the TX writer thread (constraint 1); a
  // Capacity point's cap.blast() call may occupy this call for its whole
  // dwell.
  void pump(uint64_t now_ms, PowerCtl& pwr, CapacityCtl& cap, WallCtl& wall);

  // Drains the ack on_cmd() armed: Accepted for a new or repeated sweep,
  // the refusal's result for a refused one. Nullopt for a call that
  // answered nothing at all -- a stale repeat of an EARLIER seq, or a
  // closed nonce.
  std::optional<cal::SweepAck> take_ack();

  // Drains the finished sweep's report, Capacity or Wall. Armed the
  // instant the sweep's work is done, inside pump(), and re-armed by pump()
  // kDoneFirstRepeatMs later and every kDoneRepeatMs after that while the
  // run waits: SweepDone rides a lossy downlink, and a sweep whose
  // SweepDone never reaches the GS is lost to the run. The half-period
  // offset keeps the repeats off the GS's own 200 ms resend grid, so a
  // repeat and the GS's next command never land on the same tick. The
  // repeats stop at the next command of the run -- any SweepCmd, ApplyCmd
  // or Abort naming its nonce, taken as the GS having moved on, except an
  // exact repeat of the finished sweep, which re-sends it -- or when the
  // run closes. The owner sends whatever this yields.
  std::optional<cal::SweepDone> take_done();

  static constexpr uint32_t kDoneFirstRepeatMs = 100;
  static constexpr uint32_t kDoneRepeatMs = 200;

  // Drains the ApplyCmd queued by on_apply(), if any. Once drained, the
  // caller owns it and moves the run from Applying back to Idle -- state
  // stays Applying only until this is called or the next on_cmd lands.
  std::optional<cal::ApplyCmd> take_pending_apply();

  // Drains the "this command was REFUSED after flattening the rate-diff
  // table" flag, armed once by on_cmd()'s anchor-read failure (a new run
  // zeroes before that read whatever its kind, so the table is always
  // flattened by then) and cleared by this call. A Wall refusal from
  // wall.check() never arms it: nothing was touched.
  //
  // Why the caller must act on it: a chip that cannot report its anchor
  // right after being zeroed leaves the drone with a FLATTENED table and
  // no run -- has_session_ stays false, so the caller's falling-edge
  // power restore never fires, and the flattening is permanent until the
  // caller notices this flag and restores the real operating plan itself.
  bool take_refused();

  State state() const { return state_; }

  // kFlagStbc, plus kFlagLdpc when radio.ldpc (cfg_.ldpc) is set. Stamped
  // on every Capacity frame and echoed in every SweepAck so the GS can
  // filter frames on it.
  uint8_t stamp_flags() const;

  // The drone-internal TXAGC reference of this run -- read once at the
  // run's first accepted command and never re-read for a later sweep of
  // the same run. 0 before any
  // command of this run has ever been accepted. Test-only: production
  // code has no reason to read this back, since it never leaves the
  // drone.
  int anchor_idx() const { return anchor_idx_; }

  // True once this run's TXAGC override has actually been restored to
  // the anchor read at run start -- i.e. close_session() has run and
  // there was something to restore. Test-only: production code has no
  // reason to poll this, since restoring IS the point of reaching Idle.
  bool power_restored_for_test() const { return power_restored_; }

 private:
  void build_points(const cal::SweepCmd& c);
  void pump_capacity(uint64_t now_ms, CapacityCtl& cap);
  void pump_wall(uint64_t now_ms, WallCtl& wall);
  void finish_sweep(uint64_t now_ms, const cal::SweepDone& d);
  // wall.cancel() when a Wall measurement has been started and not
  // finished; a no-op otherwise.
  void cancel_wall(WallCtl& wall);
  void close_session(PowerCtl& pwr);
  // Milliseconds left until sweep_start_ms_, clamped to 0 once it has
  // passed -- what a SweepAck's start_in_ms reports.
  uint16_t remaining_start_ms(uint64_t now_ms) const;

  CalSweepCfg cfg_;

  State state_ = State::Idle;

  // Run bookkeeping. has_session_ is the "in flight" flag active()
  // reports; nonce_/last_started_seq_ are what makes on_cmd idempotent by
  // (nonce, seq) (constraint 3).
  bool has_session_ = false;
  uint32_t nonce_ = 0;
  int last_started_seq_ = -1;  // -1 = no sweep accepted yet this run
  uint64_t hard_cap_deadline_ms_ = 0;  // whole-run ceiling
  uint64_t await_deadline_ms_ = 0;     // between-event wait

  // The last nonce this engine closed (hard cap, abort or await timeout),
  // so a straggler or a GS resend of that exact nonce arriving afterward
  // is ignored instead of reopening a fresh run -- see on_cmd().
  bool last_nonce_valid_ = false;
  uint32_t last_closed_nonce_ = 0;

  int anchor_idx_ = 0;
  bool power_restored_ = false;

  // Set by on_cmd() every time it accepts or idempotently re-arms a
  // sweep, drained by take_ack().
  std::optional<cal::SweepAck> pending_ack_;

  // Set by on_cmd() when it refuses a command after zero_rate_diffs() has
  // already touched the table, drained by take_refused().
  bool pending_refusal_ = false;

  // When the run's current sweep may start -- now_ms + 200 at the moment
  // on_cmd() accepted it, so the ack has time to reach the GS first.
  uint64_t sweep_start_ms_ = 0;

  // The (nonce, seq) of the last ApplyCmd handed out by
  // take_pending_apply(), so a GS resend of that same apply (it keeps
  // resending until it hears APPLIED) is ignored by on_apply() rather
  // than re-queued.
  bool handed_out_apply_ = false;
  uint32_t handed_out_nonce_ = 0;
  uint8_t handed_out_seq_ = 0;

  // Current sweep's plan.
  cal::Kind current_kind_ = cal::Kind::Capacity;
  uint8_t current_bw_ = 20;
  uint8_t current_channel_ = 0;  // Wall only
  uint32_t settle_ms_ = 0;       // Capacity only
  uint32_t dwell_ms_ = 0;        // Capacity only

  // Capacity: the MCS points left to run, the cursor into them, and
  // whether the current one has been set up and is settling.
  std::vector<uint8_t> points_;
  size_t cursor_ = 0;
  bool point_entered_ = false;
  uint64_t settle_deadline_ms_ = 0;

  // Wall: wall.start() has been called and the measurement has neither
  // finished nor been cancelled.
  bool wall_started_ = false;

  // Per-MCS CapacityCtl::blast results for the sweep in progress.
  std::array<uint32_t, 8> offered_{};
  std::array<uint8_t, 8> blocked_pct_{};

  std::optional<cal::SweepDone> pending_done_;
  // The finished sweep's SweepDone while it is still being repeated (see
  // take_done()), and when pump() re-arms it next.
  std::optional<cal::SweepDone> repeat_done_;
  uint64_t next_done_ms_ = 0;
  std::optional<cal::ApplyCmd> pending_apply_;
};

}  // namespace mabur
