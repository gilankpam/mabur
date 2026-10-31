#pragma once
// Drone-side calibration sweep state machine (docs/calibration.md).
//
// Walks the cells one GS-commanded SweepCmd describes -- a (kind, width)
// at a time -- stamping every Power/Verify frame it sends with the cell it
// was sent in so the GS can attribute delivery back to that cell with no
// clock sync and no sequence negotiation (mabur/cal_wire.h's header comment
// has the why). Capacity cells instead run one production-sized blast per
// MCS and report what the owner's CapacityCtl measured.
//
// Three constraints shape this class:
//
//   1. It owns no thread. RadioTx::send_body is single-caller by contract
//      (drone/src/radio_tx.h) -- its scratch buffer is not thread-safe --
//      and the TX writer thread (drone/src/main.cpp) is its sole legal
//      caller. pump() is meant to be called FROM that thread once the
//      TxQueue is quiesced; this class never spawns anything of its own.
//      A Capacity point's blast() call is allowed to block pump() for the
//      whole dwell -- that thread is the one legal driver of the device,
//      so there is no other thread for it to block.
//   2. Every non-Idle state carries a deadline. Losing the link mid-run is
//      the expected case, not an error -- a sweep deliberately walks into
//      indices where nothing decodes, and the GS is deliberately silent
//      for most of a run. pump() checks the run-wide deadline before
//      anything else, so the drone always finds its way back to Idle with
//      power restored, GS or no GS.
//   3. Idempotent by (nonce, seq), monotonically. The uplink loses
//      30-50% of frames, so the GS repeats a SweepCmd into the drone's
//      listen window, and those repeats can arrive out of order. A seq at
//      or behind the one already accepted -- a repeat of the sweep
//      running now, or a late duplicate of an EARLIER sweep arriving
//      after the run has moved on -- must be ignored or re-acked, not
//      restarted: restarting would re-zero the cell cursor and throw away
//      partial progress.
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "mabur/cal_proto.h"
#include "mabur/cal_wire.h"
#include "mabur/profile.h"
#include "radio_tx.h"

namespace mabur {

struct CalSweepCfg {
  // Absolute ceiling on one calibration run (every kind/width sweep of one
  // nonce combined -- Power, its Verify, and any Capacity sweeps that
  // follow), from the first accepted SweepCmd. The backstop for "the GS
  // never speaks again" -- see constraint 2.
  uint32_t hard_cap_ms = 180000;
  // How long to wait for the next run event -- the next SweepCmd, an
  // on_apply, or (once an apply is drained) the arm_verify it should
  // trigger -- before giving up on the run and restoring power on its
  // own. Re-armed fresh at each of those events.
  uint32_t await_next_ms = 15000;
  // radio.ldpc: whether Power/Verify/Capacity frames fly LDPC. STBC is
  // always on (an RTL8821AU GS ground card decodes STBC but not every
  // chip's HT-LDPC at every rate, which is exactly what calibration is
  // measuring for -- so the sweep rides whatever the operator has chosen
  // for the link, not a fixed "most robust" choice of its own).
  bool ldpc = true;
  // Capacity frame body size -- the production A-MPDU payload
  // (13 + bpb*(16 + symbol_size) from [fec]) -- so a Capacity point
  // measures throughput at the size real video frames actually fly at,
  // not the fixed 64-byte Power/Verify stamp.
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

// Sweeps the cells of one SweepCmd at a time -- Power, Verify (armed
// directly by the owner after an apply, not over the wire) or Capacity --
// and reports the stamped delivery frames or the measured capacity back,
// then hands off a matching ApplyCmd (once one arrives) for the owner to
// apply. Not thread-safe; on_cmd/on_apply/on_abort/pump/arm_verify are all
// meant to be called from the same thread that owns the RadioTx passed to
// pump() (see the file header).
//
// Every Power/Verify cell is SIGNED and RELATIVE to the chip's own TXAGC
// anchor -- the wire never carries an absolute index, on the command in,
// the stamped sweep frame, or the ack out. The anchor itself is read once
// per run (read_anchor_idx(), below) and stays entirely drone-internal: it
// is what pwr.set_index_override() is programmed against
// (clamp(anchor_idx_ + rel, 0, 127)) and what close_session() restores,
// but it is never reported to the GS.
class CalSweep {
 public:
  enum class State { Idle, Sweeping, Applying };

  // Test/production seam for the three knobs this class needs from the
  // drone's real power path (drone/src/power_plan.h in production):
  // parking the TXAGC index at a candidate cell, zeroing the
  // wall-equalized per-rate diff table so every rate is swept from the
  // same anchor, and reading that anchor back.
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

  explicit CalSweep(CalSweepCfg cfg) : cfg_(cfg) {}

  // Accepts a sweep command (Power or Capacity -- Verify never arrives
  // this way, see arm_verify). A brand-new nonce starts a fresh run
  // (state -> Sweeping, cell cursor reset, the chip's anchor captured
  // right here, synchronously), UNLESS that nonce is the one this engine
  // just closed (hard cap, abort, or the await timeout) -- a straggler or
  // a GS resend of an already-closed nonce is ignored outright, with no
  // ack, so a closed run can never reopen and hard_cap_ms keeps bounding
  // the run it was armed for. A chip that cannot report its anchor right
  // now (read_anchor_idx() < 0) cannot place a single relative cell: the
  // command is refused with AckResult::AnchorUnreadable rather than
  // silently dropped, so the GS does not have to infer the refusal from a
  // timeout. A table that would not zero (zero_rate_diffs() false) is
  // refused the same way, before any read -- for a later power sweep of
  // the run too, which then closes the run. A seq strictly BEHIND the one already accepted for this
  // nonce -- a late duplicate of an earlier sweep -- is ignored outright
  // with no ack. An EXACT repeat of the seq already running re-arms the
  // same Accepted ack every time (the GS resends its SweepCmd into a
  // 30-50%-lossy uplink, and a single lost ack must not cost the whole
  // sweep) without touching the cell cursor; the re-armed ack's
  // start_in_ms reports whatever is left of the original 200 ms start
  // delay, clamped to 0 once cells are already airing. A seq strictly
  // AHEAD of the one already accepted (matching nonce) starts a new sweep
  // from this command's own fields, reusing the nonce's already-captured
  // anchor, and drops any ApplyCmd still queued from an earlier sweep of
  // this run (take_pending_apply() would otherwise hand out a result for
  // a sweep this new command just superseded).
  //
  // A new run zeroes the per-rate diff table before its one anchor read,
  // whatever the kind (the chip reports reference + diff while a custom
  // table is live). Every accepted Power-kind command -- new run or a
  // later sweep of the same run -- zeroes it again (per-rate walls must be
  // measured against a common base every time Power runs); a later
  // Capacity sweep of a run never zeroes. Capacity DOES restore the TXAGC
  // override to the anchor on acceptance (a prior Power/Verify sweep of
  // the same run may have left it parked at a swept cell); the owner then
  // programs this width's production plan and its CapacityCtl clears the
  // override per point, since a live override flattens the per-rate
  // table. The anchor itself is read only once, at the run's first
  // accepted command, regardless of kind. The
  // first cell of any accepted sweep does not open until 200 ms after
  // acceptance (see pump()): the ack has to have a chance to reach the GS
  // before frames the GS is not listening for yet start using up the
  // air.
  void on_cmd(const cal::SweepCmd& c, uint64_t now_ms, PowerCtl& pwr);

  // Accepts an ApplyCmd for the run currently in flight. Ignored if there
  // is no open run, the nonce doesn't match, a sweep is actively
  // Sweeping (accepting one now would silently abandon that sweep's
  // progress), or this exact (nonce, seq) has already been handed out by
  // take_pending_apply() -- the GS resends APPLY every 200 ms until it
  // hears back APPLIED, and a repeat landing after the owner has already
  // applied it and armed Verify must not re-queue anything or disturb
  // the verify sweep running. Otherwise queues it for take_pending_apply()
  // and moves to Applying; the run stays open (close_session is not
  // called) so a silent GS still times out through the normal
  // await/hard-cap path -- checked in Applying too, not just the shared
  // hard cap.
  void on_apply(const cal::ApplyCmd& a, uint64_t now_ms);

  // Cancels the run identified by `a.nonce` immediately -- the same
  // close_session every other exit path uses, so power is restored and
  // the run is torn down synchronously rather than waiting on a timeout.
  // Ignored if there is no open run or the nonce doesn't match.
  void on_abort(const cal::Abort& a, uint64_t now_ms, PowerCtl& pwr);

  // True while a run (any sweep, or the Idle gap between sweeps/apply/
  // verify while awaiting the next event) is open -- i.e. before it has
  // timed out, been aborted, or fully drained. Distinct from state(),
  // which reports the transmit sub-state rather than whether a run is in
  // flight at all.
  bool active() const { return has_session_; }

  // The open run's nonce; meaningful only while active().
  uint32_t run_nonce() const { return nonce_; }

  // True iff on_cmd(c) would start a sweep (a new run or a later seq of
  // the open one) rather than re-ack or ignore it -- before the anchor
  // read, which can still refuse it. The owner programs what a starting
  // sweep needs from the device (aggregation off, a capacity width's
  // power table) only for a command this accepts, so a GS resend landing
  // mid-sweep never touches the device under the sweep running.
  bool starts_sweep(const cal::SweepCmd& c) const;

  // Advances the sweep by one unit of work: entering a cell, sending one
  // frame (Power/Verify) or running one Capacity point, or ending a
  // sweep/run. Checks the run-wide deadline first (constraint 2) so a
  // silent GS can never leave the drone parked mid-sweep. Must be called
  // from RadioTx's single legal caller thread (constraint 1); a Capacity
  // point's cap.blast() call may occupy this call for its whole dwell.
  void pump(uint64_t now_ms, RadioTx& tx, PowerCtl& pwr, CapacityCtl& cap);

  // Arms a Verify sweep directly -- not over the wire -- for the owner to
  // call once it has applied an ApplyCmd it drained from
  // take_pending_apply(). A no-op when there is no open run, or when a
  // Power/Capacity sweep is actively Sweeping (verify only ever follows a
  // drained apply, never interrupts a sweep in progress). Walks exactly
  // one cell per MCS, at the parked relative index `park_rel[mcs]`
  // (kWallLeave skips that MCS: nothing was measured, or the GS left the
  // file's existing wall alone), at `bw`. There is no per-sweep plan from
  // the wire to copy pacing from, so the cells run at the same
  // frames-per-cell/settle/gap the protocol's own SweepCmd defaults use
  // (20 frames, 5 ms settle, 1 ms gap) -- see cal_sweep.cpp. Never zeroes
  // the diff table and never touches PowerCtl itself (the per-cell index
  // override, like every Power/Verify cell, is programmed inside pump()).
  // Like on_cmd's sweeps, the first cell does not open until 200 ms after
  // this call. A Verify sweep emits no SweepDone -- the GS times its own
  // verify window -- so take_done() never arms for it.
  void arm_verify(const std::array<int, 8>& park_rel, uint8_t bw, uint64_t now_ms);

  // Drains the ack on_cmd() armed: Accepted for a new or repeated sweep,
  // AnchorUnreadable for a refused one. Nullopt for a call that accepted
  // nothing at all -- a stale repeat of an EARLIER seq.
  std::optional<cal::SweepAck> take_ack();

  // Drains the finished sweep's per-cell results, for a Power or Capacity
  // sweep (never for Verify -- see arm_verify). Armed the instant the last
  // cell's work is done, inside pump(), and re-armed by pump()
  // kDoneFirstRepeatMs later and every kDoneRepeatMs after that while the
  // run waits: SweepDone rides a lossy downlink, and a capacity sweep
  // whose SweepDone never reaches the GS writes nothing. The half-period
  // offset keeps the repeats off the GS's own 200 ms resend grid, so a
  // repeat and the GS's next command never land on the same tick. The
  // repeats stop at the next command of the run -- any SweepCmd, ApplyCmd
  // or Abort naming its nonce, taken as the GS having moved on -- or when
  // the run closes. The owner sends whatever this yields.
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
  // flattened by then) and cleared by this call.
  //
  // Why the caller must act on it: a chip that cannot report its anchor
  // right after being zeroed leaves the drone with a FLATTENED table and
  // no run -- has_session_ stays false, so the caller's falling-edge
  // power restore never fires, and the flattening is permanent until the
  // caller notices this flag and restores the real operating plan itself.
  bool take_refused();

  State state() const { return state_; }

  // kFlagStbc, plus kFlagLdpc when radio.ldpc (cfg_.ldpc) is set. Stamped
  // on every Power/Verify/Capacity frame and echoed in every SweepAck so
  // the GS can filter frames on it.
  uint8_t stamp_flags() const;

  // The drone-internal TXAGC reference this run's cells are programmed
  // relative to -- read once at the run's first accepted command and
  // never re-read for a later sweep of the same run. 0 before any
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
  struct Cell {
    uint8_t mcs = 0;
    int idx = 0;  // relative to anchor_idx_ for Power/Verify; unused for Capacity
  };

  void build_cells(const cal::SweepCmd& c);
  void build_verify_cells(const std::array<int, 8>& park_rel);
  void pump_power_verify(uint64_t now_ms, RadioTx& tx, PowerCtl& pwr);
  void pump_capacity(uint64_t now_ms, CapacityCtl& cap);
  void finish_sweep(uint64_t now_ms);
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

  // When the run's current sweep's first cell is allowed to open --
  // now_ms + 200 at the moment on_cmd()/arm_verify() accepted the sweep.
  // Both pump_power_verify() and pump_capacity() wait for this before
  // touching a cell, so the ack (or, for Verify, the applied config) has
  // time to reach the GS before any frame the GS is not listening for
  // yet airs.
  uint64_t sweep_start_ms_ = 0;

  // The (nonce, seq) of the last ApplyCmd handed out by
  // take_pending_apply(), so a GS resend of that same apply (it keeps
  // resending until it hears APPLIED) is ignored by on_apply() rather
  // than re-queued over a verify sweep already running from it.
  bool handed_out_apply_ = false;
  uint32_t handed_out_nonce_ = 0;
  uint8_t handed_out_seq_ = 0;

  // Current sweep's plan.
  cal::Kind current_kind_ = cal::Kind::Power;
  uint8_t current_bw_ = 20;
  uint16_t frames_per_cell_ = 0;
  uint32_t settle_ms_ = 0;
  uint32_t gap_ms_ = 0;
  uint32_t dwell_ms_ = 0;  // Capacity only

  std::vector<Cell> cells_;
  size_t cursor_ = 0;
  bool cell_entered_ = false;
  uint8_t active_mcs_ = 0;
  int active_idx_ = 0;  // relative, as stamped on the wire (Power/Verify)

  // Per-cell frame pacing (Power/Verify), copied out of the accepted
  // SweepCmd (or the hard-coded Verify constants -- see arm_verify()).
  uint16_t cell_seq_ = 0;
  uint64_t settle_deadline_ms_ = 0;
  uint64_t next_send_ms_ = 0;

  // Per-MCS accumulators for the sweep in progress: frames actually sent
  // for Power (SweepDone.offered[mcs] is the row's send count), or the
  // CapacityCtl::blast result for Capacity. Unused for Verify (no
  // SweepDone is ever built from these).
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
