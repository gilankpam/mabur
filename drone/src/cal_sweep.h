#pragma once
// Drone-side calibration sweep state machine (spec
// 2026-09-10-tx-power-calibration-design.md).
//
// Walks the (rate, TXAGC index) cells a GS-commanded CalCmd describes,
// stamping every frame it sends with the cell it was sent in so the GS can
// attribute delivery back to that cell -- see mabur/cal_wire.h's header
// comment for why attribution has to survive corrupt frames.
//
// Three constraints shape this class (see docs/superpowers/specs for the
// full design):
//
//   1. It owns no thread. RadioTx::send_body is single-caller by contract
//      (drone/src/radio_tx.h) -- its scratch buffer is not thread-safe --
//      and the TX writer thread (drone/src/main.cpp) is its sole legal
//      caller. pump() is meant to be called FROM that thread once the
//      TxQueue is quiesced; this class never spawns anything of its own.
//   2. Every non-Idle state carries a deadline. Losing the link mid-sweep
//      is the expected case, not an error -- the sweep deliberately walks
//      into indices where nothing decodes, and the GS is deliberately
//      silent for most of a session. pump() checks the session-wide
//      deadline before anything else, so the drone always finds its way
//      back to Idle with power restored, GS or no GS.
//   3. Idempotent by (nonce, phase), monotonically. The uplink loses
//      30-50% of frames, so the GS repeats a CalCmd into the drone's
//      listen window, and those repeats can arrive out of order. A phase
//      at or behind the one already accepted -- a repeat of the phase
//      running now, or a late duplicate of an EARLIER phase arriving after
//      the session has moved on -- must be ignored, not restarted:
//      restarting would re-zero the cell cursor, throw away partial
//      progress, and (if a result is sitting undrained in Applying)
//      silently discard the measured wall table itself.
#include <cstdint>
#include <optional>
#include <vector>

#include "mabur/rc_proto.h"
#include "radio_tx.h"

namespace mabur {

struct CalSweepCfg {
  // Absolute ceiling on one calibration session (all phases of one nonce
  // combined), from the first accepted CalCmd. The backstop for "the GS
  // never speaks again" -- see constraint 2.
  uint32_t hard_cap_ms = 180000;
  // Reserved for a future half-duplex listen window between cells (Task
  // 11's ack wiring); this class does not read it yet.
  uint32_t listen_ms = 1000;
  // How long to wait for the next session event -- the next phase's
  // CalCmd, an on_result, or (once a result is accepted) the verify-phase
  // CalCmd it should trigger -- before giving up on the session and
  // restoring power on its own. Re-armed fresh at each of those events
  // (see on_result(), which sets it from its own now_ms rather than
  // inheriting whatever was left over from the last phase finishing).
  uint32_t await_next_ms = 15000;
};

// Sweeps the cells of one CalCmd at a time and reports the (rate, idx)
// delivery test frames back through a RadioTx, then hands off a matching
// CalResult (once one arrives) for the caller to apply. Not thread-safe;
// on_cmd/on_result/pump are all meant to be called from the same thread
// that owns the RadioTx passed to pump() (see the file header).
class CalSweep {
 public:
  enum class State { Idle, Sweeping, Applying };

  // Test/production seam for the two knobs this class needs from the
  // drone's real power path (drone/src/power_plan.h in production):
  // parking the TXAGC index at a candidate cell, and zeroing the
  // wall-equalized per-rate diff table so every rate is swept from the
  // same base index (Global Constraint: "zero the diffs before sweeping").
  struct PowerCtl {
    virtual ~PowerCtl() = default;
    virtual bool set_index_override(int idx) = 0;
    virtual bool zero_rate_diffs() = 0;
    virtual int read_base_ref_idx() = 0;
  };

  explicit CalSweep(CalSweepCfg cfg) : cfg_(cfg) {}

  // Accepts a sweep command. A brand-new nonce starts a fresh session
  // (state -> Sweeping, cell cursor reset, per-rate diffs zeroed on the
  // next pump()). A phase at or behind the one already accepted for this
  // nonce -- an exact repeat, or a late duplicate of an earlier phase --
  // is ignored outright (constraint 3: phases are idempotent AND
  // monotonic, since coarse < fine < verify never runs backward). A phase
  // strictly ahead of the one already accepted (matching nonce) resumes
  // sweeping with a fresh cell cursor built from this command's windows,
  // without re-zeroing the diffs a prior phase of the same session already
  // zeroed.
  //
  // Takes PowerCtl (unlike on_result/pump's later, per-cell uses of it)
  // because a NEW session's diffs must be zeroed and its base_ref_idx
  // captured here, synchronously, at acceptance -- before pump() ever
  // parks the TXAGC at a swept cell's index. A later phase (fine) landing
  // after an earlier phase already did that would otherwise read the
  // override instead of the anchor if it read again (see
  // take_ack_base_ref()'s header comment).
  void on_cmd(const rc::CalCmd& c, uint64_t now_ms, PowerCtl& pwr);

  // Accepts a measured-wall result for the session currently in flight.
  // Ignored if there is no open session or the nonce doesn't match (a
  // result from a stale/foreign session) -- see stale_nonce_result_is_ignored.
  void on_result(const rc::CalResult& r, uint64_t now_ms);

  // True while a session (any phase, or the Idle gap between phases while
  // awaiting the next one) is open -- i.e. before it has timed out or been
  // fully drained. Distinct from state(), which reports the transmit
  // sub-state rather than whether a session is in flight at all.
  bool active() const { return has_session_; }

  // Advances the sweep by one unit of work: entering a cell, sending one
  // frame, or ending a phase/session. Checks the session-wide deadline
  // first (constraint 2) so a silent GS can never leave the drone parked
  // mid-sweep. Must be called from RadioTx's single legal caller thread
  // (constraint 1).
  void pump(uint64_t now_ms, RadioTx& tx, PowerCtl& pwr);

  State state() const { return state_; }

  // Drains the pending result set by on_result(), if any. Once drained,
  // the caller (Task 11's apply/verify wiring) owns it; a second call
  // returns nullopt until another on_result() lands.
  std::optional<rc::CalResult> take_pending_result();

  // Drains the ack payload on_cmd() armed for the phase it just accepted
  // (nullopt for a call that didn't accept one -- a new/rejected repeat,
  // constraint 3). The caller (Task 11's RC dispatch) must send exactly
  // one T_TELEM per drained value, before it next calls pump() for this
  // phase: CalSession::on_ack() (gs/src/cal_session.h) is what ends the
  // GS's AwaitAck state and opens its radio-silence window, and it
  // re-enters AwaitAck for the fine phase, so a drone that acked only once
  // would leave the GS transmitting into the very sweep the radio-silence
  // rule exists to keep clear.
  //
  // Always the value on_cmd() captured ONCE at session start, never a
  // fresh PowerCtl::read_base_ref_idx() taken here or inside on_cmd() for
  // a later phase: by the time a second phase (fine) is accepted, the
  // first phase has already parked the TXAGC at a swept cell's index
  // (pump_sweeping never restores between phases), and devourer's
  // GetTxPowerState reports that override, not the anchor, if read again.
  std::optional<int> take_ack_base_ref();

  // The same value take_ack_base_ref() vends, without draining it --
  // non-destructive, so the caller can still ask after already draining an
  // ack (Task 11's apply/verify wiring needs it again once a result lands,
  // by which point any ack for the phase that produced it is long gone).
  // 0 before any phase of this session has ever been accepted -- matches
  // the wire's own "0 = not read" sentinel (Telem.cal_base_ref_idx).
  int base_ref_idx() const { return base_ref_idx_; }

  // True once this session's TXAGC override has actually been restored to
  // the base reference index read at session start -- i.e. close_session()
  // has run and there was something to restore. Test-only: production
  // code has no reason to poll this, since restoring IS the point of
  // reaching Idle.
  bool power_restored_for_test() const { return power_restored_; }

 private:
  struct Cell {
    uint8_t rate = 0;
    uint8_t idx = 0;
  };

  void pump_sweeping(uint64_t now_ms, RadioTx& tx, PowerCtl& pwr);
  void close_session(PowerCtl& pwr);
  void build_cells(const std::vector<rc::CalWindow>& windows);

  CalSweepCfg cfg_;

  State state_ = State::Idle;

  // Session bookkeeping. has_session_ is the "in flight" flag active()
  // reports; nonce_/last_started_phase_ are what makes on_cmd idempotent
  // by (nonce, phase) (constraint 3).
  bool has_session_ = false;
  uint32_t nonce_ = 0;
  int last_started_phase_ = -1;  // -1 = no phase started yet this session
  uint8_t current_phase_ = 0;
  uint64_t hard_cap_deadline_ms_ = 0;  // whole-session ceiling
  uint64_t await_deadline_ms_ = 0;     // between-phase / between-result wait

  // Zeroed once per session (not per phase, so fine/verify measure against
  // the same base coarse already zeroed against), and the reference index
  // read back at that moment so close_session() can restore it.
  bool zeroed_for_session_ = false;
  int base_ref_idx_ = 0;
  bool power_restored_ = false;
  // Latched the first time this session's on_result() is accepted, so a
  // duplicated result frame (the uplink retransmits same as it does
  // commands) cannot re-arm Applying and drive a second cal_apply flash
  // write -- see the comment on on_result()'s definition.
  bool result_accepted_ = false;

  // Set by on_cmd() every time it accepts a phase (never on a rejected
  // repeat), drained by take_ack_base_ref(). See that method's header
  // comment for why this is always base_ref_idx_, not a fresh read.
  std::optional<int> pending_ack_base_ref_;

  // Current phase's cell plan and cursor.
  std::vector<Cell> cells_;
  size_t cursor_ = 0;
  bool cell_entered_ = false;
  uint8_t active_rate_ = 0;
  uint8_t active_idx_ = 0;

  // Per-cell frame pacing, copied out of the accepted CalCmd. gap_us is
  // truncated to whole milliseconds here -- pump()'s only clock is a
  // uint64_t now_ms -- so the shipped plan runs gap_us=2000 (2 ms)
  // deliberately: it lands exactly on this quantization. Lowering gap_us
  // much below ~1000 will silently collapse to "as fast as pump() is
  // called" rather than actually spacing frames tighter, since sub-ms
  // gaps have no representation at this resolution; giving pump() a
  // finer-grained clock is the fix if that's ever needed.
  uint16_t frames_per_cell_ = 0;
  uint32_t settle_ms_ = 0;
  uint32_t gap_ms_ = 0;
  uint16_t cell_seq_ = 0;
  uint64_t settle_deadline_ms_ = 0;
  uint64_t next_send_ms_ = 0;

  std::optional<rc::CalResult> pending_result_;
};

}  // namespace mabur
