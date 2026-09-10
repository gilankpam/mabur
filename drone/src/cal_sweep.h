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
//   3. Idempotent by (nonce, phase). The uplink loses 30-50% of frames, so
//      the GS repeats a CalCmd into the drone's listen window. A repeat of
//      the phase already running (or just finished) must be ignored, not
//      restarted -- restarting would re-zero the cell cursor and throw
//      away partial progress for no reason.
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
  // How long to wait, once a phase finishes, for the GS to either send the
  // next phase's CalCmd or an on_result -- before giving up on the session
  // and restoring power on its own.
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
  // next pump()). A repeat of the (nonce, phase) already running or just
  // completed is ignored outright (constraint 3). A new phase of the
  // session already in flight (matching nonce, different phase) resumes
  // sweeping with a fresh cell cursor built from this command's windows,
  // without re-zeroing the diffs a prior phase of the same session already
  // zeroed.
  void on_cmd(const rc::CalCmd& c, uint64_t now_ms);

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

  // Current phase's cell plan and cursor.
  std::vector<Cell> cells_;
  size_t cursor_ = 0;
  bool cell_entered_ = false;
  uint8_t active_rate_ = 0;
  uint8_t active_idx_ = 0;

  // Per-cell frame pacing, copied out of the accepted CalCmd.
  uint16_t frames_per_cell_ = 0;
  uint32_t settle_ms_ = 0;
  uint32_t gap_ms_ = 0;
  uint16_t cell_seq_ = 0;
  uint64_t settle_deadline_ms_ = 0;
  uint64_t next_send_ms_ = 0;

  std::optional<rc::CalResult> pending_result_;
};

}  // namespace mabur
