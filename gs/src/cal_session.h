#pragma once
// The calibration session brain: owns the sweep plan, tallies received
// frames into measurement cells, drives the coarse -> fine -> result ->
// verify phase transitions, and is the sole place that decides whether the
// GS is allowed to transmit at all while a run is in progress.
// Spec: docs/superpowers/specs/2026-09-10-tx-power-calibration-design.md.
#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "mabur/rc_proto.h"

#include "cal_analysis.h"
#include "cal_plan.h"

namespace maburgs {

struct CalSessionCfg {
  CalThresholds th;
  uint32_t ack_timeout_ms = 3000;
  uint32_t phase_slack_ms = 4000;
  double margin_db = 1.0;
};

class CalSession {
 public:
  enum class State {
    Idle,
    AwaitAck,
    Sweep,
    Analyze,
    Result,
    Verify,
    Done,
    Failed
  };

  explicit CalSession(CalSessionCfg cfg) : cfg_(cfg) {}

  // Refuses (returning false and filling `*err`) unless the link is up, the
  // peer advertised CAP_CALIBRATE, and no session is already running -- the
  // three refusals both live here so `main.cpp` has one call to make.
  bool start(uint32_t vtx_id, uint32_t nonce, uint64_t now_ms,
            std::string* err);

  // The drone accepted the outstanding T_CAL_CMD. Ignored if it names a
  // different nonce or arrives outside AwaitAck -- a late repeat of an ack
  // the session already consumed, or an ack for a session that has since
  // failed/aborted, must not resurrect anything.
  void on_ack(uint32_t nonce, int base_ref_idx, uint64_t now_ms);

  // A sweep-frame arrival. Frames outside the phase currently running, or
  // for an index this session never asked for, are dropped -- silently, by
  // design: a stray frame must never be attributed to a cell it doesn't
  // belong to.
  void on_cal_frame(int card, const mabur::cal::CalFrameInfo& f, int rssi_dbm,
                    bool crc_ok, uint64_t now_ms);

  // The command the GS should (re)send right now, or nullopt if nothing is
  // due. Advances the state machine first (see step()), so this alone can
  // discover an ack timeout.
  std::optional<mabur::rc::CalCmd> due_cmd(uint64_t now_ms);

  // The result ready to send, or nullopt. One-shot: delivered once per
  // session, then the session moves on to silently watching for the
  // drone's self-initiated verify sweep. Also advances the state machine.
  std::optional<mabur::rc::CalResult> due_result(uint64_t now_ms);

  // True whenever a GS transmit would corrupt the measurement: from the
  // moment the drone acknowledges a phase until its planned duration plus
  // the configured slack has elapsed. Every GS transmit path (RCF, DISC
  // keepalive, the calibration commands themselves) gates on this.
  bool radio_silent(uint64_t now_ms) const;

  State state() const { return state_; }

  // Per-rate analysis as it stands right now (coarse-only mid-run, merged
  // once a fine phase has completed) -- for the `status` command and the
  // eventual cal.log, not the wire.
  const std::array<RateWall, 8>& walls() const { return final_walls_; }

  std::string progress() const;

  // Operator or watchdog abort: drops straight back to Idle and reopens the
  // air immediately, from any state.
  void abort(const char* why);

  // Told by main.cpp every tick: whether the RC link is up and whether the
  // peer's last DiscAck carried CAP_CALIBRATE. start() is the only place
  // that reads these.
  void set_peer(bool linked, bool cal_capable);

  double margin_db() const { return cfg_.margin_db; }
  void set_margin_db(double db) { cfg_.margin_db = db; }

  // Test accessors into the currently-live phase's cell tally.
  uint16_t cell_received(uint8_t rate, uint8_t idx, int card) const;
  uint16_t cell_corrupt(uint8_t rate, uint8_t idx) const;

 private:
  // Advances time-driven transitions (ack timeout, phase-end analysis, the
  // verify window closing). Both due_cmd() and due_result() call this
  // before anything else, so neither one is the sole driver of a
  // transition -- main.cpp calls both once per core-loop tick, and the
  // tests exercise them independently.
  void step(uint64_t now_ms);

  // Seeds every (rate, idx) cell a plan names with expected=frames_per_cell,
  // received=0. This is what makes a drone that dies mid-phase read as loss
  // rather than as a shrinking denominator -- the count comes from the plan
  // the GS itself built, never from anything the drone reports.
  void seed_cells(const mabur::rc::CalCmd& cmd);
  void clear_cells();

  // Moves to AwaitAck with `cmd` as the pending command, (re)seeding cells
  // for it. Shared by the coarse and fine phase kick-offs.
  void begin_await(const mabur::rc::CalCmd& cmd, uint64_t now_ms);

  // Moves to Verify: builds the local (never transmitted -- the drone
  // self-initiates this sweep once it applies the result, spec step 9) plan
  // for the park indices, seeds cells for it, and computes how long the GS
  // must stay silent to cover it.
  void begin_verify(uint64_t now_ms);

  // A sweep's listen window has opened: convert this phase's cells to the
  // sorted vectors analyze_rate wants, run the analysis, and decide whether
  // a fine phase follows or the result is ready.
  void finish_phase(uint64_t now_ms);

  // Builds the final CalResult from final_walls_ and moves to Result.
  void finalize_result();

  std::vector<CalCell> sorted_cells(int rate) const;

  void fail(const char* why);

  CalSessionCfg cfg_;
  State state_ = State::Idle;

  uint32_t vtx_id_ = 0;
  uint32_t nonce_ = 0;
  bool linked_ = false;
  bool cal_capable_ = false;
  int base_ref_idx_ = 0;
  const char* fail_reason_ = "";

  // AwaitAck bookkeeping.
  mabur::rc::CalCmd pending_cmd_;
  uint64_t await_start_ms_ = 0;
  uint64_t last_sent_ms_ = 0;
  bool sent_once_ = false;

  // Sweep/Verify bookkeeping. running_phase_ is what on_cal_frame() checks
  // an arriving frame's phase byte against.
  uint8_t running_phase_ = 0;
  uint64_t phase_start_ms_ = 0;
  uint64_t phase_end_ms_ = 0;

  // Sparse per-rate cell tally for whichever phase is currently running.
  // Cleared and reseeded at each phase boundary -- once a phase's cells
  // have been folded into coarse_walls_/final_walls_ the raw tally is no
  // longer needed.
  std::array<std::map<uint8_t, CalCell>, 8> cells_;
  // Raw per-cell-per-card RSSI samples, median-reduced into CalCell::rssi_dbm
  // at phase end (sorted_cells()).
  std::array<std::map<uint8_t, std::array<std::vector<int>, 2>>, 8> rssi_raw_;

  std::array<RateWall, 8> coarse_walls_{};
  std::array<RateWall, 8> final_walls_{};

  mabur::rc::CalResult pending_result_;
  bool result_ready_ = false;
  // Park index per rate (wall - margin), -1 where undetermined. Feeds both
  // CalResult and the locally-built verify plan.
  std::array<int, 8> pending_park_{};
};

}  // namespace maburgs
