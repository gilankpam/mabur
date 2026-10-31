#pragma once
// The calibration session brain on the GS. A run is an ordered list of
// sweeps the GS composed (power and/or capacity, at 20 and/or 40 MHz); per
// sweep the session commands the drone, waits for the ack, keeps the GS
// radio silent while the drone airs stamped frames, tallies those frames,
// analyses them when the drone says it is done (or the window closes),
// sends the result for the drone to apply, verifies a power apply at the
// parked indices, and moves on to the next sweep.
//
// It talks to the drone only through plain structs (mabur::cal::CalMsg):
// the caller encodes due_mav()'s output onto the MAVLink tunnel's cal lane
// and decodes the drone's replies into on_mav(). It is also the sole place
// that decides whether the GS may transmit while a run is in progress
// (radio_silent()). No maburgs-only include, so the web GS can link it.
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mabur/cal_proto.h"
#include "mabur/cal_wire.h"
#include "mabur/rc_proto.h"

#include "cal_analysis.h"

namespace maburgs {

// Forward-declared: the session only holds pointers to these (optional
// sinks/collaborators, not owned).
class CalLog;
class CalPicker;

struct CalSessionCfg {
  CalThresholds th;
  // How long a SweepCmd is repeated without an ack before the run fails.
  uint32_t ack_timeout_ms = 3000;
  // How long past a sweep's planned end the GS keeps listening (and stays
  // silent) for late frames and the drone's SweepDone; also pads the
  // verify window.
  uint32_t phase_slack_ms = 1000;
  // After a capacity sweep's SweepDone, how long (in session time, with no
  // frame of the sweep handled at all) its frames must have been quiet
  // before the sweep closes. The drone sends SweepDone when its last
  // blast returns, ahead of the frames its TX queue is still airing; a
  // saturated stream never gaps this long, the end of the drain does.
  // Bounded by the listen window like everything else in AwaitDone.
  uint32_t tail_quiet_ms = 50;
  // How long an ApplyCmd is repeated without an Applied before the run
  // fails.
  uint32_t apply_timeout_ms = 5000;
  // Bound on the channel pick (measure round + relocate confirm).
  uint32_t pick_timeout_ms = 8000;
  // The GS's copy of the wall margin, used only to predict the verify
  // parks (mabur::park_rel_for, the rule the drone uses with its own
  // radio.wall_margin_db) and for cal.log's R line.
  double margin_db = 1.0;
};

// What one width's sweeps produced so far in this run.
struct WidthResult {
  bool have_walls = false;  // a power sweep at this width was analysed
  bool have_eff = false;    // a capacity sweep was analysed with its SweepDone
  // A capacity sweep ran but its SweepDone never arrived: offered and
  // blocked_pct are unknown, so no point can be told apart from tx_bound
  // and nothing is written.
  bool eff_unknown = false;
  bool applied = false;      // the latest apply at this width was adopted
  std::string apply_reason;  // the drone's reason on the latest apply
  std::array<RateWall, 8> walls{};
  std::array<CapResult, 8> cap{};
  std::array<uint16_t, 8> eff_x1000{};  // what was sent; 0 = leave
  // Verify parks (relative index) per rate; cal::kWallLeave = not parked.
  std::array<int, 8> park{mabur::cal::kWallLeave, mabur::cal::kWallLeave,
                          mabur::cal::kWallLeave, mabur::cal::kWallLeave,
                          mabur::cal::kWallLeave, mabur::cal::kWallLeave,
                          mabur::cal::kWallLeave, mabur::cal::kWallLeave};
  // Best-card verify delivery per rate; -1 = the rate was not verified.
  std::array<int, 8> verify_pct{-1, -1, -1, -1, -1, -1, -1, -1};
};

struct CalSnapshot;

class CalSession {
 public:
  // How many copies of each Abort due_mav() offers.
  static constexpr int kAbortCopies = 3;

  // Picking runs the channel pick before anything reaches the drone; it is
  // outside running(). Analyze is transient (entered and left within one
  // call) and only ever visible as a state name in passing.
  enum class State { Idle, Picking, AwaitAck, Sweep, AwaitDone, Analyze, Apply, Verify, Done, Failed };

  // `log` and `picker` are optional and not owned; both must outlive the
  // session. A null picker runs without a channel pick.
  explicit CalSession(CalSessionCfg cfg, CalLog* log = nullptr, CalPicker* picker = nullptr)
      : cfg_(cfg), log_(log), picker_(picker) {}

  // Starts a run over `sweeps`. The list is sorted per width ascending,
  // power before capacity (so a capacity sweep measures at the power its
  // own width's walls just parked), and deduplicated. Refuses (false, with
  // `*err`) an empty list, a Verify or non-20/40 entry, a run already in
  // progress (Picking included), a link that is down, or a peer without
  // CAP_CALIBRATE.
  bool start(uint32_t nonce, std::vector<mabur::cal::SweepSpec> sweeps, uint64_t now_ms,
             std::string* err);

  // The drone's SweepAck, SweepDone and Applied. Anything naming another
  // nonce, another sweep or arriving in the wrong state is ignored. A
  // SweepDone closes a power sweep at once and a capacity sweep once its
  // frames have been quiet for tail_quiet_ms.
  void on_mav(const mabur::cal::CalMsg& m, uint64_t now_ms);

  // The caller's once-per-pass tick, called after the pass has handled the
  // frames it drained; the capacity tail's quiet is measured from it. The
  // message to send right now, or nullopt: a SweepCmd (repeated every
  // 200 ms until acked), an ApplyCmd (repeated every 200 ms until
  // Applied), or the Abort that closes the drone's run after Done, a
  // failure or abort() -- kAbortCopies copies 200 ms apart, since the
  // uplink is lossy and the drone ignores a nonce it has already closed.
  // The caller sends it as is, never gated on
  // radio_silent(): the session only offers something when the air is the
  // GS's to use. Advances time-driven transitions first.
  std::optional<mabur::cal::CalMsg> due_mav(uint64_t now_ms);

  // A received calibration frame. `air_bytes` is the frame's size on the
  // air (dot11 header + body + FCS); capacity sweeps sum it into the
  // delivered air bits. A frame counts only if its kind is the running
  // sweep's (Verify during the verify pass), its width is the sweep's and
  // its flags are the ones the drone's ack announced; anything else is
  // counted in the snapshot's stamp_mismatch and never tallied. Before the
  // ack, the first CRC-clean frame of the commanded kind and width stands
  // in for it and its flags become the run's. `now_ms` is the frame's own
  // receive stamp (the RX thread's, not the core loop's tick): a capacity
  // point is timed by when its frames arrived.
  void on_cal_frame(int card, const mabur::cal::CalFrameInfo& f, int rssi_dbm, bool crc_ok,
                    size_t air_bytes, uint64_t now_ms);

  // True whenever a GS transmit would corrupt the measurement: from the
  // ack (or the frame that stood in for it) until the sweep's listen
  // window closes, and through the verify pass.
  bool radio_silent(uint64_t now_ms) const;

  // AwaitAck through Verify: a run owns the air. Picking is not included
  // -- the channel machinery must keep running while the pick measures.
  bool running() const;
  // running()'s set of states, for a caller holding only a snapshot.
  static bool is_running(State st);

  // Abandons the run from any state: the Aborts go out through due_mav()
  // (if the run already reached the drone), a pick in progress is
  // cancelled, the air reopens and the session goes Idle. The snapshot
  // keeps `why` as its fail_reason.
  void abort(const char* why);

  // Told every tick: link state, whether the peer advertised
  // CAP_CALIBRATE, and the pair's tag context.
  void set_peer(bool linked, bool cal_capable, mabur::rc::TagCtx ctx = {});

  // The tag context every cal message of the current run is packed under:
  // set_peer()'s ctx as of the moment the run first commands the drone
  // (start() without a pick, the end of the pick otherwise), frozen for
  // the run -- the drone latches the pair its first SweepCmd arrived
  // under and verifies the rest of the run against it.
  const mabur::rc::TagCtx& tag_ctx() const { return tag_ctx_; }

  double margin_db() const { return cfg_.margin_db; }
  State state() const { return state_; }
  CalSnapshot snapshot() const;

  // The run's sweep list after sorting and dedup.
  const std::vector<mabur::cal::SweepSpec>& sweeps() const { return sweeps_; }

  // Test accessor: CRC-clean frames tallied on `card` into the running
  // power sweep's cell (rate, idx).
  uint16_t cell_received(uint8_t rate, int idx, int card) const;

 private:
  // The session's clock: the latest stamp any entry point has seen. The
  // receive path stamps frames and replies on its own thread, so a stamp
  // can be older than a tick already seen; every entry point advances this
  // first and works from it, so elapsed-time checks never underflow.
  uint64_t advance(uint64_t now_ms) {
    if (now_ms > now_) now_ = now_ms;
    return now_;
  }
  void step(uint64_t now_ms);
  void step_picking(uint64_t now_ms);
  void finish_pick(const char* pick, uint64_t now_ms);
  void begin_sweep(size_t i, uint64_t now_ms);
  void on_accepted(uint8_t flags, uint16_t start_in_ms, uint64_t now_ms);
  void finish_sweep(uint64_t now_ms);
  bool analyze_power();            // true: an apply is due
  bool analyze_capacity_sweep();   // true: an apply is due
  void begin_apply(uint64_t now_ms);
  void on_applied(const mabur::cal::Applied& a, uint64_t now_ms);
  void begin_verify(uint64_t now_ms);
  void finish_verify(uint64_t now_ms);
  void next_sweep(uint64_t now_ms);
  void fail(const std::string& why);
  void queue_abort();
  void tally_power(int card, const mabur::cal::CalFrameInfo& f, int rssi_dbm, bool crc_ok);
  void tally_capacity(int card, const mabur::cal::CalFrameInfo& f, int rssi_dbm, bool crc_ok,
                      size_t air_bytes, uint64_t rx_ms);
  // True once no capacity frame has reached the session for tail_quiet_ms
  // of session time (a power sweep has no tail: always true).
  bool tail_quiet(uint64_t now_ms) const;
  void tally_verify(int card, const mabur::cal::CalFrameInfo& f, bool crc_ok);
  int cells_per_row() const;
  WidthResult& width_result();
  bool has_power20() const;

  CalSessionCfg cfg_;
  CalLog* log_ = nullptr;
  CalPicker* picker_ = nullptr;

  State state_ = State::Idle;
  uint32_t nonce_ = 0;
  bool linked_ = false;
  bool cal_capable_ = false;
  mabur::rc::TagCtx peer_ctx_{};
  mabur::rc::TagCtx tag_ctx_{};
  std::string fail_reason_;

  std::vector<mabur::cal::SweepSpec> sweeps_;
  size_t sweep_i_ = 0;
  mabur::cal::SweepCmd cmd_{};  // the running sweep's command

  // Picking.
  uint64_t now_ = 0;  // see advance()
  uint64_t pick_start_ms_ = 0;
  bool reloc_ordered_ = false;
  uint8_t reloc_to_ = 0;
  uint8_t channel_ = 0;
  std::string pick_note_;
  const char* pick_ = "pinned";  // cal.log R's pick token
  bool run_logged_ = false;      // R written (at the first accepted ack)

  // AwaitAck / Apply repeats, Abort.
  uint64_t phase_start_ms_ = 0;  // entered AwaitAck / Apply
  uint64_t last_sent_ms_ = 0;
  bool sent_once_ = false;
  bool reached_drone_ = false;  // at least one SweepCmd was offered
  // The Abort still owed to the drone, and how many more copies of it.
  std::optional<mabur::cal::Abort> pending_abort_;
  int aborts_left_ = 0;
  uint64_t next_abort_ms_ = 0;

  // Sweep / AwaitDone / Verify windows.
  uint8_t ack_flags_ = 0;
  uint64_t sweep_end_ms_ = 0;   // planned end: AwaitDone opens here
  uint64_t window_end_ms_ = 0;  // AwaitDone (or Verify) closes here
  std::optional<mabur::cal::SweepDone> done_;
  uint32_t stamp_mismatch_ = 0;
  uint32_t cell_i_ = 0;

  // Power tally, [rate][idx - idx_lo]; RSSI samples per card, median at
  // analysis.
  std::array<std::vector<CalCell>, 8> cells_;
  std::array<std::vector<std::array<std::vector<int>, 2>>, 8> rssi_;
  // Capacity tally per rate.
  std::array<CapCell, 8> cap_;
  std::array<std::array<std::vector<int>, 2>, 8> cap_rssi_;
  // Session time a capacity frame was last handled (tail_quiet): the clock
  // at its tally, raised to the next due_mav tick (cap_fresh_ until then).
  uint64_t last_cap_seen_ms_ = 0;
  bool cap_fresh_ = false;
  // Verify tally per rate (best card read at the close) and the index the
  // first frame of each rate arrived at.
  std::vector<std::pair<uint8_t, int>> verify_cells_;
  std::array<std::array<uint16_t, 2>, 8> verify_recv_{};
  std::array<int, 8> verify_seen_idx_{};
  std::array<bool, 8> verify_seen_{};

  mabur::cal::ApplyCmd apply_{};
  bool apply_wrote_walls_ = false;

  std::array<WidthResult, 2> width_{};
};

struct CalSnapshot {
  CalSession::State state = CalSession::State::Idle;
  size_t sweep_i = 0, sweep_n = 0;
  mabur::cal::SweepSpec cur{mabur::cal::Kind::Power, 20};
  // Progress through the running sweep's (or verify pass's) cells: the
  // cell of the latest tallied frame, 1-based, out of the sweep's total.
  uint32_t cell_i = 0, cell_n = 0;
  uint8_t channel = 0;
  std::string pick_note;
  uint8_t flags = 0;  // the stamp flags the drone's ack announced
  uint32_t stamp_mismatch = 0;  // frames dropped for a kind/width/flags mismatch
  std::array<WidthResult, 2> width;  // [0] = 20 MHz, [1] = 40 MHz
  std::string fail_reason;
};

}  // namespace maburgs
