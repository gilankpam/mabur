#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "config.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"

namespace mabur {

// The resolved operating point: the 2-slot ladder ([0]=BASE at profile_mcs
// -1, [1]=ENH at profile_mcs — spec 2026-08-29-airtime-balance-uep §2), the
// per-stream literal FEC air overhead PAIR (Task 6, RC_VERSION 5: the
// single fec_overhead scalar split into fec_ov_base/fec_ov_enh — fixed
// per-rung pairs, spec 2026-08-30-same-rate-fixed-pairs; applied directly
// to the UEP layers via apply_op_to_uep, no ladder translation), per-layer
// shed flags (failsafe-forced OR local congestion-directed; shed[1] is the
// enh shed, the old shed[3] — the old reserved layer and its shed[2] slot
// are gone), and
// a generation counter bumped only when a *new* operating point
// (ladder/FEC) is applied (BOOT/DISC/RCF/failsafe entry) — NOT on every
// publish. Congestion shed re-applies the *current* op (same ladder/FEC)
// with updated shed flags and publishes a fresh AppliedOp WITHOUT bumping
// generation, so consumers MUST NOT use generation to detect "did a new
// AppliedOp get published" — every apply_op() call (via Actuator::apply_op)
// constructs and stores a brand new object, so callers that need "did the
// published object change" should identity-compare the shared_ptr they last
// observed against the newly loaded one instead.
struct AppliedOp {
  std::array<rc::LayerTxSpec, 2> ladder;
  double fec_ov_base = 2.0;
  double fec_ov_enh = 2.0;
  std::array<bool, 2> shed = {false, false};
  uint64_t generation = 0;
};

// Volatile per-layer overhead override (bench sweeps, set via the debug
// HTTP :8301 POST /venc/set?ov_base_pct=N / ov_enh_pct=N; -1 = off, both
// must be >= 0 to take effect). main.cpp's hot loop applies the forced pair
// to the UEP layers directly (the op pair loses), and run_bitrate_policy
// reads the same pair so its target describes what is ACTUALLY flying
// rather than the commanded op. Not persisted; a daemon restart clears it.
//
// Plain atomics, no lock: the HTTP thread writes, the hot loop and the
// agent thread read. Torn reads are impossible (each field loaded
// independently) and a stale value by one tick is harmless for a rate
// target. May be null (tests), in which case no override is ever armed.
//
// This struct is all that survives of AirFeedOut. AirFeed itself — the
// measurement half of the deleted AirBalancer — was removed on 2026-09-01
// along with the bitrate blend that was its only consumer; see
// run_bitrate_policy.
struct OvOverride {
  std::atomic<int> ovr_base_pct{-1};
  std::atomic<int> ovr_enh_pct{-1};
};

// Everything RcAgent does to the outside world funnels through this
// interface, so tests can substitute a recording double instead of driving
// real radio/encoder hardware.
class Actuator {
 public:
  virtual ~Actuator() = default;
  virtual void apply_op(const AppliedOp&) = 0;                     // ladder+FEC+shed
  virtual void send_control(const std::vector<uint8_t>& body) = 0;  // DISC_ACK
  // The two encoder-parameter verbs report whether the encoder actually
  // took the value. RcAgent latches its "last commanded" state ONLY on
  // true, so a transient failure is re-sent on the next policy tick
  // instead of leaving the encoder silently diverged from the ladder for
  // the rest of the flight (the waybeam-wedge failure mode, in-process
  // edition — memory: waybeam-bitrate-wedge).
  virtual bool set_bitrate_kbps(int) = 0;
  virtual bool set_roi_qp(int) = 0;
  // No return: an IDR is a one-shot request with no latched state to keep
  // consistent, and the pacer has already spent its budget. A failed one is
  // re-raised by whatever produced it (the next chain break, or the next
  // RCF that re-enters LINKED).
  virtual void request_idr() = 0;
};

// Radio-side health signals sampled once per tick. thermal_delta is
// telemetry-only (nothing acts on it); tx_drops and txq_depth/txq_cap feed
// the congestion-shed policy (run_congestion_guard).
struct RadioHealth {
  int thermal_delta = 0;
  uint64_t tx_drops = 0;  // TxStats::failed: USB bulk-OUT failures
  // TxQueue occupancy at the tick (main.cpp). 0/0 when unknown (tests,
  // pre-radio ticks) -- never reads as pressure.
  size_t txq_depth = 0;
  size_t txq_cap = 0;
};

// Control-plane state machine: BOOT -> RENDEZVOUS -> LINKED <-> FAILSAFE.
// Consumes inbound RC frames (RCF feedback, DISC discovery beacons) and a
// periodic tick (which also carries radio health for the congestion
// guard), and drives an Actuator with the resolved operating point. All
// timing is driven by the `now_ms` parameters callers pass in — RcAgent
// makes no real-time calls of its own, so tests can simulate any clock.
class RcAgent {
 public:
  enum class State { BOOT, RENDEZVOUS, LINKED, FAILSAFE };

  // ovr is the debug-HTTP per-layer overhead override (bench sweeps); may
  // be null (tests), in which case no override is ever armed and the
  // bitrate policy always builds its target from the commanded pair.
  RcAgent(const Config& cfg, Actuator& act, OvOverride* ovr = nullptr);

  // Parses `body` as an RC frame (RCF or DISC; anything else, or a frame
  // failing CRC/vtx_id match, is silently ignored) and applies its effect.
  void on_rc_frame(const uint8_t* body, size_t len, uint64_t now_ms);

  // The encoder lost a reference frame (a ring-full drop ate it), so the
  // decode chain downstream is broken until an IDR re-seeds it. Called from
  // the venc encoder thread (venc_core.h's on_chain_break) — the ONLY
  // cross-thread entry point on RcAgent, which is why it is a bare atomic
  // set and nothing else. The request is consumed on the next tick(), on the
  // agent thread, where it goes through the same IDR pacer as every other
  // producer.
  void note_chain_break();

  // Advances the failsafe/rendezvous timers and re-evaluates the
  // congestion guard against the given health sample. On BOOT, the first
  // call applies the MAX_RANGE op and transitions to RENDEZVOUS.
  void tick(uint64_t now_ms, const RadioHealth& health);

  State state() const { return state_; }
  const AppliedOp& current() const { return applied_; }

  // Telemetry accessors (spec 2026-07-26 drone-telemetry): read-only
  // snapshots of RcAgent-internal state the T_TELEM collector needs but
  // that isn't otherwise exposed. All same-thread reads (the agent thread
  // owns both RcAgent and the telemetry collector call site in main.cpp).
  bool failsafe_shed() const { return failsafe_shed_; }
  bool have_feedback() const { return have_last_fb_; }
  uint64_t last_feedback_ms() const { return last_fb_ms_; }
  uint64_t rcf_accepted() const { return rcf_accepted_; }
  // link-rtt: seq of the RCF that last_feedback_ms/rcf_age_ms age against.
  // Empty whenever the seq window is reset (DISC re-establish, failsafe) —
  // in that state last_fb_ms_ was refreshed by a non-RCF event and echoing
  // a stale seq would let the GS fabricate an RTT sample from the wrong
  // send time.
  std::optional<uint16_t> last_feedback_seq() const {
    if (!have_last_seq_) return std::nullopt;
    return last_seq_;
  }

  // True iff the last accepted RCF carried probe3 (an s3-only MCS probe) —
  // cleared the moment a FAILSAFE/max-range apply takes over (a degraded or
  // lost link must never report itself as still probing) or a follow-up RCF
  // arrives without the flag. Spec 2026-08-05 s3-probe-promote.
  bool probing() const { return probe3_active_; }

  // Latched on a BOOT/RENDEZVOUS -> LINKED transition — the process-(re)start
  // link-up, when frames encoded so far never reached the air and the GS may
  // hold a stale frame-id cursor. The caller consumes it to re-mark the frame
  // discontinuity window (FramePipeline::mark_discontinuity). Deliberately
  // NOT latched on FAILSAFE -> LINKED: a routine RF flap re-basing the GS's
  // id space would evict its in-flight frames for nothing.
  bool take_link_established() {
    bool v = link_established_;
    link_established_ = false;
    return v;
  }

 private:
  const Config& cfg_;
  Actuator& act_;
  OvOverride* ovr_;  // may be null — see the constructor comment
  State state_ = State::BOOT;
  bool link_established_ = false;  // see take_link_established()

  AppliedOp applied_;

  uint64_t last_fb_ms_ = 0;
  bool have_last_fb_ = false;

  uint16_t last_seq_ = 0;
  bool have_last_seq_ = false;

  // Cumulative count of RCFs accepted (fresh + matching vtx_id) — feeds
  // Telem.rcf_rx. Never reset (a session-boundary reset would make the GS's
  // rate computation, which is over a measured interval, ambiguous).
  uint64_t rcf_accepted_ = 0;

  // Mirrors the last accepted RCF's probe3 bit — see probing(). Cleared in
  // apply_max_range() (FAILSAFE/boot never probes) and recomputed at the top
  // of every RCF apply.
  bool probe3_active_ = false;

  // IDR policy state (spec 2026-08-28 venc-foldin §4). Every IDR producer
  // — the GS-driven RCF-after-failsafe path and the encoder's chain-break
  // signal — funnels through idr_due(); nothing else may call
  // act_.request_idr(). chain_break_pending_ is the venc thread's handoff
  // (see note_chain_break); the two timestamps are agent-thread-only.
  std::atomic<bool> chain_break_pending_{false};
  // have_* companions rather than a 0 sentinel (the file's own idiom, cf.
  // have_last_fb_/have_last_bitrate_eval_): now_ms is a caller-supplied
  // clock that legitimately starts at 0, and a first IDR at t=0 must still
  // arm the 100 ms floor.
  uint64_t last_idr_ms_ = 0;
  bool have_last_idr_ = false;
  uint64_t last_chain_idr_ms_ = 0;
  bool have_last_chain_idr_ = false;
  bool idr_due(uint64_t now_ms, bool chain);

  // Bitrate policy state.
  int last_bitrate_kbps_ = 0;
  bool have_last_bitrate_ = false;
  uint64_t last_bitrate_eval_ms_ = 0;
  bool have_last_bitrate_eval_ = false;
  bool roi_low_ = false;
  // Set by run_bitrate_policy() whenever set_bitrate_kbps()/set_roi_qp()
  // returns false, cleared when both verbs are in the state the policy
  // wants. Drives the per-tick retry half of the periodic re-assert (see
  // kReassertMs / tick()).
  bool verb_apply_failed_ = false;
  // "run_bitrate_policy() already ran at this now_ms" — the tick-level
  // re-assert below consults it so a tick that has ALREADY run the policy
  // (a max-range/failsafe entry on this very tick, or an RCF the agent loop
  // drained at the same millisecond) does not immediately run it a second
  // time. At most one policy run per distinct now_ms.
  uint64_t last_policy_ms_ = 0;
  bool have_last_policy_ = false;

  // Congestion-shed state.
  int shed_level_ = 0;
  uint64_t last_drop_rise_ms_ = 0;
  bool have_last_drop_rise_ = false;
  uint64_t last_tx_drops_ = 0;
  bool have_last_tx_drops_ = false;

  // True for as long as MAX_RANGE is the operating point — i.e. RENDEZVOUS
  // (including BOOT's initial apply) or FAILSAFE — set in apply_max_range()
  // and cleared the moment a resolved DISC/RCF op takes the agent back to
  // LINKED (apply_ladder_op()). OR'd with the congestion-level shed
  // whenever (re)building AppliedOp.shed[1] (the enh layer — the old
  // shed[2]/[3] pair collapsed to this single slot when the reserved layer
  // was deleted), so a later congestion-guard reapply (which recomputes
  // shed from shed_level_ alone) can never silently drop the MAX_RANGE-
  // forced shed — most importantly, FAILSAFE's.
  bool failsafe_shed_ = false;

  // Periodic re-assert interval, ms. run_bitrate_policy() only pushes on
  // CHANGE, so without this nothing ever restates the commanded rate: a
  // verb that failed during a FAILSAFE entry stayed unrepaired until the
  // next RCF or rendezvous (up to rendezvous_ms, 30 s), and anything that
  // moved the encoder behind RcAgent's back — the debug endpoint's
  // POST /venc/set, an in-process re-init — won until the ladder happened
  // to change rung. 5 s is chosen against those two: short enough that a
  // stuck bitrate costs at most one GOP-ish window of wrong rate rather
  // than a flight, long enough that the re-apply is ~1/50 of the tick rate
  // (tick_ms 100) and cannot itself become a source of MI-call churn.
  static constexpr uint64_t kReassertMs = 5000;

  void apply_max_range(uint64_t now_ms);
  void apply_ladder_op(const std::array<rc::LayerTxSpec, 2>& ladder,
                        double ov_base, double ov_enh);
  void reapply_with_shed();
  void run_bitrate_policy(uint64_t now_ms, bool force);
  void run_congestion_guard(uint64_t now_ms, const RadioHealth& health);
  rc::DiscAck make_disc_ack(uint32_t nonce, uint16_t seq) const;
};

}  // namespace mabur
