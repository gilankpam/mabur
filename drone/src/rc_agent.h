#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "config.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"

namespace mabur {

// The resolved operating point: the 4-rung ladder, the commanded FEC
// overhead (the base scale that stream 1's uep_layer_overhead rescales),
// per-layer shed flags (failsafe-forced OR local congestion-directed), and
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
  std::array<rc::LayerTxSpec, 4> ladder;
  double fec_overhead = 1.0;
  std::array<bool, 4> shed = {false, false, false, false};
  uint64_t generation = 0;
};

// Everything RcAgent does to the outside world funnels through this
// interface, so tests can substitute a recording double instead of driving
// real radio/encoder hardware.
class Actuator {
 public:
  virtual ~Actuator() = default;
  virtual void apply_op(const AppliedOp&) = 0;                     // ladder+FEC+shed
  virtual void send_control(const std::vector<uint8_t>& body) = 0;  // DISC_ACK
  virtual void set_bitrate_kbps(int) = 0;
  virtual void set_roi_qp(int) = 0;
  virtual void request_idr() = 0;
};

// Radio-side health signals sampled once per tick. thermal_delta is
// telemetry-only (nothing acts on it); tx_drops feeds the congestion-shed
// policy.
struct RadioHealth {
  int thermal_delta = 0;
  uint64_t tx_drops = 0;
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

  RcAgent(const Config& cfg, Actuator& act);

  // Parses `body` as an RC frame (RCF or DISC; anything else, or a frame
  // failing CRC/vtx_id match, is silently ignored) and applies its effect.
  void on_rc_frame(const uint8_t* body, size_t len, uint64_t now_ms);

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

  // Bitrate policy state.
  int last_bitrate_kbps_ = 0;
  bool have_last_bitrate_ = false;
  uint64_t last_bitrate_eval_ms_ = 0;
  bool have_last_bitrate_eval_ = false;
  bool roi_low_ = false;

  // Congestion-shed state.
  int shed_level_ = 0;
  uint64_t last_drop_rise_ms_ = 0;
  bool have_last_drop_rise_ = false;
  uint64_t last_tx_drops_ = 0;
  bool have_last_tx_drops_ = false;

  // True for as long as MAX_RANGE is the operating point — i.e. RENDEZVOUS
  // (including BOOT's initial apply) or FAILSAFE — set in apply_max_range()
  // and cleared the moment a resolved DISC/RCF op takes the agent back to
  // LINKED (apply_ladder_op()). OR'd with the congestion-level sheds
  // whenever (re)building AppliedOp.shed[2]/[3], so a later congestion-guard
  // reapply (which recomputes shed from shed_level_ alone) can never
  // silently drop the MAX_RANGE-forced shed — most importantly, FAILSAFE's.
  bool failsafe_shed_ = false;

  void apply_max_range(uint64_t now_ms);
  void apply_ladder_op(const std::array<rc::LayerTxSpec, 4>& ladder,
                        double fec_overhead);
  void reapply_with_shed();
  void run_bitrate_policy(uint64_t now_ms, bool force);
  void run_congestion_guard(uint64_t now_ms, const RadioHealth& health);
  rc::DiscAck make_disc_ack(uint32_t nonce, uint16_t seq) const;
};

}  // namespace mabur
