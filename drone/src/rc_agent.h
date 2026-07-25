#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "config.h"
#include "mabur/profile.h"

namespace mabur {

// One fully-resolved operating point ready to hand to the radio/encoder:
// the 4-rung PHY ladder, the FEC overhead scalar (fed to
// uep_layer_overhead), the commanded TX power offset, per-layer shed flags
// (failsafe-forced OR local congestion-directed), and a generation counter
// bumped only when a *new* operating point (ladder/FEC) is applied
// (BOOT/DISC/RCF/failsafe entry) — NOT on every publish. Thermal derate and
// congestion shed re-apply the *current* op (same ladder/FEC) with updated
// pwr_offset_qdb/shed and publish a fresh AppliedOp WITHOUT bumping
// generation, so consumers MUST NOT use generation to detect "did a new
// AppliedOp get published" — every apply_op() call (via Actuator::apply_op)
// constructs and stores a brand new object, so callers that need "did the
// published object change" should identity-compare the shared_ptr they last
// observed against the newly loaded one instead.
struct AppliedOp {
  std::array<rc::LayerTxSpec, 4> ladder;
  double fec_overhead = 1.0;
  // Commanded TX power expressed as a qdB offset from the calibrated
  // wall-equalized baseline; 0 = full legal power, negative = backed off.
  // Never positive (a plan's max legal offset is always 0).
  int pwr_offset_qdb = 0;
  std::array<bool, 4> shed = {false, false, false, false};
  uint64_t generation = 0;
};

// Everything RcAgent does to the outside world funnels through this
// interface, so tests can substitute a recording double instead of driving
// real radio/encoder hardware.
class Actuator {
 public:
  virtual ~Actuator() = default;
  virtual void apply_op(const AppliedOp&) = 0;                     // ladder+power+FEC+shed
  virtual void send_control(const std::vector<uint8_t>& body) = 0;  // DISC_ACK
  virtual void set_bitrate_kbps(int) = 0;
  virtual void set_roi_qp(int) = 0;
  virtual void request_idr() = 0;
};

// Radio-side health signals sampled once per tick and fed to RcAgent's
// thermal guard / congestion-shed policies.
struct RadioHealth {
  int thermal_delta = 0;
  uint64_t tx_drops = 0;
};

// Control-plane state machine: BOOT -> RENDEZVOUS -> LINKED <-> FAILSAFE.
// Consumes inbound RC frames (RCF feedback, DISC discovery beacons) and a
// periodic tick (which also carries radio health for the thermal/congestion
// guards), and drives an Actuator with the resolved operating point. All
// timing is driven by the `now_ms` parameters callers pass in — RcAgent
// makes no real-time calls of its own, so tests can simulate any clock.
class RcAgent {
 public:
  enum class State { BOOT, RENDEZVOUS, LINKED, FAILSAFE };

  RcAgent(const Config& cfg, Actuator& act);

  // Parses `body` as an RC frame (RCF or DISC; anything else, or a frame
  // failing CRC/vtx_id match, is silently ignored) and applies its effect.
  void on_rc_frame(const uint8_t* body, size_t len, uint64_t now_ms);

  // Advances the failsafe/rendezvous timers and re-evaluates the thermal
  // and congestion guards against the given health sample. On BOOT, the
  // first call applies the MAX_RANGE op and transitions to RENDEZVOUS.
  void tick(uint64_t now_ms, const RadioHealth& health);

  State state() const { return state_; }
  const AppliedOp& current() const { return applied_; }

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
  // Last GS-commanded qdB power offset (pre-thermal-derate). MAX_RANGE
  // (apply_max_range) always sets this to 0 (full legal power); an RCF with
  // a real (non-PWR_NO_CHANGE) offset byte sets it to
  // clamp(decode_pwr_offset_qdb(byte), cfg_.radio.min_offset_qdb, 0); a DISC
  // row's pwr_offset_qdb goes through the same clamp. PWR_NO_CHANGE leaves
  // it untouched.
  int commanded_offset_qdb_ = 0;

  uint64_t last_fb_ms_ = 0;
  bool have_last_fb_ = false;

  uint16_t last_seq_ = 0;
  bool have_last_seq_ = false;

  // Bitrate policy state.
  int last_bitrate_kbps_ = 0;
  bool have_last_bitrate_ = false;
  uint64_t last_bitrate_eval_ms_ = 0;
  bool have_last_bitrate_eval_ = false;
  bool roi_low_ = false;

  // Thermal guard state.
  int thermal_derate_ = 0;

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
  void apply_ladder_op(const std::array<rc::LayerTxSpec, 4>& ladder, int pwr_offset_qdb,
                        double fec_overhead);
  void reapply_with_derate_and_shed();
  void run_bitrate_policy(uint64_t now_ms, bool force);
  void run_thermal_guard(const RadioHealth& health);
  void run_congestion_guard(uint64_t now_ms, const RadioHealth& health);
};

}  // namespace mabur
