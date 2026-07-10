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
// uep_layer_overhead), the commanded TX power index, per-layer shed flags
// (failsafe-forced OR local congestion-directed), and a generation counter
// bumped only when a *new* operating point (ladder/FEC) is applied
// (BOOT/DISC/RCF/failsafe entry) — NOT on every publish. Thermal derate and
// congestion shed re-apply the *current* op (same ladder/FEC) with updated
// pwr_idx/shed and publish a fresh AppliedOp WITHOUT bumping generation, so
// consumers MUST NOT use generation to detect "did a new AppliedOp get
// published" — every apply_op() call (via Actuator::apply_op) constructs and
// stores a brand new object, so callers that need "did the published object
// change" should identity-compare the shared_ptr they last observed against
// the newly loaded one instead.
struct AppliedOp {
  std::array<rc::LayerTxSpec, 4> ladder;
  double fec_overhead = 1.0;
  int pwr_idx = 63;
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

 private:
  const Config& cfg_;
  Actuator& act_;
  State state_ = State::BOOT;

  AppliedOp applied_;
  int commanded_pwr_idx_ = 63;  // last GS/MAX_RANGE-commanded power (pre-thermal-derate)

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
  void apply_ladder_op(const std::array<rc::LayerTxSpec, 4>& ladder, int pwr_idx,
                        double fec_overhead);
  void reapply_with_derate_and_shed();
  void run_bitrate_policy(uint64_t now_ms, bool force);
  void run_thermal_guard(const RadioHealth& health);
  void run_congestion_guard(uint64_t now_ms, const RadioHealth& health);
};

}  // namespace mabur
