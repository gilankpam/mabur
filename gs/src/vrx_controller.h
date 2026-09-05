#pragma once

#include <array>
#include <optional>
#include <vector>

#include "mabur/rc_proto.h"

#include "ladder_controller.h"
#include "op_point.h"
#include "rendezvous.h"

namespace maburgs {

struct VrxCfg {
  uint32_t vtx_id = 1;
  uint8_t op_channel = 149;
  int feedback_ms = 100;
  int beacon_keepalive_ms = 1000;
  // Keep-alive DISC cadence while no DiscAck has ever been accepted
  // (peer_acked() false): the GS is blind to peer caps and its video tail
  // is gated off, so ask fast. Relaxes to beacon_keepalive_ms after the
  // first accept. Stale-caps fix, 2026-08-28.
  int unacked_keepalive_ms = 250;
  // Measured-loss ladder controller config (see LinkCfg::ladder_cfg,
  // config.h). Consulted every tick unless pinned.
  LadderCfg ladder;
  // Static-link pin: mcs >= 0 bypasses the adaptive controller (see
  // LinkCfg::static_mcs). overhead pair used only when pinned (from
  // LinkCfg::static_overhead_base/enh).
  int pin_mcs = -1;
  double pin_overhead_base = 0.25;
  double pin_overhead_enh = 0.25;
  // link.probe.pin_mcs: static-pin mode only -- probe a fixed MCS while
  // pinned (bench validation).
  int probe_pin_mcs = -1;
};

class VrxController {
 public:
  explicit VrxController(VrxCfg cfg);
  // A video body arrived: feeds the rendezvous video-silence timer only. The
  // RSSI/SNR/seq the old ScoreWindow consumed here went nowhere but the RCF
  // score/ack_seq fields, both deleted from the wire in RC_VERSION 3.
  void on_video(double now_ms);
  void on_rc_frame(const uint8_t* buf, size_t len, double now_ms);
  struct Out {
    std::vector<uint8_t> frame;
    bool is_disc;
  };
  // health: this window's measured loss (LadderController::LinkHealth, see
  // ladder_controller.h). Ignored entirely in pin mode. The ladder's own
  // internal checks (video_starved forces the failsafe rung; sample_valid
  // gates everything else) replace the old SNR-survivor-bias special case
  // here — see ladder_controller.cpp update().
  std::optional<Out> step(double now_ms, const LinkHealth& health);
  const OpPoint& cur_op() const;
  // The ladder controller itself, for Task 6's sideport link.ctl block and
  // the "ctl: rung a->b" transition line in main.cpp. Exists even in pin
  // mode (constructed unconditionally) but is never ticked/updated there.
  const LadderController& ctl() const { return ctrl_; }
  VrxState link_state() const;
  uint16_t rcf_seq() const;
  // The probe byte the last built RCF carried (kNoProbeProfile when none).
  uint8_t probe_profile() const { return last_cmd_probe_profile_; }
  // chip_caps from the most recently accepted DiscAck; 0 before any accept.
  // Gates GS main's video tail on mabur::rc::CAP_FRAME_WIRE.
  uint16_t peer_caps() const { return peer_caps_; }
  // Whether any DiscAck has ever been accepted. peer_caps() == 0 is ambiguous
  // on its own — a peer may genuinely advertise no caps — and the rendezvous
  // starts in SESSION, so callers that want to complain about a peer's missing
  // capability must wait for this to be true or they complain about a peer
  // they have not heard from yet.
  bool peer_acked() const { return peer_acked_; }
  // Rendezvous nonce for test construction of acceptable DiscAcks.
  uint32_t rz_nonce() const { return rz_.nonce(); }

 private:
  mabur::rc::Rcf build_rcf();
  void note_cmd(const mabur::rc::Rcf& r);

  VrxCfg cfg_;
  LadderController ctrl_;
  VrxRendezvous rz_;
  double last_fb_ms_ = -1e18;
  double last_keepalive_ms_ = -1e18;
  uint16_t seq_ = 0;
  OpPoint cur_op_;
  uint16_t peer_caps_ = 0;
  bool peer_acked_ = false;
  uint8_t last_cmd_probe_profile_ = mabur::rc::kNoProbeProfile;
};

}  // namespace maburgs
