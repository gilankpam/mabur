#pragma once

#include <array>
#include <optional>
#include <vector>

#include "ladder_controller.h"
#include "op_point.h"
#include "rendezvous.h"

namespace maburgs {

struct VrxCfg {
  uint32_t vtx_id = 1;
  uint8_t op_channel = 149;
  int feedback_ms = 100;
  int beacon_keepalive_ms = 1000;
  // Measured-loss ladder controller config (see LinkCfg::ladder_cfg,
  // config.h). Consulted every tick unless pinned.
  LadderCfg ladder;
  // Static-link pin: mcs >= 0 bypasses the adaptive controller (see
  // LinkCfg::static_mcs). overhead used only when pinned.
  int pin_mcs = -1;
  double pin_overhead = 0.25;
  // RCF repeat burst (rcf-uplink-loss findings 2026-08-14): after an RCF
  // whose commanded content changed, re-send it copies times at
  // rcf_repeat_ms spacing (fresh seq each). 0 disables.
  int rcf_repeat_copies = 3;
  int rcf_repeat_ms = 10;
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
  // Repeat-burst drain: returns the next repeat RCF when one is due at
  // now_ms, rebuilt from CURRENT controller state with a fresh seq. Callers
  // send it like any control frame but must NOT treat it as a feedback
  // boundary (no decoder-window reset — window == RCF period holds for
  // step() emissions only).
  std::optional<std::vector<uint8_t>> poll_repeat(double now_ms);
  const OpPoint& cur_op() const;
  // The ladder controller itself, for Task 6's sideport link.ctl block and
  // the "ctl: rung a->b" transition line in main.cpp. Exists even in pin
  // mode (constructed unconditionally) but is never ticked/updated there.
  const LadderController& ctl() const { return ctrl_; }
  VrxState link_state() const;
  uint16_t rcf_seq() const;
  // chip_caps from the most recently accepted DiscAck; 0 before any accept.
  // Gates GS main's video tail on mabur::rc::CAP_FRAME_WIRE.
  uint16_t peer_caps() const { return peer_caps_; }
  // Whether any DiscAck has ever been accepted. peer_caps() == 0 is ambiguous
  // on its own — a peer may genuinely advertise no caps — and the rendezvous
  // starts in SESSION, so callers that want to complain about a peer's missing
  // capability must wait for this to be true or they complain about a peer
  // they have not heard from yet.
  bool peer_acked() const { return peer_acked_; }

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
  // RCF repeat burst state (see poll_repeat).
  int repeats_left_ = 0;
  double next_repeat_ms_ = 0;
  bool have_last_cmd_ = false;
  uint8_t last_cmd_profile_ = 0;
  uint8_t last_cmd_ov16_ = 0;
  bool last_cmd_probe3_ = false;
  uint8_t last_cmd_probe_profile_ = 0;
};

}  // namespace maburgs
