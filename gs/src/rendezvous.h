#pragma once

#include "mabur/rc_proto.h"

namespace maburgs {

enum class VrxState { SESSION, BEACONING };
enum class VrxAction { TxFeedback, Beacon, Idle };

struct VrxRzConfig {
  uint32_t vtx_id = 1;
  int link_lost_ms = 1000;
  int beacon_period_ms = 20;
  uint8_t op_channel = 149;
};

class VrxRendezvous {
 public:
  explicit VrxRendezvous(VrxRzConfig cfg);
  void feed_video(double now_ms);
  VrxAction tick(double now_ms);
  mabur::rc::Disc beacon();
  bool feed_disc_ack(const mabur::rc::DiscAck& ack, double now_ms);
  VrxState state() const;
  uint32_t nonce() const;

 private:
  VrxRzConfig cfg_;
  VrxState state_ = VrxState::SESSION;
  double last_video_ms_ = 0;
  double last_beacon_ms_ = -1e18;
  uint16_t seq_ = 0;
  uint32_t nonce_;
};

}  // namespace maburgs
