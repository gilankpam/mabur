#include "rendezvous.h"

namespace maburgs {

VrxRendezvous::VrxRendezvous(VrxRzConfig cfg)
    : cfg_(cfg),
      nonce_(static_cast<uint32_t>(
          (static_cast<uint64_t>(cfg.vtx_id) * 2654435761ull) & 0xFFFFFFFFull)) {}

void VrxRendezvous::feed_video(double now_ms) {
  last_video_ms_ = now_ms;
  if (state_ == VrxState::BEACONING) state_ = VrxState::SESSION;
}

VrxAction VrxRendezvous::tick(double now_ms) {
  if (state_ == VrxState::SESSION) {
    if (now_ms - last_video_ms_ > cfg_.link_lost_ms)
      state_ = VrxState::BEACONING;
    else
      return VrxAction::TxFeedback;
  }
  if (now_ms - last_beacon_ms_ >= cfg_.beacon_period_ms) {
    last_beacon_ms_ = now_ms;
    return VrxAction::Beacon;
  }
  return VrxAction::Idle;
}

mabur::rc::Disc VrxRendezvous::beacon() {
  seq_ = static_cast<uint16_t>(seq_ + 1);
  mabur::rc::Disc d;
  d.vtx_id = cfg_.vtx_id;
  d.vrx_nonce = nonce_;
  d.op_channel = cfg_.op_channel;
  d.op_width = 20;
  d.seq = seq_;
  return d;
}

bool VrxRendezvous::feed_disc_ack(const mabur::rc::DiscAck& ack, double now_ms) {
  if (ack.vtx_id != cfg_.vtx_id || ack.vrx_nonce != nonce_) return false;
  state_ = VrxState::SESSION;
  last_video_ms_ = now_ms;
  return true;
}

VrxState VrxRendezvous::state() const { return state_; }
uint32_t VrxRendezvous::nonce() const { return nonce_; }

}  // namespace maburgs
