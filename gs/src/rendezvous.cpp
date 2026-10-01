#include "rendezvous.h"
#include <random>

namespace maburgs {
namespace {
uint32_t random_nonce() {
  std::random_device rd;
  uint32_t n = 0;
  do { n = rd(); } while (n == 0);
  return n;
}
}  // namespace

VrxRendezvous::VrxRendezvous(VrxRzConfig cfg)
    : cfg_(cfg), nonce_(cfg.nonce ? cfg.nonce : random_nonce()), proposal_(cfg.op_channel) {}

void VrxRendezvous::feed_video(double now_ms) {
  last_video_ms_ = now_ms;
  // Video proves the drone is there, not that we hold its session nonce.
  if (state_ == VrxState::BEACONING && vtx_nonce_) state_ = VrxState::SESSION;
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
  if (first_flagged_ms_) ++beacons_since_flagged_;
  mabur::rc::Disc d;
  d.vrx_nonce = nonce_;
  d.op_channel = proposal_;
  d.op_width = 20;
  d.seq = seq_;
  return d;
}

bool VrxRendezvous::feed_disc_ack(const mabur::rc::DiscAck& ack, double now_ms, bool* adopted_new) {
  if (adopted_new) *adopted_new = false;
  if (ack.vrx_nonce != nonce_) return false;
  if (ack.flags & mabur::rc::kAckKeyMismatch) {
    if (!first_flagged_ms_) {
      first_flagged_ms_ = now_ms;
      beacons_since_flagged_ = 0;
    }
    // Both: the run is old enough AND enough of our DISCs went unanswered
    // by an unflagged ack (rendezvous.h). vtx_nonce_ is kept: KEY_MISMATCH
    // sends no RCFs.
    if (now_ms - *first_flagged_ms_ >= kKeyMismatchMs &&
        beacons_since_flagged_ >= kKeyMismatchBeacons)
      state_ = VrxState::KEY_MISMATCH;
    return true;
  }
  first_flagged_ms_.reset();
  beacons_since_flagged_ = 0;
  if (!vtx_nonce_ || *vtx_nonce_ != ack.vtx_nonce) {
    vtx_nonce_ = ack.vtx_nonce;
    if (adopted_new) *adopted_new = true;
  }
  state_ = VrxState::SESSION;
  last_video_ms_ = now_ms;
  return true;
}
}  // namespace maburgs
