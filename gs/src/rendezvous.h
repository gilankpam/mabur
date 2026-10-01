#pragma once
#include <optional>
#include "mabur/rc_proto.h"
namespace maburgs {

// SESSION: we hold the drone's vtx_nonce and may send RCFs. BEACONING: no
// session (never acked, or video lost > link_lost_ms). KEY_MISMATCH: the
// drone has been answering our DISCs with key_mismatch for >= kKeyMismatchMs
// and no unflagged ack arrived -- our key file differs from the drone's.
// Spec 2026-10-01 link-pairing §6/§8.
enum class VrxState { SESSION, BEACONING, KEY_MISMATCH };
enum class VrxAction { TxFeedback, Beacon, Idle };

struct VrxRzConfig {
  int link_lost_ms = 1000;
  int beacon_period_ms = 20;
  uint8_t op_channel = 149;
  // 0 = random per instance (std::random_device). A restarted GS must be a
  // NEW session on the drone (its seq32 restarts), so the nonce is never
  // derived from config. Tests and the hop-inject self-test pin one.
  uint32_t nonce = 0;
};

class VrxRendezvous {
 public:
  static constexpr double kKeyMismatchMs = 1000;
  explicit VrxRendezvous(VrxRzConfig cfg);
  void feed_video(double now_ms);
  VrxAction tick(double now_ms);
  mabur::rc::Disc beacon();
  bool feed_disc_ack(const mabur::rc::DiscAck& ack, double now_ms, bool* adopted_new);
  VrxState state() const { return state_; }
  uint32_t nonce() const { return nonce_; }
  std::optional<uint32_t> vtx_nonce() const { return vtx_nonce_; }
  void set_proposal(uint8_t ch) { proposal_ = ch; }
  uint8_t proposal() const { return proposal_; }

 private:
  VrxRzConfig cfg_;
  VrxState state_ = VrxState::BEACONING;
  double last_video_ms_ = 0;
  double last_beacon_ms_ = -1e18;
  uint16_t seq_ = 0;
  uint32_t nonce_;
  uint8_t proposal_;
  std::optional<uint32_t> vtx_nonce_;
  // Stranger rule: time of the first flagged ack in the current run of
  // flagged-only acks; reset by any unflagged ack.
  std::optional<double> first_flagged_ms_;
};
}  // namespace maburgs
