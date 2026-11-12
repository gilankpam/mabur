#pragma once
#include <optional>
#include "mabur/rc_proto.h"
namespace maburgs {

// SESSION: we hold the drone's vtx_nonce and may send RCFs. BEACONING: no
// session (never acked, or video lost > link_lost_ms). KEY_MISMATCH: our
// key file differs from the drone's -- the stranger rule: a flagged ack
// arrives while the current flagged-only run (no unflagged ack since its
// first flagged ack) is BOTH >= kKeyMismatchMs old AND has seen
// >= kKeyMismatchBeacons of our DISCs go out since that first flagged ack.
// Without vtx_id every drone on the channel answers every DISC, so a
// foreign-key drone beside ours acks flagged; the time term alone tripped
// in SESSION (1 s keep-alives) on one or two lost acks of our own drone.
// BEACONING (20 ms DISCs) still trips in ~1 s; SESSION takes ~3 s. Any
// unflagged ack resets both and leaves KEY_MISMATCH.
// See docs/link-pairing.md.
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
  static constexpr int kKeyMismatchBeacons = 3;
  // video_flowing(): link video bodies with no gap above kVideoGapMs for at
  // least kVideoRunMs. The lowest encoder rate (low-power 30 fps) spaces
  // frames 33 ms apart; the first frames of a booting encoder (~1.5 fps)
  // never qualify.
  static constexpr double kVideoGapMs = 100;
  static constexpr double kVideoRunMs = 200;
  explicit VrxRendezvous(VrxRzConfig cfg);
  void feed_video(double now_ms);
  VrxAction tick(double now_ms);
  mabur::rc::Disc beacon();
  bool feed_disc_ack(const mabur::rc::DiscAck& ack, double now_ms, bool* adopted_new);
  VrxState state() const { return state_; }
  // Real link video is flowing (feed_video only). SESSION alone is not:
  // a DISC_ACK opens it and refreshes the silence timer with no video, so
  // a freshly booted drone whose encoder is not up yet holds SESSION for
  // link_lost_ms and then drops back to BEACONING, taking the RCFs with it.
  bool video_flowing(double now_ms) const {
    return now_ms - last_real_video_ms_ <= kVideoGapMs &&
           last_real_video_ms_ - video_run_start_ms_ >= kVideoRunMs;
  }
  uint32_t nonce() const { return nonce_; }
  std::optional<uint32_t> vtx_nonce() const { return vtx_nonce_; }
  void set_proposal(uint8_t ch) { proposal_ = ch; }
  uint8_t proposal() const { return proposal_; }

 private:
  VrxRzConfig cfg_;
  VrxState state_ = VrxState::BEACONING;
  double last_video_ms_ = 0;
  // The current run of link video for video_flowing(): never touched by an ack.
  double last_real_video_ms_ = -1e18;
  double video_run_start_ms_ = 0;
  double last_beacon_ms_ = -1e18;
  uint16_t seq_ = 0;
  uint32_t nonce_;
  uint8_t proposal_;
  // Kept across KEY_MISMATCH: tick() never returns TxFeedback outside
  // SESSION, so a held nonce sends nothing there, and clearing it would make
  // our own drone's next same-nonce ack read as a NEW session -- the caller
  // would zero seq32 and the still-LINKED drone (seq32 strictly increasing)
  // would reject every RCF until failsafe. The trigger is a stranger's
  // flagged acks while three of our drone's acks are lost on the uplink.
  std::optional<uint32_t> vtx_nonce_;
  // Stranger rule: time of the first flagged ack in the current run of
  // flagged-only acks, and the DISCs beacon() built since it; both reset by
  // any unflagged ack.
  std::optional<double> first_flagged_ms_;
  int beacons_since_flagged_ = 0;
};
}  // namespace maburgs
