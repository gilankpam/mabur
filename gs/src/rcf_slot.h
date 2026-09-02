#pragma once
// RcfSlotter — slot the GS uplink into the drone's inter-AU idle
// (docs/gs-uplink-self-blanking-findings-2026-09-02.md).
//
// Why: a GS control-frame send (RCF, DISC, repeat copies) blasts the
// sibling RX card at ~-4 dBm and deafens the TX card, so any drone PPDU
// whose preamble starts within ~180 us of the send is lost on BOTH cards.
// The chip already holds the TX until the PPDU it is receiving ends, so
// what kills is the NEXT PPDU starting right behind the blast — i.e. sends
// that land inside an AU burst. Sends that land in the idle between AUs
// (~8 ms at 11 Mb/s) hit nothing.
//
// Mechanism: when video is flowing, an offered frame is held until an AU
// completes (the core loop's end-of-AU callback) at a moment where the
// send will be on air BEFORE the next AU's burst is due. The next burst is
// predicted from the first-body arrival cadence (t_first of consecutive AUs
// is flat to <1 ms; completion is not — it moves with frame size and FEC
// repair), so the rule is: release at completion only if
//   now + lead_ms < last_t_first + period − guard_ms.
// A completion too close to the next burst is skipped and the frame waits
// for the next one. A frame offered within grace_ms after a good
// completion goes out at once (the idle has just begun). A hold longer
// than hold_max_ms is released anyway so feedback never stalls on a
// stalled/absent video stream. With no recent AU (no video) everything
// passes through — that covers rendezvous, so DISC needs no bypass.
// hold_max_ms 0 disables the slotter.
//
// Single-threaded (core loop). Time is the core loop's mono ms.
#include <cstdint>
#include <deque>
#include <vector>

namespace maburgs {

struct RcfSlotCfg {
  int hold_max_ms = 30;      // 0 = off
  int video_recent_ms = 100; // no AU within this -> pass through
  int grace_ms = 2;          // offered this soon after a good AU -> send now
  int lead_ms = 3;           // send -> on air + blast (USB + chip + ~0.2 ms)
  int guard_ms = 1;          // margin before the predicted next burst
};

// A control frame plus what the sender needs at actual send time: the
// card the selector chose when it was built, and the RCF seq to stamp the
// RTT estimator with (built-time seq, since vrx.rcf_seq() moves on).
enum class SlotReason : uint8_t { Passthru = 0, Grace = 1, Au = 2, Timeout = 3 };

struct SlotFrame {
  std::vector<uint8_t> frame;
  uint16_t seq = 0;
  int card = 0;
  bool stamp_rtt = false;
  // Filled in by RcfSlotter: why/when the frame was released.
  SlotReason reason = SlotReason::Passthru;
  uint64_t offered_ms = 0;
};

class RcfSlotter {
 public:
  explicit RcfSlotter(RcfSlotCfg cfg) : cfg_(cfg) {}
  // The core loop saw an AU's first body (begin-of-AU callback): feeds
  // the burst-cadence predictor.
  void on_au_first(uint64_t now_ms);
  // The core loop finished an AU (end-of-AU callback).
  void on_au_complete(uint64_t now_ms);
  // Would a send at now_ms be on air before the next burst is due?
  bool idle_ahead(uint64_t now_ms) const;
  double period_ms() const { return period_ms_; }
  // Offer a frame for sending. Returns true if it was taken into the hold
  // (send it when take_due() hands it back); false = send it now.
  bool offer(SlotFrame& f, uint64_t now_ms, bool bypass);
  // Frames whose hold has ended, oldest first. Call once per loop
  // iteration after the offers.
  std::vector<SlotFrame> take_due(uint64_t now_ms);

  uint64_t last_au_ms() const { return last_au_ms_; }
  uint64_t released_au() const { return released_au_; }
  uint64_t released_timeout() const { return released_timeout_; }
  uint64_t passthru() const { return passthru_; }
  size_t pending() const { return pending_.size(); }

 private:
  RcfSlotCfg cfg_;
  std::deque<SlotFrame> pending_;
  uint64_t hold_start_ms_ = 0;   // oldest pending's offer time
  uint64_t last_au_ms_ = 0;      // last completion
  uint64_t last_first_ms_ = 0;   // last first-body arrival
  bool have_au_ = false, have_first_ = false;
  double period_ms_ = 16.667;    // EMA of first-body intervals
  bool release_ = false;         // an AU completed while frames were pending
  uint64_t released_au_ = 0, released_timeout_ = 0, passthru_ = 0;
};

}  // namespace maburgs
