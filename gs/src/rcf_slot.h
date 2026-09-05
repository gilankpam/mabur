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
// Probe tail (spec 2026-09-04 probe-stream, fixed 2026-09-05): the
// probe-stream body is the LAST PPDU of every ENH burst, and the AU
// completes before it lands (bench: the probe's radio stamp is 0.9 ms p50
// / 4 ms p99 after the completion stamp; a send's USB+chip latency is
// 1-1.5 ms). A send released at the completion therefore puts the blast
// exactly where the probe is -- measured 2x the enh stream's own loss,
// and a fixed "one body airtime" deferral did nothing because it moved
// the blast from one part of that spread to another
// (docs/handover-probe-blanking-2026-09-04.md). So an ENH completion
// releases NOTHING by itself: the release is the probe's own arrival
// (on_probe_tail, fed from the probe sink), which is the exact "burst is
// off air" signal at any MCS and frame size. A lost probe never arrives,
// so each ENH completion also arms a deadline at completion + tail_ub,
// where tail_ub is learned from the observed completion->probe offsets
// (a decaying max, floored at probe_tail_ms = the body's own airtime,
// plus 1 ms). Both the probe release and the deadline release re-check
// idle_ahead() at the instant they fire, with no extra delay charged;
// the completion itself charges nothing. A base-AU completion is never
// trailed by a probe and releases at once, as before. The grace window
// opens at the burst end -- the probe's arrival, or the estimate
// completion + tail_ub while it is still expected.
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
  // Airtime of the probe-stream body that trails every ENH burst (spec
  // 2026-09-04 probe-stream), ms, rounded up; 0 when no probe is commanded.
  // Floor of the learned completion->probe offset (tail_ub_ms()): the
  // burst cannot end sooner than one probe body after the completion.
  int probe_tail_ms = 0;
};

// A control frame plus what the sender needs at actual send time: the
// card the selector chose when it was built, and the RCF seq to stamp the
// RTT estimator with (built-time seq, since vrx.rcf_seq() moves on).
enum class SlotReason : uint8_t { Passthru = 0, Grace = 1, Au = 2, Timeout = 3, Probe = 4 };

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
  // The core loop finished an AU (end-of-AU callback). probe_follows says
  // whether THIS AU is trailed by a probe body (enh AU + probe commanded):
  // such a completion releases nothing itself -- see on_probe_tail.
  void on_au_complete(uint64_t now_ms, bool probe_follows);
  // The probe sink saw a probe body (any card, any profile): the ENH
  // burst is off air. Releases the hold if the idle ahead allows it, and
  // feeds the completion->probe offset into tail_ub_ms().
  void on_probe_tail(uint64_t now_ms);
  // Would a send at now_ms be on air before the next burst is due?
  bool idle_ahead(uint64_t now_ms) const;
  double period_ms() const { return period_ms_; }
  // Airtime of the probe body trailing every ENH burst, ms; 0 = none
  // commanded. Runtime: the probe MCS follows the rung.
  void set_probe_tail_ms(int ms) { cfg_.probe_tail_ms = ms < 0 ? 0 : ms; }
  // Learned upper bound on completion->probe-arrival, ms: the deadline a
  // lost probe's release falls back to. ceil(decaying max of observed
  // offsets, floored at probe_tail_ms) + 1.
  int tail_ub_ms() const;
  // Offer a frame for sending. Returns true if it was taken into the hold
  // (send it when take_due() hands it back); false = send it now.
  bool offer(SlotFrame& f, uint64_t now_ms, bool bypass);
  // Frames whose hold has ended, oldest first. Call once per loop
  // iteration after the offers.
  std::vector<SlotFrame> take_due(uint64_t now_ms);

  uint64_t last_au_ms() const { return last_au_ms_; }
  uint64_t released_au() const { return released_au_; }
  uint64_t released_timeout() const { return released_timeout_; }
  uint64_t released_probe() const { return released_probe_; }
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
  bool release_pending_ = false; // a completion/probe armed a release
  SlotReason release_reason_ = SlotReason::Au;
  // An ENH completion is waiting for its probe: the deadline releases
  // instead if the probe never shows (lost on air).
  bool probe_wait_ = false;
  uint64_t probe_wait_from_ms_ = 0;  // that completion's time
  uint64_t probe_deadline_ms_ = 0;
  // When the current burst is (expected to be) off air: a base completion,
  // the probe's arrival, or completion + tail_ub while the probe is still
  // expected. The grace window counts from here.
  uint64_t idle_from_ms_ = 0;
  double tail_est_ms_ = 0;       // decaying max of completion->probe offsets
  static constexpr double kTailMaxMs = 15.0;
  uint64_t released_au_ = 0, released_timeout_ = 0, passthru_ = 0;
  uint64_t released_probe_ = 0;
};

}  // namespace maburgs
