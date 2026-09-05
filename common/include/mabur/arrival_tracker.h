#pragma once
#include <cstdint>
#include <vector>

namespace mabur {

// Pre-FEC loss booked at ARRIVAL time, in a SwDecoder's virtual-seq space
// (spec docs/superpowers/specs/2026-09-05-arrival-loss-design.md §3).
//
// Why: the completion counters (delivered/recovered/abandoned) book a lost
// symbol only when FEC repairs it or gives up on it, tens of ms after the
// loss, and right after a rung-change re-key the first blocks complete late,
// so the first util bucket has almost no denominator and a handful of
// repairs reads as 50-100 % loss (flight-0023 probation bounces, bench
// ctl-0299). Here `expected` is sequence advance and `arrived` is what was
// heard, both booked the moment a seq crosses a settle line `guard` seqs
// behind the newest -- no dependence on decoder progress.
//
// Counters are monotonic and never rewound: a downstream sliding window
// (gs/src/s1_loss.h) treats a backward step as a counter reset. A symbol
// heard after its seq was booked missing counts `late` and is NOT un-booked.
class ArrivalTracker {
 public:
  static constexpr uint32_t kDefaultGuard = 32;  // one FEC window as flown

  explicit ArrivalTracker(uint32_t guard = kDefaultGuard, uint32_t ring_bits = 1024)
      : guard_(guard), bits_(ring_bits, 0), mask_(ring_bits - 1) {}

  // A source symbol with virtual seq v was heard. Idempotent for duplicates
  // (second card, retry) as long as v is still inside the guard. A v the
  // ring cannot hold yet (>= settle_ + ring) moves the settle line forward
  // first, booking what it passes, so its bit never aliases an unbooked seq.
  void on_source(uint64_t v, uint64_t stale_end) {
    if (!anchored_) anchor(v);
    if (v < settle_) { ++late_; return; }
    const uint64_t ring = mask_ + 1;
    if (v >= settle_ + ring) settle_to(v - ring + 1, stale_end);
    bits_[static_cast<size_t>(v & mask_)] = 1;
    if (v > newest_) newest_ = v;
  }

  // newest: highest seq seen or implied (a source's v, or a repair's
  // wend-1). stale_end: seqs below it book on the `_stale` side -- pass
  // ~0ull while a transition boundary is open (everything stale), wm+1 for
  // a closed watermark, 0 for none. Books every seq <= newest - guard.
  void advance(uint64_t newest, uint64_t stale_end) {
    if (!anchored_) anchor(newest);
    if (newest > newest_) newest_ = newest;
    if (newest_ < static_cast<uint64_t>(guard_)) return;
    settle_to(newest_ - guard_ + 1, stale_end);  // exclusive end of the settled span
  }
  // Encoder restart (SwDecoder::reset_state): drop the bitmap, re-anchor.
  // Seqs in flight at the reset are never booked; every counter is kept.
  void reset(uint64_t anchor_v) {
    for (auto& b : bits_) b = 0;
    anchored_ = false;
    anchor(anchor_v);
  }

  uint64_t expected() const { return expected_; }
  uint64_t arrived() const { return arrived_; }
  uint64_t expected_stale() const { return expected_stale_; }
  uint64_t arrived_stale() const { return arrived_stale_; }
  uint64_t late() const { return late_; }
  bool anchored() const { return anchored_; }

 private:
  void anchor(uint64_t v) {
    anchored_ = true;
    newest_ = v;
    settle_ = v;
  }
  // Book every seq in [settle_, target): read+clear its heard bit. Bits are
  // only ever meaningful for seqs in [settle_, settle_ + ring), which
  // on_source() guarantees. A jump larger than kResetSpan never reaches
  // here (SwDecoder::reset_state -> reset()), so the loop is bounded by
  // the largest in-session gap, microseconds at worst.
  void settle_to(uint64_t target, uint64_t stale_end) {
    while (settle_ < target) {
      const size_t i = static_cast<size_t>(settle_ & mask_);
      const bool heard = bits_[i] != 0;
      bits_[i] = 0;
      ++expected_;
      if (heard) ++arrived_;
      if (settle_ < stale_end) {
        ++expected_stale_;
        if (heard) ++arrived_stale_;
      }
      ++settle_;
    }
  }

  uint32_t guard_;
  std::vector<uint8_t> bits_;
  uint32_t mask_;
  bool anchored_ = false;
  uint64_t newest_ = 0;
  uint64_t settle_ = 0;  // next seq to book
  uint64_t expected_ = 0, arrived_ = 0, expected_stale_ = 0, arrived_stale_ = 0, late_ = 0;
};

}  // namespace mabur
