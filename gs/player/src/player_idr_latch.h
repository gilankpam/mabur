#ifndef MABUR_PLAYER_IDR_LATCH_H_
#define MABUR_PLAYER_IDR_LATCH_H_
#include <cstdint>

namespace maburplay {

// Latches "I have broken my own decoder reference chain -- ask the drone for
// an IDR", from maburplay's OWN bookkeeping and with no decoder
// introspection: MppBackend::concealed() was measured always-zero even while
// h265d logged reference misses (docs/idr-decoder-blindspot-2026-08-11.md).
//
// Set by a backend flush, a join/re-arm at a sid0 AU (measured: sid0 is
// NEVER an IRAP in this stream, so the decoder resumes predicting from
// pictures it never saw), or a decode-watchdog reset. Cleared only when a
// COMPLETE IDR-flagged AU reaches the decoder.
//
// A LEVEL, not an edge: the sender repeats it, so a lost datagram costs
// nothing, a flush immediately followed by a join is ONE episode, and an
// IDR that never arrives simply keeps the request alive. Spec
// docs/superpowers/specs/2026-08-12-decoder-idr-backchannel-design.md.
class PlayerIdrLatch {
 public:
  enum class Reason : uint8_t { kNone = 0, kFlush, kJoin, kWatchdog };

  // Returns true only on the clear -> set edge (the caller's cue to log and
  // to send immediately rather than waiting for the next heartbeat). Every
  // event is counted even when it does not open an episode.
  bool on_break(Reason r, uint64_t now_ms) {
    if (r == Reason::kNone) return false;
    ++counts_[static_cast<int>(r)];
    if (want_) return false;
    want_ = true;
    reason_ = r;
    since_ms_ = now_ms;
    ++episodes_;
    return true;
  }

  // An AU was handed to the decoder. Returns true when this cleared the latch.
  bool on_au_submitted(bool complete, bool idr, uint64_t now_ms) {
    if (!want_ || !complete || !idr) return false;
    want_ = false;
    last_wait_ms_ = now_ms > since_ms_ ? now_ms - since_ms_ : 0;
    return true;
  }

  bool want() const { return want_; }
  Reason reason() const { return reason_; }
  uint64_t episodes() const { return episodes_; }
  uint64_t last_wait_ms() const { return last_wait_ms_; }
  uint64_t count(Reason r) const { return counts_[static_cast<int>(r)]; }

 private:
  bool want_ = false;
  Reason reason_ = Reason::kNone;
  uint64_t since_ms_ = 0;
  uint64_t episodes_ = 0;
  uint64_t last_wait_ms_ = 0;
  uint64_t counts_[4] = {0, 0, 0, 0};
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_IDR_LATCH_H_
