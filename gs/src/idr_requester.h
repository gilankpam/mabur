#pragma once
#include <cstdint>

namespace maburgs {

// Latches "the decoder's reference chain is broken — ask the drone for an
// IDR" from FrameStream frame_lost events, and clears when a complete
// IDR-flagged frame is emitted. A LEVEL consumed by VrxController::step()
// on every RCF, so a lost RCF costs one 100 ms feedback period and nothing
// else; the drone's grant cooldown is the dedup. sid 3 (non-reference
// TRAIL_N enhance) never latches: its loss dips fps but cannot smear.
// Caller supplies the clock (mono ms). Spec 2026-08-11 idr-request.
class IdrRequester {
 public:
  // Loss event with known sid. Returns true when this call newly set the
  // latch (a clear -> set edge) — the caller's cue to log the episode.
  bool on_frame_lost(uint8_t sid, uint64_t now_ms) {
    if (sid > 2 || want_) return false;
    want_ = true;
    since_ms_ = now_ms;
    ++episodes_;
    return true;
  }

  // maburplay reported that IT broke the decoder's reference chain (flush,
  // join at a non-IRAP sid0, watchdog) -- a class of break the GS cannot
  // observe, because nothing was lost on the wire. Same latch, but its own
  // counter so the two causes stay distinguishable on the sideport. Spec
  // 2026-08-12 decoder-idr-backchannel.
  //
  // The shared latch means whichever trigger fires first owns the episode;
  // the other does not open a second one while the latch is up. That is
  // intended -- one broken chain needs one IDR, not two.
  bool on_player_break(uint64_t now_ms) {
    if (want_) return false;
    want_ = true;
    since_ms_ = now_ms;
    ++episodes_player_;
    return true;
  }

  // Emitted-frame observation (idr = FrameHdr flags carried kFlagIdr).
  // Returns true when this call cleared the latch.
  bool on_frame_emitted(bool complete, bool idr, uint64_t now_ms) {
    if (!want_ || !complete || !idr) return false;
    want_ = false;
    last_wait_ms_ = now_ms - since_ms_;
    return true;
  }

  bool want() const { return want_; }
  uint64_t episodes() const { return episodes_; }
  uint64_t episodes_player() const { return episodes_player_; }
  // Clamped elapsed time since latch. The sin fill passes a loop-top clock
  // that can lag the latch stamp taken mid-drain by a fresh mono_ms() —
  // clamp guards against uint64 underflow when caller clock lags behind the
  // latch mark.
  uint64_t wait_ms(uint64_t now_ms) const {
    return want_ && now_ms > since_ms_ ? now_ms - since_ms_ : 0;
  }
  uint64_t last_wait_ms() const { return last_wait_ms_; }

 private:
  bool want_ = false;
  uint64_t since_ms_ = 0;
  uint64_t episodes_ = 0;
  uint64_t episodes_player_ = 0;
  uint64_t last_wait_ms_ = 0;
};

}  // namespace maburgs
