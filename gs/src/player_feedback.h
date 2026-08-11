#pragma once
#include <cstdint>
#include <string>

#include "idr_requester.h"
#include "mabur/player_fb.h"

namespace maburgs {

// Receives maburplay's IDR feedback datagrams on loopback (spec 2026-08-12
// decoder-idr-backchannel). Non-blocking and polled from the core loop -- no
// thread, the same shape as the player's GsSource.
//
// The player asserts a LEVEL and repeats it, so a new episode is counted only
// on a clear -> set edge of the RECEIVED flag; a repeated assertion must not
// inflate the GS's episode counter.
class PlayerFeedback {
 public:
  PlayerFeedback() = default;
  ~PlayerFeedback();
  PlayerFeedback(const PlayerFeedback&) = delete;
  PlayerFeedback& operator=(const PlayerFeedback&) = delete;

  // Binds 127.0.0.1:port. port == 0 binds an ephemeral port (tests).
  bool open(int port, std::string* err);
  bool ok() const { return fd_ >= 0; }
  int port() const { return port_; }

  // Drains every pending datagram, keeping the newest that decoded.
  // Returns true iff the received level went clear -> set on this call.
  bool poll(uint64_t now_ms);

  // Drops the level after `stale_ms` of silence. Never produces an edge: a
  // dead player must not look like a repair request. stale_ms == 0 disables.
  void expire(uint64_t now_ms, int stale_ms);

  bool want() const { return want_; }
  bool have_any() const { return msgs_ > 0; }
  uint64_t age_ms(uint64_t now_ms) const {
    return msgs_ && now_ms > last_rx_ms_ ? now_ms - last_rx_ms_ : 0;
  }
  const mabur::playerfb::Msg& msg() const { return msg_; }
  uint64_t datagrams() const { return datagrams_; }
  uint64_t malformed() const { return malformed_; }

 private:
  int fd_ = -1;
  int port_ = 0;
  bool want_ = false;
  mabur::playerfb::Msg msg_{};
  uint64_t last_rx_ms_ = 0, msgs_ = 0, datagrams_ = 0, malformed_ = 0;
};

// One core-loop iteration of the player back-channel: drain, expire, then
// reconcile the received LEVEL against the GS latch. Returns true when this
// call newly raised the latch (the caller's cue to log the episode).
//
// LEVEL-DRIVEN, NOT EDGE-DRIVEN, and that is the whole point. The player
// asserts a level and repeats it every report_ms; our latch clears
// independently, on the emitted IDR. Acting on the RECEIVED edge lets the two
// desynchronize, and both shapes are permanently-broken-picture bugs:
//
//   * the player dies and the init wrapper respawns it inside stale_ms. Our
//     shadow want_ is still true from the dead instance, so the new instance's
//     first idr=1 is not an edge -- and if the latch had already cleared,
//     nobody ever asks for the IDR the fresh join needs.
//   * the granted IDR is emitted (latch clears) but never reaches the player
//     (ring lap, oversize drop). The player keeps asserting, the repeated
//     level produces no edge, and the request is never retried. The spec's
//     failure table promises the opposite: "latch stays set, request repeats,
//     bounded by the drone's 1 s cooldown".
//
// on_player_break() no-ops while the latch is already up, so re-raising every
// iteration costs nothing. The runaway bound holds: a re-raise requires the
// latch to be clear, which requires an IDR to have actually been emitted,
// which the drone rate-limits to one per waybeam.idr_cooldown_ms.
//
// expire() MUST run before the want() check so a dead player's stale level
// cannot raise a fresh request.
inline bool reconcile_player_idr(PlayerFeedback& fb, IdrRequester& idr,
                                 uint64_t now_ms, int stale_ms) {
  fb.poll(now_ms);  // edge return is still meaningful, but must not gate here
  fb.expire(now_ms, stale_ms);
  return fb.want() && idr.on_player_break(now_ms);
}

}  // namespace maburgs
