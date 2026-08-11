#pragma once
#include <cstdint>
#include <string>

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

}  // namespace maburgs
