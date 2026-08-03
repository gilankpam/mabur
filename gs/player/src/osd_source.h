#ifndef MABUR_PLAYER_OSD_SOURCE_H_
#define MABUR_PLAYER_OSD_SOURCE_H_

#include <cstdint>
#include <string>
#include <vector>

#include "mabur/msp_dp.h"

namespace maburplay {

// Receives MSP DisplayPort snapshots from maburgs (UDP on loopback -- the
// gs-side msp.out.{host,port} sink) and maintains the current screen.
// Non-blocking; driven from the player's main loop, no thread of its own.
class OsdSource {
 public:
  OsdSource() = default;
  ~OsdSource();
  OsdSource(const OsdSource&) = delete;
  OsdSource& operator=(const OsdSource&) = delete;

  // Binds 127.0.0.1:port. port == 0 binds an ephemeral port (tests).
  bool open(int port, std::string* err);
  // Socket-free mode: accept bytes via feed() only (host --osd-render).
  bool feed_open();
  int port() const { return port_; }

  // Drains every pending datagram. True == a complete screen is ready to
  // render now (a DRAW_SCREEN arrived and the rate limit allows it).
  bool poll(uint64_t now_ms);
  // Same contract as poll(), for bytes obtained elsewhere.
  bool feed(const uint8_t* p, size_t n, uint64_t now_ms);

  // True once nothing has arrived for stale_ms AND something arrived at
  // least once (a link that never carried MSP has nothing to blank).
  bool stale(uint64_t now_ms) const;

  const mabur::MspScreen& screen() const { return screen_; }
  uint64_t datagrams() const { return datagrams_; }
  uint64_t screens() const { return screens_; }

  void set_stale_ms(int ms) { stale_ms_ = ms; }
  void set_min_interval_ms(int ms) { min_interval_ms_ = ms; }

 private:
  bool consume_(const uint8_t* p, size_t n, uint64_t now_ms);
  bool gate_(uint64_t now_ms);

  int fd_ = -1;
  int port_ = 0;
  bool opened_ = false;
  mabur::MspParser parser_;
  mabur::MspScreen screen_;
  bool complete_ = false;
  uint64_t last_rx_ms_ = 0;
  uint64_t last_render_ms_ = 0;
  bool rendered_once_ = false;
  int stale_ms_ = 2000;
  int min_interval_ms_ = 30;
  uint64_t datagrams_ = 0;
  uint64_t screens_ = 0;
  // Per-instance receive buffer (not a function-local static): Task 8 can
  // construct more than one OsdSource in a process, and a shared buffer
  // would let one instance's poll() clobber another's in-flight datagram.
  std::vector<uint8_t> buf_ = std::vector<uint8_t>(65536);
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_SOURCE_H_
