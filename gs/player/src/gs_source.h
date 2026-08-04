#ifndef MABUR_PLAYER_GS_SOURCE_H_
#define MABUR_PLAYER_GS_SOURCE_H_

#include <cstdint>
#include <string>
#include <vector>

#include "gs_snapshot.h"

namespace maburplay {

// Receives maburgs stats-sideport datagrams (UDP on loopback, one of the
// destinations in maburgs' `stats.out` list) and keeps the newest decoded
// snapshot. Non-blocking; driven from the player's main loop, no thread of
// its own -- the same shape as OsdSource, deliberately, so main.cpp reads
// both intakes with one idiom.
//
// No rate limiting: the sideport is 2 Hz by configuration, which is already
// below any sane redraw rate. (OsdSource needs a gate because MSP snapshots
// can burst.)
class GsSource {
 public:
  GsSource() = default;
  ~GsSource();
  GsSource(const GsSource&) = delete;
  GsSource& operator=(const GsSource&) = delete;

  // Binds 127.0.0.1:port. port == 0 binds an ephemeral port (tests).
  bool open(int port, std::string* err);
  // Socket-free mode: accept bytes via feed() only (host --gs-render).
  bool feed_open();
  int port() const { return port_; }
  // Test-only: lets a test poll(2) the raw socket for readability (to know
  // datagrams are genuinely queued) without going through poll()'s own
  // drain loop. Not part of the intake contract -- callers must never read
  // or write through this fd themselves.
  int debug_fd() const { return fd_; }

  // Drains every pending datagram, keeping the LAST one that decoded. A
  // backlog must collapse to the newest sample: the OSD shows now, not a
  // queue replayed. True == a fresh snapshot is ready to render.
  bool poll(uint64_t now_ms);
  // Same contract as poll(), for bytes obtained elsewhere.
  bool feed(const uint8_t* p, size_t n, uint64_t now_ms);

  // True once stale_ms has passed since the last DECODED snapshot AND at
  // least one ever arrived -- a link that never carried stats has nothing
  // to call stale. stale_ms == 0 disables staleness entirely.
  //
  // Note the clock advances on a decoded snapshot, not on any datagram: a
  // stream of malformed datagrams is silence, and must read as such.
  bool stale(uint64_t now_ms) const;
  bool have_any() const { return snapshots_ > 0; }

  const GsSnapshot& snapshot() const { return snap_; }
  uint64_t datagrams() const { return datagrams_; }
  uint64_t snapshots() const { return snapshots_; }
  uint64_t parse_errors() const { return parse_errors_; }

  void set_stale_ms(int ms) { stale_ms_ = ms; }

 private:
  int fd_ = -1;
  int port_ = 0;
  GsSnapshot snap_;
  uint64_t last_ok_ms_ = 0;
  uint64_t datagrams_ = 0;
  uint64_t snapshots_ = 0;
  uint64_t parse_errors_ = 0;
  int stale_ms_ = 3000;  // 6 missed samples at the 500 ms sideport cadence
  // Per-instance receive buffer: a shared one would let a second GsSource
  // clobber this one's in-flight datagram (the same reasoning as
  // OsdSource::buf_).
  std::vector<uint8_t> buf_ = std::vector<uint8_t>(65536);
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_GS_SOURCE_H_
