#pragma once
#include <cstdint>
#include <string>

#include "au_ring.h"

namespace maburgs {

// Per-AU wakeup channel next to the AU ring: SOCK_SEQPACKET listener that
// holds at most one consumer (maburplay). Accept -> 16-byte geometry hello
// {magic, version, slot_bytes, slot_count}; notify() -> 8-byte {rec_no}.
// Datagram loss/overflow is harmless: state lives in the ring.
class AuDoorbell {
 public:
  AuDoorbell() = default;
  ~AuDoorbell();
  AuDoorbell(const AuDoorbell&) = delete;
  AuDoorbell& operator=(const AuDoorbell&) = delete;

  bool open(const std::string& path, AuRingGeom geom);  // unlinks stale socket
  void poll();                    // accept/reap; call every loop tick
  void notify(uint64_t rec_no);   // nonblocking; drops on EAGAIN
  bool client_connected() const { return client_ >= 0; }

 private:
  void drop_client_();
  int listen_ = -1;
  int client_ = -1;
  AuRingGeom geom_;
};

}  // namespace maburgs
