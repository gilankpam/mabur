#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

// venc_ring.h (vendored, C header) declares venc_ring_create/attach/destroy
// without extern "C", so wrap the include to get C linkage matching the
// vendored venc_ring.c translation unit (compiled as C).
extern "C" {
#include "venc_ring.h"
}

namespace mabur {

// Consumer side of the vendored venc_ring SHM ring: pulls RTP packets
// written by the waybeam venc producer. Attaches lazily (construction does
// not require the ring to already exist) and transparently reattaches if
// the producer restarts (detected via a changed/missing /dev/shm/<name>
// inode), so callers can just keep calling read() across producer
// restarts. Not thread-safe: intended for exclusive use by a single
// (agent/capture) thread.
class RingSource {
 public:
  // attach_backoff_ms: how long to wait between attach attempts while the
  // ring does not exist yet (or right after a detected producer restart).
  // Exposed mainly so tests can shorten the default ~1 s production
  // backoff.
  explicit RingSource(std::string shm_name, int attach_backoff_ms = 1000);
  ~RingSource();

  RingSource(const RingSource&) = delete;
  RingSource& operator=(const RingSource&) = delete;

  // Blocks up to timeout_ms waiting for a packet.
  //   >0  packet length, written into buf (up to buf_size bytes).
  //   0   timeout, or ring not attached yet (still backing off).
  // Handles attach/reattach internally; never throws.
  int read(uint8_t* buf, size_t buf_size, int timeout_ms);

  bool attached() const;
  uint64_t reattach_count() const;

 private:
  bool try_attach();
  void detach();
  // Snap read_idx to the current write head (skip stale backlog). Called
  // on every successful attach.
  void snap_to_head();
  // Returns true if the backing /dev/shm/<name> file has changed identity
  // (different inode) or disappeared since attach, indicating the
  // producer restarted.
  bool producer_restarted() const;

  std::string shm_name_;
  int attach_backoff_ms_;

  venc_ring_t* ring_ = nullptr;
  uint64_t reattach_count_ = 0;

  // Identity of /dev/shm/<name> at the time of the current attach, used
  // to detect producer restarts (recreate = new inode).
  bool have_inode_ = false;
  uint64_t attached_dev_ = 0;
  uint64_t attached_ino_ = 0;

  // Backoff bookkeeping: timestamp (steady clock, ms) of the last failed
  // attach attempt, so read() doesn't spin-retry attach every call.
  int64_t last_attach_attempt_ms_ = -1;
};

}  // namespace mabur
