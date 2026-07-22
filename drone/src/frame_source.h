#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

// venc_frame_ring.h (vendored, C header) declares
// venc_frame_ring_create/attach/destroy without extern "C", so wrap the
// include to get C linkage matching the vendored venc_frame_ring.c
// translation unit (compiled as C).
extern "C" {
#include "venc_frame_ring.h"
}

namespace mabur {

// Consumer side of the vendored venc_frame_ring SHM ring: pulls whole
// Annex-B encoded frames (each preceded by an 8-byte VencFrameMeta) written
// by the waybeam venc producer. Attaches lazily (construction does not
// require the ring to already exist) and transparently reattaches if the
// producer restarts (detected via a changed/missing /dev/shm/<name> inode),
// so callers can just keep calling read() across producer restarts. Not
// thread-safe: intended for exclusive use by a single (agent/capture)
// thread.
class FrameSource {
 public:
  // attach_backoff_ms: how long to wait between attach attempts while the
  // ring does not exist yet (or right after a detected producer restart).
  // Exposed mainly so tests can shorten the default ~1 s production
  // backoff.
  explicit FrameSource(std::string shm_name, int attach_backoff_ms = 1000);
  ~FrameSource();

  FrameSource(const FrameSource&) = delete;
  FrameSource& operator=(const FrameSource&) = delete;

  // Blocks up to timeout_ms waiting for a frame.
  //   >0  payload length, written into buf starting at
  //       VENC_FRAME_META_SIZE (up to buf_size bytes total: meta + payload),
  //       and the decoded meta header into *meta.
  //   0   timeout, or ring not attached yet (still backing off).
  // Handles attach/reattach internally; never throws.
  int read(uint8_t* buf, size_t buf_size, int timeout_ms, VencFrameMeta* meta);

  bool attached() const;
  uint64_t reattach_count() const;

  // Forwards to venc_frame_ring_get_fill when attached; false (out
  // untouched) when unattached.
  bool fill(venc_frame_ring_fill_t* out) const;

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

  venc_frame_ring_t* ring_ = nullptr;
  uint64_t reattach_count_ = 0;

  // Identity of /dev/shm/<name> at the time of the current attach, used
  // to detect producer restarts (recreate = new inode).
  bool have_inode_ = false;
  uint64_t attached_dev_ = 0;
  uint64_t attached_ino_ = 0;

  // Whether this FrameSource has ever completed a successful attach
  // before. The very first attach must NOT snap read_idx to the current
  // write head: unlike RingSource's RTP-packet ring (whose production
  // consumer always attaches before the producer starts writing),
  // whole-frame producers may publish a frame before the very first
  // FrameSource attach happens, and that frame must still be delivered.
  // Only attaches that follow a detected producer restart (i.e. every
  // attach after the first) should skip stale backlog via snap-to-head.
  bool ever_attached_ = false;

  // Backoff bookkeeping: timestamp (steady clock, ms) of the last failed
  // attach attempt, so read() doesn't spin-retry attach every call.
  int64_t last_attach_attempt_ms_ = -1;
};

}  // namespace mabur
