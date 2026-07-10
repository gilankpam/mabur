#include "ring_source.h"

#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <thread>

namespace mabur {

namespace {

int64_t steady_now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// Mirrors the path-building venc_ring.c does internally: shm_open() with a
// leading-'/' name maps to /dev/shm/<name-without-slash>. We build the
// equivalent /dev/shm path here (outside the vendored code) purely for
// stat()-based producer-restart detection; venc_ring itself never sees
// this string.
std::string shm_stat_path(const std::string& shm_name) {
  const std::string& n =
      (!shm_name.empty() && shm_name[0] == '/') ? shm_name.substr(1)
                                                 : shm_name;
  return "/dev/shm/" + n;
}

}  // namespace

RingSource::RingSource(std::string shm_name, int attach_backoff_ms)
    : shm_name_(std::move(shm_name)), attach_backoff_ms_(attach_backoff_ms) {}

RingSource::~RingSource() { detach(); }

void RingSource::detach() {
  if (ring_) {
    venc_ring_destroy(ring_);
    ring_ = nullptr;
  }
  have_inode_ = false;
}

void RingSource::snap_to_head() {
  if (!ring_ || !ring_->hdr) return;
  uint64_t w = __atomic_load_n(&ring_->hdr->write_idx, __ATOMIC_ACQUIRE);
  __atomic_store_n(&ring_->hdr->read_idx, w, __ATOMIC_RELEASE);
}

bool RingSource::try_attach() {
  venc_ring_t* r = venc_ring_attach(shm_name_.c_str());
  if (!r) return false;

  ring_ = r;
  snap_to_head();

  struct stat st;
  if (stat(shm_stat_path(shm_name_).c_str(), &st) == 0) {
    have_inode_ = true;
    attached_dev_ = static_cast<uint64_t>(st.st_dev);
    attached_ino_ = static_cast<uint64_t>(st.st_ino);
  } else {
    // Couldn't stat right after a successful attach (unlikely race) —
    // treat as unknown identity; next timeout's re-stat will simply
    // detect ENOENT-as-missing and force a reattach, which is safe.
    have_inode_ = false;
  }
  return true;
}

bool RingSource::producer_restarted() const {
  struct stat st;
  if (stat(shm_stat_path(shm_name_).c_str(), &st) != 0) {
    return true;  // ENOENT (or other stat failure) — treat as restarted
  }
  if (!have_inode_) return false;
  return static_cast<uint64_t>(st.st_dev) != attached_dev_ ||
         static_cast<uint64_t>(st.st_ino) != attached_ino_;
}

int RingSource::read(uint8_t* buf, size_t buf_size, int timeout_ms) {
  if (!ring_) {
    // When timeout_ms <= 0, return immediately (instant poll, no sleep).
    if (timeout_ms <= 0) {
      int64_t now = steady_now_ms();
      if (last_attach_attempt_ms_ >= 0) {
        int64_t elapsed = now - last_attach_attempt_ms_;
        if (elapsed < attach_backoff_ms_) {
          // Still in backoff window but timeout=0 means no sleep.
          return 0;
        }
      }
      last_attach_attempt_ms_ = now;
      if (!try_attach()) {
        return 0;
      }
    } else {
      int64_t now = steady_now_ms();
      if (last_attach_attempt_ms_ >= 0) {
        int64_t elapsed = now - last_attach_attempt_ms_;
        if (elapsed < attach_backoff_ms_) {
          int remaining = static_cast<int>(attach_backoff_ms_ - elapsed);
          int sleep_ms = std::min(timeout_ms, remaining);
          if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
          }
          return 0;
        }
      }
      last_attach_attempt_ms_ = now;
      if (!try_attach()) {
        return 0;
      }
    }
  }

  uint16_t out_len = 0;
  int rc;

  // Special-case timeout_ms <= 0: non-blocking poll (not infinite wait).
  // venc_ring_read_wait's contract is "timeout_ms <= 0 = infinite", but our
  // documented contract is "0 = instant poll". Use the non-blocking
  // venc_ring_read to implement the poll semantics.
  if (timeout_ms <= 0) {
    rc = venc_ring_read(ring_, buf, static_cast<uint16_t>(buf_size), &out_len);
  } else {
    rc = venc_ring_read_wait(ring_, buf, static_cast<uint16_t>(buf_size),
                              &out_len, timeout_ms);
  }

  if (rc == 0) {
    return static_cast<int>(out_len);
  }

  // Timeout (or, with timeout_ms<=0, spurious wake) — this is the
  // designated checkpoint for producer-restart detection since it's
  // the one place we're guaranteed to visit on every idle period.
  if (producer_restarted()) {
    detach();
    ++reattach_count_;
    // A restart means a (likely already-live) new ring triggered this
    // detach, so don't apply the cold-start backoff to the very next
    // attach attempt — only repeated failures to reattach should back
    // off.
    last_attach_attempt_ms_ = -1;
  }
  return 0;
}

bool RingSource::attached() const { return ring_ != nullptr; }

uint64_t RingSource::reattach_count() const { return reattach_count_; }

}  // namespace mabur
