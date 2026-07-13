#pragma once
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "mabur/uep_encoder.h"

namespace mabur {

// Bounded FIFO between the encode (hot) thread and the USB TX writer
// thread. Decouples ring drain from bulk-OUT latency: when the chip's TX
// FIFO stalls, the backlog lands HERE and overflow drops the OLDEST bodies
// — a bounded-latency, FEC-recoverable erasure the receiver's overhead
// absorbs. Without this stage the backlog landed in the waybeam SHM ring,
// whose overflow makes the RTP packetizer abort NALs mid-chain
// (sender-side slice truncation, invisible to seq-gap accounting — bench
// 2026-07-13, the PixelPilot glitch root cause).
class TxQueue {
 public:
  explicit TxQueue(size_t cap) : cap_(cap) {}

  void push(UepBody&& b) {
    {
      std::lock_guard<std::mutex> l(m_);
      if (closed_) return;
      if (q_.size() >= cap_) {
        q_.pop_front();
        ++dropped_;
      }
      q_.push_back(std::move(b));
    }
    cv_.notify_one();
  }

  // Pops up to max_n bodies into out (appended), blocking up to timeout_ms
  // for the first one. Returns the number popped (0 on timeout/closed-empty).
  size_t pop_batch(std::vector<UepBody>& out, size_t max_n, int timeout_ms) {
    std::unique_lock<std::mutex> l(m_);
    if (q_.empty()) {
      cv_.wait_for(l, std::chrono::milliseconds(timeout_ms),
                   [&] { return !q_.empty() || closed_; });
    }
    size_t n = 0;
    while (n < max_n && !q_.empty()) {
      out.push_back(std::move(q_.front()));
      q_.pop_front();
      ++n;
    }
    return n;
  }

  void close() {
    {
      std::lock_guard<std::mutex> l(m_);
      closed_ = true;
    }
    cv_.notify_all();
  }

  size_t depth() const {
    std::lock_guard<std::mutex> l(m_);
    return q_.size();
  }
  uint64_t dropped() const {
    std::lock_guard<std::mutex> l(m_);
    return dropped_;
  }

 private:
  const size_t cap_;
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::deque<UepBody> q_;
  bool closed_ = false;
  uint64_t dropped_ = 0;
};

}  // namespace mabur
