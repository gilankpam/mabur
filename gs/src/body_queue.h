#pragma once
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <vector>

#include "mabur/node.h"

namespace maburgs {

// Bounded queue from the per-card devourer RX callbacks (producers) to the
// core thread (single consumer). Mutex+condvar with batch drain — trivial
// cost at GS body rates. Overflow (consumer wedged) drops the NEWEST message
// and counts it, bounding memory while keeping the oldest (most decodable)
// backlog intact.
class BodyQueue {
 public:
  static constexpr size_t kCapacity = 8192;

  void push(mabur::node::RxBody m) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (q_.size() >= kCapacity) {
        ++dropped_;
        return;
      }
      q_.push_back(std::move(m));
    }
    cv_.notify_one();
  }

  size_t drain(std::vector<mabur::node::RxBody>& out, int timeout_ms) {
    std::unique_lock<std::mutex> lk(mu_);
    if (q_.empty() && !closed_ && timeout_ms > 0)
      cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                   [&] { return !q_.empty() || closed_; });
    const size_t n = q_.size();
    for (auto& m : q_) out.push_back(std::move(m));
    q_.clear();
    return n;
  }

  void close() {
    {
      std::lock_guard<std::mutex> lk(mu_);
      closed_ = true;
    }
    cv_.notify_all();
  }

  bool closed() const {
    std::lock_guard<std::mutex> lk(mu_);
    return closed_;
  }

  uint64_t dropped() const {
    std::lock_guard<std::mutex> lk(mu_);
    return dropped_;
  }

 private:
  mutable std::mutex mu_;
  std::condition_variable cv_;
  std::deque<mabur::node::RxBody> q_;
  bool closed_ = false;
  uint64_t dropped_ = 0;
};

}  // namespace maburgs
