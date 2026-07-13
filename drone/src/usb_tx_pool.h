#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace mabur {

// N parallel USB sender threads behind a small bounded frame queue. The
// 8822E flow-controls synchronous bulk-OUT URBs (~0.4 ms acceptance
// handshake + FIFO drain), so a single blocking sender idles the radio
// during every host round-trip and caps air throughput at ~26 Mbps
// regardless of MCS (linkbench bisect 2026-07-14); ~4 threads saturate
// (devourer docs/aggregation.md — sync bulk from multiple threads is legal
// and simply queues, giving ~N URBs in flight). Senders batch ≤3 frames
// per call, the HalMAC per-transfer descriptor limit devourer's
// send_packets packs into one URB.
//
// submit() COPIES the frame (callers reuse their build buffers) and BLOCKS
// while the queue is full: backpressure propagates to the TX writer and
// backlog lands upstream in TxQueue, where the drop-oldest/FEC-erasure
// policy lives. Keep the capacity small — just enough to keep N senders
// fed — or this queue silently becomes a second, policy-free backlog.
//
// NOTE: threads > 1 means ≤3-frame URB batches can swap order on air. The
// FEC datapath is block-id-addressed (order-agnostic) and the GS
// aggregator's delivery accounting is a max-seq high-water mark, so both
// tolerate it.
class UsbTxPool {
 public:
  // Blocking batch send; frames are valid for the duration of the call.
  // Returns how many of the batch were accepted by the radio.
  using SendBatch =
      std::function<size_t(const std::vector<std::vector<uint8_t>>&)>;

  UsbTxPool(SendBatch send, int threads, size_t cap_frames)
      : send_(std::move(send)), cap_(cap_frames) {
    if (threads < 1) threads = 1;
    threads_.reserve(static_cast<size_t>(threads));
    for (int i = 0; i < threads; ++i)
      threads_.emplace_back([this] { run(); });
  }

  ~UsbTxPool() { stop(); }

  // Idempotent. Senders drain what's queued (bounded by cap_frames), then
  // exit; joins them all. Call before tearing down the radio device.
  void stop() {
    {
      std::lock_guard<std::mutex> l(m_);
      if (stopped_) return;
      stopped_ = true;
    }
    fill_.notify_all();
    space_.notify_all();
    for (auto& t : threads_)
      if (t.joinable()) t.join();
  }

  // Copies the frame in; blocks while full. False after stop().
  bool submit(const uint8_t* frame, size_t len) {
    std::unique_lock<std::mutex> l(m_);
    space_.wait(l, [&] { return q_.size() < cap_ || stopped_; });
    if (stopped_) return false;
    q_.emplace_back(frame, frame + len);
    fill_.notify_one();
    return true;
  }

  uint64_t sent_ok() const { return sent_ok_.load(); }
  uint64_t send_fail() const { return send_fail_.load(); }
  size_t depth() const {
    std::lock_guard<std::mutex> l(m_);
    return q_.size();
  }

 private:
  void run() {
    std::vector<std::vector<uint8_t>> batch;
    for (;;) {
      {
        std::unique_lock<std::mutex> l(m_);
        fill_.wait(l, [&] { return stopped_ || !q_.empty(); });
        if (q_.empty()) return;  // stopped and drained
        const size_t n = q_.size() < 3 ? q_.size() : 3;
        for (size_t i = 0; i < n; ++i) {
          batch.push_back(std::move(q_.front()));
          q_.pop_front();
        }
      }
      space_.notify_one();
      const size_t ok = send_(batch);
      sent_ok_ += ok;
      send_fail_ += batch.size() - ok;
      batch.clear();
    }
  }

  SendBatch send_;
  const size_t cap_;
  mutable std::mutex m_;
  std::condition_variable fill_, space_;
  std::deque<std::vector<uint8_t>> q_;
  bool stopped_ = false;
  std::vector<std::thread> threads_;
  std::atomic<uint64_t> sent_ok_{0}, send_fail_{0};
};

}  // namespace mabur
