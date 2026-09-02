#pragma once
#include <atomic>
#include <chrono>
#if defined(__linux__)
#include <pthread.h>
#endif
#include <condition_variable>
#include <cstdint>
#include <cstdio>
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

  struct Frame {
    const uint8_t* data;
    size_t len;
  };

  // Grouped submit (fec.feed_batch): enqueues the group under ONE lock hold
  // with one wakeup per <=3 frames, so a grouped feed reaches one worker as
  // one full <=3-frame batch (one 3-descriptor URB) instead of splitting
  // 1+1+1 across idle workers. A group bigger than cap_ is enqueued in
  // cap_-sized chunks (blocking for each) rather than deadlocking on space
  // that can never exist. Returns frames accepted (< n only after stop()).
  size_t submit_many(const Frame* frames, size_t n) {
    size_t done = 0;
    while (done < n) {
      const size_t want = n - done < cap_ ? n - done : cap_;
      size_t notifies;
      {
        std::unique_lock<std::mutex> l(m_);
        space_.wait(l, [&] { return q_.size() + want <= cap_ || stopped_; });
        if (stopped_) return done;
        for (size_t i = 0; i < want; ++i)
          q_.emplace_back(frames[done + i].data,
                          frames[done + i].data + frames[done + i].len);
        notifies = (want + 2) / 3;  // one worker per full batch-of-3
      }
      for (size_t i = 0; i < notifies; ++i) fill_.notify_one();
      done += want;
    }
    return done;
  }

  uint64_t sent_ok() const { return sent_ok_.load(); }
  uint64_t send_fail() const { return send_fail_.load(); }
  size_t depth() const {
    std::lock_guard<std::mutex> l(m_);
    return q_.size();
  }

 private:
  static uint64_t now_us() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
  }

  void run() {
#if defined(__linux__)
    pthread_setname_np(pthread_self(), "mbr-usb");
#endif
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
      // usb_urb gauge (handover-usb-feed probe 2): wall time of each
      // send_packets call (= one blocking bulk-OUT URB round-trip on
      // jaguar3), the number of OTHER workers already inside send_ at
      // entry (overlap: do blocked URBs pipeline?), and busy-µs for the
      // window's mean-inflight figure. Unsaturated a URB completes in
      // tens of µs (FIFO has room); the saturated distribution and its
      // scaling with tx_threads is the measurement.
      const uint64_t t0 = now_us();
      const int ov = g_inflight_.fetch_add(1, std::memory_order_relaxed);
      const size_t ok = send_(batch);
      g_inflight_.fetch_sub(1, std::memory_order_relaxed);
      const uint64_t dt = now_us() - t0;
      g_calls_.fetch_add(1, std::memory_order_relaxed);
      g_bodies_.fetch_add(batch.size(), std::memory_order_relaxed);
      g_sum_us_.fetch_add(dt, std::memory_order_relaxed);
      uint64_t prev = g_max_us_.load(std::memory_order_relaxed);
      while (dt > prev &&
             !g_max_us_.compare_exchange_weak(prev, dt,
                                              std::memory_order_relaxed)) {}
      static constexpr uint64_t kEdge[5] = {50, 150, 300, 600, 1200};
      size_t bi = 5;
      for (size_t i = 0; i < 5; ++i)
        if (dt <= kEdge[i]) { bi = i; break; }
      g_dist_[bi].fetch_add(1, std::memory_order_relaxed);
      g_overlap_[ov < 7 ? ov : 7].fetch_add(1, std::memory_order_relaxed);
      // Split by batch size: fixed-per-URB acceptance would show sz3 mean
      // ≈ sz1 mean (amortizing 3x per body); per-byte would show ~3x.
      const size_t sz = batch.size() < 3 ? batch.size() : 3;
      g_sz_calls_[sz - 1].fetch_add(1, std::memory_order_relaxed);
      g_sz_sum_us_[sz - 1].fetch_add(dt, std::memory_order_relaxed);
      maybe_report(t0 + dt);

      sent_ok_ += ok;
      send_fail_ += batch.size() - ok;
      batch.clear();
    }
  }

  // One worker (whoever crosses the 5 s boundary first, CAS-elected)
  // prints and resets the shared window counters.
  void maybe_report(uint64_t now) {
    uint64_t last = g_last_report_us_.load(std::memory_order_relaxed);
    if (last == 0) {
      g_last_report_us_.compare_exchange_strong(last, now,
                                                std::memory_order_relaxed);
      return;
    }
    if (now - last < 5000000) return;
    if (!g_last_report_us_.compare_exchange_strong(
            last, now, std::memory_order_relaxed))
      return;  // another worker owns this window's report
    const uint64_t win_us = now - last;
    const uint64_t calls = g_calls_.exchange(0, std::memory_order_relaxed);
    const uint64_t bodies = g_bodies_.exchange(0, std::memory_order_relaxed);
    const uint64_t sum = g_sum_us_.exchange(0, std::memory_order_relaxed);
    const uint64_t mx = g_max_us_.exchange(0, std::memory_order_relaxed);
    uint64_t d[6], o[8], sc[3], ss[3];
    for (int i = 0; i < 6; ++i)
      d[i] = g_dist_[i].exchange(0, std::memory_order_relaxed);
    for (int i = 0; i < 8; ++i)
      o[i] = g_overlap_[i].exchange(0, std::memory_order_relaxed);
    for (int i = 0; i < 3; ++i) {
      sc[i] = g_sz_calls_[i].exchange(0, std::memory_order_relaxed);
      ss[i] = g_sz_sum_us_[i].exchange(0, std::memory_order_relaxed);
    }
    if (calls == 0) return;
    std::fprintf(
        stderr,
        "maburd usb_urb: calls=%llu bodies=%llu us/call mean=%llu max=%llu "
        "dist<=50/150/300/600/1200/inf=%llu/%llu/%llu/%llu/%llu/%llu "
        "overlap0..7=%llu/%llu/%llu/%llu/%llu/%llu/%llu/%llu "
        "mean_inflight=%llu.%02llu "
        "sz1=%llu@%llu sz2=%llu@%llu sz3=%llu@%llu\n",
        (unsigned long long)calls, (unsigned long long)bodies,
        (unsigned long long)(sum / calls), (unsigned long long)mx,
        (unsigned long long)d[0], (unsigned long long)d[1],
        (unsigned long long)d[2], (unsigned long long)d[3],
        (unsigned long long)d[4], (unsigned long long)d[5],
        (unsigned long long)o[0], (unsigned long long)o[1],
        (unsigned long long)o[2], (unsigned long long)o[3],
        (unsigned long long)o[4], (unsigned long long)o[5],
        (unsigned long long)o[6], (unsigned long long)o[7],
        (unsigned long long)(sum / win_us),
        (unsigned long long)(sum * 100 / win_us % 100),
        (unsigned long long)sc[0], (unsigned long long)(sc[0] ? ss[0] / sc[0] : 0),
        (unsigned long long)sc[1], (unsigned long long)(sc[1] ? ss[1] / sc[1] : 0),
        (unsigned long long)sc[2], (unsigned long long)(sc[2] ? ss[2] / sc[2] : 0));
  }

  SendBatch send_;
  const size_t cap_;
  mutable std::mutex m_;
  std::condition_variable fill_, space_;
  std::deque<std::vector<uint8_t>> q_;
  bool stopped_ = false;
  std::vector<std::thread> threads_;
  std::atomic<uint64_t> sent_ok_{0}, send_fail_{0};

  // usb_urb gauge window state (shared across workers, reset each report).
  std::atomic<int> g_inflight_{0};
  std::atomic<uint64_t> g_calls_{0}, g_bodies_{0}, g_sum_us_{0}, g_max_us_{0};
  std::atomic<uint64_t> g_dist_[6] = {};
  std::atomic<uint64_t> g_overlap_[8] = {};
  std::atomic<uint64_t> g_sz_calls_[3] = {}, g_sz_sum_us_[3] = {};
  std::atomic<uint64_t> g_last_report_us_{0};
};

}  // namespace mabur
