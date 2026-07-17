#pragma once
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace mabur {

class SwEncoder;

// One queued repair: everything SwEncoder::execute_repair_job needs to build
// the envelope later. Coordinates only — the encoder's ring keeps the
// referenced window rows readable (kSlackRows) until the job runs.
struct FecRepairJob {
  SwEncoder* eng = nullptr;
  uint32_t repair_key = 0;
  uint32_t header_seq = 0;  // h.seq (window start) captured at credit time
  int window_len = 0;
  size_t start_slot = 0;    // ring slot of the window's oldest row
};

// Shared FEC worker thread: single producer (maburd's hot thread owns every
// layer encoder), single consumer, bounded spin-then-sleep pickup.
//
// ARMv7 rules (measured on the SSC338Q, bench/fecbench/RESULTS.md — both
// are load-bearing, do not "simplify"):
//  - Spin loops read with memory_order_relaxed and fence-acquire once after
//    the awaited change is seen. An acquire load in the loop is LDR+DMB;
//    an idle spinner's barrier storm slowed the OTHER core 10-22%.
//  - The worker sleeps (futex) after kSpinIters; a permanent spinner keeps
//    the scheduler migrating the hot thread and erases the offload gain.
class FecWorker {
 public:
  // cpu >= 0 pins the worker thread to that core (Linux; no-op elsewhere).
  // queue_slots is a test surface — a tiny queue forces the caller's
  // inline-fallback path; production uses the default.
  explicit FecWorker(int cpu = -1, uint32_t queue_slots = 256);
  ~FecWorker();
  FecWorker(const FecWorker&) = delete;
  FecWorker& operator=(const FecWorker&) = delete;

  // Single-producer enqueue. False = queue full; the caller must build that
  // repair inline (graceful degradation, never blocks, never drops).
  bool try_enqueue(const FecRepairJob& job);

 private:
  static constexpr long kSpinIters = 16384;  // ~50-100 us @ 800 MHz A7

  void loop();

  const int cpu_;
  std::vector<FecRepairJob> q_;
  std::atomic<uint32_t> head_{0}, tail_{0};
  std::atomic<bool> quit_{false}, sleeping_{false};
  std::mutex m_;
  std::condition_variable cv_;
  std::thread th_;
};

}  // namespace mabur
