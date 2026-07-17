#include "mabur/fec_worker.h"

#include "mabur/sw_encoder.h"

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

namespace mabur {

FecWorker::FecWorker(int cpu, uint32_t queue_slots)
    : cpu_(cpu), q_(queue_slots), th_([this] { loop(); }) {}

FecWorker::~FecWorker() {
  quit_.store(true, std::memory_order_seq_cst);
  {
    std::lock_guard<std::mutex> l(m_);
  }
  cv_.notify_one();
  th_.join();
}

bool FecWorker::try_enqueue(const FecRepairJob& job) {
  const uint32_t t = tail_.load(std::memory_order_relaxed);
  if (t - head_.load(std::memory_order_acquire) >= q_.size()) return false;
  q_[t % q_.size()] = job;
  // seq_cst store + seq_cst sleeping_ read: the worker sets sleeping_ under
  // m_ and re-checks tail_ in its wait predicate, so either it sees this
  // job before sleeping or we see sleeping_ and notify. Weaker orders can
  // reorder the StoreLoad pair on ARM and lose the wake.
  tail_.store(t + 1, std::memory_order_seq_cst);
  if (sleeping_.load(std::memory_order_seq_cst)) {
    {
      std::lock_guard<std::mutex> l(m_);
    }
    cv_.notify_one();
  }
  return true;
}

void FecWorker::loop() {
#if defined(__linux__)
  if (cpu_ >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu_, &set);
    pthread_setaffinity_np(pthread_self(), sizeof set, &set);
  }
#endif
  uint32_t seen = 0;
  for (;;) {
    long spins = 0;
    while (tail_.load(std::memory_order_relaxed) == seen) {
      if (quit_.load(std::memory_order_relaxed)) return;
      if (++spins >= kSpinIters) {
        std::unique_lock<std::mutex> l(m_);
        sleeping_.store(true, std::memory_order_seq_cst);
        cv_.wait(l, [&] {
          return tail_.load(std::memory_order_relaxed) != seen ||
                 quit_.load(std::memory_order_relaxed);
        });
        sleeping_.store(false, std::memory_order_seq_cst);
        spins = 0;
      }
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    FecRepairJob j = q_[seen % q_.size()];
    head_.store(seen + 1, std::memory_order_release);
    ++seen;
    j.eng->execute_repair_job(j);
  }
}

}  // namespace mabur
