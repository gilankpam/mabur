#pragma once
// Fixed-slot ring of sealed source envelopes keyed by wire seq (spec
// 2026-10-05 fec-nack §4.1). Single writer (the drone hot thread), any
// reader (the RX thread answering a T_NACK). No lock: each slot carries a
// seqlock-style state word; a reader that catches a slot mid-write, or a
// slot that was overwritten under it, gets `false` and treats it as a miss.
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>
namespace mabur {

class RetxRing {
 public:
  RetxRing(size_t slots, size_t env_len)
      : slots_(pow2_at_least(slots)), env_len_(env_len),
        state_(new std::atomic<uint32_t>[slots_]), seq_(new std::atomic<uint32_t>[slots_]),
        data_(slots_ * env_len_) {
    for (size_t i = 0; i < slots_; ++i) { state_[i].store(0); seq_[i].store(0); }
  }
  // Slots that hold ring_ms of base-layer sources at the maximum bitrate
  // (base_share of the stream is base; symbol_size is the source payload).
  static size_t slots_for(int bitrate_max_kbps, int ring_ms, int symbol_size, double base_share = 0.6) {
    const double bytes = bitrate_max_kbps * 1000.0 / 8.0 * base_share * (ring_ms / 1000.0);
    return pow2_at_least(static_cast<size_t>(bytes / symbol_size) + 1);
  }
  // Hot path: one memcpy of the envelope body (the shared bytes a seqlock
  // deliberately data-races over -- formally UB, same accepted tradeoff as
  // Linux's seqlock_t / folly's SeqLock, made safe by the state recheck in
  // get() below) plus three atomic stores (state_[i]=1, seq_[i]=seq,
  // state_[i]=2). No allocation, no lock.
  void put(uint32_t seq, const uint8_t* env, size_t len) {
    if (len != env_len_) return;
    const size_t i = seq & (slots_ - 1);
    state_[i].store(1, std::memory_order_relaxed);          // writing
    std::atomic_thread_fence(std::memory_order_release);
    std::memcpy(&data_[i * env_len_], env, env_len_);
    seq_[i].store(seq, std::memory_order_relaxed);
    state_[i].store(2, std::memory_order_release);          // valid
  }
  bool get(uint32_t seq, uint8_t* out) const {
    const size_t i = seq & (slots_ - 1);
    if (state_[i].load(std::memory_order_acquire) != 2 || seq_[i].load(std::memory_order_relaxed) != seq) return false;
    std::memcpy(out, &data_[i * env_len_], env_len_);
    // Recheck must itself be an acquire load, not a fence-guarded relaxed
    // one: if the writer overwrote this slot (and even finished, leaving
    // state back at 2) during the memcpy above, only an acquire load here
    // that actually observes that store synchronizes-with it and forces
    // the following seq_ read to see the NEW seq -- a relaxed load could
    // be reordered ahead of the writer's release and still read the OLD
    // (matching) seq after the writer's new state is already visible,
    // which is exactly the torn-read-passes-as-a-hit bug this guards.
    return state_[i].load(std::memory_order_acquire) == 2 && seq_[i].load(std::memory_order_relaxed) == seq;
  }
  size_t slots() const { return slots_; }
  size_t env_len() const { return env_len_; }

 private:
  static size_t pow2_at_least(size_t n) { size_t p = 1; while (p < n) p <<= 1; return p; }
  size_t slots_, env_len_;
  std::unique_ptr<std::atomic<uint32_t>[]> state_, seq_;
  std::vector<uint8_t> data_;
};
}  // namespace mabur
