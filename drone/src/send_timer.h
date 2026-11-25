// Longest radio send per watchdog interval. Sender threads bracket each send
// with begin/end; the watchdog thread calls take_longest_us. Lock-free so a
// sender stuck inside the chip can never block the watchdog, and a send that
// never returns still reads as slow through its growing age.
#pragma once
#include <atomic>
#include <cstdint>

namespace mabur {

class SendTimer {
 public:
  static constexpr int kSlots = 16;  // radio.tx_threads is at most 8

  // Slot for end(); -1 when none is free (that send goes untimed).
  int begin(uint64_t now_us) {
    const uint64_t t = now_us ? now_us : 1;  // 0 marks a free slot
    for (int i = 0; i < kSlots; ++i) {
      uint64_t free_v = 0;
      if (start_[i].compare_exchange_strong(free_v, t)) return i;
    }
    return -1;
  }

  void end(int slot, uint64_t now_us) {
    if (slot < 0) return;
    const uint64_t s = start_[slot].load();
    const uint64_t d = now_us > s ? now_us - s : 0;
    // Publish the duration before freeing the slot, so a concurrent take sees
    // the send as running or finished, never neither.
    uint64_t m = max_done_.load();
    while (d > m && !max_done_.compare_exchange_weak(m, d)) {}
    start_[slot].store(0);
  }

  // Longest send since the previous call: sends that finished in between,
  // and the age so far of any still in progress.
  uint64_t take_longest_us(uint64_t now_us) {
    uint64_t longest = max_done_.exchange(0);
    for (auto& a : start_) {
      const uint64_t s = a.load();
      if (s && now_us > s && now_us - s > longest) longest = now_us - s;
    }
    return longest;
  }

 private:
  std::atomic<uint64_t> start_[kSlots] = {};
  std::atomic<uint64_t> max_done_{0};
};

}  // namespace mabur
