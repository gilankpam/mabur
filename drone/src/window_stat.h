#pragma once
#include <cstdint>

namespace mabur {

// Count, sum and max of one gauge over a reporting window. The owning
// thread add()s per sample and reset()s after printing; mean() is
// truncated and 0 for an empty window, so a report can print it without a
// divide-by-zero guard. Inline and allocation-free for per-frame use.
struct WindowStat {
  uint64_t n = 0, sum = 0, max = 0;
  void add(uint64_t v) {
    ++n;
    sum += v;
    if (v > max) max = v;
  }
  uint64_t mean() const { return n ? sum / n : 0; }
  void reset() { n = sum = max = 0; }
};

}  // namespace mabur
