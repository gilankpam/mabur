#ifndef MABURGS_LAT_WINDOW_H_
#define MABURGS_LAT_WINDOW_H_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace maburgs {

// Rolling-window per-segment aggregates for the sideport (head segments
// only: enc/dq/air/fec — the tail lives in maburplay). Percentiles are
// per-segment here (aggregate export); the player's real-frame rule
// applies to DISPLAYED BREAKDOWNS, which this is not.
//
// Fixed-cap accumulation: add() beyond kCap samples per segment is a
// silent no-op rather than growing unbounded or evicting -- a stats tick
// firing late (send stall, huge interval_ms) degrades to "the window is
// capped", never to unbounded memory growth on the core loop.
class LatWindow {
 public:
  // idx: 0=enc, 1=dq, 2=air, 3=fec (µs each).
  void add(uint32_t enc_us, uint32_t dq_us, uint32_t air_us, uint32_t fec_us) {
    if (seg_[0].size() >= kCap) return;
    seg_[0].push_back(enc_us);
    seg_[1].push_back(dq_us);
    seg_[2].push_back(air_us);
    seg_[3].push_back(fec_us);
  }

  struct Out {
    int n = 0;
    uint32_t p50[4] = {0, 0, 0, 0};
    uint32_t p99[4] = {0, 0, 0, 0};
  };

  // Returns the aggregates over every sample seen since the last flush()
  // (or construction) and clears the window for the next one.
  Out flush() {
    Out out;
    out.n = static_cast<int>(seg_[0].size());
    for (int i = 0; i < 4; ++i) {
      out.p50[i] = percentile(seg_[i], 50);
      out.p99[i] = percentile(seg_[i], 99);
      seg_[i].clear();
    }
    return out;
  }

  static constexpr std::size_t kCap = 4096;

 private:
  static uint32_t percentile(std::vector<uint32_t>& v, int pct) {
    if (v.empty()) return 0;
    const std::size_t n = v.size();
    std::size_t idx = (n * static_cast<std::size_t>(pct)) / 100;
    if (idx >= n) idx = n - 1;
    std::nth_element(v.begin(), v.begin() + static_cast<long>(idx), v.end());
    return v[idx];
  }

  std::vector<uint32_t> seg_[4];
};

}  // namespace maburgs

#endif  // MABURGS_LAT_WINDOW_H_
