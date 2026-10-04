#pragma once
// USB delivery lateness (host arrival - chip RX TSF, above the per-second
// minimum), rxprobe's measure, on CRC-good frames. Producer: the RX thread
// (add); consumer: the stats tick (take). Moved from web/src/web_main.cpp.
#include <algorithm>
#include <cstdint>
#include <mutex>
#include <vector>

namespace maburgs {
struct UsbLate {
  void add(int64_t host_us, uint32_t tsf) {
    std::lock_guard<std::mutex> lk(mu_);
    if (tsf < last_tsf_) tsf_hi_ += int64_t{1} << 32;
    last_tsf_ = tsf;
    offs_.push_back(host_us - (tsf_hi_ + tsf));
  }
  void take(int64_t& p99_us, int64_t& max_us) {
    std::vector<int64_t> v;
    { std::lock_guard<std::mutex> lk(mu_); v.swap(offs_); }
    p99_us = max_us = 0;
    if (v.empty()) return;
    std::sort(v.begin(), v.end());
    const int64_t mn = v.front();
    p99_us = v[v.size() * 99 / 100] - mn;
    max_us = v.back() - mn;
  }
 private:
  std::mutex mu_;
  std::vector<int64_t> offs_;
  uint32_t last_tsf_ = 0;
  int64_t tsf_hi_ = 0;
};
}  // namespace maburgs
