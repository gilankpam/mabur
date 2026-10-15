#ifndef MABUR_PLAYER_LATENESS_WINDOW_H_
#define MABUR_PLAYER_LATENESS_WINDOW_H_

#include <algorithm>
#include <array>
#include <cstdint>

namespace maburplay {

// Rolling p1 / p99 of frame transit (arrival - capture pts, µs, so it
// carries the two clocks' offset; p99 - p1 cancels it) over the last
// kSize frames -- ~10 s at 60 fps. The capture-time
// scheduler takes p1 as the delivery floor and p99 - p1 as the clean-link
// spread its allowance must cover. Both are exact order statistics of the
// window, recomputed per sample with nth_element on a scratch copy: 600
// values, a few µs per frame. A sample enters at once (the allowance grows
// within one frame of a late arrival) and leaves only when it ages out
// (shrinks after ~10 s), which keeps the allowance from flapping.
class LatenessWindow {
 public:
  static constexpr int kSize = 600;

  void add(int64_t v) {
    ring_[static_cast<size_t>(head_)] = v;
    head_ = (head_ + 1) % kSize;
    if (n_ < kSize) ++n_;
    std::copy(ring_.begin(), ring_.begin() + n_, scratch_.begin());
    const int k = n_ / 100;  // 1 % of the window, 6 at full size
    auto b = scratch_.begin(), e = scratch_.begin() + n_;
    std::nth_element(b, b + k, e);
    lo_ = scratch_[static_cast<size_t>(k)];
    std::nth_element(b, b + (n_ - 1 - k), e);
    hi_ = scratch_[static_cast<size_t>(n_ - 1 - k)];
  }

  bool empty() const { return n_ == 0; }
  int64_t lo() const { return lo_; }  // p1: the delivery floor
  int64_t hi() const { return hi_; }  // p99: the late tail

 private:
  std::array<int64_t, kSize> ring_{};
  std::array<int64_t, kSize> scratch_{};
  int head_ = 0;
  int n_ = 0;
  int64_t lo_ = 0, hi_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_LATENESS_WINDOW_H_
