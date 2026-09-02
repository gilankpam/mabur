#include "rtt_estimator.h"

namespace maburgs {

void RttEstimator::on_rcf_sent(uint16_t seq, uint64_t t_us) {
  ring_[ring_next_] = Sent{seq, t_us, true};
  ring_next_ = (ring_next_ + 1) % kRingCap;
}

bool RttEstimator::on_telem(uint16_t seq_echo, bool echo_valid,
                            uint16_t age_ms, uint64_t pts_at_build_us,
                            uint64_t rx_us) {
  if (!echo_valid || age_ms >= kMaxAgeMs) return false;

  const Sent* hit = nullptr;
  for (const auto& s : ring_) {
    if (s.used && s.seq == seq_echo) {
      hit = &s;
      break;
    }
  }
  if (!hit || rx_us < hit->t_us) return false;

  int64_t rtt_us = static_cast<int64_t>(rx_us - hit->t_us) -
                   static_cast<int64_t>(age_ms) * 1000;
  if (rtt_us < kNegClampUs || rtt_us > kMaxPlausibleUs) return false;
  if (rtt_us < 0) rtt_us = 0;

  const double r = static_cast<double>(rtt_us);
  if (n_ == 0) {
    rtt_ewma_us_ = r;
    rtt_min_us_ = r;
  } else {
    rtt_ewma_us_ += kEwmaAlpha * (r - rtt_ewma_us_);
    if (r < rtt_min_us_) rtt_min_us_ = r;
  }
  ++n_;

  if (pts_at_build_us != 0) {
    const int64_t off = static_cast<int64_t>(pts_at_build_us) -
                        static_cast<int64_t>(rx_us) + rtt_us / 2;
    off_win_[off_next_] = OffSample{rtt_us, off};
    off_next_ = (off_next_ + 1) % kOffWindow;
    if (off_n_ < kOffWindow) ++off_n_;
  }
  return true;
}

int64_t RttEstimator::pts_off_us() const {
  int best = 0;
  for (int i = 1; i < off_n_; ++i)
    if (off_win_[i].rtt_us < off_win_[best].rtt_us) best = i;
  return off_win_[best].off_us;
}

}  // namespace maburgs
