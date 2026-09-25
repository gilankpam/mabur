#pragma once
#include <cstddef>
#include <cstdint>

namespace maburgs {

// Our own video's airtime, reconstructed on the GS from the frames a card
// decoded (spec 2026-09-25-nhm-airtime §5): payload at the HT-1SS PHY rate
// plus one HT-mixed preamble per PPDU. devourer marks a PPDU's FIRST
// subframe with the PHY status (RxAtrib.physt) and only that subframe
// carries bw/stbc, so the accumulator latches them there and times the
// PPDU's later subframes at the same width.
constexpr uint32_t kHtPreambleUs = 40;  // L-STF/LTF/SIG + HT-SIG/STF/LTF1
constexpr uint32_t kStbcExtraUs = 4;    // STBC 1SS -> 2 space-time streams: one extra HT-LTF

inline double ht_rate_mbps(uint8_t mcs, uint8_t bw_mhz, bool sgi) {
  static constexpr double k20[8] = {6.5, 13, 19.5, 26, 39, 52, 58.5, 65};
  if (mcs > 7) return 0.0;
  double r = k20[mcs] * (bw_mhz == 40 ? 2.0769230769230769 : 1.0);  // 40 MHz: 108/52 subcarriers
  if (sgi) r = r * 10.0 / 9.0;
  return r;
}
inline double mpdu_air_us(size_t bytes, uint8_t mcs, uint8_t bw_mhz, bool sgi) {
  const double r = ht_rate_mbps(mcs, bw_mhz, sgi);
  return r > 0 ? static_cast<double>(bytes) * 8.0 / r : 0.0;
}
inline uint32_t ppdu_overhead_us(bool stbc) { return kHtPreambleUs + (stbc ? kStbcExtraUs : 0); }

class OwnAirAcc {
 public:
  void on_frame(size_t bytes, uint8_t mcs, bool physt, uint8_t bw_mhz, bool stbc, bool sgi) {
    if (mcs > 7) return;
    if (physt) { bw_ = bw_mhz; stbc_ = stbc; sgi_ = sgi; }
    acc_us_ += mpdu_air_us(bytes, mcs, bw_, sgi_) + (physt ? ppdu_overhead_us(stbc_) : 0);
  }
  uint64_t total_us() const { return static_cast<uint64_t>(acc_us_ + 0.5); }

 private:
  uint8_t bw_ = 20;
  bool stbc_ = false, sgi_ = false;
  double acc_us_ = 0.0;
};

}  // namespace maburgs
