#pragma once

#include <cstdint>
#include <deque>
#include <optional>

namespace maburgs {

struct ScoreConfig {
  double window_s = 0.5;
  double rssi_lo = -80.0;
  double rssi_hi = -40.0;
  double snr_lo = 5.0;
  double snr_hi = 30.0;
  double rssi_weight = 0.3;
  double snr_weight = 0.7;
  double loss_penalty = 1000.0;
};

class ScoreWindow {
 public:
  explicit ScoreWindow(ScoreConfig cfg = {});
  void add_frame(double rssi, double snr, bool crc_err, uint16_t seq,
                 double now_s);
  size_t n() const;
  std::optional<double> snr_estimate() const;
  std::optional<double> rssi_estimate() const;
  double fcs_loss() const;
  double seq_gap_loss() const;  // 12-bit wrap-safe forward sum
  uint16_t ack_seq() const;     // all-time max (Python parity)
  int score(std::optional<double> residual_loss = std::nullopt) const;

 private:
  struct Frame {
    double t, rssi, snr;
    bool crc_err;
    uint16_t seq;
  };
  ScoreConfig cfg_;
  std::deque<Frame> frames_;
  uint16_t max_seq_ = 0;
  bool has_max_ = false;
};

}  // namespace maburgs
