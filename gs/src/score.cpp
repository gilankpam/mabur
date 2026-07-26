#include "score.h"

#include <algorithm>

namespace maburgs {
namespace {
double lin(double x, double lo, double hi) {
  if (hi == lo) return 1000.0;
  const double t = (x - lo) / (hi - lo);
  return 1000.0 + 1000.0 * std::max(0.0, std::min(1.0, t));
}
}  // namespace

ScoreWindow::ScoreWindow(ScoreConfig cfg) : cfg_(cfg) {}

void ScoreWindow::add_frame(double rssi, double snr, bool crc_err, uint16_t seq,
                            double now_s) {
  frames_.push_back(Frame{now_s, rssi, snr, crc_err, seq});
  if (!has_max_ || seq > max_seq_) { max_seq_ = seq; has_max_ = true; }
  const double cutoff = now_s - cfg_.window_s;
  while (!frames_.empty() && frames_.front().t < cutoff) frames_.pop_front();
}

size_t ScoreWindow::n() const { return frames_.size(); }

std::optional<double> ScoreWindow::snr_estimate() const {
  if (frames_.empty()) return std::nullopt;
  double s = 0;
  for (const auto& f : frames_) s += f.snr;
  return s / static_cast<double>(frames_.size());
}

std::optional<double> ScoreWindow::rssi_estimate() const {
  if (frames_.empty()) return std::nullopt;
  double s = 0;
  for (const auto& f : frames_) s += f.rssi;
  return s / static_cast<double>(frames_.size());
}

double ScoreWindow::fcs_loss() const {
  if (frames_.empty()) return 0.0;
  size_t bad = 0;
  for (const auto& f : frames_) bad += f.crc_err ? 1 : 0;
  return static_cast<double>(bad) / static_cast<double>(frames_.size());
}

double ScoreWindow::seq_gap_loss() const {
  if (frames_.size() < 2) return 0.0;
  long span = 1;
  for (size_t i = 1; i < frames_.size(); ++i)
    span += (frames_[i].seq - frames_[i - 1].seq) & 0x0FFF;  // forward, wrap-safe
  if (span == 0) return 0.0;
  const double loss = 1.0 - static_cast<double>(frames_.size()) / static_cast<double>(span);
  return loss > 0.0 ? loss : 0.0;
}

uint16_t ScoreWindow::ack_seq() const { return has_max_ ? max_seq_ : 0; }

int ScoreWindow::score(std::optional<double> residual_loss) const {
  if (frames_.empty()) return 1000;
  const double rssi_s = lin(*rssi_estimate(), cfg_.rssi_lo, cfg_.rssi_hi);
  const double snr_s = lin(*snr_estimate(), cfg_.snr_lo, cfg_.snr_hi);
  double s = cfg_.rssi_weight * rssi_s + cfg_.snr_weight * snr_s;
  const double loss = residual_loss.has_value() ? *residual_loss : seq_gap_loss();
  s -= cfg_.loss_penalty * loss;
  const double clamped = std::max(1000.0, std::min(2000.0, s));
  return static_cast<int>(clamped);
}

}  // namespace maburgs
