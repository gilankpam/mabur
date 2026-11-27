#include "wall_sweep.h"

#include <algorithm>
#include <cmath>

namespace mabur {

namespace {
constexpr int kParkIndex = 28, kParkGain = 0;
}

WallSweep::WallSweep(CaptureSession& s, uint8_t bw, int anchor, uint64_t now_ms, Measure measure)
    : s_(s), bw_(bw), start_(now_ms), measure_(measure) {
  curve_.anchor = anchor;
}

WallSweep::~WallSweep() { finish(cal::WallResult::Aborted); }

void WallSweep::cancel() { finish(cal::WallResult::Aborted); }

void WallSweep::finish(cal::WallResult r) {
  if (done_) return;
  done_ = true;
  result_ = r;
  if (begun_) s_.end();
}

bool WallSweep::step(uint64_t now_ms) {
  if (done_) return false;
  // now_ms is monotonic, so the unsigned difference cannot wrap.
  if (now_ms - start_ >= kDeadlineMs) { finish(cal::WallResult::Deadline); return false; }

  if (phase_ == Phase::Begin) {
    begun_ = true;
    if (!s_.begin(bw_)) { finish(cal::WallResult::DeviceError); return false; }
    phase_ = Phase::Unparked;
  }
  if (phase_ == Phase::Parked && !s_.park(true)) { finish(cal::WallResult::Unparked); return false; }

  const bool points = phase_ == Phase::Points;
  const int idx = points ? wall_point_index(k_) : kParkIndex;
  const int gain = points ? gain_ : kParkGain;
  if (!s_.capture(idx, gain, &buf_)) { finish(cal::WallResult::DeviceError); return false; }
  const CrossEvm m = measure_(buf_.data(), buf_.size(), bw_);

  switch (phase_) {
    case Phase::Unparked:
      unparked_env_ = m.envelope_db;
      phase_ = Phase::Parked;
      break;
    case Phase::Parked: {
      // An all-zero capture has envelope -inf: a rise of +inf or NaN is no evidence of a park.
      const float rise = m.envelope_db - unparked_env_;
      if (!std::isfinite(rise)) { finish(cal::WallResult::Unparked); return false; }
      park_rise_ = rise;
      if (rise < kParkRiseDb) { finish(cal::WallResult::Unparked); return false; }
      phase_ = Phase::Points;
      break;
    }
    default:
      take_point(m);
  }
  return !done_;
}

void WallSweep::take_point(const CrossEvm& m) {
  // Level is only defined with a frame; a strong frameless capture may be foreign energy, not the probe.
  const bool hot = m.clipped || (m.frame && m.level_db > kHeadroomDb);
  if (hot && gain_ > 0) {
    // Values taken at the stronger gain are not comparable with the weaker one:
    // restart this index at gain - 1 with a fresh retry budget. Gain never goes back up.
    --gain_;
    vals_.clear();
    tries_ = 0;
    return;
  }
  if (m.clipped) { vals_.clear(); next_point(); return; }   // clipped at the weakest gain: the point stays empty

  ++tries_;
  // A demodulated "frame" below kMinLevelDb is read out of noise, not the probe: a retry.
  if (m.frame && m.level_db >= kMinLevelDb) vals_.push_back(m.evm_db);
  if ((int)vals_.size() == kCapturesPerPoint || tries_ == kCapturesPerPoint + kMaxRetries) next_point();
}

void WallSweep::next_point() {
  if (!vals_.empty()) {
    std::sort(vals_.begin(), vals_.end());
    const size_t n = vals_.size();
    const float v = n % 2 ? vals_[n / 2] : (vals_[n / 2 - 1] + vals_[n / 2]) / 2;
    curve_.evm_db[k_] = std::round(v * 10) / 10.0f;
  }
  vals_.clear();
  tries_ = 0;
  if (++k_ == kWallPoints) finish(check_curve(curve_));
}

}  // namespace mabur
