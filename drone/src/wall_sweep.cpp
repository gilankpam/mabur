#include "wall_sweep.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mabur {

namespace {
constexpr int kParkIndex = 28, kParkGain = 0;
constexpr int kTopIndex = wall_point_index(kWallPoints - 1);
constexpr float kUntried = std::numeric_limits<float>::quiet_NaN();
}

WallSweep::WallSweep(CaptureSession& s, uint8_t bw, int anchor, uint64_t now_ms, WallOps ops)
    : s_(s), bw_(bw), anchor_(anchor), start_(now_ms), ops_(ops) {
  curve_.anchor = anchor;
}

WallSweep::~WallSweep() { finish(cal::WallResult::Aborted); }

void WallSweep::cancel() { finish(cal::WallResult::Aborted); }

void WallSweep::finish(cal::WallResult r) {
  if (done_) return;
  done_ = true;
  result_ = r;
  kept_.clear();
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
  if (phase_ == Phase::Parked && !parked_) {
    parked_ = true;
    if (!s_.park(true)) { finish(cal::WallResult::Unparked); return false; }
  }

  if (phase_ == Phase::Fit) {
    kept_.emplace_back();
    if (!s_.capture(idx_, gain_, &kept_.back())) { finish(cal::WallResult::DeviceError); return false; }
    fit_probe();
    return !done_;
  }

  const bool hunting = phase_ == Phase::Hunt;
  if (!s_.capture(hunting ? idx_ : kParkIndex, hunting ? gain_ : kParkGain, &buf_)) {
    finish(cal::WallResult::DeviceError);
    return false;
  }
  const CrossEvm m = ops_.measure(buf_.data(), buf_.size(), bw_);
  if (hunting) { hunt(m); return !done_; }

  // A park-check capture can hold no TX at all (only the removed DC left, or a "frame" read
  // out of that noise below kMinLevelDb): it says nothing about the park, so retry it. The
  // last attempt is used whatever it holds.
  const bool clean_frame = m.frame && m.level_db >= kMinLevelDb;
  if (!clean_frame && m.envelope_db < kMinLevelDb && tries_ < kMaxRetries) { ++tries_; return true; }
  tries_ = 0;

  if (phase_ == Phase::Unparked) {
    unparked_env_ = m.envelope_db;
    phase_ = Phase::Parked;
    return true;
  }
  // An all-zero capture has envelope -inf: a rise of +inf or NaN is no evidence of a park.
  const float rise = m.envelope_db - unparked_env_;
  if (!std::isfinite(rise)) { finish(cal::WallResult::Unparked); return false; }
  park_rise_ = rise;
  if (rise < kWeakParkRiseDb) { finish(cal::WallResult::Unparked); return false; }
  curve_.weak_park = rise < kParkRiseDb;
  const int first = (int)std::lround((anchor_ + 40) / 4.0) * 4;
  start_hunt(std::clamp(first, 60, kTopIndex), kWallIdxLo);
  return true;
}

void WallSweep::start_hunt(int first, int lowest) {
  phase_ = Phase::Hunt;
  idx_ = first;
  lowest_ = lowest;
  hunted_ = 0;
  tries_ = 0;
  tried_.fill(kUntried);
}

void WallSweep::hunt(const CrossEvm& m) {
  // Level is only defined with a frame; a strong frameless capture may be foreign energy, not the probe.
  const bool hot = m.clipped || (m.frame && m.level_db > kHeadroomDb);
  if (hot && gain_ > 0) {
    // Same index one gain word weaker, with a fresh retry budget. Gain never goes back up:
    // a fit's captures must share one gain.
    --gain_;
    tries_ = 0;
    return;
  }
  // A demodulated "frame" below kMinLevelDb is read out of noise, not the probe.
  if (m.clipped || !m.frame || m.level_db < kMinLevelDb || !std::isfinite(m.evm_db)) {
    if (tries_++ < kMaxRetries) return;
    finish(cal::WallResult::TooFewPoints);
    return;
  }
  tries_ = 0;

  const bool one = probe_ == 1;
  const float lo = one ? kProbe1Lo : kProbe2Lo, hi = one ? kProbe1Hi : kProbe2Hi;
  const float aim = one ? kProbe1Aim : kProbe2Aim, slope = one ? kProbe1Slope : kProbe2Slope;
  const float own = m.evm_db;
  tried_[(idx_ - kWallIdxLo) / kWallIdxStep] = own;
  ++hunted_;
  auto tried_at = [&](int idx) {   // NaN when untried or off the grid; NaN compares false
    return idx < kWallIdxLo || idx > kTopIndex ? kUntried : tried_[(idx - kWallIdxLo) / kWallIdxStep];
  };

  // Inside the window; at the top index still below it; or a tried pair one grid step apart
  // straddling it, where the upper index is the probe.
  if (own >= lo && own <= hi) { start_fit(idx_, true); return; }
  if (own < lo && idx_ == kTopIndex) { start_fit(idx_, true); return; }
  if (own < lo && tried_at(idx_ + kWallIdxStep) > hi) { start_fit(idx_ + kWallIdxStep, false); return; }
  if (own > hi && tried_at(idx_ - kWallIdxStep) < lo) { start_fit(idx_, true); return; }
  // Below the window at the top index already ended the hunt above.
  if (hunted_ == kHuntMaxIndices) { finish(cal::WallResult::TooFewPoints); return; }

  const int dir = own < lo ? 1 : -1;
  const int mag = std::clamp(std::abs((int)std::lround((aim - own) / slope / 4)) * 4, 4, 16);
  int next = std::clamp(idx_ + dir * mag, lowest_, kTopIndex);
  while (next >= lowest_ && next <= kTopIndex && !std::isnan(tried_at(next))) next += dir * kWallIdxStep;
  if (next < lowest_ || next > kTopIndex) { finish(cal::WallResult::TooFewPoints); return; }
  idx_ = next;
}

void WallSweep::start_fit(int idx, bool keep_last) {
  phase_ = Phase::Fit;
  idx_ = idx;
  retakes_ = 0;
  kept_.clear();
  // The capture that selected the probe is reused; a probe picked by a bracket is an earlier
  // index, possibly at a stronger gain, so its captures are taken afresh.
  if (!keep_last) return;
  kept_.emplace_back().swap(buf_);
  fit_probe();
}

void WallSweep::fit_probe() {
  IqSpan spans[kProbeCaptures];
  const int n = (int)kept_.size();
  for (int i = 0; i < n; ++i) spans[i] = {kept_[i].data(), kept_[i].size()};
  const PaFit f = ops_.fit(spans, n, bw_);
  if (!f.ok) {
    if (f.frames < 2 && n < kProbeCaptures) return;   // take another capture
    finish(cal::WallResult::TooFewPoints);
    return;
  }
  kept_.clear();
  // A residual the fit cannot explain is foreign RF inside the probe or a garbage fit.
  if (f.residual_db - f.own_evm_db > residual_limit_db(bw_, probe_)) {
    if (retakes_++ < kProbeRetakes) return;
    finish(cal::WallResult::Disturbed);
    return;
  }
  if (probe_ == 2) {
    fit2_ = f;
    probe2_ = idx_;
    assemble();
    return;
  }
  fit1_ = f;
  probe1_ = idx_;
  if (probe1_ == kTopIndex) { assemble(); return; }   // no index left for probe 2
  // Probe 1 is a measured point of the probe-2 hunt too: it and probe1 + 4 can bracket the window.
  const float own1 = tried_[(probe1_ - kWallIdxLo) / kWallIdxStep];
  probe_ = 2;
  start_hunt(std::min(kTopIndex, probe1_ + 24), probe1_ + kWallIdxStep);
  tried_[(probe1_ - kWallIdxLo) / kWallIdxStep] = own1;
}

void WallSweep::assemble() {
  std::array<float, kWallPoints> v;
  for (int k = 0; k < kWallPoints; ++k) {
    const int idx = wall_point_index(k);
    if (idx <= probe1_) v[k] = ops_.predict(fit1_, bw_, probe1_ - idx);
    else if (probe2_ && idx <= probe2_) v[k] = ops_.predict(fit2_, bw_, probe2_ - idx);
    else v[k] = probe2_ ? fit2_.own_evm_db : fit1_.own_evm_db;   // above the top probe: not predicted
  }
  // Going down from each probe a point never reads worse than the one above it: a PA's EVM
  // only worsens with the index, and a fit's prediction rises again far below its probe,
  // where few samples reach the low knots.
  for (int k = kWallPoints - 2; k >= 0; --k) {
    const int idx = wall_point_index(k);
    if (idx < probe1_ || (probe2_ && idx > probe1_ && idx < probe2_)) v[k] = std::min(v[k], v[k + 1]);
  }
  for (float x : v)
    if (!std::isfinite(x)) { finish(cal::WallResult::TooFewPoints); return; }
  for (int k = 0; k < kWallPoints; ++k) curve_.evm_db[k] = std::round(v[k] * 10) / 10.0f;
  finish(check_curve(curve_));
}

}  // namespace mabur
