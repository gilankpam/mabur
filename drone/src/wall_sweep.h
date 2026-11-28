// One TX-power wall measurement, one cross-path capture per step(): a park
// check, then two probe indices hunted by the probe's own EVM, a PA fit of
// each, and the 25-point curve filled from the fits' predictions. The radio
// sits behind CaptureSession so the state machine runs on the host against a fake.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "cross_evm.h"
#include "mabur/cal_proto.h"
#include "pa_fit.h"
#include "wall_curve.h"

namespace mabur {

// What the sweep needs of the radio. The real one wraps devourer's cross-path calls.
struct CaptureSession {
  virtual ~CaptureSession() = default;
  virtual bool begin(uint8_t bw) = 0;
  virtual bool park(bool parked) = 0;
  virtual bool capture(int tx_index, int gain, std::vector<devourer::IqSample>* out) = 0;
  virtual void end() = 0;     // safe after a failed begin or capture, and when called twice
};

// The measurement code the sweep calls, replaceable so tests can script it.
struct WallOps {
  CrossEvm (*measure)(const devourer::IqSample*, size_t, uint8_t) = cross_evm;
  PaFit (*fit)(const IqSpan*, int, uint8_t) = pa_fit;
  float (*predict)(const PaFit&, uint8_t, int) = pa_predict;
};

class WallSweep {
 public:
  static constexpr uint32_t kDeadlineMs = cal::kWallDeadlineMs;
  static constexpr float kHeadroomDb = 57.0f, kMinLevelDb = 30.0f;
  // Park rise: at or above kParkRiseDb parked; from kWeakParkRiseDb up, measured and flagged weak.
  static constexpr float kParkRiseDb = 12.0f, kWeakParkRiseDb = 6.0f;
  static constexpr int kMaxRetries = 2, kGains = 4;
  // Own-EVM windows the two probes are hunted into; the hunt aims at kProbe*Aim assuming
  // kProbe*Slope dB per index step.
  static constexpr float kProbe1Lo = -12.3f, kProbe1Hi = -11.0f, kProbe1Aim = -11.6f, kProbe1Slope = 0.25f;
  static constexpr float kProbe2Lo = -9.0f, kProbe2Hi = -8.3f, kProbe2Aim = -8.65f, kProbe2Slope = 0.1f;
  static constexpr int kHuntMaxIndices = 6, kProbeCaptures = 3, kProbeRetakes = 2;
  // Highest fit residual minus own EVM a probe (1 or 2) is trusted with.
  static constexpr float residual_limit_db(uint8_t bw, int probe) {
    return bw == 40 ? (probe == 1 ? -5.0f : -3.0f) : (probe == 1 ? -8.0f : -5.5f);
  }

  WallSweep(CaptureSession& s, uint8_t bw, int anchor, uint64_t now_ms, WallOps ops = {});
  // end() is called once on finishing, and only if begin() was called.
  ~WallSweep();                    // ends the session if it is still open
  WallSweep(const WallSweep&) = delete;
  WallSweep& operator=(const WallSweep&) = delete;

  bool step(uint64_t now_ms);      // at most one session call that captures; false once finished
  void cancel();                   // finishes with Aborted
  bool finished() const { return done_; }
  cal::WallResult result() const { return result_; }  // valid once finished
  const WallCurve& curve() const { return curve_; }
  // Parked minus unparked envelope; 0 before the check, and 0 when the rise is
  // not finite (an all-zero capture has envelope -inf).
  float park_rise_db() const { return park_rise_; }

 private:
  enum class Phase { Begin, Unparked, Parked, Hunt, Fit };
  void finish(cal::WallResult r);
  void start_hunt(int first, int lowest);
  void hunt(const CrossEvm& m);
  void start_fit(int idx, bool keep_last);
  void fit_probe();
  void assemble();

  CaptureSession& s_;
  uint8_t bw_;
  int anchor_;
  uint64_t start_;
  WallOps ops_;
  Phase phase_ = Phase::Begin;
  bool begun_ = false, parked_ = false, done_ = false;
  cal::WallResult result_ = cal::WallResult::Aborted;
  WallCurve curve_;
  float unparked_env_ = 0, park_rise_ = 0;
  int gain_ = kGains - 1;
  int tries_ = 0;      // failed captures at idx_, or park-check retries
  int probe_ = 1;      // the probe being hunted or fitted
  int idx_ = 0;        // the index being hunted or fitted
  int lowest_ = 0;     // lowest index this hunt may try
  int hunted_ = 0;     // indices this hunt has measured
  int retakes_ = 0;    // probes retaken over a bad residual
  int probe1_ = 0, probe2_ = 0;   // 0: not found
  PaFit fit1_, fit2_;
  std::array<float, kWallPoints> tried_{};   // own EVM per point index this hunt measured, NaN untried
  std::vector<devourer::IqSample> buf_;
  std::vector<std::vector<devourer::IqSample>> kept_;   // the probe's captures, at most kProbeCaptures
};

}  // namespace mabur
