// One TX-power wall measurement: a park check, then 25 TXAGC points, one
// cross-path capture per step(). The radio sits behind CaptureSession so the
// state machine runs on the host against a fake.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "cross_evm.h"
#include "mabur/cal_proto.h"
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

class WallSweep {
 public:
  using Measure = CrossEvm (*)(const devourer::IqSample*, size_t, uint8_t);
  static constexpr uint32_t kDeadlineMs = cal::kWallDeadlineMs;
  static constexpr float kHeadroomDb = 57.0f, kMinLevelDb = 30.0f, kParkRiseDb = 10.0f;
  static constexpr int kCapturesPerPoint = 3, kMaxRetries = 2, kGains = 4;

  WallSweep(CaptureSession& s, uint8_t bw, int anchor, uint64_t now_ms, Measure measure = cross_evm);
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
  enum class Phase { Begin, Unparked, Parked, Points };
  void finish(cal::WallResult r);
  void take_point(const CrossEvm& m);
  void next_point();

  CaptureSession& s_;
  uint8_t bw_;
  uint64_t start_;
  Measure measure_;
  Phase phase_ = Phase::Begin;
  bool begun_ = false, parked_ = false, done_ = false;
  cal::WallResult result_ = cal::WallResult::Aborted;
  WallCurve curve_;
  float unparked_env_ = 0, park_rise_ = 0;
  int k_ = 0, gain_ = kGains - 1, tries_ = 0;   // tries_: captures of point k_ at gain_, or park-check retries
  std::vector<float> vals_;
  std::vector<devourer::IqSample> buf_;
};

}  // namespace mabur
