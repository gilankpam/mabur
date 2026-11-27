#pragma once
// Calibration's wall actuator on the radio: CalSweep::WallCtl driven by
// devourer's cross-path capture, run on the TX writer thread (CalSweep's
// single owner thread) like every other sweep.

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
#include <shared_mutex>
#include <vector>

#include "cal_sweep.h"
#include "config.h"
#include "wall_sweep.h"

#include "IRtlRadio.h"

namespace mabur::maburd {

struct RealMode;

// The cross-path calls, each waiting for the retune gate and taking it
// shared for the one call, as DevicePowerCtl does: they are control-plane
// calls and must never overlap a FastRetune.
struct DeviceCaptureSession : CaptureSession {
  IRtlRadio* dev = nullptr;
  std::shared_mutex* gate = nullptr;
  std::atomic<bool>* gate_waiting = nullptr;
  int captures = 0;    // CrossPathCapture calls since the last reset
  bool end_ok = true;  // false once CrossPathEnd reported a failed restore

  bool begin(uint8_t bw) override;
  bool park(bool parked) override;
  bool capture(int tx_index, int gain, std::vector<devourer::IqSample>* out) override;
  void end() override;
};

// The fixed device-call order around one WallSweep: start() takes the radio
// to the commanded channel and width, finish() and cancel() put it back. Any
// decision logic lives in WallSweep and CalSweep, which are host-tested.
struct DeviceWallCtl : CalSweep::WallCtl {
  explicit DeviceWallCtl(RealMode& m);

  // The config the run's plans are derived from (re-read from disk, so a
  // parameter set made since boot counts).
  std::function<Config()> reload_config;

  cal::AckResult check(uint8_t channel, uint8_t bw) override;
  void start(uint8_t channel, uint8_t bw, uint64_t now_ms) override;
  bool step(uint64_t now_ms) override;
  cal::SweepDone finish() override;
  void cancel() override;

 private:
  void restore();
  void log_result(cal::WallResult r, const std::array<int8_t, 8>& walls, int16_t floor_x10,
                  bool ldpc) const;

  RealMode& m_;
  DeviceCaptureSession sess_;
  std::optional<WallSweep> sweep_;
  bool open_ = false;      // between start() and the restore
  bool retuned_ = false;   // start() moved the channel
  bool narrowed_ = false;  // start() took a 40 MHz tuning to 20
  uint8_t ch_ = 0, bw_ = 0;
  int anchor_ = -1;        // -1: start() could not set the radio up
  uint64_t t0_ms_ = 0;
};

}  // namespace mabur::maburd
