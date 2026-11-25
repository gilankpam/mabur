// Channel-busy reading cadence for the IGI floor watchdog.
//
// Foreign airtime fills the chip's TX queue and a higher floor does not cure
// it, so a window on a busy channel is not judged. One 240 ms CCX window is
// armed per second and read at the next; a reading counts for 2 s. A radio
// that cannot arm the engine is asked no more. The caller owns the engine:
// this class only decides when to arm and read, and drops the reading on any
// retune (ineligible tick) or USB error.
#pragma once

#include <cstdint>

namespace mabur {

class BusyWindow {
 public:
  struct Dev {
    virtual ~Dev() = default;
    // Busy percent of the armed window; -1 when there is no valid reading. May throw.
    virtual int read_busy_pct() = 0;
    // Arms a window; false when the engine is unsupported. May throw.
    virtual bool arm(uint32_t window_us) = 0;
  };

  static constexpr uint32_t kWindowUs = 240000;

  // Returns the busy percent (-1 = unknown). *went_unsupported is set on the
  // one call where the engine is found unsupported.
  int step(uint64_t now_ms, bool eligible, Dev& dev, bool* went_unsupported) {
    if (!eligible) {
      // A retune spoils the armed window; the next eligible tick re-arms.
      armed_ = false;
      pct_ = -1;
      read_ms_ = 0;
    } else if (ok_ && now_ms - read_ms_ >= 1000) {
      read_ms_ = now_ms;
      try {
        if (armed_) pct_ = dev.read_busy_pct();
        armed_ = false;
        if (dev.arm(kWindowUs)) {
          armed_ = true;
        } else {
          ok_ = false;
          pct_ = -1;
          *went_unsupported = true;
        }
      } catch (...) {
        armed_ = false;
        pct_ = -1;
      }
    }
    if (pct_ >= 0 && now_ms - read_ms_ > 2000) pct_ = -1;
    return pct_;
  }

 private:
  bool ok_ = true, armed_ = false;
  int pct_ = -1;
  uint64_t read_ms_ = 0;
};

}  // namespace mabur
