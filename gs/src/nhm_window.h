#pragma once
#include <cstdint>

#include "nhm_busy.h"
#include "scout_radio.h"

namespace maburgs {

// Per-card bookkeeping for the verdict window's NHM arm (spec
// 2026-09-25-nhm-airtime §6): a read is ours only if WE armed it, with the
// period devourer still reports (a scout dwell re-arms with its own), on the
// channel the card is still on (a window spanning a retune binned two
// channels). Pure; main.cpp keeps one per card.
class NhmWindowTracker {
 public:
  void armed(uint8_t ch, uint16_t period) { armed_ = true; ch_ = ch; period_ = period; }
  void invalidate() { armed_ = false; }
  bool usable(const NhmBusyRead& r, uint8_t ch_now) const {
    return armed_ && r.valid && r.period == period_ && ch_now == ch_;
  }

 private:
  bool armed_ = false;
  uint8_t ch_ = 0;
  uint16_t period_ = 0;
};

}  // namespace maburgs
