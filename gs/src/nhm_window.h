#pragma once
#include <cstdint>

#include "nhm_busy.h"
#include "scout_radio.h"

namespace maburgs {

// Per-card bookkeeping for the verdict window's NHM arm (spec
// 2026-09-25-nhm-airtime §6): a read is ours only if WE armed it, with the
// period devourer still reports (a scout dwell re-arms with its own), on the
// channel the card is still on (a window spanning a retune binned two
// channels), AND no in-flight scout dwell has touched the card since we
// armed. The period/channel checks alone miss the FastRetune case (fix
// round 1, task-6 review): InflightScout::dwell() retunes via FastRetune,
// which does not clear devourer's NHM-ready state, so the period we armed
// with is still reported, and the card is back on its own channel by the
// next tick -- so a ~15 ms dwell that lands inside our NHM window is
// invisible to both checks, contaminating the busy reading with whatever
// the candidate channel saw. main.cpp bumps a per-card generation counter
// after every completed dwell (success or fail); `gen` pins the reading to
// the generation we armed under. Pure; main.cpp keeps one per card.
class NhmWindowTracker {
 public:
  void armed(uint8_t ch, uint16_t period, uint32_t gen) {
    armed_ = true; ch_ = ch; period_ = period; gen_ = gen;
  }
  void invalidate() { armed_ = false; }
  bool usable(const NhmBusyRead& r, uint8_t ch_now, uint32_t gen_now) const {
    return armed_ && r.valid && r.period == period_ && ch_now == ch_ && gen_now == gen_;
  }

 private:
  bool armed_ = false;
  uint8_t ch_ = 0;
  uint16_t period_ = 0;
  uint32_t gen_ = 0;
};

}  // namespace maburgs
