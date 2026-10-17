#pragma once
#include <cstdint>

#include "nhm_busy.h"
#include "scout_radio.h"

namespace maburgs {

// Per-card bookkeeping for the verdict window's NHM arm: a read is ours
// only if WE armed it, with the period devourer still reports (a scout
// dwell re-arms with its own), on the channel the card is still on (a
// window spanning a retune binned two channels), AND no in-flight scout
// dwell has touched the card since we armed. The period/channel checks
// alone miss the FastRetune case: InflightScout::dwell() retunes via
// FastRetune, which does not clear devourer's NHM-ready state, so the
// period we armed with is still reported, and the card is back on its own
// channel by the next tick -- so a ~15 ms dwell that lands inside our NHM
// window is invisible to both checks, contaminating the busy reading with
// whatever the candidate channel saw. ChannelCore::inflight_body_ bumps a
// per-card dwell generation after every completed dwell (success or fail);
// the arm is pinned to the generation it was made under. Pure;
// WindowSampler keeps one per card.
//
// The core thread's own hop retune, width resync and dwell burst can leave
// and re-enter the armed channel inside one window, which the channel check
// cannot see; ChannelCore bumps a per-card tune generation around each one
// and the arm is pinned to that too.
class NhmWindowTracker {
 public:
  void armed(uint8_t ch, uint16_t period, uint32_t dwell_gen, uint32_t tune_gen) {
    armed_ = true; ch_ = ch; period_ = period; dwell_gen_ = dwell_gen; tune_gen_ = tune_gen;
  }
  void invalidate() { armed_ = false; }
  bool usable(const NhmBusyRead& r, uint8_t ch_now, uint32_t dwell_gen_now,
              uint32_t tune_gen_now) const {
    return armed_ && r.valid && r.period == period_ && ch_now == ch_ &&
           dwell_gen_now == dwell_gen_ && tune_gen_now == tune_gen_;
  }

 private:
  bool armed_ = false;
  uint8_t ch_ = 0;
  uint16_t period_ = 0;
  uint32_t dwell_gen_ = 0;
  uint32_t tune_gen_ = 0;
};

}  // namespace maburgs
