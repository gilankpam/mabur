#pragma once
#include <cstdint>
#include <vector>

namespace maburgs {

// Per-card s1-class inputs to the RF-label selection below. Mirrors the
// fields of Aggregator's ClassTrack that the choice depends on, so the choice
// itself stays pure (no clock, no IO, no Aggregator) and is unit-testable —
// the main loop is otherwise unreachable from the host suite.
struct S1CardLabelInput {
  bool has_ema = false;      // card has folded at least one s1 frame ever
  uint64_t frames = 0;       // cumulative s1-class frame count
  uint64_t prev_frames = 0;  // same counter at the previous feedback window
  double snr_ema = 0.0;      // s1 SNR EMA, devourer raw half-dB
};

// Picks the card whose s1 RF labels (SNR/EVM/RSSI) go to the ladder
// controller this window, or -1 for "no card measured s1 this window" — the
// caller then NaNs all three labels, which leaves the predictive fade trigger
// inert (spec 2026-08-14 fade-demote §3).
//
// The freshness term is the point: ClassTrack's EMAs are never decayed, they
// simply freeze at their last value when frames stop arriving — exactly what
// a deep fade does. A frozen EMA must never reach the trigger as if it were a
// live measurement, and it must not win this argmax either: a wedged
// front-end whose EMA froze high would otherwise outrank a live sibling card
// forever, blanking the labels on a multi-card GS for as long as it stayed
// wedged.
inline int select_s1_label_card(const std::vector<S1CardLabelInput>& cards) {
  int best = -1;
  for (size_t i = 0; i < cards.size(); ++i) {
    const S1CardLabelInput& c = cards[i];
    if (!c.has_ema) continue;                 // snr_ema not meaningful yet
    if (c.frames <= c.prev_frames) continue;  // frozen EMA, not a measurement
    if (best < 0 || c.snr_ema > cards[static_cast<size_t>(best)].snr_ema)
      best = static_cast<int>(i);
  }
  return best;
}

}  // namespace maburgs
