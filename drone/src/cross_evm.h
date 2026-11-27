// Signal quality of one cross-path capture: the drone's PA A transmits the
// probe frame (HT MCS1, one stream) and chain B records it. The EVM of the
// demodulated QPSK data symbols is the measure the TX-power walls come from.
#pragma once

#include <cstddef>
#include <cstdint>

#include "CrossPathCapture.h"

namespace mabur {

struct CrossEvm {
  bool frame = false;     // a frame was demodulated; evm_db, level_db and flagged are valid
  float evm_db = 0;
  float level_db = 0;     // mean power over the chosen frame, dB re one LSB squared
  float envelope_db = 0;  // 90th percentile of the smoothed power; always valid
  bool clipped = false;   // any I or Q at or beyond +-2040 once the DC (median) is removed
  int flagged = 0;        // samples marked bad inside the chosen frame's demodulated span
};

// bw is 20 or 40 (MHz). Of every frame in the capture and every symbol timing
// tried, the result is the one with the fewest flagged samples, then the
// lowest EVM.
CrossEvm cross_evm(const devourer::IqSample* s, size_t n, uint8_t bw);

} // namespace mabur
