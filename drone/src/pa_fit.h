// The PA's gain against its input amplitude, fitted to cross-path captures of
// the probe frame (see cross_evm.h), and the EVM the probe would read at a
// lower TXAGC index. The fit models the received tones as the ideal frame
// waveform through a memoryless gain G(envelope), then a per-tone channel;
// G is piecewise linear in the envelope's dB, so lowering the drive by n
// steps (0.25 dB each) slides the waveform down the same curve.
#pragma once

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>

#include "CrossPathCapture.h"

namespace mabur {

constexpr int kPaKnots = 18;  // -25..+9 dB re the frame's rms, 2 dB apart

struct IqSpan {
  const devourer::IqSample* p;
  size_t n;
};

struct PaFit {
  int frames = 0;        // usable frames found over the captures; the two least-flagged are fitted
  bool ok = false;       // frames >= 2 and every solved value is finite
  bool clipped = false;  // some capture clipped; a clipped capture contributes no frame
  float own_evm_db = 0;  // mean measured EVM of the two fitted frames
  float residual_db = 0; // unexplained over received power after the fit
  float level_db = 0;    // mean level of the two fitted frames
  std::array<std::complex<float>, kPaKnots> gain{};  // G at the knots, gain[0] == 1
  float offset_db = 0;   // own_evm_db minus the raw prediction at 0 steps
};

// ncaps captures (1..3) of one TXAGC index; bw is 20 or 40 (MHz). A frame is
// usable when its capture is unclipped, all of its symbol timings fit in the
// capture and its level is at least 30 dB.
PaFit pa_fit(const IqSpan* caps, int ncaps, uint8_t bw);

// EVM (dB) the probe would read `steps` TXAGC steps lower: the raw prediction plus offset_db,
// power-added to the assumed -27 dB listener floor. steps >= 0; f.ok must hold.
float pa_predict(const PaFit& f, uint8_t bw, int steps);

} // namespace mabur
