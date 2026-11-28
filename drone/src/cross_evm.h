// Signal quality of one cross-path capture: the drone's PA A transmits the
// probe frame (HT MCS1, one stream) and chain B records it. The EVM of the
// demodulated QPSK data symbols is the measure the TX-power walls come from.
#pragma once

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "CrossPathCapture.h"

namespace mabur {

struct CrossEvm {
  bool frame = false;     // a frame was demodulated; evm_db, level_db and flagged are valid
  float evm_db = 0;
  float level_db = 0;     // mean power over the chosen frame, dB re one LSB squared
  float envelope_db = 0;  // 90th percentile of the smoothed power; always valid
  bool clipped = false;   // any I or Q at or beyond +-2040 once the DC (median) is removed
  int flagged = 0;        // samples marked bad inside the chosen frame's demodulated span
  size_t start = 0;       // sample where the chosen frame rises
};

// bw is 20 or 40 (MHz). Of every frame in the capture and every symbol timing
// tried, the result is the one with the fewest flagged samples, then the
// lowest EVM.
CrossEvm cross_evm(const devourer::IqSample* s, size_t n, uint8_t bw);

// The pieces cross_evm is built from, shared with pa_fit. S is 1 at 20 MHz, 2 at 40.
namespace detail {

constexpr int kSyms = 20;        // HT data symbols demodulated after the HT-LTF
constexpr int kSymLen = 80;      // samples per symbol with its guard interval, at 20 MHz
constexpr int kLtfOffset = 656;  // frame rise to the HT-LTF's FFT window, at 20 MHz

struct Capture {
  struct Frame {
    size_t start;  // sample where the frame rises
    int flagged;   // samples marked bad over its demodulated span
  };
  std::vector<std::complex<float>> x;  // the samples with the DC (median) removed
  std::vector<double> pw;              // prefix sums of |x|^2
  std::vector<double> sorted;          // scratch for the percentile
  double p90 = 0;                      // 90th percentile of the smoothed power
  bool clipped = false;                // any I or Q at or beyond +-2040
  std::vector<Frame> frames;           // every candidate frame, fewest flagged first, then by start
  // Mean power over a frame's start+700*S .. start+1800*S, dB re one LSB squared.
  double level_db(size_t start, int S) const;
};

// Fills c from n > 0 samples. c's buffers are reused across calls.
void prepare(const devourer::IqSample* s, size_t n, int S, Capture& c);

struct Timing {
  size_t ltf = 0;         // sample where the chosen timing's HT-LTF window starts
  float evm_db = INFINITY;
};
// Of the symbol timings of the frame rising at `start`, the one with the
// lowest EVM, if any reads below stop_db (evm_db stays +inf otherwise).
Timing best_timing(const Capture& c, size_t start, int S, float stop_db = INFINITY);

// Iterative radix-2 FFT of n points in place; tw[k] = exp(-2*pi*j*k/n) gives
// the forward transform, its conjugate the unscaled inverse.
void fft(std::complex<float>* a, int n, const std::complex<float>* tw);

} // namespace detail
} // namespace mabur
