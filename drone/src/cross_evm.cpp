#include "cross_evm.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

namespace mabur {
namespace detail {
namespace {

using cf = std::complex<float>;

constexpr int kMaxFft = 128;     // 64 samples per symbol at 20 MHz, 128 at 40
constexpr int kMaxTones = 108;   // data tones at 40 MHz
constexpr int kMinFrame = 1900;  // shortest frame that holds the probe, at 20 MHz
constexpr float kClip = 2040.0f;

// Median of 12-bit samples by counting: the two middle order statistics
// read off a 4096-bin histogram, linear where a selection over the samples
// is not. A value outside 12 bits is clamped, which moves the median only if
// the median itself is out of range.
template <class Get>
double median12(const devourer::IqSample* s, size_t n, Get get) {
  uint32_t hist[4096] = {};
  for (size_t k = 0; k < n; ++k) ++hist[std::clamp(int(get(s[k])), -2048, 2047) + 2048];
  // The values at sorted positions h - 1 and h; the median is h for odd n.
  const size_t h = n / 2;
  int below = -1, at = -1;
  size_t seen = 0;
  for (int b = 0; b < 4096 && at < 0; ++b) {
    seen += hist[b];
    if (below < 0 && seen >= h) below = b;  // sorted position h - 1 is in bin b
    if (seen > h) at = b;
  }
  if (n % 2) return at - 2048;
  return (double(below - 2048) + (at - 2048)) / 2;
}

// std::complex's * and / take a NaN-recovery path (/ is a libgcc call); the
// values here are finite, and these loops are the whole cost of a capture.
cf mul(cf a, cf b) {
  return cf(a.real() * b.real() - a.imag() * b.imag(), a.real() * b.imag() + a.imag() * b.real());
}
cf inv(cf a) { return std::conj(a) / std::norm(a); }

// The nearest QPSK point, each axis by numpy's sign (0 stays 0); copysign
// and a select instead of an int-to-float conversion, the demod's costliest
// line.
float slice_axis(float v) {
  const float a = std::copysign(float(M_SQRT1_2), v);
  return v == 0 ? 0.0f : a;
}
cf slice_qpsk(cf z) { return cf(slice_axis(z.real()), slice_axis(z.imag())); }

// The LTF window (row 0) and the kSyms data symbols after it, as data tones.
using Windows = cf[kSyms + 1][kMaxTones];

struct Demod {
  int fft_n, tones, sym;
  int bin[kMaxTones];
  cf rot[kMaxTones];  // exp(+2*pi*j*bin/N): a bin's factor for a one-sample advance
  cf tw[kMaxFft / 2];

  explicit Demod(int S) : fft_n(64 * S), tones(0), sym(kSymLen * S) {
    // Data tones (pilots, DC and the guard band excluded); negative k is bin k + N.
    const int edge = S == 2 ? 58 : 28;
    for (int k = -edge; k <= edge; ++k) {
      const int a = std::abs(k);
      const bool skip = S == 2 ? (a <= 1 || a == 11 || a == 25 || a == 53)
                               : (a == 0 || a == 7 || a == 21);
      if (!skip) bin[tones++] = k < 0 ? k + fft_n : k;
    }
    for (int t = 0; t < tones; ++t) rot[t] = std::polar(1.0f, float(2 * M_PI * bin[t] / fft_n));
    for (int k = 0; k < fft_n / 2; ++k)
      tw[k] = std::polar(1.0f, float(-2 * M_PI * k / fft_n));
  }

  void tones_at(const cf* x, cf* out) const {
    cf a[kMaxFft];
    std::copy(x, x + fft_n, a);
    fft(a, fft_n, tw);
    for (int t = 0; t < tones; ++t) out[t] = a[bin[t]];
  }

  // The windows of the symbol timing whose LTF window starts at x[0].
  void windows_at(const cf* x, Windows& w) const {
    for (int m = 0; m <= kSyms; ++m) tones_at(x + sym * m, w[m]);
  }

  // w, the windows at x[0], becomes the windows at x[1]: a sliding DFT, one
  // complex multiply per bin in place of an FFT per window.
  void advance(const cf* x, Windows& w) const {
    for (int m = 0; m <= kSyms; ++m) {
      const cf* xm = x + sym * m;
      const cf delta = xm[fft_n] - xm[0];
      for (int t = 0; t < tones; ++t) w[m][t] = mul(w[m][t] + delta, rot[t]);
    }
  }

  // Decision-directed EVM with each tone's channel taken from the HT-LTF,
  // then refined twice on the sliced data. Tone by tone, so it gives up
  // (+inf) once the error so far already puts the result at or above
  // `stop_db`: a timing that cannot beat the best one costs only part of the
  // work.
  float evm_db(const Windows& w, float stop_db = INFINITY) const {
    // |D|^2 is at most 1, so err / (kSyms * tones) bounds err / ref from below.
    const double stop_err = std::pow(10.0, stop_db / 10.0) * kSyms * tones;
    double err = 0, ref = 0;
    cf Y[kSyms], D[kSyms];  // one tone's symbols and decisions
    for (int t = 0; t < tones; ++t) {
      // Decisions on the LTF's channel, and the sums the first pass needs.
      const cf hinv = inv(w[0][t]);
      cf num = 0;
      float den = 0;
      for (int m = 0; m < kSyms; ++m) {
        Y[m] = w[m + 1][t];
        D[m] = slice_qpsk(mul(Y[m], hinv));
        num += mul(Y[m], std::conj(D[m]));
        den += std::norm(D[m]);
      }
      // Each pass re-slices on the channel the last decisions give, summing
      // for the next pass and for the error as it goes. A pass that leaves
      // the decisions unchanged would give the next one the same channel,
      // so the same Z and D: the tone is done.
      double terr = 0, tref = 0;
      for (int pass = 0; pass < 2; ++pass) {
        const cf h2inv = inv(num) * den;  // 1 / (num / den)
        num = 0;
        den = 0;
        terr = tref = 0;
        bool same = true;
        for (int m = 0; m < kSyms; ++m) {
          const cf z = mul(Y[m], h2inv);
          const cf d = slice_qpsk(z);
          same = same && d == D[m];
          D[m] = d;
          num += mul(Y[m], std::conj(d));
          den += std::norm(d);
          terr += std::norm(z - d);
          tref += std::norm(d);
        }
        if (same) break;
      }
      err += terr;
      ref += tref;
      if (err >= stop_err) return INFINITY;
    }
    return float(10 * std::log10(err / ref));
  }
};

} // namespace

void fft(cf* a, int n, const cf* tw) {
  for (int i = 1, j = 0; i < n; ++i) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) std::swap(a[i], a[j]);
  }
  for (int len = 2; len <= n; len <<= 1) {
    const int step = n / len, half = len / 2;
    for (int i = 0; i < n; i += len)
      for (int k = 0; k < half; ++k) {
        const cf t = mul(a[i + k + half], tw[k * step]);
        a[i + k + half] = a[i + k] - t;
        a[i + k] += t;
      }
  }
}

void prepare(const devourer::IqSample* s, size_t n, int S, Capture& c) {
  const double mi = median12(s, n, [](const devourer::IqSample& v) { return v.i; });
  const double mq = median12(s, n, [](const devourer::IqSample& v) { return v.q; });

  auto& x = c.x;
  auto& pw = c.pw;
  x.resize(n);
  pw.resize(n + 1);  // prefix sums of |x|^2
  pw[0] = 0;
  c.clipped = false;
  for (size_t k = 0; k < n; ++k) {
    x[k] = cf(float(s[k].i - mi), float(s[k].q - mq));
    if (std::fabs(x[k].real()) >= kClip || std::fabs(x[k].imag()) >= kClip) c.clipped = true;
    pw[k + 1] = pw[k] + std::norm(std::complex<double>(s[k].i - mi, s[k].q - mq));
  }

  // Power smoothed over x[k-8 .. k+7], zero outside the capture.
  auto p = [&](size_t k) {
    const size_t a = k >= 8 ? k - 8 : 0, b = std::min(n, k + 8);
    return (pw[b] - pw[a]) / 16;
  };
  auto& sorted = c.sorted;
  sorted.resize(n);
  for (size_t k = 0; k < n; ++k) sorted[k] = p(k);
  const double rank = 0.9 * double(n - 1);
  const size_t lo = size_t(rank);
  std::nth_element(sorted.begin(), sorted.begin() + lo, sorted.end());
  double p90 = sorted[lo];
  if (lo + 1 < n) {
    const double next = *std::min_element(sorted.begin() + lo + 1, sorted.end());
    p90 += (rank - double(lo)) * (next - p90);
  }
  c.p90 = p90;

  // Candidate frames, each with the flagged count over its demodulated span,
  // fewest flagged first: a frame flagged more than one that demodulated
  // cannot win.
  const double thr = 0.1 * p90;
  c.frames.clear();
  for (size_t st = 1; st < n; ++st) {
    if (!(p(st) > thr) || p(st - 1) > thr) continue;
    size_t e = st + 1;
    while (e < n && p(e) > thr) ++e;
    if (e - st < size_t(kMinFrame) * S) continue;
    const size_t f0 = std::min(n, st + 640 * S);
    const size_t f1 = std::min(n, st + size_t(kLtfOffset + kSymLen * (kSyms + 1)) * S);
    int nb = 0;
    for (size_t k = f0; k < f1; ++k) nb += s[k].bad;
    c.frames.push_back({st, nb});
  }
  std::stable_sort(c.frames.begin(), c.frames.end(),
                   [](const Capture::Frame& a, const Capture::Frame& b) {
                     return a.flagged < b.flagged;
                   });
}

double Capture::level_db(size_t start, int S) const {
  const size_t a = start + 700 * S, b = start + 1800 * S;
  return 10 * std::log10((pw[b] - pw[a]) / double(b - a));
}

Timing best_timing(const Capture& c, size_t start, int S, float stop_db) {
  static const Demod d20(1), d40(2);
  const Demod& dm = S == 2 ? d40 : d20;
  const size_t n = c.x.size();
  const size_t span = size_t(kSymLen) * S * (kSyms + 1);
  Timing t;
  t.evm_db = stop_db;
  bool have = false;
  Windows win;
  // Every symbol timing from 12 20 MHz samples early to 8 late is tried, in
  // order, so each one's windows slide from the last's. EVM against timing is
  // no flat floor: the best timing can sit in a pocket one to three samples
  // wide at the floor's edge, so none is skipped. The float drift over the
  // slides stays far below a hundredth of a dB.
  const size_t b0 = start + kLtfOffset * S - 12 * S;
  for (int k = 0; k < 21 * S; ++k) {
    const size_t b = b0 + k;
    if (b + span > n) break;
    if (k == 0)
      dm.windows_at(c.x.data() + b, win);
    else
      dm.advance(c.x.data() + b - 1, win);
    const float ev = dm.evm_db(win, t.evm_db);
    if (std::isfinite(ev) && ev < t.evm_db) {
      have = true;
      t.ltf = b;
      t.evm_db = ev;
    }
  }
  if (!have) t.evm_db = INFINITY;
  return t;
}

} // namespace detail

CrossEvm cross_evm(const devourer::IqSample* s, size_t n, uint8_t bw) {
  CrossEvm r;
  if (n == 0) return r;
  const int S = bw == 40 ? 2 : 1;

  // Kept across calls: fresh buffers (0.4 MB at 40 MHz) go back to the
  // kernel on free and fault in again, zeroed, on every capture.
  static thread_local detail::Capture c;
  detail::prepare(s, n, S, c);
  r.clipped = c.clipped;
  r.envelope_db = float(10 * std::log10(c.p90));

  bool have = false;
  int best_nb = 0;
  float best_evm = INFINITY;
  size_t best_s = 0;
  for (const detail::Capture::Frame& f : c.frames) {
    if (have && f.flagged > best_nb) break;
    const detail::Timing t = detail::best_timing(c, f.start, S, best_evm);
    if (std::isfinite(t.evm_db) && t.evm_db < best_evm) {
      have = true;
      best_nb = f.flagged;
      best_evm = t.evm_db;
      best_s = f.start;
    }
  }
  if (!have) return r;
  r.frame = true;
  r.evm_db = best_evm;
  r.flagged = best_nb;
  r.start = best_s;
  r.level_db = float(c.level_db(best_s, S));
  return r;
}

} // namespace mabur
