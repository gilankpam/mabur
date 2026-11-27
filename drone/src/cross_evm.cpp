#include "cross_evm.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

namespace mabur {
namespace {

using cf = std::complex<float>;

constexpr int kMaxFft = 128;     // 64 samples per symbol at 20 MHz, 128 at 40
constexpr int kMaxTones = 108;   // data tones at 40 MHz
constexpr int kSyms = 20;        // HT data symbols demodulated after the HT-LTF
constexpr int kSymLen = 80;      // samples per symbol with its guard interval, at 20 MHz
constexpr int kLtfOffset = 656;  // frame rise to the HT-LTF's FFT window, at 20 MHz
constexpr int kMinFrame = 1900;  // shortest frame that holds the probe, at 20 MHz
constexpr float kClip = 2040.0f;

double median(std::vector<int16_t>& v) {
  const size_t h = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + h, v.end());
  if (v.size() % 2) return v[h];
  const int16_t hi = v[h];
  const int16_t lo = *std::max_element(v.begin(), v.begin() + h);
  return (double(lo) + hi) / 2;
}

// std::complex's * and / take a NaN-recovery path (/ is a libgcc call); the
// values here are finite, and these loops are the whole cost of a capture.
cf mul(cf a, cf b) {
  return cf(a.real() * b.real() - a.imag() * b.imag(), a.real() * b.imag() + a.imag() * b.real());
}
cf inv(cf a) { return std::conj(a) / std::norm(a); }

// Iterative radix-2 DIT FFT of n points in place; tw[k] = exp(-2*pi*j*k/n).
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

// numpy's sign: 0 stays 0.
float sgn(float v) { return float((v > 0) - (v < 0)); }
cf slice_qpsk(cf z) { return cf(sgn(z.real()), sgn(z.imag())) * float(M_SQRT1_2); }

struct Demod {
  int fft_n, tones, sym;
  int bin[kMaxTones];
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
    for (int k = 0; k < fft_n / 2; ++k)
      tw[k] = std::polar(1.0f, float(-2 * M_PI * k / fft_n));
  }

  void tones_at(const cf* x, cf* out) const {
    cf a[kMaxFft];
    std::copy(x, x + fft_n, a);
    fft(a, fft_n, tw);
    for (int t = 0; t < tones; ++t) out[t] = a[bin[t]];
  }

  // Decision-directed EVM with the channel taken from the HT-LTF at x[0],
  // then refined twice on the sliced data. Caller guarantees x holds
  // (kSyms + 1) symbols.
  float evm_db(const cf* x) const {
    cf H[kMaxTones], Y[kSyms][kMaxTones], Z[kSyms][kMaxTones], D[kSyms][kMaxTones];
    tones_at(x, H);
    for (int t = 0; t < tones; ++t) H[t] = inv(H[t]);
    for (int m = 0; m < kSyms; ++m) tones_at(x + sym * (m + 1), Y[m]);
    for (int m = 0; m < kSyms; ++m)
      for (int t = 0; t < tones; ++t) {
        Z[m][t] = mul(Y[m][t], H[t]);
        D[m][t] = slice_qpsk(Z[m][t]);
      }
    for (int pass = 0; pass < 2; ++pass)
      for (int t = 0; t < tones; ++t) {
        cf num = 0;
        float den = 0;
        for (int m = 0; m < kSyms; ++m) {
          num += mul(Y[m][t], std::conj(D[m][t]));
          den += std::norm(D[m][t]);
        }
        const cf h2inv = inv(num) * den;  // 1 / (num / den)
        for (int m = 0; m < kSyms; ++m) {
          Z[m][t] = mul(Y[m][t], h2inv);
          D[m][t] = slice_qpsk(Z[m][t]);
        }
      }
    double err = 0, ref = 0;
    for (int m = 0; m < kSyms; ++m)
      for (int t = 0; t < tones; ++t) {
        err += std::norm(Z[m][t] - D[m][t]);
        ref += std::norm(D[m][t]);
      }
    return float(10 * std::log10(err / ref));
  }
};

} // namespace

CrossEvm cross_evm(const devourer::IqSample* s, size_t n, uint8_t bw) {
  CrossEvm r;
  if (n == 0) return r;
  const int S = bw == 40 ? 2 : 1;

  std::vector<int16_t> tmp(n);
  for (size_t k = 0; k < n; ++k) tmp[k] = s[k].i;
  const double mi = median(tmp);
  for (size_t k = 0; k < n; ++k) tmp[k] = s[k].q;
  const double mq = median(tmp);

  std::vector<cf> x(n);
  std::vector<double> pw(n + 1, 0.0);  // prefix sums of |x|^2
  for (size_t k = 0; k < n; ++k) {
    x[k] = cf(float(s[k].i - mi), float(s[k].q - mq));
    if (std::fabs(x[k].real()) >= kClip || std::fabs(x[k].imag()) >= kClip) r.clipped = true;
    pw[k + 1] = pw[k] + std::norm(std::complex<double>(s[k].i - mi, s[k].q - mq));
  }
  auto mean_pow = [&](size_t a, size_t b) { return (pw[b] - pw[a]) / double(b - a); };

  // Power smoothed over x[k-8 .. k+7], zero outside the capture.
  std::vector<double> p(n);
  for (size_t k = 0; k < n; ++k) {
    const size_t a = k >= 8 ? k - 8 : 0, b = std::min(n, k + 8);
    p[k] = (pw[b] - pw[a]) / 16;
  }
  std::vector<double> sorted(p);
  const double rank = 0.9 * double(n - 1);
  const size_t lo = size_t(rank);
  std::nth_element(sorted.begin(), sorted.begin() + lo, sorted.end());
  double p90 = sorted[lo];
  if (lo + 1 < n) {
    const double next = *std::min_element(sorted.begin() + lo + 1, sorted.end());
    p90 += (rank - double(lo)) * (next - p90);
  }
  r.envelope_db = float(10 * std::log10(p90));

  const Demod dm(S);
  const double thr = 0.1 * p90;
  const size_t span = size_t(kSymLen) * S * (kSyms + 1);
  bool have = false;
  int best_nb = 0;
  float best_evm = 0;
  size_t best_s = 0;
  for (size_t st = 1; st < n; ++st) {
    if (!(p[st] > thr) || p[st - 1] > thr) continue;
    size_t e = st + 1;
    while (e < n && p[e] > thr) ++e;
    if (e - st < size_t(kMinFrame) * S) continue;
    const size_t f0 = std::min(n, st + 640 * S);
    const size_t f1 = std::min(n, st + size_t(kLtfOffset + kSymLen * (kSyms + 1)) * S);
    int nb = 0;
    for (size_t k = f0; k < f1; ++k) nb += s[k].bad;
    if (have && nb > best_nb) continue;  // cannot win: flagged count ranks first
    for (int d = -12 * S; d < 9 * S; ++d) {
      const long b = long(st) + kLtfOffset * S + d;
      if (b < 0 || size_t(b) + span > n) continue;
      const float ev = dm.evm_db(x.data() + b);
      if (!std::isfinite(ev)) continue;
      if (!have || nb < best_nb || (nb == best_nb && ev < best_evm)) {
        have = true;
        best_nb = nb;
        best_evm = ev;
        best_s = st;
      }
    }
  }
  if (!have) return r;
  r.frame = true;
  r.evm_db = best_evm;
  r.flagged = best_nb;
  r.level_db = float(10 * std::log10(mean_pow(best_s + 700 * S, best_s + 1800 * S)));
  return r;
}

} // namespace mabur
