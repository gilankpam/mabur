#include "pa_fit.h"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

#include "cross_evm.h"

namespace mabur {
namespace {

using cf = std::complex<float>;
using cd = std::complex<double>;

constexpr int J = kPaKnots;
constexpr int kTri = J * (J + 1) / 2;  // a Hermitian JxJ's upper triangle, row by row
constexpr int kOs = 4;                 // oversampling of the ideal waveform
using detail::kMaxUsed;
constexpr int kMaxOsFft = detail::kMaxFft * kOs;
constexpr float kKnot0 = -25.0f, kKnotStep = 2.0f;
constexpr double kLambda = 0.05;  // weight of the curve's second differences
constexpr int kRounds = 10;
constexpr float kFloorDb = -27.0f;
constexpr double kMinLevelDb = 30.0;
constexpr float kStepDb = 0.25f;  // one TXAGC step
// Enough symbols that the fixtures predict within 0.2 dB of the reference's
// 200 random ones; below that the error is set more by which symbols are
// drawn than by how many.
constexpr int kPredSyms = 40;

// The HT-LTF's 52-tone sign sequence (the legacy L-LTF's).
constexpr int8_t kLL[52] = {1,  1,  -1, -1, 1,  1,  -1, 1,  -1, 1,  1,  1,  1, 1, 1, -1, -1, 1,
                            1,  -1, 1,  -1, 1,  1,  1,  1,  1,  -1, -1, 1,  1, -1, 1, -1, 1, -1,
                            -1, -1, -1, -1, 1,  1,  -1, -1, 1,  -1, 1,  -1, 1, 1, 1, 1};

// The receiver's tone layout, plus what the ideal waveform needs of each
// used tone: its bin in the oversampled FFT, its HT-LTF sign and rotation.
struct Tones {
  const detail::ToneLayout& lay;
  int os_n;
  int binos[kMaxUsed];
  float ltf[kMaxUsed];
  cf gamma[kMaxUsed];  // the upper half of an HT40 frame is sent rotated by j
  cf tw_os[kMaxOsFft / 2], itw_os[kMaxOsFft / 2];

  explicit Tones(int S) : lay(detail::tone_layout(S)), os_n(lay.fft_n * kOs) {
    int8_t seq[kMaxUsed];
    int q = 0;
    auto put = [&](const int8_t* a, int n) { for (int i = 0; i < n; ++i) seq[q++] = a[i]; };
    const int8_t one[] = {1}, two_p[] = {1, 1}, two_m[] = {-1, -1};
    const int8_t mid[] = {-1, -1, -1, 1, -1, 1, 1, -1};
    if (S == 1) {
      put(two_p, 2), put(kLL, 52), put(two_m, 2);
    } else {
      for (int h = 0; h < 2; ++h) {
        put(kLL, 26), put(one, 1), put(kLL + 26, 26);
        if (h == 0) put(mid, 8);
      }
    }
    for (int u = 0; u < lay.used; ++u) {
      const int k = lay.k[u];
      binos[u] = k < 0 ? k + os_n : k;
      ltf[u] = seq[u];
      gamma[u] = S == 2 && k > 0 ? cf(0, 1) : cf(1, 0);
    }
    for (int k = 0; k < os_n / 2; ++k) {
      tw_os[k] = std::polar(1.0f, float(-2 * M_PI * k / os_n));
      itw_os[k] = std::conj(tw_os[k]);
    }
  }
};

const Tones& tones(int S) {
  static const Tones t20(1), t40(2);
  return S == 2 ? t40 : t20;
}

// numpy's sign: 0 stays 0.
float sgn(float v) { return v > 0 ? 1.0f : v < 0 ? -1.0f : 0.0f; }
cf decide(cf z, bool pilot) {
  if (pilot) return cf(sgn(z.real()), 0);
  return cf(sgn(z.real()), sgn(z.imag())) * float(M_SQRT1_2);
}

// The hat basis around envelope a (dB re rms): knot j weighs 1 - t, knot
// j + 1 weighs t; flat beyond the first and last knot.
void knot(float a, int& j, float& t) {
  const float u = (a - kKnot0) / kKnotStep;
  j = std::clamp(int(std::floor(u)), 0, J - 2);
  t = std::clamp(u - float(j), 0.0f, 1.0f);
}

float env_db(cf v) { return 10 * std::log10(std::max(std::norm(v), 1e-12f)); }

// The fixed QPSK/BPSK-pilot symbol set the prediction drives through the
// curve: its ideal waveform at rms 1, that waveform's envelope (dB) and the
// symbols. It depends only on the width, so it is built once per width.
struct PredSet {
  std::vector<cf> v, D;    // kPredSyms x os_n samples, kPredSyms x used tones
  std::vector<float> env;  // dB, per sample of v
  double Q[kMaxUsed] = {};  // sum |D|^2 per tone over the symbols
};

PredSet make_pred_set(int S) {
  const Tones& tn = tones(S);
  const int T = tn.lay.used, n = tn.os_n;
  // Each symbol's waveform has rms sqrt(T) unscaled: every used tone is unit power.
  const float scale = 1.0f / std::sqrt(float(T));
  PredSet ps;
  ps.v.assign(size_t(kPredSyms) * n, cf(0));
  ps.D.resize(size_t(kPredSyms) * T);
  ps.env.resize(ps.v.size());
  uint32_t rng = 1;
  auto bit = [&rng] {
    rng = rng * 1664525u + 1013904223u;
    return (rng >> 31) ? 1.0f : -1.0f;
  };
  for (int s = 0; s < kPredSyms; ++s) {
    cf* v = &ps.v[size_t(s) * n];
    cf* D = &ps.D[size_t(s) * T];
    for (int t = 0; t < T; ++t) {
      D[t] = tn.lay.pilot[t] ? cf(bit(), 0) : cf(bit(), bit()) * float(M_SQRT1_2);
      v[tn.binos[t]] = D[t] * tn.gamma[t];
      ps.Q[t] += std::norm(D[t]);
    }
    detail::fft(v, n, tn.itw_os);
    for (int k = 0; k < n; ++k) {
      v[k] *= scale;
      ps.env[size_t(s) * n + k] = env_db(v[k]);
    }
  }
  return ps;
}

const PredSet& pred_set(int S) {
  static const PredSet p20 = make_pred_set(1), p40 = make_pred_set(2);
  return S == 2 ? p40 : p20;
}

// The data symbols' EVM (dB) of the fixed symbol set through `gain`, driven
// `steps` TXAGC steps lower, each tone's channel estimated against the known
// symbols.
float pred_raw(const std::array<cf, J>& gain, int S, int steps) {
  const Tones& tn = tones(S);
  const PredSet& ps = pred_set(S);
  const int T = tn.lay.used, n = tn.os_n;
  const float shift = kStepDb * steps;
  cd P[kMaxUsed] = {};
  double E[kMaxUsed] = {};
  cf y[kMaxOsFft];
  for (int s = 0; s < kPredSyms; ++s) {
    const cf* v = &ps.v[size_t(s) * n];
    const float* env = &ps.env[size_t(s) * n];
    const cf* D = &ps.D[size_t(s) * T];
    for (int k = 0; k < n; ++k) {
      int j;
      float w;
      knot(env[k] - shift, j, w);
      y[k] = v[k] * (gain[j] * (1 - w) + gain[j + 1] * w);
    }
    detail::fft(y, n, tn.tw_os);
    for (int t = 0; t < T; ++t) {
      const cd yt = y[tn.binos[t]];
      P[t] += yt * std::conj(cd(D[t]));
      E[t] += std::norm(yt);
    }
  }
  // sum |Y/h - D|^2 with h = P/Q is Q (E Q / |P|^2 - 1).
  double err = 0, ref = 0;
  for (int t = 0; t < T; ++t) {
    if (tn.lay.pilot[t]) continue;
    err += ps.Q[t] * (E[t] * ps.Q[t] / std::norm(P[t]) - 1);
    ref += ps.Q[t];
  }
  return float(10 * std::log10(err / ref));
}

// Solves A x = y for Hermitian positive definite A (both triangles filled)
// by Cholesky; false when a pivot is not positive.
bool cholesky_solve(cd (&A)[J][J], cd (&y)[J], std::array<cd, J>& x) {
  cd L[J][J] = {};
  for (int j = 0; j < J; ++j) {
    double d = A[j][j].real();
    for (int k = 0; k < j; ++k) d -= std::norm(L[j][k]);
    if (!(d > 0)) return false;
    L[j][j] = std::sqrt(d);
    for (int i = j + 1; i < J; ++i) {
      cd s = A[i][j];
      for (int k = 0; k < j; ++k) s -= L[i][k] * std::conj(L[j][k]);
      L[i][j] = s / L[j][j].real();
    }
  }
  cd z[J];
  for (int i = 0; i < J; ++i) {
    cd s = y[i];
    for (int k = 0; k < i; ++k) s -= L[i][k] * z[k];
    z[i] = s / L[i][i].real();
  }
  for (int i = J - 1; i >= 0; --i) {
    cd s = z[i];
    for (int k = i + 1; k < J; ++k) s -= std::conj(L[k][i]) * x[k];
    x[i] = s / L[i][i].real();
  }
  return true;
}

} // namespace

PaFit pa_fit(const IqSpan* caps, int ncaps, uint8_t bw) {
  PaFit f;
  const int S = bw == 40 ? 2 : 1;
  const Tones& tn = tones(S);
  const int T = tn.lay.used, NOS = tn.os_n;
  constexpr int M = detail::kSyms;

  // Reused across calls, as cross_evm's; re-prepared when the fitted frames
  // come from another capture than the last one read.
  static thread_local detail::Capture cap;
  int ready = -1;
  auto prepare = [&](int ci) {
    if (ready != ci) detail::prepare(caps[ci].p, caps[ci].n, S, cap);
    ready = ci;
  };

  struct Cand {
    int flagged, ci;
    size_t start;
    double level;
  };
  static thread_local std::vector<Cand> cand;
  cand.clear();
  // Every symbol timing tried, 8 samples late at most, fits in the capture.
  const size_t need = size_t(detail::kLtfOffset + 8 + detail::kSymLen * (M + 1)) * S;
  for (int ci = 0; ci < ncaps; ++ci) {
    if (caps[ci].n == 0) continue;
    prepare(ci);
    if (cap.clipped) {
      f.clipped = true;
      continue;
    }
    for (const auto& fr : cap.frames) {
      if (fr.start + need > caps[ci].n) continue;
      const double lv = cap.level_db(fr.start, S);
      if (lv >= kMinLevelDb) cand.push_back({fr.flagged, ci, fr.start, lv});
    }
  }
  f.frames = int(cand.size());
  if (f.frames < 2) return f;
  std::sort(cand.begin(), cand.end(), [](const Cand& a, const Cand& b) {
    return std::tie(a.flagged, a.ci, a.start) < std::tie(b.flagged, b.ci, b.start);
  });

  // Per fitted frame and tone: the Gram matrix of the knots' basis tones over
  // the symbols (upper triangle), their products with the received tones,
  // and the received power. The least-squares rounds need nothing else.
  // Kept across calls, as cross_evm's buffers are (0.7 MB at 40 MHz).
  static thread_local std::vector<cd> G, C, H;
  static thread_local std::vector<double> E;
  static thread_local std::vector<cf> R, D, bufs;
  static thread_local std::vector<int> kj;
  static thread_local std::vector<float> kt;
  G.assign(2 * T * kTri, 0);
  C.assign(2 * T * J, 0);
  E.assign(2 * T, 0);
  H.resize(2 * T);
  R.resize(M * T);
  D.resize(M * T);
  bufs.resize(J * NOS);
  kj.resize(NOS);
  kt.resize(NOS);
  double wt2[2];  // each frame's rows weigh 1 / rms(R)
  double own = 0, level = 0;
  for (int fi = 0; fi < 2; ++fi) {
    const Cand& c = cand[fi];
    prepare(c.ci);
    const detail::Timing tm = detail::best_timing(cap, c.start, S);
    if (!std::isfinite(tm.evm_db)) return f;
    own += tm.evm_db;
    level += c.level;

    // The HT-LTF window and the data symbols at the chosen timing.
    cf w0[kMaxUsed];
    for (int m = 0; m <= M; ++m)
      tn.lay.tones_at(cap.x.data() + tm.ltf + size_t(detail::kSymLen) * S * m,
                      m ? &R[(m - 1) * T] : w0);
    // Decisions on the HT-LTF's channel, then refined twice on the sliced symbols.
    double dpow = 0;
    for (int t = 0; t < T; ++t) {
      cf h = w0[t] * tn.ltf[t];
      for (int pass = 0; pass <= 2; ++pass) {
        if (pass) {
          cf num = 0;
          float den = 0;
          for (int m = 0; m < M; ++m) {
            num += R[m * T + t] * std::conj(D[m * T + t]);
            den += std::norm(D[m * T + t]);
          }
          h = num / den;
        }
        for (int m = 0; m < M; ++m) D[m * T + t] = decide(R[m * T + t] / h, tn.lay.pilot[t]);
      }
      for (int m = 0; m < M; ++m) dpow += std::norm(D[m * T + t]);
    }

    // The ideal waveform is the decisions' oversampled IFFT at rms 1; by
    // Parseval its unscaled mean power is sum |D|^2 over the symbols.
    const float scale = float(1 / std::sqrt(dpow / M));
    cd* g = &G[fi * T * kTri];
    cd* cc = &C[fi * T * J];
    double* e = &E[fi * T];
    double rpow = 0;
    for (int m = 0; m < M; ++m) {
      cf* v = &bufs[0];  // knot 0's buffer doubles as the waveform's
      std::fill(v, v + NOS, cf(0));
      for (int t = 0; t < T; ++t) v[tn.binos[t]] = D[m * T + t] * tn.gamma[t];
      detail::fft(v, NOS, tn.itw_os);
      for (int k = 0; k < NOS; ++k) {
        v[k] *= scale;
        knot(env_db(v[k]), kj[k], kt[k]);
      }
      // Knot j's basis waveform is hat_j(envelope) times the waveform.
      for (int k = 0; k < NOS; ++k) {
        const cf vk = v[k];
        for (int j = 0; j < J; ++j) bufs[j * NOS + k] = 0;
        bufs[kj[k] * NOS + k] = vk * (1 - kt[k]);
        bufs[(kj[k] + 1) * NOS + k] = vk * kt[k];
      }
      for (int j = 0; j < J; ++j) detail::fft(&bufs[j * NOS], NOS, tn.tw_os);
      for (int t = 0; t < T; ++t) {
        cd b[J];
        for (int j = 0; j < J; ++j) b[j] = cd(bufs[j * NOS + tn.binos[t]]) / double(kOs);
        const cd r = R[m * T + t];
        e[t] += std::norm(r);
        rpow += std::norm(r);
        cd* gt = g + t * kTri;
        for (int i = 0, p = 0; i < J; ++i) {
          const cd bi = std::conj(b[i]);
          cc[t * J + i] += bi * r;
          for (int j = i; j < J; ++j) gt[p++] += bi * b[j];
        }
      }
    }
    wt2[fi] = double(M) * T / rpow;
  }
  f.own_evm_db = float(own / 2);
  f.level_db = float(level / 2);

  // Alternating least squares: each tone's channel against the model, then
  // the knots over both frames with a smoothness penalty, scaled to G(low) = 1.
  std::array<cd, J> th;
  th.fill(1);
  auto model = [&](int ft, cd& P, double& Q) {  // sum R conj(M) and sum |M|^2 over the symbols
    const cd* gt = &G[ft * kTri];
    const cd* ct = &C[ft * J];
    P = 0;
    Q = 0;
    for (int i = 0, p = 0; i < J; ++i) {
      P += std::conj(th[i]) * ct[i];
      Q += std::norm(th[i]) * gt[p++].real();
      for (int j = i + 1; j < J; ++j) Q += 2 * (std::conj(th[i]) * gt[p++] * th[j]).real();
    }
  };
  for (int round = 0; round < kRounds; ++round) {
    cd A[J][J] = {}, y[J] = {};
    for (int ft = 0; ft < 2 * T; ++ft) {
      const double w2 = wt2[ft / T];
      cd P;
      double Q;
      model(ft, P, Q);
      const cd h = P / Q;
      H[ft] = h;
      const double s = w2 * std::norm(h);
      const cd* gt = &G[ft * kTri];
      for (int i = 0, p = 0; i < J; ++i) {
        y[i] += w2 * std::conj(h) * C[ft * J + i];
        for (int j = i; j < J; ++j) A[i][j] += s * gt[p++];
      }
    }
    double trace = 0;
    for (int i = 0; i < J; ++i) trace += A[i][i].real();
    const double c2 = kLambda * kLambda * trace / J;
    const double d[3] = {1, -2, 1};
    for (int r = 0; r + 2 < J; ++r)
      for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j) A[r + i][r + j] += c2 * d[i] * d[j];
    for (int i = 0; i < J; ++i)
      for (int j = 0; j < i; ++j) A[i][j] = std::conj(A[j][i]);
    if (!cholesky_solve(A, y, th)) return f;
    const cd th0 = th[0];
    for (cd& v : th) v /= th0;
  }

  // What the fitted curve, with the last round's channels, leaves unexplained.
  double num = 0, den = 0;
  for (int ft = 0; ft < 2 * T; ++ft) {
    cd P;
    double Q;
    model(ft, P, Q);
    num += E[ft] - 2 * (std::conj(P) * H[ft]).real() + std::norm(H[ft]) * Q;
    den += E[ft];
  }
  f.residual_db = float(10 * std::log10(num / den));

  bool finite = std::isfinite(f.own_evm_db) && std::isfinite(f.residual_db) &&
                std::isfinite(f.level_db);
  for (int j = 0; j < J; ++j) {
    f.gain[j] = cf(th[j]);
    finite = finite && std::isfinite(f.gain[j].real()) && std::isfinite(f.gain[j].imag());
  }
  if (!finite) return f;
  f.offset_db = f.own_evm_db - pred_raw(f.gain, S, 0);
  f.ok = std::isfinite(f.offset_db);
  return f;
}

float pa_predict(const PaFit& f, uint8_t bw, int steps) {
  const float raw = pred_raw(f.gain, bw == 40 ? 2 : 1, steps) + f.offset_db;
  return 10 * std::log10(std::pow(10.0f, raw / 10) + std::pow(10.0f, kFloorDb / 10));
}

} // namespace mabur
