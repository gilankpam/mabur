#include "mtest.h"
#include "pa_fit.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

using mabur::IqSpan;
using mabur::PaFit;
using mabur::pa_fit;
using mabur::pa_predict;

namespace {

// A fixture holds the high dword of each capture word (tests/fixtures/crosspath/README.md).
std::vector<devourer::IqSample> load_hi32(const std::string& name) {
  std::ifstream f(std::string(MABUR_FIXTURE_DIR) + "/crosspath/" + name + ".hi32",
                  std::ios::binary);
  REQUIRE(f.good());
  std::vector<devourer::IqSample> s;
  uint8_t b[4];
  while (f.read(reinterpret_cast<char*>(b), 4)) {
    const uint32_t hi = b[0] | b[1] << 8 | b[2] << 16 | uint32_t(b[3]) << 24;
    s.push_back(devourer::cross_path_sample(uint64_t{hi} << 32));
  }
  return s;
}

IqSpan span(const std::vector<devourer::IqSample>& s) { return {s.data(), s.size()}; }

struct Row {
  const char* base;
  uint8_t bw;
  float own, residual, level;
  float pred[11];  // at 0, 4, .. 40 steps
};

// Reference values: pafit_ref.py on the row's two captures.
const Row kRows[] = {
    {"d144_20_i080", 20, -11.82f, -23.04f, 58.35f, {-11.69f, -12.42f, -13.29f, -14.32f, -15.53f, -16.93f, -18.48f, -20.12f, -21.65f, -22.78f, -23.30f}},
    {"d144_20_i104", 20, -9.01f, -18.16f, 59.00f, {-8.94f, -9.23f, -9.62f, -10.12f, -10.72f, -11.40f, -12.15f, -12.97f, -13.84f, -14.74f, -15.61f}},
    {"d64_20_i100", 20, -11.91f, -23.07f, 53.06f, {-11.77f, -12.75f, -13.81f, -14.96f, -16.25f, -17.70f, -19.33f, -21.10f, -22.88f, -24.37f, -25.22f}},
    {"h144_20_i104", 20, -10.78f, -20.90f, 43.22f, {-10.68f, -11.39f, -12.26f, -13.31f, -14.54f, -15.94f, -17.48f, -19.12f, -20.76f, -22.17f, -23.06f}},
    {"d144_40_i076", 40, -12.11f, -19.05f, 58.43f, {-11.97f, -13.02f, -14.20f, -15.52f, -17.00f, -18.59f, -20.22f, -21.73f, -22.94f, -23.76f, -24.20f}},
    {"d144_40_i096", 40, -8.91f, -13.67f, 59.41f, {-8.84f, -9.44f, -10.18f, -11.04f, -12.00f, -13.08f, -14.25f, -15.47f, -16.69f, -17.88f, -18.99f}},
};

void check_row(const Row& r, const PaFit& f) {
  float worst = 0;
  for (int k = 0; k <= 10; ++k) {
    const float p = pa_predict(f, r.bw, 4 * k);
    worst = std::fmax(worst, std::fabs(p - r.pred[k]));
    CHECK(std::fabs(p - r.pred[k]) <= 0.2f);
  }
  std::printf("  %-14s frames %d own %7.2f residual %7.2f level %6.2f worst pred %.3f\n", r.base,
              f.frames, f.own_evm_db, f.residual_db, f.level_db, worst);
  CHECK(f.ok);
  CHECK(f.frames >= 2);
  CHECK(!f.clipped);
  CHECK(std::fabs(f.own_evm_db - r.own) <= 0.1f);
  CHECK(std::fabs(f.residual_db - r.residual) <= 0.3f);
  CHECK(std::fabs(f.level_db - r.level) <= 0.1f);
}

PaFit fit_one(const std::string& name, uint8_t bw) {
  const auto s = load_hi32(name);
  const IqSpan c = span(s);
  return pa_fit(&c, 1, bw);
}

} // namespace

TEST(fits_match_the_reference) {
  for (const Row& r : kRows) {
    const auto a = load_hi32(std::string(r.base) + "_a");
    const auto b = load_hi32(std::string(r.base) + "_b");
    const IqSpan caps[] = {span(a), span(b)};
    check_row(r, pa_fit(caps, 2, r.bw));
  }
}

TEST(one_capture_with_two_frames_is_enough) {
  check_row(kRows[0], fit_one("d144_20_i080_a", 20));
}

TEST(prediction_is_bounded_by_the_floor) {
  const auto a = load_hi32("d144_20_i080_a");
  const auto b = load_hi32("d144_20_i080_b");
  const IqSpan caps[] = {span(a), span(b)};
  const float p = pa_predict(pa_fit(caps, 2, 20), 20, 96);
  std::printf("  96 steps: %.2f\n", p);
  CHECK(std::isfinite(p));
  CHECK(p >= -27.05f);
}

TEST(a_capture_without_two_frames_is_not_ok) {
  const PaFit none = fit_one("c144_40_noframe", 40);
  CHECK(none.frames == 0 && !none.ok);
  CHECK(!fit_one("c40_20_noise", 20).ok);
}

TEST(a_clipped_capture_gives_no_frame) {
  const PaFit f = fit_one("c144_20_i028_clip", 20);
  CHECK(f.clipped && f.frames == 0 && !f.ok);
}

TEST(identical_frames_do_not_produce_a_non_finite_fit) {
  // The capture's first 4000 samples hold only its first frame, so both
  // fitted frames are the same samples.
  const auto a = load_hi32("d144_20_i080_a");
  const IqSpan one = {a.data(), 4000};
  const IqSpan caps[] = {one, one};
  const PaFit f = pa_fit(caps, 2, 20);
  std::printf("  frames %d ok %d residual %.2f offset %.2f\n", f.frames, f.ok, f.residual_db,
              f.offset_db);
  CHECK(f.frames == 2);
  bool finite = std::isfinite(f.offset_db) && std::isfinite(f.residual_db);
  for (const auto& g : f.gain) finite = finite && std::isfinite(g.real()) && std::isfinite(g.imag());
  CHECK(!f.ok || finite);
}

TEST(a_foreign_burst_raises_the_residual) {
  auto a = load_hi32("d144_20_i080_a");
  const auto b = load_hi32("d64_20_i100_b");
  REQUIRE(a.size() == b.size());
  const size_t n = a.size();
  double pa = 0, pb = 0;
  for (size_t k = 0; k < n; ++k) {
    pa += double(a[k].i) * a[k].i + double(a[k].q) * a[k].q;
    pb += double(b[k].i) * b[k].i + double(b[k].q) * b[k].q;
  }
  // The other capture's samples, 137 earlier and wrapping, 10 dB under this one's power.
  const double g = std::sqrt(0.1 * pa / pb);
  for (size_t k = 0; k < n; ++k) {
    const auto& o = b[(k + n - 137) % n];
    a[k].i = int16_t(std::lround(a[k].i + g * o.i));
    a[k].q = int16_t(std::lround(a[k].q + g * o.q));
  }
  const IqSpan c = span(a);
  const PaFit f = pa_fit(&c, 1, 20);
  std::printf("  ok %d clipped %d residual - own %.2f\n", f.ok, f.clipped,
              f.residual_db - f.own_evm_db);
  CHECK(f.ok && !f.clipped);
  CHECK(!f.ok || f.residual_db - f.own_evm_db > -8.0f);
}

// Host time per pa_fit call and per 25 pa_predict calls; runs only with
// PA_FIT_TIME set, so the suite does not pay for it.
TEST(time_per_call) {
  if (!std::getenv("PA_FIT_TIME")) return;
  for (const char* base : {"d144_20_i080", "d144_40_i076"}) {
    const uint8_t bw = std::string(base).find("_40_") != std::string::npos ? 40 : 20;
    const auto a = load_hi32(std::string(base) + "_a");
    const auto b = load_hi32(std::string(base) + "_b");
    const IqSpan caps[] = {span(a), span(b)};
    const int reps = 20;
    using clk = std::chrono::steady_clock;
    PaFit f;
    auto t0 = clk::now();
    for (int i = 0; i < reps; ++i) f = pa_fit(caps, 2, bw);
    const double fit_ms = std::chrono::duration<double, std::milli>(clk::now() - t0).count() / reps;
    float sink = 0;
    t0 = clk::now();
    for (int i = 0; i < reps; ++i)
      for (int k = 0; k < 25; ++k) sink += pa_predict(f, bw, k);
    const double pred_ms =
        std::chrono::duration<double, std::milli>(clk::now() - t0).count() / reps;
    std::printf("  %d MHz: pa_fit %.2f ms, 25 pa_predict %.2f ms (sink %.1f)\n", bw, fit_ms,
                pred_ms, sink);
  }
}

MTEST_MAIN
