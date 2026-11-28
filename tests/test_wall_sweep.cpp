#include "mtest.h"
#include "wall_sweep.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <set>
#include <string>
#include <utility>

using namespace mabur;

namespace {
// One capture as the script sees it: ids 0 and 1 are the park check (unless it retries); nth
// counts the earlier captures at the same index after the first two, whatever their gain.
struct Cap { int id, idx, gain, nth; bool measured = false; };

// The model PA: EVM the probe reads at an index. 100 -> -14.75, 112 -> -11.75, 124 -> -8.75.
float model(int idx) { return std::max(-30.0f, -30.0f + (idx - 39) * 0.25f); }
float rounded(float v) { return std::round(v * 10) / 10.0f; }

struct FakeSession;
FakeSession* g_fake = nullptr;
CrossEvm default_script(const Cap& c);

struct FakeSession : CaptureSession {
  std::vector<std::string> calls;
  std::vector<Cap> caps;
  std::function<float(int)> evm = model;
  std::function<CrossEvm(const Cap&)> script = default_script;
  // The fit's verdict for a probe at idx; nth counts the earlier fits at that index.
  std::function<PaFit(int idx, int nth)> fit_script;
  // The prediction from a fit of the probe at idx, `steps` lower.
  std::function<float(int idx, int steps)> predict_script;
  std::vector<std::pair<int, int>> fits;   // (index, captures) per fit call
  std::vector<int> fit_at;                 // captures taken when each fit ran
  bool begin_ok = true, park_ok = true;
  int fail_at_index = -1;

  FakeSession() {
    fit_script = [this](int idx, int) {
      PaFit f;
      f.ok = true;
      f.frames = 2;
      f.own_evm_db = evm(idx);
      f.residual_db = f.own_evm_db - 12;
      return f;
    };
    predict_script = [this](int idx, int steps) { return evm(idx - steps); };
  }
  bool begin(uint8_t bw) override { calls.push_back("begin " + std::to_string(bw)); return begin_ok; }
  bool park(bool p) override { calls.push_back(std::string("park ") + (p ? "1" : "0")); return park_ok; }
  bool capture(int idx, int gain, std::vector<devourer::IqSample>* out) override {
    calls.push_back("cap " + std::to_string(idx) + " " + std::to_string(gain));
    if (idx == fail_at_index) return false;
    int id = (int)caps.size(), nth = 0;
    for (const auto& c : caps) nth += c.id >= 2 && c.idx == idx;
    caps.push_back({id, idx, gain, id >= 2 ? nth : 0});
    out->assign(1, devourer::IqSample{});
    (*out)[0].i = (int16_t)id;
    return true;
  }
  void end() override { calls.push_back("end"); }
  int count(const std::string& prefix) const {
    int n = 0;
    for (const auto& c : calls) n += c.rfind(prefix, 0) == 0;
    return n;
  }
  int fits_at(int idx) const {
    int n = 0;
    for (const auto& f : fits) n += f.first == idx;
    return n;
  }
  // Captures the hunt read (it measures them; a fit's captures are not measured), after the
  // park check, split at the first fit.
  std::vector<Cap> hunted(bool second) const {
    std::vector<Cap> v;
    const int split = fit_at.empty() ? (int)caps.size() : fit_at[0];
    for (const auto& c : caps)
      if (c.measured && c.id >= 2 && (c.id >= split) == second) v.push_back(c);
    return v;
  }
};

CrossEvm good(int idx) {
  CrossEvm m;
  m.frame = true;
  m.evm_db = g_fake->evm(idx);
  m.level_db = 50;
  m.envelope_db = 50;
  return m;
}
CrossEvm default_script(const Cap& c) {
  CrossEvm m = good(c.idx);
  if (c.id == 0) m.envelope_db = 35;   // unparked
  return m;
}

CrossEvm measure(const devourer::IqSample* s, size_t n, uint8_t) {
  REQUIRE(n == 1 && g_fake);
  Cap& c = g_fake->caps[s[0].i];
  c.measured = true;
  return g_fake->script(c);
}
PaFit fit(const IqSpan* spans, int n, uint8_t) {
  REQUIRE(g_fake && n >= 1 && n <= 3);
  const Cap& c0 = g_fake->caps[spans[0].p[0].i];
  for (int i = 0; i < n; ++i) {   // a fit only ever sees one index at one gain
    REQUIRE(spans[i].n == 1);
    const Cap& c = g_fake->caps[spans[i].p[0].i];
    REQUIRE(c.idx == c0.idx && c.gain == c0.gain);
  }
  PaFit f = g_fake->fit_script(c0.idx, g_fake->fits_at(c0.idx));
  g_fake->fits.push_back({c0.idx, n});
  g_fake->fit_at.push_back((int)g_fake->caps.size());
  f.offset_db = (float)c0.idx;   // the probe's index, for predict
  return f;
}
float predict(const PaFit& f, uint8_t, int steps) {
  REQUIRE(f.ok && steps >= 0);
  return g_fake->predict_script((int)f.offset_db, steps);
}
const WallOps kOps{measure, fit, predict};

constexpr uint64_t kT0 = 1000;
void run_to_end(WallSweep& sw) {
  for (int i = 0; i < 1000 && sw.step(kT0); ++i) {}
  REQUIRE(sw.finished());
}
int k_of(int idx) { return (idx - 28) / 4; }
bool no_values(const WallCurve& c) {
  for (const auto& v : c.evm_db)
    if (v) return false;
  return true;
}
}  // namespace

#define SWEEP_BW(fake, sw, bw) FakeSession fake; g_fake = &fake; WallSweep sw(fake, bw, 60, kT0, kOps)
#define SWEEP(fake, sw) SWEEP_BW(fake, sw, 20)

TEST(two_probes_fill_the_curve) {   // 100 -> 112 (in the window), then 124
  SWEEP(fake, sw);
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  CHECK(fake.calls[0] == "begin 20" && fake.calls[1] == "cap 28 0" && fake.calls[2] == "park 1" && fake.calls[3] == "cap 28 0");
  CHECK(fake.calls[4] == "cap 100 3");                 // probe 1 starts at anchor + 40, strongest gain
  REQUIRE(fake.fits.size() == 2);
  CHECK(fake.fits[0].first == 112 && fake.fits[1].first == 124);
  for (int k = 0; k < kWallPoints; ++k)
    CHECK(sw.curve().evm_db[k] && *sw.curve().evm_db[k] == rounded(model(wall_point_index(k))));
  CHECK(fake.count("cap") - 2 <= 8 && fake.count("end") == 1);
  CHECK(sw.curve().anchor == 60 && !sw.curve().weak_park && sw.park_rise_db() == 15.0f);
  CHECK(!sw.step(kT0) && fake.count("end") == 1);     // finished stays finished
}
TEST(the_hunt_never_tries_an_index_twice) {   // measured EVM +-0.6 dB, alternating per capture
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    CrossEvm m = default_script(c);
    m.evm_db += c.id % 2 ? 0.6f : -0.6f;
    return m;
  };
  run_to_end(sw);
  for (int second = 0; second < 2; ++second) {
    const auto h = fake.hunted(second);
    std::set<int> idx;
    std::set<std::pair<int, int>> idx_gain;
    for (const auto& c : h) { idx.insert(c.idx); idx_gain.insert({c.idx, c.gain}); }
    CHECK(idx_gain.size() == h.size() && idx.size() <= 6);
  }
  CHECK(!fake.hunted(false).empty() && !fake.hunted(true).empty());
}
TEST(a_bracketing_pair_takes_the_upper_index) {   // 100 -> 112 (-10.8, above) -> 108 (-12.75, below)
  SWEEP(fake, sw);
  fake.evm = [](int idx) { return idx <= 108 ? std::min(model(idx), -12.6f) : std::max(model(idx), -10.8f); };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  REQUIRE(!fake.fits.empty());
  CHECK(fake.fits[0].first == 112);
  CHECK(fake.count("cap 108") == 1);
}
TEST(probe_1_and_the_next_index_can_bracket_probe_2) {   // 112 -11.75, 116 and up -7: 124 -> 116
  SWEEP(fake, sw);
  fake.evm = [](int idx) { return idx <= 112 ? model(idx) : -7.0f; };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  REQUIRE(fake.fits.size() == 2);
  CHECK(fake.fits[0].first == 112 && fake.fits[1].first == 116);
  CHECK(fake.count("cap 112") == 1 && fake.count("cap 116") == 1 && fake.count("cap 124") == 1);
}
TEST(a_pa_that_never_reaches_the_window_is_not_saturated) {
  SWEEP(fake, sw);
  fake.evm = [](int idx) { return std::min(model(idx), -13.0f); };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::NotSaturated);
  CHECK(fake.calls[fake.calls.size() - 2] == "cap 124 3");   // the hunt ended at 124
  CHECK(fake.fits.size() == 1 && fake.fits[0].first == 124);
}
TEST(no_probe_2_when_probe_1_is_124) {   // -11.5 (inside the probe-1 window) only at 124
  SWEEP(fake, sw);
  fake.evm = [](int idx) { return idx == 124 ? -11.5f : std::min(model(idx), -14.0f); };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::NotSaturated);
  CHECK(!fake.fits.empty());
  for (const auto& f : fake.fits) CHECK(f.first == 124);
  CHECK(sw.curve().evm_db[24] && *sw.curve().evm_db[24] == -11.5f);
}
TEST(a_pa_always_above_the_window_runs_out_of_indices) {   // 100, 84, 68, 52, 36, 28
  SWEEP(fake, sw);
  fake.evm = [](int) { return -5.0f; };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::TooFewPoints && fake.fits.empty());
  CHECK(fake.count("cap") - 2 == 6 && fake.count("cap 28 3") == 1);
  CHECK(no_values(sw.curve()));
}
TEST(a_hot_capture_steps_the_gain_down_and_stays_down) {
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    CrossEvm m = default_script(c);
    if (c.id >= 2 && c.gain == 3) m.level_db = 60;
    return m;
  };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  CHECK(fake.calls[4] == "cap 100 3" && fake.calls[5] == "cap 100 2");
  for (size_t i = 5; i < fake.calls.size(); ++i)
    if (fake.calls[i].rfind("cap ", 0) == 0) CHECK(fake.calls[i].back() <= '2');
}
TEST(a_clip_at_the_weakest_gain_is_retried_then_refused) {   // every capture clips
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) { CrossEvm m = default_script(c); if (c.id >= 2) m.clipped = true; return m; };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::TooFewPoints);
  CHECK(fake.count("cap 100 3") == 1 && fake.count("cap 100 2") == 1 && fake.count("cap 100 1") == 1);
  CHECK(fake.count("cap 100 0") == 3 && fake.count("cap") == 2 + 6);
}
TEST(a_capture_without_a_frame_is_retried_twice) {
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {   // at 100: no frame, a noise "frame" under 30 dB, then a frame
    CrossEvm m = default_script(c);
    if (c.id >= 2 && c.idx == 100 && c.nth == 0) m.frame = false;
    if (c.id >= 2 && c.idx == 100 && c.nth == 1) m.level_db = 12;
    return m;
  };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok && fake.count("cap 100") == 3);
  SWEEP(fake2, sw2);
  fake2.script = [](const Cap& c) { CrossEvm m = default_script(c); if (c.id >= 2) m.frame = false; return m; };
  run_to_end(sw2);
  CHECK(sw2.result() == cal::WallResult::TooFewPoints && fake2.count("cap 100") == 3 && fake2.count("cap") == 5);
}
TEST(one_frame_is_not_enough) {
  SWEEP(fake, sw);
  fake.fit_script = [&](int idx, int nth) {
    PaFit f;
    f.frames = nth < 2 ? 1 : 2;
    f.ok = f.frames == 2;
    f.own_evm_db = model(idx);
    f.residual_db = f.own_evm_db - 12;
    return f;
  };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  CHECK(fake.count("cap 112") == 3 && fake.count("cap 124") == 3);
  CHECK(fake.fits_at(112) == 3 && fake.fits[2].second == 3);

  SWEEP(fake2, sw2);
  fake2.fit_script = [](int, int) { PaFit f; f.frames = 1; return f; };
  run_to_end(sw2);
  CHECK(sw2.result() == cal::WallResult::TooFewPoints && no_values(sw2.curve()));
  CHECK(fake2.count("cap 112") == 3 && fake2.count("cap 124") == 0);
}
PaFit fit_with_residual(int idx, float residual_minus_own) {
  PaFit f;
  f.ok = true;
  f.frames = 2;
  f.own_evm_db = model(idx);
  f.residual_db = f.own_evm_db + residual_minus_own;
  return f;
}
TEST(a_bad_residual_retakes_the_probe_then_gives_up) {
  SWEEP(fake, sw);
  fake.fit_script = [](int idx, int nth) { return fit_with_residual(idx, idx == 112 && nth == 0 ? -2.0f : -12.0f); };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  CHECK(fake.count("cap 112") == 2 && fake.fits_at(112) == 2);
  CHECK(fake.fits[1].second == 1);   // the retake drops the bad fit's captures

  SWEEP(fake2, sw2);
  fake2.fit_script = [](int idx, int) { return fit_with_residual(idx, -2.0f); };
  run_to_end(sw2);
  CHECK(sw2.result() == cal::WallResult::Disturbed && no_values(sw2.curve()));
  CHECK(fake2.fits_at(112) == 3 && fake2.count("cap 112") == 3 && fake2.count("cap 124") == 0);
}
TEST(residual_limits_follow_width_and_probe) {   // limits: 20 MHz -8 / -5.5, 40 MHz -5 / -3
  {
    SWEEP_BW(fake, sw, 40);
    fake.fit_script = [](int idx, int) { return fit_with_residual(idx, idx == 112 ? -6.0f : -12.0f); };
    run_to_end(sw);
    CHECK(sw.result() == cal::WallResult::Ok && fake.fits_at(112) == 1);
  }
  {
    SWEEP_BW(fake, sw, 20);
    fake.fit_script = [](int idx, int nth) { return fit_with_residual(idx, idx == 112 && nth == 0 ? -6.0f : -12.0f); };
    run_to_end(sw);
    CHECK(sw.result() == cal::WallResult::Ok && fake.fits_at(112) == 2);
  }
  {
    SWEEP_BW(fake, sw, 20);   // probe 2 at 20 MHz takes -6
    fake.fit_script = [](int idx, int) { return fit_with_residual(idx, idx == 124 ? -6.0f : -12.0f); };
    run_to_end(sw);
    CHECK(sw.result() == cal::WallResult::Ok && fake.fits_at(124) == 1);
  }
  CHECK(WallSweep::residual_limit_db(20, 1) == -8.0f && WallSweep::residual_limit_db(20, 2) == -5.5f);
  CHECK(WallSweep::residual_limit_db(40, 1) == -5.0f && WallSweep::residual_limit_db(40, 2) == -3.0f);
}
TEST(weak_park_is_flagged_not_refused) {
  const float unparked[3] = {42, 45, 37};   // parked 50: rise 8, 5, 13
  const cal::WallResult want[3] = {cal::WallResult::Ok, cal::WallResult::Unparked, cal::WallResult::Ok};
  for (int i = 0; i < 3; ++i) {
    SWEEP(fake, sw);
    const float u = unparked[i];
    fake.script = [u](const Cap& c) { CrossEvm m = default_script(c); if (c.id == 0) m.envelope_db = u; return m; };
    run_to_end(sw);
    CHECK(sw.result() == want[i] && sw.park_rise_db() == 50 - u);
    CHECK(sw.curve().weak_park == (i == 0));
    if (i == 1) CHECK(fake.count("cap") == 2 && fake.count("end") == 1);
  }
}
TEST(a_park_rise_of_exactly_12_is_not_weak) {   // envelopes 38 then 50
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) { CrossEvm m = default_script(c); if (c.id == 0) m.envelope_db = 38; return m; };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok && sw.park_rise_db() == 12.0f && !sw.curve().weak_park);
}
TEST(a_prediction_that_rises_again_is_flattened) {
  SWEEP(fake, sw);
  // More than 40 steps under its probe the prediction reads 3 dB above the 40-step point.
  fake.predict_script = [](int idx, int steps) { return steps > 40 ? model(idx - 40) + 3.0f : model(idx - steps); };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  const auto& e = sw.curve().evm_db;
  for (int k = 0; k < kWallPoints; ++k) REQUIRE(e[k]);
  for (int idx = 28; idx < 112 - 40; idx += 4) CHECK(*e[k_of(idx)] == *e[k_of(112 - 40)]);
  for (int k = 1; k < kWallPoints; ++k) CHECK(*e[k] >= *e[k - 1]);
}
TEST(above_probe_2_the_curve_keeps_rising) {   // probe 2 at 116: hunt -8.95, fit own -9.05
  SWEEP(fake, sw);
  fake.evm = [](int idx) { return idx <= 112 ? model(idx) : idx == 116 ? -8.95f : -7.0f; };
  fake.fit_script = [](int idx, int) {
    PaFit f = fit_with_residual(idx, -12.0f);
    if (idx == 116) f.own_evm_db = -9.05f, f.residual_db = -21.05f;
    return f;
  };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  REQUIRE(fake.fits.size() == 2 && fake.fits[1].first == 116);
  const int w3 = walls_from_curve(sw.curve(), true)[3];
  std::printf("  mcs3 wall %d\n", w3);
  CHECK(w3 <= 116 - 60 + 1);
}
TEST(fit_2_never_reads_below_probe_1) {   // fit 2 predicts 1 dB under probe 1 at 116
  SWEEP(fake, sw);
  fake.predict_script = [](int idx, int steps) {
    return idx == 124 && idx - steps == 116 ? model(112) - 1.0f : model(idx - steps);
  };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  REQUIRE(fake.fits.size() == 2 && fake.fits[0].first == 112 && fake.fits[1].first == 124);
  const auto& e = sw.curve().evm_db;
  REQUIRE(e[k_of(112)] && e[k_of(116)]);
  CHECK(*e[k_of(116)] == *e[k_of(112)]);
}
TEST(a_non_finite_prediction_is_never_stored) {
  SWEEP(fake, sw);
  fake.predict_script = [](int idx, int steps) {
    return idx - steps == 60 ? std::numeric_limits<float>::quiet_NaN() : model(idx - steps);
  };
  run_to_end(sw);
  CHECK(sw.result() != cal::WallResult::Ok && no_values(sw.curve()));
}
TEST(park_rise_is_zero_before_the_check) {
  SWEEP(fake, sw);
  CHECK(sw.park_rise_db() == 0.0f && !sw.finished());
}
TEST(a_park_that_does_not_take_is_refused) {
  SWEEP(fake, sw);
  fake.park_ok = false;
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Unparked);
  CHECK(fake.count("cap") == 1 && fake.count("end") == 1);
}
TEST(an_empty_envelope_is_not_a_park) {   // an all-zero capture has envelope -inf: rise +inf or NaN
  const float ninf = -std::numeric_limits<float>::infinity();
  for (int which = 0; which < 2; ++which) {
    SWEEP(fake, sw);
    fake.script = [&](const Cap& c) {
      CrossEvm m = default_script(c);
      if (c.id == 0) m.envelope_db = ninf;                 // unparked -inf: rise +inf
      if (c.id == 1 && which == 1) m.envelope_db = ninf;   // both -inf: rise NaN
      return m;
    };
    run_to_end(sw);
    CHECK(sw.result() == cal::WallResult::Unparked);
    CHECK(fake.count("cap") == 2 && fake.count("end") == 1);
    CHECK(sw.park_rise_db() == 0.0f);                      // non-finite rise reports 0
  }
}
// A park-check capture that caught no TX: no frame, and the envelope (DC removed) is low.
CrossEvm no_tx() { CrossEvm m; m.envelope_db = 3; return m; }
TEST(a_parked_capture_without_tx_is_retried) {   // unparked 35, parked: no TX, then 50
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) { return c.id == 1 ? no_tx() : default_script(c); };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok && sw.park_rise_db() == 15.0f);
  CHECK(fake.calls[1] == "cap 28 0" && fake.calls[2] == "park 1" && fake.calls[3] == "cap 28 0" &&
        fake.calls[4] == "cap 28 0" && fake.calls[5] == "cap 100 3");   // 3 park-check captures
  CHECK(fake.count("park") == 1);                                       // parked once, not per retry
}
TEST(an_unparked_capture_without_tx_is_retried) {
  // unparked: no TX (3), then leakage 35; parked 40. Without the retry the rise would be 37 and pass.
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    if (c.id == 0) return no_tx();
    CrossEvm m = default_script(c);
    if (c.id == 1) m.envelope_db = 35;
    if (c.id == 2) m.envelope_db = 40;
    return m;
  };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Unparked && sw.park_rise_db() == 5.0f);
  CHECK(fake.count("cap") == 3 && fake.count("park") == 1 && fake.calls[2] == "cap 28 0" && fake.calls[3] == "park 1");
}
TEST(a_park_check_still_without_tx_after_two_retries_uses_the_last) {   // parked: no TX three times
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) { return c.id >= 1 ? no_tx() : default_script(c); };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Unparked && sw.park_rise_db() == -32.0f);
  CHECK(fake.count("cap") == 4 && fake.count("park") == 1);           // 1 unparked + 3 parked
}
TEST(each_park_check_capture_has_its_own_retries) {   // unparked: no TX twice, then 35; parked: no TX once, then 50
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    if (c.id == 0 || c.id == 1 || c.id == 3) return no_tx();
    CrossEvm m = default_script(c);
    if (c.id == 2) m.envelope_db = 35;
    return m;
  };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok && sw.park_rise_db() == 15.0f);
  CHECK(fake.calls[3] == "cap 28 0" && fake.calls[4] == "park 1" && fake.calls[7] == "cap 100 3");   // 3 + 2 park-check captures
}
// With nothing on the air the demodulator can still read a "frame" out of the noise.
CrossEvm noise_frame() { CrossEvm m; m.frame = true; m.level_db = 13; m.envelope_db = 16; m.evm_db = -3; return m; }
TEST(a_parked_capture_with_only_a_noise_frame_is_retried) {   // unparked 35, parked: noise frame, then 50
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) { return c.id == 1 ? noise_frame() : default_script(c); };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok && sw.park_rise_db() == 15.0f);
  CHECK(fake.calls[3] == "cap 28 0" && fake.calls[4] == "cap 28 0" && fake.calls[5] == "cap 100 3");
}
TEST(a_park_check_frame_with_a_low_envelope_is_not_retried) {   // only "no frame AND low envelope" is no TX
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) { CrossEvm m = default_script(c); if (c.id == 1) m.envelope_db = 20; return m; };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Unparked && fake.count("cap") == 2);
}
TEST(the_deadline_ends_the_sweep) {
  SWEEP(fake, sw);
  for (int i = 0; i < 3; ++i) CHECK(sw.step(kT0 + 29999));   // park check + the first hunt capture
  int caps = fake.count("cap");
  CHECK(!sw.step(kT0 + 30000));
  CHECK(sw.finished() && sw.result() == cal::WallResult::Deadline);
  CHECK(fake.count("cap") == caps && fake.count("end") == 1);   // the deadline is checked before capturing
}
TEST(cancel_ends_the_session_once) {
  FakeSession fake;  g_fake = &fake;
  {
    WallSweep sw(fake, 20, 60, kT0, kOps);
    for (int i = 0; i < 3; ++i) sw.step(kT0);
    CHECK(!sw.finished());
    sw.cancel();
    CHECK(sw.finished() && sw.result() == cal::WallResult::Aborted && fake.count("end") == 1);
    sw.cancel();
    CHECK(sw.result() == cal::WallResult::Aborted && fake.count("end") == 1);
    CHECK(!sw.step(kT0));
  }
  CHECK(fake.count("end") == 1);
}
TEST(the_destructor_ends_an_open_session) {
  FakeSession fake;  g_fake = &fake;
  { WallSweep sw(fake, 20, 60, kT0, kOps);  sw.step(kT0);  sw.step(kT0); }
  CHECK(fake.count("end") == 1);
}
TEST(a_failed_capture_is_a_device_error) {
  SWEEP(fake, sw);
  fake.fail_at_index = 112;
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::DeviceError && fake.count("end") == 1);
  CHECK(fake.count("cap 112") == 1 && fake.count("cap 124") == 0);
}
TEST(a_failed_begin_is_a_device_error) {
  SWEEP(fake, sw);
  fake.begin_ok = false;
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::DeviceError);
  CHECK(fake.count("end") == 1 && fake.count("cap") == 0);
}

MTEST_MAIN
