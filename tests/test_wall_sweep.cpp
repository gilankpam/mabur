#include "mtest.h"
#include "wall_sweep.h"

#include <cmath>
#include <functional>
#include <limits>
#include <string>

using namespace mabur;

namespace {
// One capture as the script sees it: ids 0 and 1 are the park check; nth counts the
// earlier point-phase captures at the same index, whatever their gain.
struct Cap { int id, idx, gain, nth; };

CrossEvm good(int idx) {
  CrossEvm m;
  m.frame = true;
  m.evm_db = -30.0f + (idx - 28) * 0.25f;
  m.level_db = 50;
  m.envelope_db = 50;
  return m;
}
CrossEvm default_script(const Cap& c) {
  CrossEvm m = good(c.idx);
  if (c.id == 0) m.envelope_db = 35;   // unparked
  return m;
}

struct FakeSession : CaptureSession {
  std::vector<std::string> calls;
  std::vector<Cap> caps;
  std::function<CrossEvm(const Cap&)> script = default_script;
  bool begin_ok = true, park_ok = true;
  int fail_at_index = -1;
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
};

FakeSession* g_fake = nullptr;
CrossEvm measure(const devourer::IqSample* s, size_t n, uint8_t) {
  REQUIRE(n == 1 && g_fake);
  return g_fake->script(g_fake->caps[s[0].i]);
}

constexpr uint64_t kT0 = 1000;
void run_to_end(WallSweep& sw) {
  for (int i = 0; i < 1000 && sw.step(kT0); ++i) {}
  REQUIRE(sw.finished());
}
int k_of(int idx) { return (idx - 28) / 4; }
}  // namespace

#define SWEEP(fake, sw) FakeSession fake; g_fake = &fake; WallSweep sw(fake, 20, 39, kT0, measure)

TEST(happy_path) {            // 2 park-check captures + 25 x 3
  SWEEP(fake, sw);
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  CHECK(fake.count("cap") == 77 && fake.count("end") == 1);
  CHECK(fake.calls[0] == "begin 20" && fake.calls[1] == "cap 28 0" && fake.calls[2] == "park 1" && fake.calls[3] == "cap 28 0");
  CHECK(fake.calls[4] == "cap 28 3");                 // points start at the strongest gain
  CHECK(*sw.curve().evm_db[0] == -30.0f && sw.curve().anchor == 39);
  CHECK(*sw.curve().evm_db[24] == -6.0f);
  CHECK(sw.park_rise_db() == 15.0f);
  CHECK(!sw.step(kT0) && fake.count("end") == 1);     // finished stays finished
}
TEST(park_rise_is_zero_before_the_check) {
  SWEEP(fake, sw);
  CHECK(sw.park_rise_db() == 0.0f && !sw.finished());
}
TEST(an_unparked_listener_is_refused) {   // envelopes 35 then 44
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) { CrossEvm m = default_script(c); if (c.id == 1) m.envelope_db = 44; return m; };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Unparked);
  CHECK(fake.count("cap") == 2 && fake.count("end") == 1);
  CHECK(sw.park_rise_db() == 9.0f);
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
TEST(gain_steps_down_and_never_back_up) {
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    CrossEvm m = default_script(c);
    if (c.id >= 2 && c.gain == 3) m.clipped = true;
    if (c.id >= 2 && c.gain == 2 && c.idx >= 60) m.level_db = 58;
    return m;
  };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  CHECK(fake.count("cap 28 3") == 1);
  CHECK(fake.count("cap 28 2") == 3);
  for (int idx = 32; idx < 60; idx += 4) CHECK(fake.count("cap " + std::to_string(idx) + " 2") == 3);
  CHECK(fake.count("cap 60 2") == 1 && fake.count("cap 60 1") == 3);
  for (int idx = 64; idx <= 124; idx += 4) {
    CHECK(fake.count("cap " + std::to_string(idx) + " 1") == 3);
    CHECK(fake.count("cap " + std::to_string(idx) + " 2") == 0 && fake.count("cap " + std::to_string(idx) + " 3") == 0);
  }
  CHECK(*sw.curve().evm_db[k_of(60)] == good(60).evm_db);
}
TEST(a_step_down_discards_the_stronger_gains_values_and_restarts_the_index) {
  // index 32 at gain 3: no frame, a value of -99, clipped; then at gain 2: no frame twice, three good
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    CrossEvm m = default_script(c);
    if (c.id < 2 || c.idx != 32) return m;
    if (c.nth == 0 || c.nth == 3 || c.nth == 4) m.frame = false;
    if (c.nth == 1) m.evm_db = -99;
    if (c.nth == 2) m.clipped = true;
    return m;
  };
  run_to_end(sw);
  CHECK(fake.count("cap 32 3") == 3 && fake.count("cap 32 2") == 5);
  CHECK(sw.curve().evm_db[k_of(32)] && *sw.curve().evm_db[k_of(32)] == good(32).evm_db);
  CHECK(fake.count("cap 36 2") == 3 && fake.count("cap 36 3") == 0);
}
TEST(the_weakest_gain_over_headroom_is_used) {
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    CrossEvm m = default_script(c);
    if (c.id >= 2 && c.gain > 0) m.clipped = true;
    if (c.id >= 2 && c.gain == 0) m.level_db = 58;
    return m;
  };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  CHECK(fake.count("cap 28 0") == 2 + 3);
  for (const auto& v : sw.curve().evm_db) CHECK(v.has_value());
}
TEST(clipped_at_the_weakest_gain_leaves_the_point_empty) {
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    CrossEvm m = default_script(c);
    if (c.id >= 2 && c.gain > 0) m.clipped = true;
    if (c.id >= 2 && c.gain == 0 && c.idx == 40) m.clipped = true;
    return m;
  };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok);
  CHECK(!sw.curve().evm_db[k_of(40)].has_value());
  CHECK(fake.count("cap 40 0") == 1);
  CHECK(sw.curve().evm_db[k_of(44)].has_value() && sw.curve().evm_db[k_of(36)].has_value());
}
TEST(a_clip_at_the_weakest_gain_discards_values_already_taken) {
  // every gain above 0 clips; at index 40 gain 0 gives good, good, clipped
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    CrossEvm m = default_script(c);
    if (c.id >= 2 && c.gain > 0) m.clipped = true;
    if (c.id >= 2 && c.idx == 40 && c.nth == 2) m.clipped = true;
    return m;
  };
  run_to_end(sw);
  CHECK(fake.count("cap 40 0") == 3);
  CHECK(!sw.curve().evm_db[k_of(40)].has_value());
}
TEST(a_level_of_exactly_30_is_usable) {
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) { CrossEvm m = default_script(c); m.level_db = 30.0f; return m; };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok && fake.count("cap") == 77);
  for (const auto& v : sw.curve().evm_db) CHECK(v.has_value());
}
TEST(a_level_of_exactly_57_does_not_step_down) {
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) { CrossEvm m = default_script(c); m.level_db = 57.0f; return m; };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok && fake.count("cap") == 77);
  CHECK(fake.count("cap 124 3") == 3);
}
TEST(a_park_rise_of_exactly_10_passes) {   // envelopes 40 then 50
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) { CrossEvm m = default_script(c); if (c.id == 0) m.envelope_db = 40; return m; };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::Ok && sw.park_rise_db() == 10.0f);
}
TEST(a_capture_without_a_frame_is_retried_twice) {
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    CrossEvm m = default_script(c);
    if (c.id < 2) return m;
    if (c.idx == 40) {          // no frame, no frame, then -20, -23, -20.5: median -20.5 (mean would be -21.2)
      const float v[5] = {0, 0, -20.0f, -23.0f, -20.5f};
      m.frame = c.nth >= 2;  m.evm_db = v[c.nth];
    }
    if (c.idx == 44) m.frame = false;
    if (c.idx == 48) {          // frame, no, no, frame, no: mean of -20 and -21
      m.frame = c.nth == 0 || c.nth == 3;  m.evm_db = c.nth == 0 ? -20.0f : -21.0f;
    }
    return m;
  };
  run_to_end(sw);
  CHECK(fake.count("cap 40") == 5 && fake.count("cap 44") == 5 && fake.count("cap 48") == 5);
  CHECK(fake.count("cap 36") == 3 && fake.count("cap 52") == 3);   // retries are per index
  CHECK(*sw.curve().evm_db[k_of(40)] == -20.5f);
  CHECK(!sw.curve().evm_db[k_of(44)].has_value());
  CHECK(*sw.curve().evm_db[k_of(48)] == -20.5f);
}
TEST(a_frame_read_out_of_noise_is_not_a_value) {
  // index 52: -20, a noise "frame" (EVM -2.8 dB at level 12.3 dB), -21, -22: median -21, not -20
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    CrossEvm m = default_script(c);
    if (c.id < 2 || c.idx != 52) return m;
    const float v[4] = {-20.0f, -2.8f, -21.0f, -22.0f};
    m.evm_db = v[c.nth];
    if (c.nth == 1) m.level_db = 12.3f;
    return m;
  };
  run_to_end(sw);
  CHECK(fake.count("cap 52") == 4);
  CHECK(*sw.curve().evm_db[k_of(52)] == -21.0f);
}
TEST(values_are_rounded_to_a_tenth) {
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    CrossEvm m = default_script(c);
    const float v[3] = {-20.04f, -20.06f, -20.26f};
    if (c.id >= 2 && c.idx == 56) m.evm_db = v[c.nth];
    return m;
  };
  run_to_end(sw);
  CHECK(*sw.curve().evm_db[k_of(56)] == -20.1f);
}
TEST(the_deadline_ends_the_sweep) {
  SWEEP(fake, sw);
  for (int i = 0; i < 10; ++i) CHECK(sw.step(kT0 + 19999));
  int caps = fake.count("cap");
  CHECK(!sw.step(kT0 + 20000));
  CHECK(sw.finished() && sw.result() == cal::WallResult::Deadline);
  CHECK(fake.count("cap") == caps && fake.count("end") == 1);   // the deadline is checked before capturing
}
TEST(cancel_ends_the_session_once) {
  FakeSession fake;  g_fake = &fake;
  {
    WallSweep sw(fake, 20, 39, kT0, measure);
    for (int i = 0; i < 10; ++i) sw.step(kT0);
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
  { WallSweep sw(fake, 20, 39, kT0, measure);  sw.step(kT0);  sw.step(kT0); }
  CHECK(fake.count("end") == 1);
}
TEST(a_failed_capture_is_a_device_error) {
  SWEEP(fake, sw);
  fake.fail_at_index = 36;
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::DeviceError && fake.count("end") == 1);
  CHECK(fake.count("cap 36") == 1 && fake.count("cap 40") == 0);
}
TEST(a_failed_begin_is_a_device_error) {
  SWEEP(fake, sw);
  fake.begin_ok = false;
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::DeviceError);
  CHECK(fake.count("end") == 1 && fake.count("cap") == 0);
}
TEST(an_invalid_curve_reports_why) {
  SWEEP(fake, sw);
  fake.script = [](const Cap& c) {
    CrossEvm m = default_script(c);
    m.evm_db = -24.0f + (c.idx - 28) * 0.2f;
    return m;
  };
  run_to_end(sw);
  CHECK(sw.result() == cal::WallResult::FloorTooHigh && fake.count("end") == 1);
}

MTEST_MAIN
