#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <utility>
#include <vector>

#include "display_lock_core.h"
#include "mtest.h"

using maburplay::DisplayLockCore;
using maburplay::ModeTiming;
using maburplay::RefreshSync;
using maburplay::parse_refresh_sync;
using maburplay::refresh_sync_name;

namespace {

constexpr int kVtotal = 1125;
constexpr double kPm = 16666.667;
constexpr double kLine = kPm / kVtotal;
constexpr double kM = 4000.0;          // the target margin, µs
constexpr double kFloorUs = 20000.0;   // capture to `want`: a constant delivery floor
constexpr double kLeadUs = 5000.0;     // the release point this much before its vblank
constexpr double kSec = 1e6;

const char* const kWhyFaster = "the camera is faster than the display's mode";
const char* const kWhyNotRate = "the refresh is not the camera's frame rate";
const char* const kWhyReach = "the camera drifts more than a line a frame";

// A 1080p60 mode against a camera of period Pc, vblank by vblank. Each
// frame is captured at c_j = c0 + j*Pc and wants out at c_j + floor; at the
// first vblank whose release point (vb - lead) is at or after that, the
// regulator's margin is (vb - lead) - want, in [0, P). The vblank moves by
// the lines the core asked for and the test applied -- as the display
// thread does: a release puts the mode's own back, a step asking other
// lines than the register's writes them, and nothing else is a write.
struct Sim {
  ModeTiming m{kVtotal, 5, kLine, kPm};
  DisplayLockCore core{kM / 1000.0};
  double Pc;
  double vb;              // the last vblank, µs
  double c0;
  uint32_t seq = 0;
  int d = 0;              // the lines the register holds
  uint64_t j = 0;         // the next frame to sample
  uint32_t pts0 = 0;      // frame 0's capture stamp
  double margin_off = 0;  // added to every margin fed
  bool feed = true;       // margins and stamps fed at all
  bool apply = true;      // the core's step applied (applied() called)
  DisplayLockCore::Step last{0, false};
  std::deque<std::pair<double, double>> margins;  // (vb, margin) of the last 2 s

  Sim(double pc, RefreshSync mode, int fps, double c0_off = kPm / 3,
      ModeTiming mt = ModeTiming{kVtotal, 5, kLine, kPm})
      : m(mt), Pc(pc), vb(1e9), c0(1e9 + c0_off) {
    core.set_mode(mode);
    core.set_camera_fps(fps);
  }

  double t() const { return vb - 1e9; }  // simulated time, µs

  DisplayLockCore::Step tick() {
    ++seq;
    vb += (m.vtotal + d) * m.line_us;
    for (;;) {
      const double c = c0 + static_cast<double>(j) * Pc;
      const double want = c + kFloorUs;
      if (want > vb - kLeadUs) break;
      const double margin = (vb - kLeadUs) - want;
      if (feed) {
        core.on_margin(margin + margin_off,
                       static_cast<uint64_t>(std::llround(std::min(want + 2000.0, vb))));
        core.on_pts(static_cast<uint32_t>(pts0 + static_cast<uint64_t>(std::llround(c - c0))));
        margins.push_back({vb, margin});
      }
      ++j;
    }
    while (!margins.empty() && margins.front().first + 2 * kSec < vb) margins.pop_front();
    const uint64_t vbu = static_cast<uint64_t>(std::llround(vb));
    last = core.step(seq, vbu, vbu + 300, m);
    if (apply) {
      if (last.release) {
        core.applied(0);
        d = 0;
      } else if (last.lines != core.lines()) {
        core.applied(last.lines);
        d = last.lines;
      }
    }
    return last;
  }

  // Ticks until pred() or `limit_us` more simulated time; true if pred().
  template <typename F>
  bool run_until(double limit_us, F pred) {
    const double end = t() + limit_us;
    while (t() < end) {
      tick();
      if (pred()) return true;
    }
    return false;
  }
  void run_for(double us) {
    run_until(us, [] { return false; });
  }

  double median_margin() const {
    std::vector<double> v;
    for (const auto& p : margins) v.push_back(p.second);
    std::sort(v.begin(), v.end());
    return v.empty() ? 0 : v[v.size() / 2];
  }
};

}  // namespace

TEST(parse_refresh_sync_names) {
  for (RefreshSync m : {RefreshSync::Off, RefreshSync::Nudge, RefreshSync::Follow}) {
    RefreshSync back = RefreshSync::Off;
    REQUIRE(parse_refresh_sync(refresh_sync_name(m), &back));
    CHECK(back == m);
  }
  CHECK(std::strcmp(refresh_sync_name(RefreshSync::Off), "off") == 0);
  CHECK(std::strcmp(refresh_sync_name(RefreshSync::Nudge), "nudge") == 0);
  CHECK(std::strcmp(refresh_sync_name(RefreshSync::Follow), "follow") == 0);
  RefreshSync out = RefreshSync::Nudge;
  CHECK(!parse_refresh_sync("free", &out));
  CHECK(!parse_refresh_sync("on", &out));
  CHECK(out == RefreshSync::Nudge);
  CHECK(std::strcmp(maburplay::display_lock_state_name(DisplayLockCore::State::Waiting), "wait") == 0);
  CHECK(std::strcmp(maburplay::display_lock_state_name(DisplayLockCore::State::Acquiring), "acq") == 0);
  CHECK(std::strcmp(maburplay::display_lock_state_name(DisplayLockCore::State::Locked), "lock") == 0);
}

// A 60.000 Hz mode against a 59.988 fps camera: the screen is the faster,
// so NUDGE makes frames longer only, and holds the margin at the target.
TEST(core_nudge_faster_screen_locks_and_holds) {
  Sim s(16670.0, RefreshSync::Nudge, 60);
  REQUIRE(s.run_until(20 * kSec, [&] { return s.core.locked(); }));
  CHECK(!s.core.nudge_shorter());
  const double end = s.t() + 20 * kSec;
  double worst = 0;
  int min_l = 0, max_l = 0;
  while (s.t() < end) {
    s.tick();
    worst = std::max(worst, std::fabs(s.core.error_ms()));
    min_l = std::min(min_l, s.d);
    max_l = std::max(max_l, s.d);
    CHECK(s.core.locked());
  }
  // line_step holds still until the vblank is 3 lines off, and the error is
  // read after a vblank's drift has moved it on (3.333 us = 0.23 line a
  // frame here, plus the target's own step): 4.5 lines.
  CHECK(worst < 4.5 * kLine / 1000.0);
  CHECK(min_l == 0 && max_l == 1);
  // The hold band again: the vblank waits until it is 3 lines off and then
  // steps back to within half a line, a one-sided sawtooth, so the median
  // margin sits within 3 lines of M.
  CHECK(std::fabs(s.median_margin() - kM) <= 3 * kLine);
}

// A 60.000 Hz mode against a 60.04 fps camera: the screen is the slower.
// Until the camera's period is measured the core steps longer on the
// nominal one; from then on, shorter only, and it holds.
TEST(core_nudge_slower_screen_steps_shorter) {
  Sim s(16658.1, RefreshSync::Nudge, 60);
  REQUIRE(s.run_until(30 * kSec, [&] { return s.core.nudge_shorter(); }));
  CHECK(std::fabs(s.core.camera_period_us() - 16658.1) < 1.0);
  int min_l = 0, max_l = 0;
  bool locked = false;
  const double t_dir = s.t();
  while (s.t() < t_dir + 20 * kSec) {
    s.tick();
    CHECK(s.core.nudge_shorter());
    min_l = std::min(min_l, s.d);
    max_l = std::max(max_l, s.d);
    locked = locked || s.core.locked();
  }
  CHECK(locked);
  CHECK(min_l == -1 && max_l == 0);
}

// Before the camera's period is measured it is taken as the nominal one --
// the mode's own here -- so the screen is not the slower: longer.
TEST(core_nudge_direction_before_period_is_longer) {
  Sim s(16658.1, RefreshSync::Nudge, 60);
  s.run_for(5 * kSec);  // ~300 stamps, fewer than kCamMinStamps
  CHECK(s.core.camera_period_us() == 0);
  CHECK(!s.core.nudge_shorter());
}

// A camera 0.3 % slower than the mode: FOLLOW adds lines to each frame to
// match it, and learns the grid to the camera's measured period.
TEST(core_follow_slower_camera_adds_lines) {
  const double pc = kPm * 1.003;
  Sim s(pc, RefreshSync::Follow, 60);
  REQUIRE(s.run_until(15 * kSec, [&] { return s.core.locked(); }));
  double diff_sum = 0;
  int n = 0, worst_abs = 0;
  while (s.t() < 60 * kSec) {
    s.tick();
    diff_sum += std::fabs(s.core.lines() - s.core.grid_lines());
    ++n;
    worst_abs = std::max(worst_abs, std::abs(s.core.lines()));
  }
  CHECK(diff_sum / n <= 1.0);
  CHECK(worst_abs <= 22);
  CHECK(std::fabs(s.core.grid_lines() - (pc / kLine - kVtotal)) < 0.5);
  CHECK(s.core.locked());
}

// A camera 0.3 % faster than the mode, inside the 0.5 % FOLLOW takes off a
// frame: fewer lines, never more than the grid's own fewer.
TEST(core_follow_faster_camera_within_half_percent) {
  const double pc = kPm * 0.997;
  Sim s(pc, RefreshSync::Follow, 60);
  REQUIRE(s.run_until(60 * kSec, [&] { return s.core.locked(); }));
  double sum = 0;
  int n = 0, min_l = 0;
  const double end = s.t() + 20 * kSec;
  while (s.t() < end) {
    s.tick();
    sum += s.core.lines();
    ++n;
    min_l = std::min(min_l, s.core.lines());
  }
  CHECK(sum / n <= 0.0);
  CHECK(min_l >= -5);
  CHECK(s.core.locked());
}

// A camera 0.7 % faster than the mode is past what FOLLOW takes off a frame:
// once its period is measured, the core lets go and waits.
TEST(core_follow_camera_too_fast_waits) {
  Sim s(kPm * 0.993, RefreshSync::Follow, 60);
  bool ever_locked = false, said = false, released = false;
  int after = 0;
  while (s.t() < 40 * kSec) {
    const DisplayLockCore::Step st = s.tick();
    ever_locked = ever_locked || s.core.locked();
    const char* why = s.core.why();
    if (!said && why && std::strcmp(why, kWhyFaster) == 0) {
      said = true;
      // The step the measurement landed on writes 0 lines over the grid's
      // (the register off the mode's own), the next lets go of it.
      CHECK(st.lines == 0);
      CHECK(!st.release);
      after = 1;
    } else if (after == 1) {
      released = st.lines == 0 && st.release;
      after = 2;
    }
  }
  CHECK(!ever_locked);
  CHECK(said);
  CHECK(released);
  CHECK(s.core.state() == DisplayLockCore::State::Waiting);
  CHECK(s.core.lines() == 0);
  const char* why = s.core.why();
  REQUIRE(why != nullptr);
  CHECK(std::strcmp(why, kWhyFaster) == 0);
}

// A camera 2 % slower than the mode (a ~58.8 fps camera, reported as 59) is
// no whole-refresh match for NUDGE: from the nominal rate and from the
// measurement, it waits.
TEST(core_nudge_rate_mismatch_waits) {
  Sim s(kPm * 1.02, RefreshSync::Nudge, 59);
  s.run_for(1 * kSec);
  CHECK(s.core.state() == DisplayLockCore::State::Waiting);
  REQUIRE(s.core.why() != nullptr);
  CHECK(std::strcmp(s.core.why(), kWhyNotRate) == 0);
  bool ever = false;
  while (s.t() < 35 * kSec) {
    s.tick();
    ever = ever || s.core.state() != DisplayLockCore::State::Waiting;
  }
  CHECK(!ever);
  CHECK(std::fabs(s.core.camera_period_us() - kPm * 1.02) < 1.0);
  REQUIRE(s.core.why() != nullptr);
  CHECK(std::strcmp(s.core.why(), kWhyNotRate) == 0);
  CHECK(s.last.lines == 0);
}

// A write that did not land (applied() not called) leaves the reference
// where it was, and the step asks for the same again.
TEST(core_skipped_write_leaves_g_alone) {
  Sim s(16670.0, RefreshSync::Nudge, 60);
  s.apply = false;
  bool asked = false;
  while (!asked && s.t() < 20 * kSec) {
    const DisplayLockCore::Step st = s.tick();
    asked = st.lines == 1 && s.core.lines() == 0 && s.core.error_ms() != 0;
    if (!asked) {
      s.core.applied(st.lines);
      s.d = st.lines;
    }
  }
  REQUIRE(asked);  // the step asked +1 and the write did not land
  const double e0 = s.core.error_ms();
  const DisplayLockCore::Step st = s.tick();
  CHECK(std::fabs(s.core.error_ms() - e0) < 0.01);
  CHECK(st.lines == 1);
  CHECK(s.core.lines() == 0);
}

// A pts discontinuity forgets the samples and the judgements, not the shift
// or the register: FOLLOW keeps its grid's lines until samples return.
TEST(core_discontinuity_clears_window_keeps_lines) {
  Sim s(kPm * 1.003, RefreshSync::Follow, 60);
  s.run_for(30 * kSec);  // the camera's period measured and in use
  REQUIRE(s.core.locked());
  s.core.applied(3);
  s.d = 3;
  s.core.on_discontinuity();
  CHECK(s.core.state() == DisplayLockCore::State::Waiting);
  CHECK(s.core.lines() == 3);
  CHECK(s.core.error_ms() == 0);
  s.feed = false;
  for (int i = 0; i < 30; ++i) {
    CHECK(s.tick().lines == 3);
    CHECK(s.core.state() == DisplayLockCore::State::Waiting);
    CHECK(s.core.error_ms() == 0);
  }
  s.feed = true;
  REQUIRE(s.run_until(1 * kSec, [&] { return s.core.state() != DisplayLockCore::State::Waiting; }));
  CHECK(s.core.state() == DisplayLockCore::State::Acquiring);
}

// Paused (the display rate does not match the camera's): let go at once.
TEST(core_paused_releases) {
  Sim s(16670.0, RefreshSync::Nudge, 60);
  REQUIRE(s.run_until(20 * kSec, [&] { return s.core.locked(); }));
  REQUIRE(s.run_until(5 * kSec, [&] { return s.core.lines() == 1; }));
  s.core.set_paused(true);
  // The register at +1: first an immediate 0 lines (written early in a
  // frame, as every write is), and only then the release.
  DisplayLockCore::Step st = s.tick();
  CHECK(st.lines == 0);
  CHECK(!st.release);
  CHECK(s.core.lines() == 0);
  CHECK(s.core.state() == DisplayLockCore::State::Waiting);
  REQUIRE(s.core.why() != nullptr);
  CHECK(std::strcmp(s.core.why(), "rate mismatch") == 0);
  st = s.tick();
  CHECK(st.lines == 0);
  CHECK(st.release);
  st = s.tick();
  CHECK(st.lines == 0);
  CHECK(!st.release);
}

// Capture stamps are µs mod 2^32: the period is measured across the wrap,
// here 5 s into the 10 s window (a wrap early on would be outlived by a
// window refilled after it).
TEST(core_camera_period_survives_pts_wrap) {
  Sim s(16670.0, RefreshSync::Nudge, 60);
  s.pts0 = 0xFFFFFFFFu - 5000000u;
  s.run_for(10.2 * kSec);
  CHECK(std::fabs(s.core.camera_period_us() - 16670.0) < 1.0);
}

// The camera changed rate: its measured period is forgotten.
TEST(core_camera_fps_change_clears_period) {
  Sim s(16670.0, RefreshSync::Nudge, 60);
  s.run_for(21 * kSec);
  REQUIRE(s.core.camera_period_us() > 0);
  s.core.set_camera_fps(100);
  CHECK(s.core.camera_period_us() == 0);
}

// A rate change carries nothing of the old camera into the new lock: the
// samples, the state, FOLLOW's learnt trim. (60 -> 59 -> 60: two changes,
// back to a nominal this mode can hold.)
TEST(core_camera_fps_change_restarts_lock) {
  Sim s(kPm * 1.003, RefreshSync::Follow, 60);
  REQUIRE(s.run_until(15 * kSec, [&] { return s.core.locked(); }));
  s.run_for(15 * kSec - s.t());  // trim learnt, the period not yet adopted
  REQUIRE(s.core.grid_lines() > 1.0);
  s.core.set_camera_fps(59);
  CHECK(s.core.state() == DisplayLockCore::State::Waiting);
  CHECK(s.core.error_ms() == 0);
  s.core.set_camera_fps(60);
  s.feed = false;
  s.tick();
  CHECK(std::fabs(s.core.grid_lines() - ((1e6 / 60) / kLine - kVtotal)) < 1e-6);
  CHECK(s.core.state() == DisplayLockCore::State::Waiting);
  CHECK(s.core.error_ms() == 0);
}

// ... nor the period in use or its stamps: NUDGE's direction goes back to
// the nominal's, and with no new frames no reading comes from the old ones.
TEST(core_camera_fps_change_forgets_period_and_stamps) {
  Sim s(16658.1, RefreshSync::Nudge, 60);
  REQUIRE(s.run_until(30 * kSec, [&] { return s.core.nudge_shorter(); }));
  s.core.set_camera_fps(59);
  s.core.set_camera_fps(60);
  s.feed = false;
  s.tick();
  CHECK(!s.core.nudge_shorter());
  CHECK(s.core.camera_period_us() == 0);
  s.run_for(10.5 * kSec);  // past the restarted reading's 10 s
  CHECK(s.core.camera_period_us() == 0);
}

// The register at +1, a step writes 0 (an override, not the mode's own),
// then the core cannot hold: it asks for the release.
TEST(core_unusable_releases_an_override_of_zero) {
  Sim s(16670.0, RefreshSync::Nudge, 60);
  REQUIRE(s.run_until(20 * kSec, [&] { return s.core.locked(); }));
  int prev = s.core.lines();
  REQUIRE(s.run_until(5 * kSec, [&] {
    const bool hit = prev == 1 && s.last.lines == 0 && !s.last.release;
    prev = s.core.lines();
    return hit;
  }));
  REQUIRE(s.core.lines() == 0);
  s.core.set_paused(true);
  // The 0 already holds: no second write of it, the release at once, and
  // once released no more asks.
  DisplayLockCore::Step st = s.tick();
  CHECK(st.lines == 0);
  CHECK(st.release);
  st = s.tick();
  CHECK(st.lines == 0);
  CHECK(!st.release);
}

// Usable, the register holding an override of 0 and the step wanting 0:
// release, and once released, no more asks.
TEST(core_usable_zero_releases_once) {
  Sim s(16670.0, RefreshSync::Nudge, 60);
  REQUIRE(s.run_until(20 * kSec, [&] { return s.core.locked(); }));
  int prev = s.core.lines();
  REQUIRE(s.run_until(5 * kSec, [&] {
    const bool hit = prev == 1 && s.last.lines == 0 && !s.last.release;
    prev = s.core.lines();
    return hit;
  }));
  // line_step just arrived: it holds 0 until the vblank drifts 3 lines off.
  DisplayLockCore::Step st = s.tick();
  CHECK(st.lines == 0);
  CHECK(st.release);
  st = s.tick();
  CHECK(st.lines == 0);
  CHECK(!st.release);
}

// A mode change starts over: Waiting, and FOLLOW's grid without the trim
// it had learnt.
TEST(core_set_mode_resets) {
  Sim s(kPm * 1.003, RefreshSync::Follow, 60);
  REQUIRE(s.run_until(15 * kSec, [&] { return s.core.locked(); }));
  s.run_for(3 * kSec);
  REQUIRE(s.t() < 19 * kSec);  // still on the nominal period: B = 0
  REQUIRE(s.core.grid_lines() > 0.3);  // trim learnt
  s.core.set_mode(RefreshSync::Follow);
  CHECK(s.core.state() == DisplayLockCore::State::Waiting);
  CHECK(!s.core.locked());
  s.feed = false;
  s.tick();
  // B on the nominal period: 1e6/60 against this mode's 16666.667 µs.
  CHECK(std::fabs(s.core.grid_lines() - ((1e6 / 60) / kLine - kVtotal)) < 1e-6);
  CHECK(s.core.error_ms() == 0);
}

// Locked, then the margins jump 5 ms: two judgements over kLockOutMs and it
// is acquiring again. The median takes half the window (1 s) to follow the
// jump, the first judgement after that lands up to kJudgeUs later and the
// second a kJudgeUs after that: 3 s, plus the ~25 ms a frame's margin takes
// to arrive after its capture -- 3.5 s with slack.
TEST(core_lock_out_after_two_bad_judgements) {
  Sim s(16670.0, RefreshSync::Nudge, 60);
  REQUIRE(s.run_until(20 * kSec, [&] { return s.core.locked(); }));
  s.run_for(2 * kSec);
  REQUIRE(s.core.locked());
  s.margin_off = 5000.0;
  CHECK(s.run_until(3.5 * kSec, [&] { return !s.core.locked(); }));
  CHECK(s.core.state() == DisplayLockCore::State::Acquiring);
}

// A register the lock found bent (left by a run that did not put it back)
// is brought back as any other: an immediate 0 lines, then the release.
// Even with refresh sync off.
TEST(core_found_bent_register_written_back_then_released) {
  Sim s(16670.0, RefreshSync::Off, 60);
  REQUIRE(s.core.found_register(4, true, s.m));
  s.d = 4;
  CHECK(s.core.lines() == 4);
  DisplayLockCore::Step st = s.tick();
  CHECK(st.lines == 0);
  CHECK(!st.release);
  CHECK(s.core.lines() == 0);
  st = s.tick();
  CHECK(st.lines == 0);
  CHECK(st.release);
  st = s.tick();
  CHECK(!st.release);
}

// A held immediate write of the mode's own total: nothing to write, the
// release only.
TEST(core_found_held_register_released) {
  Sim s(16670.0, RefreshSync::Nudge, 60);
  REQUIRE(s.core.found_register(0, true, s.m));
  DisplayLockCore::Step st = s.tick();
  CHECK(st.lines == 0);
  CHECK(st.release);
  st = s.tick();
  CHECK(!st.release);
}

// A 100 Hz mode (1125 lines, 8.9 us a line) against a camera on the drone's
// trim, 0.111 Hz under: it drifts 11.1 us = 1.25 lines a frame, faster than
// one line a frame pulls back. Once the period is measured NUDGE waits, says
// why, and leaves the register at the mode's own: no writes past bringing
// it back.
TEST(core_nudge_beyond_reach_waits) {
  const ModeTiming m100{kVtotal, 5, 10000.0 / kVtotal, 10000.0};
  Sim s(1e6 / (100.0 - 0.111), RefreshSync::Nudge, 100, 10000.0 / 3, m100);
  bool said = false;
  int writes_after = 0, releases_after = 0, ticks_after = 0;
  while (s.t() < 60 * kSec) {
    const int before = s.core.lines();
    const DisplayLockCore::Step st = s.tick();
    const char* why = s.core.why();
    if (!said && why && std::strcmp(why, kWhyReach) == 0) said = true;
    if (!said) continue;
    ++ticks_after;
    if (st.lines != before) ++writes_after;
    if (st.release) ++releases_after;
    CHECK(st.lines == 0);
    CHECK(s.core.state() == DisplayLockCore::State::Waiting);
    CHECK(s.core.beyond_reach());
  }
  REQUIRE(said);
  CHECK(ticks_after > 100 * 30);  // said within the first 30 s
  CHECK(writes_after <= 1);       // the register brought back to 0 lines, at most once
  CHECK(releases_after == 1);
  CHECK(s.core.lines() == 0);
  REQUIRE(s.core.why() != nullptr);
  CHECK(std::strcmp(s.core.why(), kWhyReach) == 0);
}

// The same trim at 60 Hz is 0.04 Hz under: 11.1 us = 0.75 line a frame,
// inside the reach. NUDGE pulls in at a quarter line a frame (the long way
// round is at most three quarters of a refresh, ~56 s) and holds.
TEST(core_nudge_three_quarter_line_drift_holds) {
  Sim s(kPm + 0.75 * kLine, RefreshSync::Nudge, 60);
  bool ever_reach = false;
  REQUIRE(s.run_until(90 * kSec, [&] {
    ever_reach = ever_reach || s.core.beyond_reach();
    return s.core.locked();
  }));
  CHECK(!ever_reach);
  int min_l = 0, max_l = 0;
  const double end = s.t() + 20 * kSec;
  while (s.t() < end) {
    s.tick();
    min_l = std::min(min_l, s.d);
    max_l = std::max(max_l, s.d);
    CHECK(s.core.locked());
    CHECK(!s.core.beyond_reach());
  }
  CHECK(min_l == 0 && max_l == 1);
}

// A vblank the thread slept through (seq up by 2): the register's lines ran
// both frames, so G moves by both. Margins anchored at the first vblank's
// G = 0 put the error at -G.
TEST(core_seq_gap_integrates_each_frame) {
  DisplayLockCore core(kM / 1000.0);
  core.set_mode(RefreshSync::Nudge);
  core.set_camera_fps(60);  // the nominal: the mode's own period, no drift
  const ModeTiming m{kVtotal, 5, kLine, kPm};
  const uint64_t t0 = 1'000'000'000;
  core.step(1, t0, t0 + 300, m);
  core.applied(4);
  for (int i = 0; i < 60; ++i) core.on_margin(kM, t0 + 1 + static_cast<uint64_t>(i));
  const uint64_t t1 = t0 + static_cast<uint64_t>(std::llround(2 * (kVtotal + 4) * kLine));
  core.step(3, t1, t1 + 300, m);
  CHECK(std::fabs(core.error_ms() + 8 * kLine / 1000.0) < 0.01 * kLine / 1000.0);
}

MTEST_MAIN
