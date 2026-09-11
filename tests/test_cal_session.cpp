#include "mtest.h"
#include "cal_session.h"
#include "cal_plan.h"
#include "cal_log.h"

#include <cstdlib>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <vector>

using namespace maburgs;

namespace {

// Mirrors tests/test_cal_log.cpp's own fresh_dir(): a clean scratch
// directory per test, under the build tree (MABUR_TEST_SCRATCH_DIR), not
// /tmp. Best-effort cleanup, deliberately unchecked (see that file's
// comment on system()'s warn_unused_result).
std::string fresh_dir(const char* name) {
  const std::string d = std::string(MABUR_TEST_SCRATCH_DIR) + "/" + name;
  if (std::system(("rm -rf " + d).c_str()) != 0) { /* best-effort */ }
  ::mkdir(d.c_str(), 0755);
  return d;
}

std::vector<std::string> cal_log_lines(const std::string& dir) {
  std::ifstream f(dir + "/cal.log");
  std::vector<std::string> out;
  for (std::string l; std::getline(f, l);) out.push_back(l);
  return out;
}

// The real TXAGC transfer curve is a flat floor below idx ~28, a ~0.3 dB/idx
// ramp to ~91, then a flat ceiling to 127 (docs/txagc-calibration.md). A
// CONSTANT curve is not a simplification of that, it is a different physical
// claim -- power that never rises -- whose knee correctly sits at the bottom
// of the sweep. Feeding one here made this test assert against a degenerate
// case rather than against the behavior it is named for.
int ramp_rssi(int idx) {
  if (idx <= 28) return -80;
  if (idx <= 91) return -80 + (idx - 28) * 3 / 10;
  return -80 + (91 - 28) * 3 / 10;
}

// Feeds a whole phase's worth of frames at `pct` delivery on card 0.
void feed_phase(CalSession& s, const mabur::rc::CalCmd& c, int pct,
                uint64_t now_ms) {
  uint16_t seq = 0;
  for (const auto& w : c.windows) {
    for (int i = w.idx_lo; i <= w.idx_hi; i += w.idx_step) {
      const int n = c.frames_per_cell * pct / 100;
      for (int k = 0; k < n; ++k) {
        mabur::cal::CalFrameInfo f{w.rate, static_cast<uint8_t>(i), c.phase,
                                   seq++};
        s.on_cal_frame(0, f, ramp_rssi(i), /*crc_ok=*/true, now_ms);
      }
    }
  }
}

// Feeds one phase, delivering pct(rate, idx) percent of each cell -- the
// variable-delivery sibling of feed_phase, needed to produce a row that
// actually dips.
template <typename F>
void feed_phase_fn(CalSession& s, const mabur::rc::CalCmd& c, F pct,
                   uint64_t now_ms) {
  uint16_t seq = 0;
  for (const auto& w : c.windows) {
    for (int i = w.idx_lo; i <= w.idx_hi; i += w.idx_step) {
      const int n = c.frames_per_cell * pct(w.rate, i) / 100;
      for (int k = 0; k < n; ++k) {
        mabur::cal::CalFrameInfo f{w.rate, static_cast<uint8_t>(i), c.phase,
                                   seq++};
        s.on_cal_frame(0, f, ramp_rssi(i), /*crc_ok=*/true, now_ms);
      }
    }
  }
}

}  // namespace

TEST(start_refuses_without_capability) {
  CalSession s(CalSessionCfg{});
  std::string err;
  s.set_peer(/*linked=*/false, /*cal_capable=*/true);
  CHECK(!s.start(1, 1, 0, &err));
  CHECK(err.find("link") != std::string::npos);

  s.set_peer(/*linked=*/true, /*cal_capable=*/false);
  CHECK(!s.start(1, 1, 0, &err));
  CHECK(err.find("CAP_CALIBRATE") != std::string::npos);
}

TEST(start_refuses_while_a_session_runs) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 1, 0, &err));
  CHECK(!s.start(1, 2, 10, &err));
  CHECK(err.find("running") != std::string::npos);
}

TEST(repeats_command_until_acknowledged) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 42, 0, &err));
  const auto a = s.due_cmd(0);
  REQUIRE(a.has_value());
  CHECK(a->phase == mabur::cal::kPhaseCoarse);
  CHECK(a->nonce == 42);
  // Still unacknowledged: the GS must offer it again.
  const auto b = s.due_cmd(300);
  CHECK(b.has_value());
  s.on_ack(42, 53, 400);
  // Acknowledged: nothing more to send, and the air must go quiet.
  CHECK(!s.due_cmd(500).has_value());
  CHECK(s.radio_silent(500));
}

TEST(radio_is_silent_for_the_whole_phase_and_opens_after) {
  // A GS send deafens both RX cards for ~180 us; during a sweep that loss is
  // counted as PA compression. Silence is what makes the numbers mean what
  // they say.
  CalSessionCfg cfg;
  cfg.phase_slack_ms = 1000;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 7, 0, &err));
  REQUIRE(s.due_cmd(0).has_value());
  s.on_ack(7, 53, 100);

  const auto plan = make_coarse_plan(1, 7);
  const uint32_t dur = plan_duration_ms(plan);
  CHECK(s.radio_silent(100));
  CHECK(s.radio_silent(100 + dur / 2));
  CHECK(s.radio_silent(100 + dur));            // still inside the slack
  CHECK(!s.radio_silent(100 + dur + 1001));    // listen window is open
}

TEST(ack_timeout_fails_the_session_without_writing_anything) {
  CalSessionCfg cfg;
  cfg.ack_timeout_ms = 3000;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 1, 0, &err));
  s.due_cmd(0);
  CHECK(s.state() == CalSession::State::AwaitAck);
  s.due_cmd(3500);
  CHECK(s.state() == CalSession::State::Failed);
  CHECK(!s.due_result(3600).has_value());
}

TEST(coarse_then_fine_then_result) {
  CalSessionCfg cfg;
  cfg.phase_slack_ms = 0;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 5, 0, &err));
  s.due_cmd(0);
  s.on_ack(5, 53, 1);

  // Coarse: every rate clean everywhere -> all no-dip, knee at peak RSSI.
  const auto coarse = make_coarse_plan(1, 5);
  feed_phase(s, coarse, 100, 10);
  const uint64_t t1 = 1 + plan_duration_ms(coarse) + 1;

  // With no dips there is no fine phase; the session goes straight to result.
  const auto res = s.due_result(t1 + 2000);
  REQUIRE(res.has_value());
  CHECK(res->nonce == 5);
  // Every rate is clean everywhere, so every row is no-dip and takes its
  // wall from the RSSI knee: peak -62 dBm is first reached at idx 88. The
  // result carries that RAW wall verbatim -- no margin subtracted here.
  // margin_db is applied exactly once, on the drone, by power_plan.h's
  // diff[r] = walls[r] - m - base_ref_idx (spec: two independently
  // configured margins in one derivation is the bug this shape avoids).
  for (int r = 0; r < 8; ++r) CHECK(res->walls[r] == 88);
}

// The coarse pass locates a dip at 4-index resolution; the fine pass is what
// turns that estimate into the index actually written to the drone's config.
// This is the path that produces the number, so it gets pinned end to end.
TEST(fine_phase_sharpens_a_real_dip_and_flags_drift) {
  CalSessionCfg cfg;
  cfg.phase_slack_ms = 0;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 11, 0, &err));
  s.due_cmd(0);
  s.on_ack(11, 53, 1);

  // Every rate clean except rate 5, which dips above idx 56. At coarse
  // resolution (step 4) the last clean cell is therefore 56.
  const auto coarse = make_coarse_plan(1, 11);
  feed_phase_fn(s, coarse,
                [](uint8_t r, int i) { return r != 5 ? 100 : (i <= 56 ? 100 : 10); },
                10);
  const uint64_t t1 = 1 + plan_duration_ms(coarse) + 1;

  // A fine phase must now be commanded, for rate 5 alone.
  const auto fine = s.due_cmd(t1);
  REQUIRE(fine.has_value());
  CHECK(fine->phase == mabur::cal::kPhaseFine);
  REQUIRE(fine->windows.size() == 1);
  CHECK(fine->windows[0].rate == 5);
  CHECK(fine->windows[0].idx_step == 1);
  s.on_ack(11, 53, t1 + 1);

  // At full resolution the true wall is 54 -- two steps below the coarse
  // estimate. The merge must adopt 54 and flag the disagreement.
  feed_phase_fn(s, *fine, [](uint8_t, int i) { return i <= 54 ? 100 : 10; },
                t1 + 2);
  const uint64_t t2 = t1 + 1 + plan_duration_ms(*fine) + 1;

  const auto res = s.due_result(t2 + 5000);
  REQUIRE(res.has_value());
  CHECK(res->walls[5] == 54);              // fine wall, raw -- no margin subtracted
  CHECK(res->walls[0] == 88);              // untouched rows keep their raw knee wall
  CHECK((s.walls()[5].flags & kCalDrift) != 0);   // coarse 56 vs fine 54
}

TEST(undetermined_rate_reaches_the_result_as_minus_one) {
  CalSessionCfg cfg;
  cfg.phase_slack_ms = 0;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 6, 0, &err));
  s.due_cmd(0);
  s.on_ack(6, 53, 1);
  // Nothing heard at all: every rate is undetermined.
  const auto coarse = make_coarse_plan(1, 6);
  const uint64_t t1 = 1 + plan_duration_ms(coarse) + 1;
  const auto res = s.due_result(t1 + 5000);
  REQUIRE(res.has_value());
  for (int r = 0; r < 8; ++r) CHECK(res->walls[r] == -1);
}

TEST(verify_phase_tallies_by_rate_despite_a_margin_mismatch) {
  // Review fix (Important 4): the GS seeds verify cells at wall - m_gs; the
  // drone independently derives wall - m_drone. If the two configs'
  // margin_db values ever disagree (nothing enforces they match -- both
  // default to 1.0, hand-set on each end separately), an exact-index match
  // would read ZERO delivery on every rate: a silent, total verify failure
  // sitting right next to a correct table already written to disk.
  // Tallying by rate alone -- there is exactly one verify cell per rate,
  // so the index carries no information the rate does not already -- is
  // what removes that coupling entirely.
  CalSessionCfg cfg;
  cfg.phase_slack_ms = 0;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 9, 0, &err));
  s.due_cmd(0);
  s.on_ack(9, 53, 1);

  // Every rate clean everywhere -> knee wall 88, this session's own park
  // index 88 - 4 = 84 (1 dB margin -> 4 TXAGC steps).
  const auto coarse = make_coarse_plan(1, 9);
  feed_phase(s, coarse, 100, 10);
  const uint64_t t1 = 1 + plan_duration_ms(coarse) + 1;
  REQUIRE(s.due_result(t1 + 2000).has_value());  // arms verify (begin_verify)

  // The drone parks one index off from what THIS session predicted --
  // standing in for a disagreeing drone-side margin_db -- yet every frame
  // must still land and be counted.
  for (int r = 0; r < 8; ++r) {
    mabur::cal::CalFrameInfo f{static_cast<uint8_t>(r), 85,
                              mabur::cal::kPhaseVerify, 0};
    s.on_cal_frame(0, f, -60, /*crc_ok=*/true, t1 + 3000);
  }
  for (int r = 0; r < 8; ++r) CHECK(s.cell_received(r, 84, 0) == 1);
}

TEST(corrupt_frames_count_as_loss_not_delivery) {
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 8, 0, &err));
  s.due_cmd(0);
  s.on_ack(8, 53, 1);
  mabur::cal::CalFrameInfo f{0, 40, mabur::cal::kPhaseCoarse, 0};
  for (int k = 0; k < 20; ++k)
    s.on_cal_frame(0, f, -70, /*crc_ok=*/false, 10);
  CHECK(s.cell_received(0, 40, 0) == 0);
  CHECK(s.cell_corrupt(0, 40) == 20);
}

TEST(frames_from_a_stale_phase_are_ignored) {
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 9, 0, &err));
  s.due_cmd(0);
  s.on_ack(9, 53, 1);
  mabur::cal::CalFrameInfo f{0, 40, mabur::cal::kPhaseVerify, 0};  // wrong phase
  s.on_cal_frame(0, f, -70, true, 10);
  CHECK(s.cell_received(0, 40, 0) == 0);
}

TEST(abort_returns_to_idle_and_reopens_the_air) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 1, 0, &err));
  s.due_cmd(0);
  s.on_ack(1, 53, 1);
  CHECK(s.radio_silent(100));
  s.abort("operator");
  CHECK(!s.radio_silent(100));
  CHECK(s.state() == CalSession::State::Idle);
}

TEST(radio_silent_is_true_from_ack_until_abort) {
  // Named for what it can actually check. main.cpp must gate RCF, the
  // slotter drain, the DISC keepalive and T_CAL_CMD on radio_silent()
  // (T_CAL_RESULT is the deliberate exception -- see due_result()), but
  // nothing here can fail against an ungated call site; only reading
  // main.cpp, or the loopback in tests/test_cal_e2e.cpp -- which watches
  // the invariant continuously against a transcription of those call
  // sites -- can. This pins the predicate itself: shut from the ack,
  // open again the instant a session is aborted.
  CalSessionCfg cfg;
  cfg.phase_slack_ms = 500;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 3, 0, &err));
  s.due_cmd(0);
  s.on_ack(3, 53, 10);
  CHECK(s.radio_silent(11));
  // ... and after abort, everything may transmit again immediately.
  s.abort("test");
  CHECK(!s.radio_silent(12));
}

TEST(null_log_sink_is_inert) {
  // The gap this covers: CalLog::cell()/wall()/verify() went unwired for
  // eleven tasks with nothing catching it -- every existing test above
  // constructs a CalSession with no CalLog* and none of them noticed a
  // real cal.log was never getting written. This makes the "no sink ->
  // nothing happens, no crash" contract an explicit assertion rather than
  // an accident of every other test's setup.
  CalSessionCfg cfg;
  cfg.phase_slack_ms = 0;
  CalSession s(cfg);  // no CalLog* -- log_ defaults to nullptr
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 30, 0, &err));
  s.due_cmd(0);
  s.on_ack(30, 53, 1);
  const auto coarse = make_coarse_plan(1, 30);
  feed_phase(s, coarse, 100, 10);
  const uint64_t t1 = 1 + plan_duration_ms(coarse) + 1;
  // finish_phase() and finalize_result() run their log_ calls right here;
  // with log_ == nullptr this must complete exactly as every other test
  // above does, not crash or silently change the result.
  const auto res = s.due_result(t1 + 2000);
  REQUIRE(res.has_value());
  for (int r = 0; r < 8; ++r) CHECK(res->walls[r] == 88);
}

TEST(cal_log_records_cells_and_walls_at_phase_end) {
  // The actual regression test for the gap: a real CalLog wired in must
  // come out of a coarse-only run holding a C row per swept cell and a W
  // row per rate, with an unheard rate's wall/floor written as the
  // documented -1 sentinels (cal_log.h) rather than 0 or garbage.
  const std::string dir = fresh_dir("cal_session_log_cells");
  {
    // Scoped: LogWriter's writer thread flushes on ~LogWriter (joined from
    // CalLog's destructor), so the file is only guaranteed complete once
    // `log` (and the CalSession pointing at it) goes out of scope --
    // exactly the pattern tests/test_cal_log.cpp uses.
    CalLog log(dir);
    log.header();

    CalSessionCfg cfg;
    cfg.phase_slack_ms = 0;
    CalSession s(cfg, &log);
    s.set_peer(true, true);
    std::string err;
    REQUIRE(s.start(1, 31, 0, &err));
    log.run(31, 53, s.margin_db());
    s.due_cmd(0);
    s.on_ack(31, 53, 1);

    // Every rate clean except rate 3, which hears nothing at all -> that
    // row never reaches deliver_pct anywhere and comes out kCalUndetermined.
    const auto coarse = make_coarse_plan(1, 31);
    feed_phase_fn(s, coarse, [](uint8_t r, int) { return r == 3 ? 0 : 100; }, 10);
    const uint64_t t1 = 1 + plan_duration_ms(coarse) + 1;
    // Undetermined and no-dip rows both skip the fine phase (make_fine_plan),
    // so this coarse-only run goes straight to Result -- finish_phase() has
    // already logged everything by the time due_result() returns.
    REQUIRE(s.due_result(t1 + 2000).has_value());
  }

  const auto lines = cal_log_lines(dir);
  int c_count = 0, w_count = 0;
  bool saw_undetermined_wall = false;
  for (const auto& l : lines) {
    if (l.rfind("C ", 0) == 0) ++c_count;
    if (l.rfind("W ", 0) == 0) {
      ++w_count;
      if (l == "W 3 -1 -1 0 2") saw_undetermined_wall = true;
    }
  }
  // Coarse sweeps idx 0..124 step 4 = 32 cells/rate * 8 rates.
  CHECK(c_count == 256);
  CHECK(w_count == 8);
  CHECK(saw_undetermined_wall);
}

TEST(cal_log_records_verify_results) {
  const std::string dir = fresh_dir("cal_session_log_verify");
  {
    // Scoped for the same reason as the test above: read only after both
    // `log` and `s` (which holds a pointer into it) are gone.
    CalLog log(dir);
    log.header();

    CalSessionCfg cfg;
    cfg.phase_slack_ms = 0;
    CalSession s(cfg, &log);
    s.set_peer(true, true);
    std::string err;
    REQUIRE(s.start(1, 32, 0, &err));
    log.run(32, 53, s.margin_db());
    s.due_cmd(0);
    s.on_ack(32, 53, 1);

    // Every rate clean everywhere -> knee wall 88 (see coarse_then_fine_
    // then_result), this session's own park index 88 - 4 = 84 (1 dB margin).
    const auto coarse = make_coarse_plan(1, 32);
    feed_phase(s, coarse, 100, 10);
    const uint64_t t1 = 1 + plan_duration_ms(coarse) + 1;
    REQUIRE(s.due_result(t1 + 2000).has_value());  // arms verify

    // 97/100 on card 0 for rate 0's verify cell.
    for (int k = 0; k < 97; ++k) {
      mabur::cal::CalFrameInfo f{0, 84, mabur::cal::kPhaseVerify,
                                static_cast<uint16_t>(k)};
      s.on_cal_frame(0, f, -60, /*crc_ok=*/true, t1 + 3000);
    }
    // Close the verify window: phase_slack_ms is 0, and a verify plan with
    // all 8 rates parked (none undetermined here) runs well under a
    // minute, so this tick is guaranteed past phase_end_ms_ regardless of
    // the exact per-cell timing math.
    s.due_cmd(t1 + 2000 + 60000);
    CHECK(s.state() == CalSession::State::Done);
  }

  const auto lines = cal_log_lines(dir);
  bool saw_rate0_verify = false;
  for (const auto& l : lines)
    if (l == "V 0 84 97") saw_rate0_verify = true;
  CHECK(saw_rate0_verify);
}

TEST(result_is_repeated_until_a_verify_frame_acks_it) {
  // T_CAL_RESULT carries the whole run's measured table over an uplink
  // that loses 30-50% of frames, and there is no explicit ack for it. A
  // single send that lost the coin flip ended the run with the drone's
  // config untouched -- while the report claimed it had been written.
  // The drone's verify sweep is the implicit ack (it only sweeps verify
  // after a successful apply), so the repeats run until the first
  // verify-phase frame arrives and then stop for good.
  CalSessionCfg cfg;
  cfg.phase_slack_ms = 0;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 21, 0, &err));
  s.due_cmd(0);
  s.on_ack(21, 53, 1);
  const auto coarse = make_coarse_plan(1, 21);
  feed_phase(s, coarse, 100, 10);
  const uint64_t t1 = 1 + plan_duration_ms(coarse) + 1;

  const uint64_t t_first = t1 + 2000;
  REQUIRE(s.due_result(t_first).has_value());   // first send, arms verify
  CHECK(s.radio_silent(t_first));               // ...and the window is open
  // Too soon: the cadence matches T_CAL_CMD's own ~200 ms.
  CHECK(!s.due_result(t_first + 50).has_value());
  const auto again = s.due_result(t_first + 200);
  REQUIRE(again.has_value());
  CHECK(again->nonce == 21);
  CHECK(again->walls[0] == 88);                 // the same table, verbatim
  REQUIRE(s.due_result(t_first + 400).has_value());

  // The drone applied and started sweeping: one verify frame is the ack.
  mabur::cal::CalFrameInfo f{0, 84, mabur::cal::kPhaseVerify, 0};
  s.on_cal_frame(0, f, -60, /*crc_ok=*/true, t_first + 500);
  CHECK(!s.due_result(t_first + 600).has_value());
  CHECK(!s.due_result(t_first + 5000).has_value());
}

TEST(a_crc_bad_verify_frame_still_stops_the_repeats) {
  // Attribution survives corruption by design (cal_wire.h), and a corrupt
  // verify frame proves just as much as a clean one: the drone applied and
  // is on the air. Continuing to transmit into that sweep is exactly what
  // the radio-silence rule forbids.
  CalSessionCfg cfg;
  cfg.phase_slack_ms = 0;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 22, 0, &err));
  s.due_cmd(0);
  s.on_ack(22, 53, 1);
  const auto coarse = make_coarse_plan(1, 22);
  feed_phase(s, coarse, 100, 10);
  const uint64_t t1 = 1 + plan_duration_ms(coarse) + 1;
  REQUIRE(s.due_result(t1 + 2000).has_value());
  mabur::cal::CalFrameInfo f{3, 84, mabur::cal::kPhaseVerify, 0};
  s.on_cal_frame(0, f, -60, /*crc_ok=*/false, t1 + 2100);
  CHECK(!s.due_result(t1 + 2200).has_value());
}

TEST(result_repeats_are_bounded_when_the_drone_never_applies) {
  // The drone is gone (or its apply was refused): no verify frame will
  // ever arrive. The repeats must stop on their own rather than filling
  // the whole verify window with uplink traffic.
  CalSessionCfg cfg;
  cfg.phase_slack_ms = 0;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 23, 0, &err));
  s.due_cmd(0);
  s.on_ack(23, 53, 1);
  const auto coarse = make_coarse_plan(1, 23);
  feed_phase(s, coarse, 100, 10);
  const uint64_t t1 = 1 + plan_duration_ms(coarse) + 1;

  uint64_t t = t1 + 2000;
  int sends = 0;
  for (int i = 0; i < 200; ++i) {
    if (s.due_result(t).has_value()) ++sends;
    t += 200;
    if (s.state() != CalSession::State::Verify) break;
  }
  // First send plus a bounded number of repeats -- not one per tick for
  // the whole window.
  CHECK(sends >= 2);
  CHECK(sends <= 20);
}

TEST(no_verify_row_is_logged_for_a_rate_that_heard_nothing) {
  // A `V` row means "this rate was verified", never "a window closed".
  // maburcal reads any V row as proof the drone applied (the drone sweeps
  // verify only after a successful apply), so emitting an all-zero row per
  // rate at window close made a run whose T_CAL_RESULT was lost -- or
  // whose apply was refused -- print `written: /etc/mabur.toml`.
  const std::string dir = fresh_dir("cal_session_log_verify_empty");
  {
    CalLog log(dir);
    log.header();
    CalSessionCfg cfg;
    cfg.phase_slack_ms = 0;
    CalSession s(cfg, &log);
    s.set_peer(true, true);
    std::string err;
    REQUIRE(s.start(1, 33, 0, &err));
    log.run(33, 53, s.margin_db());
    s.due_cmd(0);
    s.on_ack(33, 53, 1);
    const auto coarse = make_coarse_plan(1, 33);
    feed_phase(s, coarse, 100, 10);
    const uint64_t t1 = 1 + plan_duration_ms(coarse) + 1;
    REQUIRE(s.due_result(t1 + 2000).has_value());  // arms verify
    // ...and nothing at all comes back: the result never reached the
    // drone, or the drone refused to apply it.
    s.due_cmd(t1 + 2000 + 60000);
    CHECK(s.state() == CalSession::State::Done);
  }
  for (const auto& l : cal_log_lines(dir))
    CHECK(l.rfind("V ", 0) != 0);   // not one V row in the file
}

TEST(records_are_on_disk_as_soon_as_the_run_reaches_done) {
  // LogWriter flushes at 1 Hz; `maburcal start` polls `status` every
  // 500 ms and reads cal.log the instant it sees state=done. The V
  // records are written at the Verify->Done transition, so without a
  // flush at that transition roughly half of successful runs rendered
  // from a file missing its tail -- and printed "no verify pass
  // completed" for a run that worked. Read here WITHOUT destroying the
  // CalLog first (the destructor would flush and hide the bug).
  const std::string dir = fresh_dir("cal_session_flush_on_done");
  CalLog log(dir);
  log.header();
  CalSessionCfg cfg;
  cfg.phase_slack_ms = 0;
  CalSession s(cfg, &log);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 34, 0, &err));
  log.run(34, 53, s.margin_db());
  s.due_cmd(0);
  s.on_ack(34, 53, 1);
  const auto coarse = make_coarse_plan(1, 34);
  feed_phase(s, coarse, 100, 10);
  const uint64_t t1 = 1 + plan_duration_ms(coarse) + 1;
  REQUIRE(s.due_result(t1 + 2000).has_value());
  for (int k = 0; k < 97; ++k) {
    mabur::cal::CalFrameInfo f{0, 84, mabur::cal::kPhaseVerify,
                              static_cast<uint16_t>(k)};
    s.on_cal_frame(0, f, -60, /*crc_ok=*/true, t1 + 3000);
  }
  s.due_cmd(t1 + 2000 + 60000);
  REQUIRE(s.state() == CalSession::State::Done);

  bool saw_verify = false;
  for (const auto& l : cal_log_lines(dir))
    if (l == "V 0 84 97") saw_verify = true;
  CHECK(saw_verify);
}

TEST(a_sweep_frame_is_an_implicit_ack_for_the_phase_it_names) {
  // The drone's ack rides a Telem, and Telem is lost like anything else.
  // Until one arrives the session sits in AwaitAck, where it (a) keeps
  // transmitting T_CAL_CMD repeats every 200 ms straight into the drone's
  // live sweep -- contaminating the measurement the radio-silence rule
  // exists to protect, and blanking its own RX cards with every send --
  // and (b) DROPS every frame that arrives meanwhile, so those cells keep
  // their seeded 0-received tally and score as total loss.
  //
  // A sweep frame for the phase being commanded proves exactly what the
  // ack would have: the drone accepted this phase and is on the air with
  // it. Found by tests/test_cal_e2e.cpp.
  CalSessionCfg cfg;
  cfg.phase_slack_ms = 0;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 71, 0, &err));
  REQUIRE(s.due_cmd(0).has_value());
  CHECK(!s.radio_silent(10));   // no ack yet: the air is still open

  mabur::cal::CalFrameInfo f{0, 8, mabur::cal::kPhaseCoarse, 0};
  s.on_cal_frame(0, f, -70, /*crc_ok=*/true, 100);
  CHECK(s.state() == CalSession::State::Sweep);
  CHECK(s.radio_silent(101));               // ...and now it is shut
  CHECK(!s.due_cmd(300).has_value());       // no more repeats into the sweep
  CHECK(s.cell_received(0, 8, 0) == 1);     // and the frame itself counted

  // The window is sized from the frame's arrival, so the phase still gets
  // its full planned duration of silence.
  const uint32_t dur = plan_duration_ms(make_coarse_plan(1, 71));
  CHECK(s.radio_silent(100 + dur - 1));
  CHECK(!s.radio_silent(100 + dur + 1));
}

TEST(a_frame_from_another_phase_is_not_an_implicit_ack) {
  // Only the phase currently being commanded counts. A straggler from a
  // phase this session already left (or one it has not asked for yet)
  // must not open the sweep window early or be tallied into it.
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1, 72, 0, &err));
  REQUIRE(s.due_cmd(0).has_value());
  mabur::cal::CalFrameInfo f{0, 8, mabur::cal::kPhaseVerify, 0};
  s.on_cal_frame(0, f, -70, true, 100);
  CHECK(s.state() == CalSession::State::AwaitAck);
  CHECK(s.cell_received(0, 8, 0) == 0);
}

MTEST_MAIN
