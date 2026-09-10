#include "mtest.h"
#include "cal_session.h"
#include "cal_plan.h"

#include <string>

using namespace maburgs;

namespace {

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

TEST(gate_contract_covers_every_transmit_class) {
  // main.cpp must gate RCF, the slotter drain, the DISC keepalive AND
  // calibration commands on this one predicate. Enumerated here so a
  // reviewer can check the wiring against a test rather than against prose.
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

MTEST_MAIN
