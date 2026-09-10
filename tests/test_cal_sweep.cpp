#include "mtest.h"
#include "cal_sweep.h"
#include "mabur/cal_wire.h"
#include "radio_tx.h"

#include <vector>

using namespace mabur;

namespace {

// Captures every built frame so the test can parse the payloads back out.
struct CaptureSink : FrameSink {
  std::vector<std::vector<uint8_t>> frames;
  bool send(const uint8_t* f, size_t n) override {
    frames.emplace_back(f, f + n);
    return true;
  }
};

struct FakePowerCtl : CalSweep::PowerCtl {
  std::vector<int> index_writes;
  int zero_diff_calls = 0;
  int base_ref = 53;
  bool set_index_override(int idx) override {
    index_writes.push_back(idx);
    return true;
  }
  bool zero_rate_diffs() override { ++zero_diff_calls; return true; }
  int read_base_ref_idx() override { return base_ref; }
};

// 2 rates x 4 indices x 5 frames.
rc::CalCmd small_cmd(uint8_t phase = cal::kPhaseCoarse, uint32_t nonce = 1) {
  rc::CalCmd c;
  c.vtx_id = 1;
  c.nonce = nonce;
  c.phase = phase;
  c.frames_per_cell = 5;
  c.settle_ms = 0;
  c.gap_us = 0;
  c.windows = {{0, 0, 12, 4}, {5, 40, 52, 4}};
  return c;
}

rc::CalCmd coarse_cmd() { return small_cmd(cal::kPhaseCoarse, 7); }

// Runs pump() until the sweep leaves Sweeping or the budget is spent.
void run_to_quiescence(CalSweep& s, RadioTx& tx, FakePowerCtl& pwr,
                       uint64_t start_ms = 0, uint64_t step_ms = 1,
                       int max_steps = 100000) {
  uint64_t t = start_ms;
  for (int i = 0; i < max_steps && s.state() == CalSweep::State::Sweeping;
       ++i) {
    s.pump(t, tx, pwr);
    t += step_ms;
  }
}

// Parses a captured frame's calibration payload. Frames are
// radiotap | dot11(26) | body, so the body is found by scanning for the
// magic rather than by hard-coding a header length the builder owns.
bool payload_of(const std::vector<uint8_t>& frame, cal::CalFrameInfo* out) {
  for (size_t i = 0; i + cal::kCalPayloadLen <= frame.size(); ++i)
    if (cal::parse_cal_payload(frame.data() + i, cal::kCalPayloadLen, out))
      return true;
  return false;
}

}  // namespace

TEST(walks_every_cell_in_the_plan) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(small_cmd(), 0, pwr);
  run_to_quiescence(s, tx, pwr);
  // 8 cells x 5 frames.
  CHECK(sink.frames.size() == 40);
  // One index write per cell, in plan order.
  REQUIRE(pwr.index_writes.size() == 8);
  CHECK(pwr.index_writes[0] == 0);
  CHECK(pwr.index_writes[3] == 12);
  CHECK(pwr.index_writes[4] == 40);
  CHECK(pwr.index_writes[7] == 52);
}

TEST(stamps_each_frame_with_its_rate_and_index) {
  // Attribution IS the measurement: a frame stamped with the wrong cell
  // corrupts that cell's delivery ratio and moves the wall.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(small_cmd(), 0, pwr);
  run_to_quiescence(s, tx, pwr);
  REQUIRE(sink.frames.size() == 40);
  int rate0 = 0, rate5 = 0;
  for (const auto& f : sink.frames) {
    cal::CalFrameInfo info;
    REQUIRE(payload_of(f, &info));
    CHECK(info.phase == cal::kPhaseCoarse);
    if (info.rate == 0) { ++rate0; CHECK(info.idx % 4 == 0); CHECK(info.idx <= 12); }
    else { ++rate5; CHECK(info.rate == 5); CHECK(info.idx >= 40 && info.idx <= 52); }
  }
  CHECK(rate0 == 20);
  CHECK(rate5 == 20);
}

TEST(zeroes_rate_diffs_before_sweeping) {
  // Per-rate walls must be measured against a common base, so the
  // wall-equalized diff table comes off before the first frame.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(small_cmd(), 0, pwr);
  s.pump(0, tx, pwr);
  CHECK(pwr.zero_diff_calls == 1);
}

TEST(ignores_a_repeat_of_the_running_phase) {
  // The uplink loses 30-50% of frames, so the GS repeats. A repeat must not
  // restart the phase or re-zero the cell cursor.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  const auto c = small_cmd();
  s.on_cmd(c, 0, pwr);
  s.pump(0, tx, pwr);
  s.pump(1, tx, pwr);
  const size_t after_two_pumps = sink.frames.size();
  s.on_cmd(c, 2, pwr);                       // exact repeat: same nonce+phase
  CHECK(pwr.zero_diff_calls == 1);      // not re-entered
  s.pump(3, tx, pwr);
  CHECK(sink.frames.size() > after_two_pumps);  // still advancing, not reset
}

TEST(accepts_the_next_phase_after_the_current_one_finishes) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(small_cmd(cal::kPhaseCoarse, 7), 0, pwr);
  run_to_quiescence(s, tx, pwr);
  const size_t after_coarse = sink.frames.size();
  s.on_cmd(small_cmd(cal::kPhaseFine, 7), 1000, pwr);
  CHECK(s.state() == CalSweep::State::Sweeping);
  run_to_quiescence(s, tx, pwr, 1000);
  CHECK(sink.frames.size() > after_coarse);
  cal::CalFrameInfo info;
  REQUIRE(payload_of(sink.frames.back(), &info));
  CHECK(info.phase == cal::kPhaseFine);
}

TEST(stale_earlier_phase_duplicate_is_ignored_mid_fine_sweep) {
  // Constraint 3 must reject a phase at or BEHIND the one already
  // accepted, not just an exact repeat of the current one: a delayed
  // coarse retransmission can arrive after fine has already started.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(small_cmd(cal::kPhaseCoarse, 7), 0, pwr);
  run_to_quiescence(s, tx, pwr);
  const size_t after_coarse = sink.frames.size();
  s.on_cmd(small_cmd(cal::kPhaseFine, 7), 1000, pwr);
  // Advance partway into the fine sweep -- not finished.
  s.pump(1000, tx, pwr);
  s.pump(1001, tx, pwr);
  s.pump(1002, tx, pwr);
  const size_t mid_fine = sink.frames.size();
  const size_t fine_so_far = mid_fine - after_coarse;
  REQUIRE(fine_so_far > 0);
  REQUIRE(fine_so_far < 40);

  // The stale coarse retransmission finally lands, late.
  s.on_cmd(small_cmd(cal::kPhaseCoarse, 7), 1003, pwr);
  CHECK(s.state() == CalSweep::State::Sweeping);  // still the fine plan
  CHECK(sink.frames.size() == mid_fine);          // cursor did not rewind

  // Two more pumps: enough to re-enter a cell and send its first frame if
  // the (buggy) duplicate had rewound the cursor to coarse's window 0.
  s.pump(1004, tx, pwr);
  s.pump(1005, tx, pwr);
  CHECK(sink.frames.size() == mid_fine + 2);
  cal::CalFrameInfo info;
  REQUIRE(payload_of(sink.frames.back(), &info));
  CHECK(info.phase == cal::kPhaseFine);           // not reset to coarse
}

TEST(stale_earlier_phase_duplicate_does_not_discard_an_undrained_result) {
  // The more serious half of the same bug: a stale duplicate arriving
  // after a result has landed (but before it's drained) must not reset
  // state_ to Sweeping and silently throw the measured wall table away.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(small_cmd(cal::kPhaseCoarse, 7), 0, pwr);
  run_to_quiescence(s, tx, pwr);
  s.on_cmd(small_cmd(cal::kPhaseFine, 7), 1000, pwr);
  run_to_quiescence(s, tx, pwr, 1000);
  const size_t after_fine = sink.frames.size();

  rc::CalResult r;
  r.vtx_id = 1;
  r.nonce = 7;
  r.walls = {88, 88, 88, 95, 73, 54, 51, 49};
  r.legacy_wall = 88;
  s.on_result(r, 2000);
  CHECK(s.state() == CalSweep::State::Applying);

  // A stale coarse retransmission arrives after the result is in.
  s.on_cmd(small_cmd(cal::kPhaseCoarse, 7), 2100, pwr);
  CHECK(s.state() == CalSweep::State::Applying);  // not reset to Sweeping
  CHECK(sink.frames.size() == after_fine);        // no cells re-walked

  const auto pending = s.take_pending_result();
  REQUIRE(pending.has_value());                   // result survived
  CHECK(pending->walls[5] == 54);
}

TEST(hard_cap_returns_to_idle_even_with_the_gs_silent) {
  // The open-loop guarantee: no command, no result, no link -- the drone
  // still restores itself. Losing the link mid-sweep is the EXPECTED case.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweepCfg cfg;
  cfg.hard_cap_ms = 500;
  CalSweep s(cfg);
  rc::CalCmd big = small_cmd();
  big.frames_per_cell = 60000;   // would never finish on its own
  s.on_cmd(big, 0, pwr);
  CHECK(s.state() == CalSweep::State::Sweeping);
  s.pump(600, tx, pwr);
  CHECK(s.state() == CalSweep::State::Idle);
  CHECK(!s.active());
}

TEST(await_next_timeout_returns_to_idle_without_applying) {
  // Phase 2 never arrives: end the session, write nothing.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweepCfg cfg;
  cfg.await_next_ms = 1000;
  CalSweep s(cfg);
  s.on_cmd(small_cmd(), 0, pwr);
  run_to_quiescence(s, tx, pwr);
  s.pump(5000, tx, pwr);
  CHECK(s.state() == CalSweep::State::Idle);
  CHECK(!s.take_pending_result().has_value());
  // This exit path is reached via await_deadline_ms_, not hard_cap_ms
  // (still 180000 ms away) -- restores_power_state_on_every_exit_path only
  // exercises the hard-cap branch, so this is the only test that pins
  // "restore power" for the await-timeout path specifically.
  CHECK(s.power_restored_for_test());
}

TEST(result_moves_to_applying_then_verify) {
  // Applying is what arms the verify pass -- no further command needed.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(small_cmd(), 0, pwr);
  run_to_quiescence(s, tx, pwr);
  rc::CalResult r;
  r.vtx_id = 1;
  r.nonce = 1;
  r.walls = {88, 88, 88, 95, 73, 54, 51, 49};
  r.legacy_wall = 88;
  s.on_result(r, 100);
  CHECK(s.state() == CalSweep::State::Applying);
  const auto pending = s.take_pending_result();
  REQUIRE(pending.has_value());
  CHECK(pending->walls[5] == 54);
}

TEST(stale_nonce_result_is_ignored) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(small_cmd(cal::kPhaseCoarse, 7), 0, pwr);
  run_to_quiescence(s, tx, pwr);
  rc::CalResult r;
  r.vtx_id = 1;
  r.nonce = 999;   // a different session
  s.on_result(r, 100);
  CHECK(s.state() != CalSweep::State::Applying);
  CHECK(!s.take_pending_result().has_value());
}

TEST(restores_power_state_on_every_exit_path) {
  // Timeout and completion alike must leave the radio as they found it, or a
  // failed calibration silently changes the drone's operating power.
  for (int path = 0; path < 2; ++path) {
    CaptureSink sink;
    RadioTx tx(sink);
    FakePowerCtl pwr;
    CalSweepCfg cfg;
    cfg.hard_cap_ms = 500;
    CalSweep s(cfg);
    rc::CalCmd c = small_cmd();
    if (path == 1) c.frames_per_cell = 60000;   // force the timeout path
    s.on_cmd(c, 0, pwr);
    run_to_quiescence(s, tx, pwr);
    s.pump(600, tx, pwr);
    CHECK(s.state() == CalSweep::State::Idle);
    CHECK(s.power_restored_for_test());
  }
}

TEST(duplicate_result_for_accepted_nonce_is_ignored) {
  // NOT the same bug as stale_nonce_result_is_ignored (a DIFFERENT nonce):
  // this is a repeat of the nonce the sweeper already accepted. The uplink
  // can duplicate a result frame same as it duplicates commands, and
  // letting it re-arm Applying would mean cal_apply's flash write runs a
  // second time -- Task 9 exists partly to hold that line at one write
  // per session.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(small_cmd(), 0, pwr);
  run_to_quiescence(s, tx, pwr);
  rc::CalResult r;
  r.vtx_id = 1;
  r.nonce = 1;
  r.walls = {88, 88, 88, 95, 73, 54, 51, 49};
  r.legacy_wall = 88;
  s.on_result(r, 100);
  REQUIRE(s.take_pending_result().has_value());
  s.on_result(r, 150);   // duplicate of the already-accepted result
  CHECK(s.state() != CalSweep::State::Applying);
  CHECK(!s.take_pending_result().has_value());
}

TEST(zero_frames_per_cell_sends_nothing) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  rc::CalCmd c = small_cmd();
  c.frames_per_cell = 0;
  s.on_cmd(c, 0, pwr);
  run_to_quiescence(s, tx, pwr);
  CHECK(sink.frames.size() == 0);
}

TEST(base_ref_readback_feeds_the_ack) {
  // The GS range-checks a candidate table against THIS unit's base_ref_idx
  // (power_plan.h derives every diff from it), so the readback must reach
  // the acknowledgment or the whole table is unusable on another VTX.
  FakePowerCtl pwr;
  pwr.base_ref = 53;
  CalSweep s(CalSweepCfg{});
  rc::CalCmd c = coarse_cmd();
  s.on_cmd(c, 0, pwr);
  CHECK(s.take_ack_base_ref() == 53);
}

TEST(ack_base_ref_is_not_re_read_through_a_parked_override) {
  // A second phase's on_cmd() lands with the FIRST phase's TXAGC override
  // still parked (pump_sweeping never restores between phases -- see
  // cal_sweep.h). If the ack re-read base_ref_idx at THIS point instead of
  // reusing the one value latched at session start, it would report the
  // override, not the anchor -- devourer/src/TxPower.h: GetTxPowerState's
  // representative indices report chip truth "for the current moment,"
  // flat during an override.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  pwr.base_ref = 53;
  CalSweep s(CalSweepCfg{});
  s.on_cmd(small_cmd(cal::kPhaseCoarse, 7), 0, pwr);
  CHECK(s.take_ack_base_ref() == 53);
  run_to_quiescence(s, tx, pwr);
  // Coarse finished with the TXAGC parked at its last swept cell; simulate
  // that drift being visible to a readback taken now.
  pwr.base_ref = 12;
  s.on_cmd(small_cmd(cal::kPhaseFine, 7), 1000, pwr);
  CHECK(s.take_ack_base_ref() == 53);  // still the session's original value
}

TEST(a_rejected_repeat_arms_no_ack) {
  // Constraint 3's early return (a stale/behind-phase repeat) must not
  // arm a second ack for a phase that was already acknowledged.
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  CalSweep s(CalSweepCfg{});
  const auto c = small_cmd();
  s.on_cmd(c, 0, pwr);
  CHECK(s.take_ack_base_ref().has_value());
  s.on_cmd(c, 2, pwr);  // exact repeat: same nonce+phase
  CHECK(!s.take_ack_base_ref().has_value());
}

MTEST_MAIN
