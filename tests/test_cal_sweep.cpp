#include "mtest.h"
#include "cal_sweep.h"
#include "mabur/cal_proto.h"
#include "mabur/cal_wire.h"
#include "radio_tx.h"

#include <array>
#include <set>
#include <utility>
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
  int read_anchor_calls = 0;
  int anchor = 53;
  bool set_index_override(int idx) override {
    index_writes.push_back(idx);
    return true;
  }
  bool zero_rate_diffs() override { ++zero_diff_calls; return true; }
  int read_anchor_idx() override { ++read_anchor_calls; return anchor; }
};

struct FakeCapacityCtl : CalSweep::CapacityCtl {
  std::vector<std::pair<uint8_t, uint8_t>> points;  // (mcs, bw) per set_point call
  std::vector<std::vector<uint8_t>> blasts;         // captured frames per blast call
  BlastResult to_return;
  bool set_point(uint8_t mcs, uint8_t bw) override {
    points.emplace_back(mcs, bw);
    return true;
  }
  BlastResult blast(const uint8_t* frame, size_t len, uint32_t dwell_ms) override {
    (void)dwell_ms;
    blasts.emplace_back(frame, frame + len);
    return to_return;
  }
};

cal::SweepCmd power_cmd(uint32_t nonce, uint8_t seq, uint8_t mcs_mask = 0xFF, int8_t lo = -41,
                       int8_t hi = 63, uint8_t bw = 20, uint16_t frames_per_cell = 2,
                       uint16_t settle_ms = 0, uint16_t gap_us = 0) {
  cal::SweepCmd c;
  c.nonce = nonce;
  c.seq = seq;
  c.kind = cal::Kind::Power;
  c.bw = bw;
  c.frames_per_cell = frames_per_cell;
  c.settle_ms = settle_ms;
  c.gap_us = gap_us;
  c.idx_lo = lo;
  c.idx_hi = hi;
  c.mcs_mask = mcs_mask;
  return c;
}

cal::SweepCmd capacity_cmd(uint32_t nonce, uint8_t seq, uint8_t mcs_mask = 0xFF, uint8_t bw = 20,
                          uint32_t dwell_ms = 0, uint16_t settle_ms = 0) {
  cal::SweepCmd c;
  c.nonce = nonce;
  c.seq = seq;
  c.kind = cal::Kind::Capacity;
  c.bw = bw;
  c.mcs_mask = mcs_mask;
  c.dwell_ms = dwell_ms;
  c.settle_ms = settle_ms;
  return c;
}

// Runs pump() until the sweep leaves Sweeping or the budget is spent.
void run_to_quiescence(CalSweep& s, RadioTx& tx, FakePowerCtl& pwr, FakeCapacityCtl& cap,
                       uint64_t start_ms = 0, uint64_t step_ms = 1, int max_steps = 1000000) {
  uint64_t t = start_ms;
  for (int i = 0; i < max_steps && s.state() == CalSweep::State::Sweeping; ++i) {
    s.pump(t, tx, pwr, cap);
    t += step_ms;
  }
}

// Parses a captured frame's calibration payload. Frames are
// radiotap | dot11(26) | body, so the body is found by scanning for the
// magic rather than by hard-coding a header length the builder owns.
bool payload_of(const std::vector<uint8_t>& frame, cal::CalFrameInfo* out) {
  for (size_t i = 0; i + cal::kCalHeaderLen <= frame.size(); ++i)
    if (cal::parse_cal_payload(frame.data() + i, frame.size() - i, out)) return true;
  return false;
}

}  // namespace

TEST(power_sweep_walks_every_cell_and_stamps_kind_bw_flags) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0, /*mask*/ 0x03, /*lo*/ -2, /*hi*/ 1, /*bw*/ 40, /*fpc*/ 2);
  s.on_cmd(c, 0, pwr);
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::Accepted);
  CHECK(ack->flags == 3);  // kFlagStbc | kFlagLdpc (radio.ldpc defaults true)
  run_to_quiescence(s, tx, pwr, cap);
  CHECK(sink.frames.size() == 16);  // 4 idx x 2 mcs x 2 frames/cell
  std::set<std::pair<int, int>> seen;
  for (const auto& f : sink.frames) {
    cal::CalFrameInfo info;
    REQUIRE(payload_of(f, &info));
    CHECK(info.kind == cal::Kind::Power);
    CHECK(info.bw == 40);
    CHECK(info.flags == 3);
    CHECK(info.rate <= 1);
    CHECK(info.idx >= -2 && info.idx <= 1);
    seen.insert({info.rate, info.idx});
  }
  CHECK(seen.size() == 8);  // every (mcs, idx) cell visited exactly once
}

TEST(capacity_sweep_sets_each_point_and_reports_offered_and_blocked) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  cap.to_return = BlastResult{500, 90};
  CalSweep s(CalSweepCfg{});
  auto c = capacity_cmd(1, 0, /*mask*/ 0xFF, /*bw*/ 20, /*dwell*/ 0, /*settle*/ 0);
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap);
  REQUIRE(cap.points.size() == 8);
  for (int m = 0; m < 8; ++m) {
    CHECK(cap.points[m].first == m);
    CHECK(cap.points[m].second == 20);
  }
  auto done = s.take_done();
  REQUIRE(done.has_value());
  for (int m = 0; m < 8; ++m) {
    CHECK(done->offered[m] == 500);
    CHECK(done->blocked_pct[m] == 90);
  }
}

TEST(a_second_power_sweep_zeroes_again_but_reads_the_anchor_once) {
  FakePowerCtl pwr;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0);
  s.on_cmd(c, 0, pwr);
  CHECK(pwr.zero_diff_calls == 1);
  CHECK(pwr.read_anchor_calls == 1);
  c.seq = 1;  // next power sweep, same run
  s.on_cmd(c, 1000, pwr);
  CHECK(pwr.zero_diff_calls == 2);
  CHECK(pwr.read_anchor_calls == 1);  // not re-read
}

TEST(verify_and_capacity_do_not_zero) {
  FakePowerCtl pwr;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = capacity_cmd(1, 0, 0x01);
  s.on_cmd(c, 0, pwr);
  CHECK(pwr.zero_diff_calls == 0);

  std::array<int, 8> park{};
  park.fill(cal::kWallLeave);
  park[0] = 5;
  s.arm_verify(park, 20, 1000);
  CHECK(pwr.zero_diff_calls == 0);
  CHECK(s.state() == CalSweep::State::Sweeping);
}

TEST(seq_behind_is_ignored_equal_rearms_ack_ahead_starts) {
  FakePowerCtl pwr;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 5);
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());

  // Equal seq: re-arm the ack, no restart (no extra zero).
  s.on_cmd(c, 10, pwr);
  auto ack2 = s.take_ack();
  REQUIRE(ack2.has_value());
  CHECK(ack2->result == cal::AckResult::Accepted);
  CHECK(pwr.zero_diff_calls == 1);  // not re-entered

  // Behind seq: ignored outright, no ack.
  auto behind = c;
  behind.seq = 4;
  s.on_cmd(behind, 20, pwr);
  CHECK(!s.take_ack().has_value());

  // Ahead seq: a new sweep starts.
  auto ahead = c;
  ahead.seq = 6;
  ahead.mcs_mask = 0x02;
  s.on_cmd(ahead, 30, pwr);
  auto ack3 = s.take_ack();
  REQUIRE(ack3.has_value());
  CHECK(ack3->result == cal::AckResult::Accepted);
  CHECK(s.state() == CalSweep::State::Sweeping);
  CHECK(pwr.zero_diff_calls == 2);  // the new sweep zeroed again (Power kind)
}

TEST(done_is_armed_once_after_the_last_cell) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0, /*mask*/ 0x01, /*lo*/ 0, /*hi*/ 0, /*bw*/ 20, /*fpc*/ 3);
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  CHECK(!s.take_done().has_value());
  run_to_quiescence(s, tx, pwr, cap);
  auto done = s.take_done();
  REQUIRE(done.has_value());
  CHECK(done->nonce == 1);
  CHECK(done->seq == 0);
  CHECK(done->offered[0] == 3);
  CHECK(!s.take_done().has_value());  // drained once
}

TEST(abort_restores_power_immediately) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0, 0xFF, -41, 63, 20, /*fpc*/ 60000);  // would never finish on its own
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  s.pump(0, tx, pwr, cap);
  s.pump(1, tx, pwr, cap);
  CHECK(s.state() == CalSweep::State::Sweeping);

  cal::Abort a{1};
  s.on_abort(a, 2, pwr);
  CHECK(s.state() == CalSweep::State::Idle);
  CHECK(!s.active());
  CHECK(s.power_restored_for_test());
  CHECK(pwr.index_writes.back() == 53);
}

TEST(hard_cap_restores_power_with_the_gs_silent) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweepCfg cfg;
  cfg.hard_cap_ms = 500;
  CalSweep s(cfg);
  auto c = power_cmd(1, 0, 0xFF, -41, 63, 20, /*fpc*/ 60000);
  s.on_cmd(c, 0, pwr);
  CHECK(s.state() == CalSweep::State::Sweeping);
  s.pump(600, tx, pwr, cap);
  CHECK(s.state() == CalSweep::State::Idle);
  CHECK(!s.active());
  CHECK(s.power_restored_for_test());
}

TEST(unreadable_anchor_refuses_with_anchor_unreadable) {
  FakePowerCtl pwr;
  pwr.anchor = -1;
  CalSweep s(CalSweepCfg{});
  auto c = power_cmd(1, 0);
  s.on_cmd(c, 0, pwr);
  CHECK(!s.active());
  auto ack = s.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::AnchorUnreadable);
  CHECK(ack->nonce == 1);
  CHECK(ack->seq == 0);
  CHECK(s.take_refused());
  CHECK(!s.take_refused());               // drained: one refusal, one restore
  CHECK(pwr.zero_diff_calls == 1);         // flattened before the failed read
  CHECK(pwr.index_writes.empty());
}

TEST(arm_verify_skips_leave_entries) {
  CaptureSink sink;
  RadioTx tx(sink);
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  pwr.anchor = 53;
  CalSweep s(CalSweepCfg{});
  // Open a run first -- arm_verify is called after an apply within a live
  // run, never standing alone in production.
  auto c = power_cmd(1, 0, /*mask*/ 0x01, /*lo*/ 0, /*hi*/ 0, /*bw*/ 20, /*fpc*/ 1);
  s.on_cmd(c, 0, pwr);
  REQUIRE(s.take_ack().has_value());
  run_to_quiescence(s, tx, pwr, cap);
  sink.frames.clear();

  std::array<int, 8> park{};
  park.fill(cal::kWallLeave);
  park[0] = 10;
  park[3] = -5;
  s.arm_verify(park, 20, 1000);
  CHECK(s.state() == CalSweep::State::Sweeping);
  run_to_quiescence(s, tx, pwr, cap, 1000);

  CHECK(!sink.frames.empty());
  std::set<int> mcs_seen;
  for (const auto& f : sink.frames) {
    cal::CalFrameInfo info;
    REQUIRE(payload_of(f, &info));
    CHECK(info.kind == cal::Kind::Verify);
    CHECK((info.rate == 0 || info.rate == 3));
    if (info.rate == 0) CHECK(info.idx == 10);
    if (info.rate == 3) CHECK(info.idx == -5);
    mcs_seen.insert(info.rate);
  }
  CHECK(mcs_seen.size() == 2);        // only the two parked MCS, every other skipped
  CHECK(!s.take_done().has_value());  // Verify emits no SweepDone
}

MTEST_MAIN
