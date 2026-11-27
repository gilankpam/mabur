#include "mtest.h"
#include "cal_sweep.h"
#include "mabur/cal_proto.h"
#include "mabur/cal_wire.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

using namespace mabur;

namespace {

struct FakePowerCtl : CalSweep::PowerCtl {
  std::vector<int> index_writes;
  int zero_diff_calls = 0;
  int read_anchor_calls = 0;
  int anchor = 53;
  bool set_index_override(int idx) override {
    index_writes.push_back(idx);
    return true;
  }
  bool zero_ok = true;
  bool zero_rate_diffs() override { ++zero_diff_calls; return zero_ok; }
  int read_anchor_idx() override { ++read_anchor_calls; return anchor; }
  bool untouched() const {
    return index_writes.empty() && zero_diff_calls == 0 && read_anchor_calls == 0;
  }
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

// Logs every call; step() returns false on its `finish_after`-th call.
struct FakeWall : CalSweep::WallCtl {
  std::vector<std::string> calls;
  cal::AckResult check_result = cal::AckResult::Accepted;
  int finish_after = 2;
  int steps = 0;
  uint8_t ch = 0, width = 0;
  cal::AckResult check(uint8_t channel, uint8_t bw) override {
    calls.push_back("check " + std::to_string(channel) + " " + std::to_string(bw));
    return check_result;
  }
  void start(uint8_t channel, uint8_t bw, uint64_t) override {
    calls.push_back("start " + std::to_string(channel) + " " + std::to_string(bw));
    ch = channel;
    width = bw;
    steps = 0;
  }
  bool step(uint64_t) override {
    calls.push_back("step");
    return ++steps < finish_after;
  }
  cal::SweepDone finish() override {
    calls.push_back("finish");
    cal::SweepDone d;
    d.wall_result = cal::WallResult::Ok;
    d.channel = ch;
    d.bw = width;
    d.walls = {63, 63, 63, 63, 44, 26, 24, 20};
    d.floor_evm_x10 = -306;
    d.anchor = 39;
    return d;
  }
  void cancel() override { calls.push_back("cancel"); }
  int count(const std::string& verb) const {
    return static_cast<int>(std::count_if(calls.begin(), calls.end(), [&](const std::string& c) {
      return c.compare(0, verb.size(), verb) == 0;
    }));
  }
};

cal::SweepCmd cap_cmd(uint32_t nonce, uint8_t seq, uint8_t bw = 20, uint8_t mcs_mask = 0xFF,
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

cal::SweepCmd wall_cmd(uint32_t nonce, uint8_t seq, uint8_t channel, uint8_t bw) {
  cal::SweepCmd c;
  c.nonce = nonce;
  c.seq = seq;
  c.kind = cal::Kind::Wall;
  c.channel = channel;
  c.bw = bw;
  return c;
}

// The engine and its three fakes, with the calls the tests make spelled
// short.
struct Rig {
  FakePowerCtl pwr;
  FakeCapacityCtl cap;
  FakeWall wall;
  CalSweep sw;
  explicit Rig(CalSweepCfg cfg = CalSweepCfg{}) : sw(cfg) {}
  void cmd(const cal::SweepCmd& c, uint64_t t) { sw.on_cmd(c, t, pwr, wall); }
  void abort(uint32_t nonce, uint64_t t) { sw.on_abort(cal::Abort{nonce}, t, pwr, wall); }
  void pump(uint64_t t) { sw.pump(t, pwr, cap, wall); }
  // Pumps until the sweep leaves Sweeping or the budget is spent; returns
  // the ms of the last pump.
  uint64_t run(uint64_t from, int max_steps = 100000) {
    uint64_t t = from;
    for (int i = 0; i < max_steps && sw.state() == CalSweep::State::Sweeping; ++i) pump(t++);
    return t - 1;
  }
};

// A one-point capacity sweep accepted at t = 0 and pumped until it leaves
// Sweeping; returns the ms it finished at (its SweepDone armed).
uint64_t finish_one_point(Rig& r, uint32_t nonce, uint8_t seq) {
  r.cmd(cap_cmd(nonce, seq, 20, 0x01), 0);
  REQUIRE(r.sw.take_ack().has_value());
  const uint64_t end = r.run(0);
  REQUIRE(r.sw.state() == CalSweep::State::Idle);
  return end;
}

}  // namespace

TEST(capacity_sweep_sets_each_point_and_reports_offered_and_blocked) {
  Rig r;
  r.cap.to_return = BlastResult{500, 90};
  r.cmd(cap_cmd(1, 0), 0);
  const auto ack = r.sw.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::Accepted);
  CHECK(ack->flags == 3);  // kFlagStbc | kFlagLdpc (radio.ldpc defaults true)
  r.run(0);
  REQUIRE(r.cap.points.size() == 8);
  for (int m = 0; m < 8; ++m) {
    CHECK(r.cap.points[m].first == m);
    CHECK(r.cap.points[m].second == 20);
  }
  cal::CalFrameInfo info;
  REQUIRE(!r.cap.blasts.empty());
  CHECK(cal::parse_cal_payload(r.cap.blasts[0].data(), r.cap.blasts[0].size(), &info));
  CHECK(info.kind == cal::Kind::Capacity && info.len == CalSweepCfg{}.capacity_frame_len);
  const auto done = r.sw.take_done();
  REQUIRE(done.has_value());
  CHECK(done->nonce == 1 && done->seq == 0);
  for (int m = 0; m < 8; ++m) {
    CHECK(done->offered[m] == 500);
    CHECK(done->blocked_pct[m] == 90);
  }
  CHECK(!r.sw.take_done().has_value());  // drained once
}

// A run's anchor is read with the diffs zeroed whatever kind opens it (a
// custom table live would read reference + diff); a later sweep of the run
// neither zeroes nor reads again.
TEST(a_new_run_zeroes_before_its_one_anchor_read_and_later_sweeps_do_not) {
  Rig r;
  r.cmd(cap_cmd(1, 0, 20, 0x01), 0);
  CHECK(r.pwr.zero_diff_calls == 1);
  CHECK(r.pwr.read_anchor_calls == 1);
  CHECK(r.sw.anchor_idx() == 53);
  r.run(0);
  r.cmd(cap_cmd(1, 1, 40, 0x01), 2000);
  r.cmd(wall_cmd(1, 2, 112, 20), 3000);
  CHECK(r.sw.take_ack()->result == cal::AckResult::Accepted);
  CHECK(r.pwr.zero_diff_calls == 1);
  CHECK(r.pwr.read_anchor_calls == 1);
}

TEST(a_wall_run_zeroes_and_reads_the_anchor_like_any_new_run) {
  Rig r;
  r.cmd(wall_cmd(1, 0, 112, 20), 0);
  CHECK(r.sw.take_ack()->result == cal::AckResult::Accepted);
  CHECK(r.pwr.zero_diff_calls == 1 && r.pwr.read_anchor_calls == 1);
  CHECK(r.sw.active() && r.sw.state() == CalSweep::State::Sweeping);
}

// The table is flattened before every new run's anchor read, so a refusal
// owes the caller a restore.
TEST(unreadable_anchor_refuses_with_anchor_unreadable) {
  Rig r;
  r.pwr.anchor = -1;
  r.cmd(cap_cmd(1, 0), 0);
  CHECK(!r.sw.active());
  const auto ack = r.sw.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::AnchorUnreadable);
  CHECK(ack->nonce == 1 && ack->seq == 0);
  CHECK(r.sw.take_refused());
  CHECK(!r.sw.take_refused());          // drained: one refusal, one restore
  CHECK(r.pwr.zero_diff_calls == 1);    // flattened before the failed read
  CHECK(r.pwr.index_writes.empty());
}

// A table that would not zero leaves reference + diff where the anchor
// read expects the reference. Refused like an unreadable anchor, and the
// owner restores.
TEST(a_table_that_would_not_zero_refuses_a_new_run) {
  Rig r;
  r.pwr.zero_ok = false;
  r.cmd(cap_cmd(1, 0), 0);
  CHECK(!r.sw.active());
  const auto ack = r.sw.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::AnchorUnreadable);
  CHECK(r.pwr.read_anchor_calls == 0);  // a read off a live table is never taken
  CHECK(r.sw.take_refused());
  CHECK(r.pwr.index_writes.empty());
}

TEST(seq_behind_is_ignored_equal_rearms_ack_ahead_starts) {
  Rig r;
  const auto c = cap_cmd(1, 5);
  r.cmd(c, 0);
  REQUIRE(r.sw.take_ack().has_value());

  // Equal seq: re-arm the ack, no restart.
  r.cmd(c, 10);
  const auto ack2 = r.sw.take_ack();
  REQUIRE(ack2.has_value());
  CHECK(ack2->result == cal::AckResult::Accepted);
  CHECK(ack2->start_in_ms == 190);  // what is left of the original delay
  CHECK(r.pwr.zero_diff_calls == 1);

  // Behind seq: ignored outright, no ack.
  r.cmd(cap_cmd(1, 4), 20);
  CHECK(!r.sw.take_ack().has_value());

  // Ahead seq: a new sweep starts.
  r.cmd(cap_cmd(1, 6, 20, 0x02), 30);
  const auto ack3 = r.sw.take_ack();
  REQUIRE(ack3.has_value());
  CHECK(ack3->result == cal::AckResult::Accepted);
  CHECK(r.sw.state() == CalSweep::State::Sweeping);
  r.run(30);
  REQUIRE(r.cap.points.size() == 1);
  CHECK(r.cap.points[0].first == 1);  // the new sweep's mask, not the old one's
}

TEST(done_repeats_from_100_ms_then_every_200_ms_until_the_next_command) {
  // SweepDone rides a lossy downlink, and a capacity sweep without it writes
  // nothing: it is re-armed 100 ms after the sweep ends -- half a period
  // off the GS's 200 ms resend grid -- then every 200 ms until the GS shows
  // it moved on.
  Rig r;
  const uint64_t end = finish_one_point(r, 7, 0);
  const auto first = r.sw.take_done();
  REQUIRE(first.has_value());
  for (uint64_t t = end + 1; t < end + 100; ++t) {
    r.pump(t);
    CHECK(!r.sw.take_done().has_value());
  }
  r.pump(end + 100);
  const auto again = r.sw.take_done();
  REQUIRE(again.has_value());
  CHECK(again->nonce == 7 && again->seq == 0 && again->offered == first->offered);
  r.pump(end + 299);
  CHECK(!r.sw.take_done().has_value());
  r.pump(end + 300);
  CHECK(r.sw.take_done().has_value());
  r.pump(end + 499);
  CHECK(!r.sw.take_done().has_value());
  r.pump(end + 500);
  CHECK(r.sw.take_done().has_value());

  // The next command of the run -- here the APPLY -- stops it.
  cal::ApplyCmd a;
  a.nonce = 7;
  a.seq = 0;
  r.sw.on_apply(a, end + 550);
  REQUIRE(r.sw.take_pending_apply().has_value());
  for (uint64_t t = end + 550; t < end + 2000; t += 10) {
    r.pump(t);
    CHECK(!r.sw.take_done().has_value());
  }
}

TEST(done_repeats_stop_at_a_sweep_an_abort_or_the_close) {
  auto silent_after = [](Rig& r, uint64_t from) {
    bool quiet = true;
    for (uint64_t t = from; t < from + 1000; t += 10) {
      r.pump(t);
      if (r.sw.take_done()) quiet = false;
    }
    return quiet;
  };
  {
    // Any other SWEEP of the run, even one the drone refuses.
    Rig r;
    const uint64_t end = finish_one_point(r, 8, 0);
    REQUIRE(r.sw.take_done().has_value());
    r.wall.check_result = cal::AckResult::Unsupported;
    r.cmd(wall_cmd(8, 1, 112, 20), end + 10);
    (void)r.sw.take_ack();
    CHECK(silent_after(r, end + 10));
  }
  {
    // An ABORT of the run.
    Rig r;
    const uint64_t end = finish_one_point(r, 9, 0);
    REQUIRE(r.sw.take_done().has_value());
    r.abort(9, end + 10);
    CHECK(silent_after(r, end + 10));
  }
  {
    // The await timer closing the run.
    CalSweepCfg cfg;
    cfg.await_next_ms = 1000;
    Rig r(cfg);
    const uint64_t end = finish_one_point(r, 10, 0);
    REQUIRE(r.sw.take_done().has_value());
    int repeats = 0;
    for (uint64_t t = end + 1; t < end + 3000; ++t) {
      r.pump(t);
      if (r.sw.take_done()) ++repeats;
    }
    CHECK(!r.sw.active());
    CHECK(repeats == 5);  // +100, +300 .. +900; the run closed at +1000
  }
}

TEST(abort_restores_power_immediately) {
  Rig r;
  r.cmd(cap_cmd(1, 0), 0);
  REQUIRE(r.sw.take_ack().has_value());
  r.pump(0);
  CHECK(r.sw.state() == CalSweep::State::Sweeping);
  r.abort(1, 2);
  CHECK(r.sw.state() == CalSweep::State::Idle);
  CHECK(!r.sw.active());
  CHECK(r.sw.power_restored_for_test());
  CHECK(r.pwr.index_writes.back() == 53);
}

TEST(hard_cap_restores_power_with_the_gs_silent) {
  CalSweepCfg cfg;
  cfg.hard_cap_ms = 500;
  Rig r(cfg);
  r.cmd(cap_cmd(1, 0), 0);
  CHECK(r.sw.state() == CalSweep::State::Sweeping);
  r.pump(600);
  CHECK(r.sw.state() == CalSweep::State::Idle);
  CHECK(!r.sw.active());
  CHECK(r.sw.power_restored_for_test());
}

TEST(start_delay_blocks_set_point_before_t200) {
  Rig r;
  r.cmd(cap_cmd(1, 0, 20, 0x01), 0);
  REQUIRE(r.sw.take_ack().has_value());
  for (uint64_t t = 0; t < 200; t += 20) r.pump(t);
  CHECK(r.cap.points.empty());
  r.run(200);
  CHECK(!r.cap.points.empty());
}

TEST(a_closed_nonce_does_not_reopen) {
  Rig r;
  const auto c = cap_cmd(1, 0);
  r.cmd(c, 0);
  REQUIRE(r.sw.take_ack().has_value());
  r.abort(1, 100);
  REQUIRE(r.sw.power_restored_for_test());
  const size_t writes_before = r.pwr.index_writes.size();

  // A straggler/resend of the aborted nonce+seq arrives after the close.
  r.cmd(c, 101);
  CHECK(!r.sw.active());
  CHECK(!r.sw.take_ack().has_value());
  CHECK(r.pwr.index_writes.size() == writes_before);  // power untouched by the attempt
  r.pump(301);
  r.pump(302);
  CHECK(r.cap.points.empty());  // nothing airs
  CHECK(r.sw.power_restored_for_test());  // still restored -- no new run reopened
}

TEST(capacity_sweep_puts_the_override_at_the_anchor_on_accept) {
  Rig r;
  r.cmd(cap_cmd(1, 0, 20, 0x01), 0);
  REQUIRE(r.sw.take_ack().has_value());
  CHECK(r.pwr.index_writes.back() == 53);
  r.run(0);
  REQUIRE(!r.cap.points.empty());
  CHECK(r.pwr.index_writes.back() == 53);
}

TEST(a_repeated_apply_already_handed_out_is_not_queued_again) {
  Rig r;
  const uint64_t end = finish_one_point(r, 1, 0);
  cal::ApplyCmd a;
  a.nonce = 1;
  a.bw = 20;
  r.sw.on_apply(a, end + 10);
  CHECK(r.sw.state() == CalSweep::State::Applying);
  REQUIRE(r.sw.take_pending_apply().has_value());
  CHECK(r.sw.state() == CalSweep::State::Idle);

  // The GS resends the same APPLY until it hears APPLIED.
  r.sw.on_apply(a, end + 210);
  CHECK(r.sw.state() == CalSweep::State::Idle);  // did not re-enter Applying
  CHECK(!r.sw.take_pending_apply().has_value());
}

TEST(a_new_seq_at_the_same_width_is_a_new_apply) {
  // Two sweeps applying at 20 MHz in one run: the handed-out latch keys on
  // the sweep's seq, not on the width.
  Rig r;
  finish_one_point(r, 1, 0);
  cal::ApplyCmd first;
  first.nonce = 1;
  first.seq = 0;
  first.bw = 20;
  r.sw.on_apply(first, 2000);
  REQUIRE(r.sw.take_pending_apply().has_value());

  r.cmd(cap_cmd(1, 1, 20, 0x01), 3000);
  REQUIRE(r.sw.take_ack().has_value());
  r.run(3000);
  REQUIRE(r.sw.state() == CalSweep::State::Idle);

  cal::ApplyCmd second = first;
  second.seq = 1;
  r.sw.on_apply(second, 6000);
  CHECK(r.sw.state() == CalSweep::State::Applying);
  const auto got = r.sw.take_pending_apply();
  REQUIRE(got.has_value());
  CHECK(got->seq == 1);

  // Its own repeat is then ignored, as the first apply's was.
  r.sw.on_apply(second, 6200);
  CHECK(!r.sw.take_pending_apply().has_value());
}

TEST(await_timeout_while_applying_closes_the_run) {
  CalSweepCfg cfg;
  cfg.await_next_ms = 100;
  Rig r(cfg);
  const uint64_t end = finish_one_point(r, 1, 0);
  cal::ApplyCmd a;
  a.nonce = 1;
  a.bw = 20;
  r.sw.on_apply(a, end + 10);
  CHECK(r.sw.state() == CalSweep::State::Applying);
  r.pump(end + 200);  // past the await deadline (end + 10 + 100)
  CHECK(r.sw.state() == CalSweep::State::Idle);
  CHECK(!r.sw.active());
  CHECK(r.sw.power_restored_for_test());
}

TEST(on_apply_is_ignored_while_a_sweep_is_sweeping) {
  Rig r;
  r.cmd(cap_cmd(1, 0), 0);
  REQUIRE(r.sw.take_ack().has_value());
  CHECK(r.sw.state() == CalSweep::State::Sweeping);
  cal::ApplyCmd a;
  a.nonce = 1;
  a.bw = 20;
  r.sw.on_apply(a, 100);
  CHECK(r.sw.state() == CalSweep::State::Sweeping);  // undisturbed
  CHECK(!r.sw.take_pending_apply().has_value());     // not queued
}

TEST(an_ahead_seq_sweep_clears_a_stale_pending_apply) {
  Rig r;
  finish_one_point(r, 1, 0);
  cal::ApplyCmd a;
  a.nonce = 1;
  a.bw = 20;
  r.sw.on_apply(a, 1000);
  CHECK(r.sw.state() == CalSweep::State::Applying);
  r.cmd(cap_cmd(1, 1, 20, 0x01), 1100);  // ahead seq
  CHECK(r.sw.state() == CalSweep::State::Sweeping);
  CHECK(!r.sw.take_pending_apply().has_value());  // cleared, not handed out
}

TEST(power_restored_flag_resets_when_a_new_run_opens) {
  Rig r;
  r.cmd(cap_cmd(1, 0), 0);
  REQUIRE(r.sw.take_ack().has_value());
  r.abort(1, 300);
  CHECK(r.sw.power_restored_for_test());
  r.cmd(cap_cmd(2, 0), 301);  // a different, new nonce
  CHECK(!r.sw.power_restored_for_test());
}

// starts_sweep() predicts on_cmd(): a new nonce (unless it is the one just
// closed) or a seq ahead of the running one starts a sweep; a repeat or a
// late duplicate does not. run_nonce() names the open run.
TEST(starts_sweep_predicts_on_cmd_and_run_nonce_names_the_run) {
  Rig r;
  CHECK(r.sw.starts_sweep(cap_cmd(5, 0)));
  r.cmd(cap_cmd(5, 1), 1000);
  REQUIRE(r.sw.active());
  CHECK(r.sw.run_nonce() == 5);
  CHECK(!r.sw.starts_sweep(cap_cmd(5, 1)));  // repeat
  CHECK(!r.sw.starts_sweep(cap_cmd(5, 0)));  // late duplicate
  CHECK(r.sw.starts_sweep(wall_cmd(5, 2, 112, 20)));
  CHECK(r.sw.starts_sweep(cap_cmd(6, 0)));   // another run
  r.abort(5, 1100);
  REQUIRE(!r.sw.active());
  CHECK(!r.sw.starts_sweep(cap_cmd(5, 3)));  // the closed nonce never reopens
  CHECK(r.sw.starts_sweep(cap_cmd(6, 0)));
}

// ---- Wall sweeps ----------------------------------------------------------

// Revert (pump skipping the 200 ms start delay for Wall): "start" lands at
// t = 1100. Revert (finish's report sent without nonce/seq filled in): the
// done reads nonce 0.
TEST(a_wall_sweep_runs_start_steps_finish) {
  Rig r;
  r.cmd(wall_cmd(/*nonce*/ 7, /*seq*/ 0, /*ch*/ 112, /*bw*/ 20), 1000);
  CHECK(r.sw.take_ack()->result == cal::AckResult::Accepted);
  r.pump(1100);
  CHECK(r.wall.calls == std::vector<std::string>{"check 112 20"});  // start delay
  r.pump(1200);
  CHECK(r.wall.calls == (std::vector<std::string>{"check 112 20", "start 112 20"}));
  CHECK(!r.sw.take_done().has_value());
  r.pump(1201);
  r.pump(1202);  // FakeWall: the second step returns false
  CHECK(r.wall.calls == (std::vector<std::string>{"check 112 20", "start 112 20", "step", "step",
                                                   "finish"}));
  const auto d = r.sw.take_done();
  REQUIRE(d.has_value());
  CHECK(d->nonce == 7 && d->seq == 0 && d->channel == 112 && d->bw == 20);
  CHECK(d->walls[7] == 20 && d->floor_evm_x10 == -306 && d->anchor == 39);
  CHECK(d->wall_result == cal::WallResult::Ok);
  CHECK(r.sw.state() == CalSweep::State::Idle && r.sw.active());
  CHECK(r.cap.points.empty());  // a Wall sweep never drives the capacity actuator
}

// Revert (the refusal opening a run, or zeroing/reading before check()):
// active() or the PowerCtl counters fail. Revert (a refusal arming
// take_refused): the restore flag fails.
TEST(a_refused_wall_sweep_opens_nothing) {
  Rig r;
  r.wall.check_result = cal::AckResult::BadChannel;
  r.cmd(wall_cmd(7, 0, 99, 20), 1000);
  const auto ack = r.sw.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::BadChannel);
  CHECK(ack->nonce == 7 && ack->seq == 0);
  CHECK(!r.sw.active() && !r.sw.take_refused() && r.pwr.untouched());
  r.cmd(cap_cmd(7, 1, 20), 1200);  // the same run goes on to capacity
  CHECK(r.sw.take_ack()->result == cal::AckResult::Accepted && r.sw.active());
  r.run(1200);
  CHECK(r.wall.count("start") == 0 && !r.cap.points.empty());
}

// Revert (a refusal superseding the running sweep): the capacity sweep never
// finishes or its done carries seq 1.
TEST(a_refused_wall_sweep_leaves_the_open_run_and_its_sweep_alone) {
  Rig r;
  r.cmd(cap_cmd(3, 0, 20, 0x01), 0);
  REQUIRE(r.sw.take_ack()->result == cal::AckResult::Accepted);
  const size_t writes = r.pwr.index_writes.size();
  r.wall.check_result = cal::AckResult::Unsupported;
  r.cmd(wall_cmd(3, 1, 112, 20), 50);
  CHECK(r.sw.take_ack()->result == cal::AckResult::Unsupported);
  CHECK(r.sw.active() && r.sw.state() == CalSweep::State::Sweeping);
  CHECK(r.pwr.index_writes.size() == writes && r.pwr.zero_diff_calls == 1);
  CHECK(!r.sw.take_refused());
  r.run(50);
  const auto d = r.sw.take_done();
  REQUIRE(d.has_value());
  CHECK(d->seq == 0 && r.cap.points.size() == 1);
}

// The GS repeats its SWEEP every 200 ms. Revert (a repeat re-running check
// or start): the counts read 2.
TEST(a_repeat_during_the_measurement_only_re_acks) {
  Rig r;
  r.wall.finish_after = 100;
  const auto c = wall_cmd(7, 0, 112, 20);
  r.cmd(c, 1000);
  REQUIRE(r.sw.take_ack()->result == cal::AckResult::Accepted);
  r.pump(1200);  // start
  r.pump(1201);
  r.pump(1202);  // two steps
  r.cmd(c, 1203);
  const auto ack = r.sw.take_ack();
  REQUIRE(ack.has_value());
  CHECK(ack->result == cal::AckResult::Accepted && ack->start_in_ms == 0);
  CHECK(r.wall.count("check") == 1 && r.wall.count("start") == 1);
  CHECK(r.wall.count("cancel") == 0);
  r.pump(1204);
  CHECK(r.wall.count("step") == 3 && r.wall.count("start") == 1);
  CHECK(r.pwr.zero_diff_calls == 1 && r.pwr.read_anchor_calls == 1);
}

// Revert (on_abort not cancelling): no "cancel" logged and the radio stays
// on the measured channel.
TEST(an_abort_mid_measurement_cancels) {
  Rig r;
  r.wall.finish_after = 100;
  r.cmd(wall_cmd(7, 0, 112, 20), 1000);
  r.pump(1200);
  r.pump(1201);
  r.abort(7, 1202);
  CHECK(r.wall.count("cancel") == 1 && r.wall.count("finish") == 0);
  CHECK(!r.sw.active());
  CHECK(!r.sw.take_done().has_value());
  r.pump(1203);
  CHECK(r.wall.count("step") == 1 && r.wall.count("cancel") == 1);
}

// Revert (the hard cap closing without cancelling): no "cancel" logged.
TEST(the_hard_cap_mid_measurement_cancels) {
  Rig r;
  r.wall.finish_after = 1000000;
  r.cmd(wall_cmd(7, 0, 112, 20), 1000);
  r.pump(1200);
  r.pump(1201);
  r.pump(1000 + 180000);
  CHECK(r.wall.count("cancel") == 1 && r.wall.count("finish") == 0);
  CHECK(!r.sw.active());
  CHECK(r.sw.power_restored_for_test());
}

// Revert (a closed run's measurement still starting on the next pump):
// "start" is logged.
TEST(an_abort_before_start_does_not_cancel_anything_open) {
  Rig r;
  r.cmd(wall_cmd(7, 0, 112, 20), 1000);
  r.abort(7, 1100);  // inside the 200 ms start delay
  CHECK(!r.sw.active());
  for (uint64_t t = 1100; t < 1600; t += 10) r.pump(t);
  CHECK(r.wall.count("start") == 0 && r.wall.count("cancel") == 0);
}

// A new sweep of the run arriving mid-measurement takes the radio back
// first. Revert (no cancel on supersede): the capacity points run with the
// radio still off-channel and no "cancel" is logged.
TEST(a_new_sweep_during_the_measurement_cancels_it) {
  Rig r;
  r.wall.finish_after = 100;
  r.cmd(wall_cmd(7, 0, 112, 20), 1000);
  r.pump(1200);
  r.pump(1201);
  r.cmd(cap_cmd(7, 1, 20, 0x01), 1202);
  CHECK(r.sw.take_ack()->result == cal::AckResult::Accepted);
  CHECK(r.wall.count("cancel") == 1);
  r.run(1202);
  CHECK(r.cap.points.size() == 1 && r.wall.count("finish") == 0);
  CHECK(r.sw.take_done()->seq == 1);
}

// Revert (the wall report not entering the repeat path): the second
// take_done is empty.
TEST(the_report_repeats_until_the_next_command) {
  Rig r;
  r.cmd(wall_cmd(7, 0, 112, 20), 1000);
  r.pump(1200);
  r.pump(1201);
  r.pump(1202);
  const auto first = r.sw.take_done();
  REQUIRE(first.has_value());
  r.pump(1202 + CalSweep::kDoneFirstRepeatMs - 1);
  CHECK(!r.sw.take_done().has_value());
  r.pump(1202 + CalSweep::kDoneFirstRepeatMs);
  const auto again = r.sw.take_done();
  REQUIRE(again.has_value());
  CHECK(again->nonce == 7 && again->walls == first->walls && again->channel == 112);
  r.cmd(cap_cmd(7, 1, 20, 0x01), 1400);
  (void)r.sw.take_ack();
  r.run(1400);
  const auto cap_done = r.sw.take_done();
  REQUIRE(cap_done.has_value());
  CHECK(cap_done->seq == 1 && cap_done->channel == 0);  // the wall report stopped
  CHECK(r.wall.count("finish") == 1);
}

// A Wall sweep that finishes before the GS hears its ack: the GS keeps
// repeating the SWEEP, and the answer it needs is the report. Revert (the
// repeat clearing the report and re-acking): an Accepted ack comes back and
// take_done stays empty. Revert (the repeat re-arming the schedule from
// now): the report due at end + 100 does not come.
TEST(a_repeat_of_a_finished_wall_sweep_re_sends_its_done) {
  Rig r;
  const auto c = wall_cmd(7, 0, 112, 20);
  r.cmd(c, 1000);
  REQUIRE(r.sw.take_ack()->result == cal::AckResult::Accepted);
  r.pump(1200);
  r.pump(1201);
  r.pump(1202);  // finished
  const auto first = r.sw.take_done();
  REQUIRE(first.has_value());
  r.cmd(c, 1250);
  CHECK(!r.sw.take_ack().has_value());
  const auto again = r.sw.take_done();
  REQUIRE(again.has_value());
  CHECK(again->nonce == 7 && again->seq == 0 && again->channel == 112);
  CHECK(again->walls == first->walls && again->anchor == first->anchor);
  CHECK(r.wall.count("check") == 1 && r.wall.count("start") == 1 &&
        r.wall.count("finish") == 1);
  CHECK(r.sw.state() == CalSweep::State::Idle && r.sw.active());
  r.pump(1202 + CalSweep::kDoneFirstRepeatMs - 1);
  CHECK(!r.sw.take_done().has_value());
  r.pump(1202 + CalSweep::kDoneFirstRepeatMs);  // the schedule is the original one
  CHECK(r.sw.take_done().has_value());
  CHECK(r.pwr.zero_diff_calls == 1 && r.pwr.read_anchor_calls == 1);
}

// The same for a capacity sweep: its report is a better answer than an
// ack for a sweep that is over. Revert (re-ack and clear): an ack comes
// back and take_done stays empty.
TEST(a_repeat_of_a_finished_capacity_sweep_re_sends_its_done) {
  Rig r;
  r.cap.to_return = BlastResult{500, 90};
  const uint64_t end = finish_one_point(r, 4, 0);
  REQUIRE(r.sw.take_done().has_value());
  const size_t points = r.cap.points.size();
  const size_t writes = r.pwr.index_writes.size();
  r.cmd(cap_cmd(4, 0, 20, 0x01), end + 10);
  CHECK(!r.sw.take_ack().has_value());
  const auto again = r.sw.take_done();
  REQUIRE(again.has_value());
  CHECK(again->seq == 0 && again->offered[0] == 500 && again->blocked_pct[0] == 90);
  CHECK(r.sw.state() == CalSweep::State::Idle);
  CHECK(r.cap.points.size() == points && r.pwr.index_writes.size() == writes);
}

MTEST_MAIN
