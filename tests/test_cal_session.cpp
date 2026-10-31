#include "mtest.h"
#include "cal_session.h"
#include "cal_wiring.h"
#include "cal_analysis.h"
#include "cal_log.h"
#include "cal_picker.h"
#include "cal_plan.h"

#include "mabur/cal_park.h"
#include "mabur/cal_proto.h"
#include "mabur/cal_wire.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <string>
#include <sys/stat.h>
#include <vector>

using namespace maburgs;
namespace cal = mabur::cal;

namespace {

using cal::Kind;
using State = CalSession::State;

constexpr uint8_t kFlags = cal::kFlagLdpc | cal::kFlagStbc;
constexpr size_t kPowerAir = cal_air_bytes(cal::kCalPayloadLen);
constexpr int kRows = kSweepHi - kSweepLo + 1;  // cells per rate row

struct FakePicker : CalPicker {
  Mode m = Mode::Auto;
  uint8_t op_ch = 36;
  std::optional<uint8_t> best;
  bool accept_reloc = true;
  std::optional<bool> settled;
  int measures = 0, relocs = 0, cancels = 0;
  uint8_t reloc_to = 0;
  Mode mode() const override { return m; }
  uint8_t op() const override { return op_ch; }
  void begin_measure(uint64_t) override { ++measures; }
  std::optional<uint8_t> measured_best() override { return best; }
  bool order_relocate(uint8_t ch) override {
    ++relocs;
    reloc_to = ch;
    return accept_reloc;
  }
  std::optional<bool> relocate_settled() override { return settled; }
  void cancel() override { ++cancels; }
};

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

// Every frame goes through the real stamp codec, as maburgs' RX path does.
cal::CalFrameInfo stamp(Kind k, uint8_t rate, int idx, uint8_t bw, uint8_t flags,
                        uint32_t seq = 0) {
  const auto p = cal::build_cal_payload(k, rate, static_cast<int8_t>(idx), bw, flags, seq);
  cal::CalFrameInfo f;
  REQUIRE(cal::parse_cal_payload(p.data(), p.size(), &f));
  return f;
}

template <class T>
std::optional<T> as(const std::optional<cal::CalMsg>& m) {
  if (!m) return std::nullopt;
  if (const T* p = std::get_if<T>(&*m)) return *p;
  return std::nullopt;
}

void ack(CalSession& s, uint32_t nonce, uint8_t seq, uint64_t now, uint8_t flags = kFlags,
         cal::AckResult r = cal::AckResult::Accepted) {
  s.on_mav(cal::SweepAck{nonce, seq, r, /*start_in_ms=*/0, flags}, now);
}

// Feeds one power sweep at `bw`: rate r delivers `per_cell` frames in every
// cell up to walls[r] and nothing above it; kNoWall delivers nothing.
void feed_power(CalSession& s, uint8_t bw, const std::array<int, 8>& walls, uint64_t now,
                uint8_t flags = kFlags, int per_cell = kPowerFrames) {
  for (uint8_t r = 0; r < 8; ++r)
    for (int idx = kSweepLo; idx <= kSweepHi; ++idx) {
      if (walls[r] == kNoWall || idx > walls[r]) continue;
      for (int k = 0; k < per_cell; ++k)
        s.on_cal_frame(0, stamp(Kind::Power, r, idx, bw, flags, k), -60, true, kPowerAir, now);
    }
}

cal::SweepDone power_done(uint32_t nonce, uint8_t seq, int per_cell = kPowerFrames) {
  cal::SweepDone d;
  d.nonce = nonce;
  d.seq = seq;
  d.offered.fill(static_cast<uint32_t>(kRows * per_cell));
  return d;
}

// Drives one power sweep from its SweepCmd to the first ApplyCmd.
cal::ApplyCmd run_power(CalSession& s, uint32_t nonce, uint8_t seq, uint8_t bw,
                        const std::array<int, 8>& walls, uint64_t t) {
  const auto cmd = as<cal::SweepCmd>(s.due_mav(t));
  REQUIRE(cmd.has_value());
  REQUIRE(cmd->kind == Kind::Power && cmd->bw == bw && cmd->seq == seq);
  ack(s, nonce, seq, t + 10);
  feed_power(s, bw, walls, t + 20);
  s.on_mav(power_done(nonce, seq), t + 30);
  const auto a = as<cal::ApplyCmd>(s.due_mav(t + 40));
  REQUIRE(a.has_value());
  REQUIRE(a->seq == seq);
  return *a;
}

const std::array<int, 8> kWalls = {20, 10, 5, 0, -5, -10, -20, kNoWall};

}  // namespace

TEST(start_sorts_and_dedups_the_sweep_list) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(1,
                  {{Kind::Capacity, 40}, {Kind::Power, 20}, {Kind::Capacity, 20},
                   {Kind::Power, 40}, {Kind::Power, 20}, {Kind::Capacity, 40}},
                  0, &err));
  const auto& v = s.sweeps();
  REQUIRE(v.size() == 4);
  CHECK(v[0].kind == Kind::Power && v[0].bw == 20);
  CHECK(v[1].kind == Kind::Capacity && v[1].bw == 20);
  CHECK(v[2].kind == Kind::Power && v[2].bw == 40);
  CHECK(v[3].kind == Kind::Capacity && v[3].bw == 40);
  CHECK(s.snapshot().sweep_n == 4);
  CHECK(s.snapshot().sweep_i == 0);

  const auto cmd = as<cal::SweepCmd>(s.due_mav(0));
  REQUIRE(cmd.has_value());
  const auto want = make_power_sweep(1, 0, 20);
  CHECK(cmd->nonce == 1 && cmd->seq == 0 && cmd->kind == Kind::Power && cmd->bw == 20);
  CHECK(cmd->frames_per_cell == want.frames_per_cell && cmd->idx_lo == want.idx_lo &&
        cmd->idx_hi == want.idx_hi && cmd->settle_ms == want.settle_ms);

  CalSession t(CalSessionCfg{});
  t.set_peer(true, true);
  CHECK(!t.start(2, {}, 0, &err));
  CHECK(!t.start(2, {{Kind::Verify, 20}}, 0, &err));
  CHECK(!t.start(2, {{Kind::Power, 30}}, 0, &err));
  CHECK(t.state() == State::Idle);
}

TEST(start_refuses_without_capability_or_link) {
  CalSession s(CalSessionCfg{});
  std::string err;
  s.set_peer(/*linked=*/false, /*cal_capable=*/true);
  CHECK(!s.start(1, {{Kind::Power, 20}}, 0, &err));
  CHECK(err.find("link") != std::string::npos);

  s.set_peer(/*linked=*/true, /*cal_capable=*/false);
  CHECK(!s.start(1, {{Kind::Power, 20}}, 0, &err));
  CHECK(err.find("CAP_CALIBRATE") != std::string::npos);

  s.set_peer(true, true);
  REQUIRE(s.start(1, {{Kind::Power, 20}}, 0, &err));
  CHECK(!s.start(2, {{Kind::Power, 20}}, 10, &err));
  CHECK(err.find("running") != std::string::npos);

  // A pick in progress is a run too, though not yet running().
  FakePicker p;
  CalSession q(CalSessionCfg{}, nullptr, &p);
  q.set_peer(true, true);
  REQUIRE(q.start(3, {{Kind::Power, 20}}, 0, &err));
  REQUIRE(q.state() == State::Picking);
  CHECK(!q.start(4, {{Kind::Power, 20}}, 10, &err));
}

TEST(picking_relocates_to_the_best_member_then_awaits_ack) {
  FakePicker p;
  p.op_ch = 36;
  CalSession s(CalSessionCfg{}, nullptr, &p);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(5, {{Kind::Power, 20}}, 0, &err));
  CHECK(s.state() == State::Picking);
  CHECK(p.measures == 1);
  CHECK(!s.running());
  CHECK(!s.radio_silent(0));
  CHECK(!s.due_mav(100).has_value());  // round still measuring

  p.best = 44;
  CHECK(!s.due_mav(200).has_value());
  CHECK(p.relocs == 1 && p.reloc_to == 44);
  CHECK(s.state() == State::Picking);
  CHECK(!s.due_mav(400).has_value());  // relocate pending
  CHECK(p.relocs == 1);                // ordered once

  p.settled = true;
  p.op_ch = 44;
  const auto cmd = as<cal::SweepCmd>(s.due_mav(600));
  REQUIRE(cmd.has_value());
  CHECK(s.state() == State::AwaitAck);
  CHECK(s.running());
  const auto snap = s.snapshot();
  CHECK(snap.channel == 44);
  CHECK(snap.pick_note.find("44") != std::string::npos);
  CHECK(p.cancels == 0);

  // Already on the best member: no relocate at all.
  FakePicker q;
  q.op_ch = 36;
  q.best = 36;
  CalSession t(CalSessionCfg{}, nullptr, &q);
  t.set_peer(true, true);
  REQUIRE(t.start(6, {{Kind::Power, 20}}, 0, &err));
  CHECK(as<cal::SweepCmd>(t.due_mav(10)).has_value());
  CHECK(q.relocs == 0);
  CHECK(t.snapshot().channel == 36);
}

TEST(pinned_picker_skips_picking_with_a_note) {
  std::string err;
  FakePicker p;
  p.m = CalPicker::Mode::Pinned;
  p.op_ch = 149;
  CalSession s(CalSessionCfg{}, nullptr, &p);
  s.set_peer(true, true);
  REQUIRE(s.start(7, {{Kind::Power, 20}}, 0, &err));
  CHECK(s.state() == State::AwaitAck);
  CHECK(p.measures == 0);
  CHECK(s.snapshot().pick_note == "pinned");
  CHECK(s.snapshot().channel == 149);
  CHECK(as<cal::SweepCmd>(s.due_mav(0)).has_value());

  FakePicker q;
  q.m = CalPicker::Mode::OneCard;
  CalSession t(CalSessionCfg{}, nullptr, &q);
  t.set_peer(true, true);
  REQUIRE(t.start(8, {{Kind::Power, 20}}, 0, &err));
  CHECK(t.state() == State::AwaitAck);
  CHECK(q.measures == 0);
  CHECK(t.snapshot().pick_note == "onecard");

  // No picker at all: same as pinned -- the run does not choose.
  CalSession u(CalSessionCfg{});
  u.set_peer(true, true);
  REQUIRE(u.start(9, {{Kind::Power, 20}}, 0, &err));
  CHECK(u.state() == State::AwaitAck);
  CHECK(u.snapshot().pick_note == "pinned");
}

TEST(unconfirmed_relocate_stays_on_op_and_proceeds) {
  std::string err;
  FakePicker p;
  p.op_ch = 36;
  p.best = 44;
  CalSession s(CalSessionCfg{}, nullptr, &p);
  s.set_peer(true, true);
  REQUIRE(s.start(10, {{Kind::Power, 20}}, 0, &err));
  CHECK(!s.due_mav(10).has_value());
  REQUIRE(p.relocs == 1);
  p.settled = false;
  CHECK(as<cal::SweepCmd>(s.due_mav(20)).has_value());
  CHECK(s.state() == State::AwaitAck);
  CHECK(s.snapshot().channel == 36);
  CHECK(s.snapshot().pick_note == "stayed on 36, relocate unconfirmed");
  CHECK(p.cancels == 1);

  // A refused order stays the same way.
  FakePicker q;
  q.op_ch = 36;
  q.best = 44;
  q.accept_reloc = false;
  CalSession t(CalSessionCfg{}, nullptr, &q);
  t.set_peer(true, true);
  REQUIRE(t.start(11, {{Kind::Power, 20}}, 0, &err));
  CHECK(as<cal::SweepCmd>(t.due_mav(10)).has_value());
  CHECK(t.snapshot().channel == 36);
  CHECK(t.snapshot().pick_note.find("stayed on 36") == 0);
  CHECK(q.cancels == 1);
}

TEST(sweep_command_repeats_until_acked_then_silence_opens) {
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(42, {{Kind::Power, 20}}, 0, &err));
  REQUIRE(as<cal::SweepCmd>(s.due_mav(0)).has_value());
  CHECK(!s.due_mav(100).has_value());
  CHECK(as<cal::SweepCmd>(s.due_mav(200)).has_value());
  CHECK(!s.radio_silent(250));

  ack(s, 42, /*seq=*/1, 280);  // another sweep's ack: ignored
  CHECK(s.state() == State::AwaitAck);
  ack(s, 41, /*seq=*/0, 290);  // another run's ack: ignored
  CHECK(s.state() == State::AwaitAck);

  s.on_mav(cal::SweepAck{42, 0, cal::AckResult::Accepted, /*start_in_ms=*/200, kFlags}, 300);
  CHECK(s.state() == State::Sweep);
  CHECK(!s.due_mav(400).has_value());
  CHECK(s.radio_silent(400));

  const uint64_t end = 300 + 200 + sweep_duration_ms(make_power_sweep(42, 0, 20));
  CHECK(!s.due_mav(end - 1).has_value());
  CHECK(s.state() == State::Sweep);
  CHECK(!s.due_mav(end).has_value());
  CHECK(s.state() == State::AwaitDone);
  CHECK(s.radio_silent(end + cfg.phase_slack_ms - 1));
  CHECK(!s.radio_silent(end + cfg.phase_slack_ms));
  s.due_mav(end + cfg.phase_slack_ms);
  CHECK(s.state() != State::AwaitDone);
  CHECK(!s.radio_silent(end + cfg.phase_slack_ms));
}

TEST(a_frame_with_the_wrong_width_or_flags_is_not_tallied) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(12, {{Kind::Power, 20}}, 0, &err));
  REQUIRE(s.due_mav(0).has_value());
  ack(s, 12, 0, 10, kFlags);
  s.on_cal_frame(0, stamp(Kind::Power, 0, 0, 40, kFlags), -60, true, kPowerAir, 20);
  s.on_cal_frame(0, stamp(Kind::Power, 0, 0, 20, cal::kFlagStbc), -60, true, kPowerAir, 20);
  s.on_cal_frame(0, stamp(Kind::Capacity, 0, 0, 20, kFlags), -60, true, kPowerAir, 20);
  s.on_cal_frame(0, stamp(Kind::Verify, 0, 0, 20, kFlags), -60, true, kPowerAir, 20);
  CHECK(s.cell_received(0, 0, 0) == 0);
  CHECK(s.snapshot().stamp_mismatch == 4);
  s.on_cal_frame(0, stamp(Kind::Power, 0, 0, 20, kFlags), -60, true, kPowerAir, 20);
  CHECK(s.cell_received(0, 0, 0) == 1);
  CHECK(s.snapshot().stamp_mismatch == 4);
  // CRC-bad frames are loss, never delivery.
  s.on_cal_frame(1, stamp(Kind::Power, 0, 0, 20, kFlags), -60, false, kPowerAir, 20);
  CHECK(s.cell_received(0, 0, 1) == 0);
}

TEST(done_closes_the_window_early) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(13, {{Kind::Power, 20}}, 0, &err));
  REQUIRE(s.due_mav(0).has_value());
  ack(s, 13, 0, 10);
  feed_power(s, 20, kWalls, 20);
  CHECK(s.radio_silent(1000));
  s.on_mav(power_done(99, 0), 1000);  // another run's Done: ignored
  CHECK(s.radio_silent(1000));
  s.on_mav(power_done(13, 0), 1000);
  CHECK(!s.radio_silent(1000));
  CHECK(s.state() == State::Apply);
  CHECK(as<cal::ApplyCmd>(s.due_mav(1000)).has_value());
}

TEST(power_sweep_yields_an_apply_with_leave_for_undetermined) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(14, {{Kind::Power, 20}}, 0, &err));
  const auto a = run_power(s, 14, 0, 20, kWalls, 0);
  CHECK(a.nonce == 14 && a.bw == 20);
  CHECK(a.has_walls && !a.has_eff);
  for (size_t r = 0; r < 7; ++r) CHECK(a.walls[r] == kWalls[r]);
  CHECK(a.walls[7] == cal::kWallLeave);
  CHECK(a.legacy == 20);  // mcs0's wall at 20 MHz
  const auto snap = s.snapshot();
  CHECK(snap.width[0].have_walls);
  CHECK(snap.width[0].walls[7].flags & kCalUndetermined);
  CHECK(snap.width[0].walls[0].wall == 20);
  // The apply repeats until Applied.
  CHECK(!s.due_mav(100).has_value());
  CHECK(as<cal::ApplyCmd>(s.due_mav(240)).has_value());
}

TEST(expected_per_cell_comes_from_the_done) {
  // The drone aired 10 frames per cell, not the commanded 20: delivery is
  // judged against what it says it offered.
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(15, {{Kind::Power, 20}}, 0, &err));
  REQUIRE(s.due_mav(0).has_value());
  ack(s, 15, 0, 10);
  feed_power(s, 20, kWalls, 20, kFlags, /*per_cell=*/10);
  s.on_mav(power_done(15, 0, /*per_cell=*/10), 30);
  const auto a = as<cal::ApplyCmd>(s.due_mav(40));
  REQUIRE(a.has_value());
  CHECK(a->walls[0] == 20);
  CHECK(a->walls[6] == -20);
}

TEST(applied_ok_opens_verify_and_tallies_by_rate) {
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(16, {{Kind::Power, 20}}, 0, &err));
  const auto a = run_power(s, 16, 0, 20, kWalls, 0);
  const auto parks = mabur::park_rel_for(a.walls, cfg.margin_db);

  s.on_mav(cal::Applied{16, 0, 40, true, ""}, 100);  // other width: ignored
  CHECK(s.state() == State::Apply);
  s.on_mav(cal::Applied{16, 0, 20, true, ""}, 100);
  CHECK(s.state() == State::Verify);
  CHECK(s.radio_silent(100));
  CHECK(!s.due_mav(400).has_value());  // no more apply repeats

  for (int k = 0; k < 20; ++k)
    s.on_cal_frame(0, stamp(Kind::Verify, 0, parks[0], 20, kFlags, k), -60, true, kPowerAir, 200);
  // Tallied by rate: an index other than the predicted park still counts.
  for (int k = 0; k < 10; ++k)
    s.on_cal_frame(1, stamp(Kind::Verify, 1, parks[1] + 1, 20, kFlags, k), -60, true, kPowerAir,
                   200);
  s.on_cal_frame(0, stamp(Kind::Verify, 2, parks[2], 40, kFlags), -60, true, kPowerAir, 200);
  CHECK(s.snapshot().stamp_mismatch == 1);

  const uint64_t close = 100 + verify_duration_ms(7) + cfg.phase_slack_ms;
  CHECK(s.radio_silent(close - 1));
  s.due_mav(close);
  CHECK(!s.radio_silent(close));
  CHECK(s.state() == State::Done);
  const auto w = s.snapshot().width[0];
  CHECK(w.applied);
  CHECK(w.verify_pct[0] == 100);
  CHECK(w.verify_pct[1] == 50);
  CHECK(w.verify_pct[2] == 0);   // parked, heard nothing
  CHECK(w.verify_pct[7] == -1);  // never parked
}

TEST(refused_apply_fails_with_the_drones_reason) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(17, {{Kind::Power, 20}, {Kind::Capacity, 20}}, 0, &err));
  run_power(s, 17, 0, 20, kWalls, 0);
  s.on_mav(cal::Applied{17, 0, 20, false, "config line not found"}, 100);
  CHECK(s.state() == State::Failed);
  const auto snap = s.snapshot();
  CHECK(snap.fail_reason.find("config line not found") != std::string::npos);
  CHECK(!snap.width[0].applied);
  CHECK(snap.width[0].apply_reason == "config line not found");
  CHECK(!s.radio_silent(100));
}

TEST(capacity_sweep_yields_eff_and_zero_for_tx_bound) {
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(18, {{Kind::Capacity, 20}}, 0, &err));
  const auto cmd = as<cal::SweepCmd>(s.due_mav(0));
  REQUIRE(cmd.has_value());
  CHECK(cmd->kind == Kind::Capacity && cmd->dwell_ms == kCapDwellMs);
  ack(s, 18, 0, 10, cal::kFlagStbc);
  constexpr size_t kAir = 1500;
  for (uint8_t r = 0; r < 2; ++r)
    for (int k = 0; k < 900; ++k)
      s.on_cal_frame(0, stamp(Kind::Capacity, r, 0, 20, cal::kFlagStbc, k), -60, true, kAir,
                     100);
  cal::SweepDone d;
  d.nonce = 18;
  d.seq = 0;
  d.offered = {1000, 1000, 0, 0, 0, 0, 0, 0};
  d.blocked_pct = {80, 10, 0, 0, 0, 0, 0, 0};
  s.on_mav(d, 200);
  const auto a = as<cal::ApplyCmd>(s.due_mav(210));
  REQUIRE(a.has_value());
  CHECK(a->has_eff && !a->has_walls);
  CHECK(a->bw == 20);

  CapCell c0;
  c0.offered = 1000;
  c0.blocked_pct = 80;
  c0.frames = {900, 0};
  c0.air_bits = {900ull * kAir * 8, 0};
  c0.rssi_dbm = {-60, 0};
  c0.have_rssi = {true, false};
  const CapResult r0 = analyze_capacity(c0, 0, 20, kCapDwellMs, cfg.th);
  REQUIRE(!(r0.flags & kCalTxBound));
  CHECK(a->eff_x1000[0] == static_cast<uint16_t>(std::lround(r0.eff * 1000.0)));
  CHECK(a->eff_x1000[0] > 0);
  CHECK(a->eff_x1000[1] == 0);  // blocked 10 %: tx_bound, never written
  for (size_t r = 2; r < 8; ++r) CHECK(a->eff_x1000[r] == 0);
  const auto w = s.snapshot().width[0];
  CHECK(w.have_eff);
  CHECK(w.cap[1].flags & kCalTxBound);

  s.on_mav(cal::Applied{18, 0, 20, true, ""}, 300);
  CHECK(s.state() == State::Done);  // no verify after a capacity apply
  CHECK(!s.radio_silent(300));
}

TEST(a_capacity_point_that_lost_over_half_its_frames_is_not_written) {
  // Under half of what was offered arrived: the dwell measured the loss,
  // not the air, and is kept out of the config like a tx_bound point. From
  // half up it is lossy and still written.
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(19, {{Kind::Capacity, 20}}, 0, &err));
  REQUIRE(as<cal::SweepCmd>(s.due_mav(0)).has_value());
  ack(s, 19, 0, 10, cal::kFlagStbc);
  constexpr size_t kAir = 1500;
  const int got[3] = {10, 499, 600};  // of 1000 offered each
  for (uint8_t r = 0; r < 3; ++r)
    for (int k = 0; k < got[r]; ++k)
      s.on_cal_frame(0, stamp(Kind::Capacity, r, 0, 20, cal::kFlagStbc, k), -60, true, kAir,
                     100);
  cal::SweepDone d;
  d.nonce = 19;
  d.seq = 0;
  d.offered = {1000, 1000, 1000, 0, 0, 0, 0, 0};
  d.blocked_pct = {80, 80, 80, 0, 0, 0, 0, 0};
  s.on_mav(d, 200);
  const auto a = as<cal::ApplyCmd>(s.due_mav(210));
  REQUIRE(a.has_value());
  CHECK(a->has_eff);
  CHECK(a->eff_x1000[0] == 0);
  CHECK(a->eff_x1000[1] == 0);
  CHECK(a->eff_x1000[2] > 0);
  const auto w = s.snapshot().width[0];
  CHECK(w.eff_x1000[0] == 0 && w.eff_x1000[1] == 0 && w.eff_x1000[2] > 0);
  CHECK((w.cap[0].flags & kCalLost) && (w.cap[0].flags & kCalLossy));
  CHECK((w.cap[1].flags & kCalLost) && (w.cap[1].flags & kCalLossy));
  CHECK(!(w.cap[2].flags & kCalLost) && (w.cap[2].flags & kCalLossy));
}

TEST(a_capacity_sweep_that_lost_every_point_writes_nothing) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(20, {{Kind::Capacity, 20}}, 0, &err));
  REQUIRE(as<cal::SweepCmd>(s.due_mav(0)).has_value());
  ack(s, 20, 0, 10, cal::kFlagStbc);
  for (int k = 0; k < 5; ++k)
    s.on_cal_frame(0, stamp(Kind::Capacity, 0, 0, 20, cal::kFlagStbc, k), -60, true, 1500, 100);
  cal::SweepDone d;
  d.nonce = 20;
  d.seq = 0;
  d.offered = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
  d.blocked_pct = {80, 80, 80, 80, 80, 80, 80, 80};
  s.on_mav(d, 200);
  for (size_t r = 0; r < 8; ++r) CHECK(s.snapshot().width[0].eff_x1000[r] == 0);
}

TEST(done_sends_three_aborts_so_the_drone_resumes_at_once) {
  // The drone otherwise sits in its await window (power parked, video
  // off) until it times out; its close keeps the written config. Three
  // copies 200 ms apart, since the uplink is lossy and the drone ignores
  // a nonce it already closed.
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(35, {{Kind::Power, 20}}, 0, &err));
  run_power(s, 35, 0, 20, kWalls, 0);
  s.on_mav(cal::Applied{35, 0, 20, true, ""}, 100);
  const uint64_t close = 100 + verify_duration_ms(7) + cfg.phase_slack_ms;
  std::vector<uint64_t> at;
  for (uint64_t t = close; t < close + 2000; t += 50) {
    const auto m = s.due_mav(t);
    if (const auto a = as<cal::Abort>(m)) {
      at.push_back(t);
      CHECK(a->nonce == 35);
    }
    CHECK(!as<cal::SweepCmd>(m).has_value());
  }
  CHECK(s.state() == State::Done);
  REQUIRE(at.size() == 3);
  CHECK(at[1] - at[0] == 200);
  CHECK(at[2] - at[1] == 200);
}

TEST(a_repeated_done_after_analyze_is_ignored) {
  // The drone repeats SweepDone every 200 ms until the GS's next command
  // reaches it, so repeats keep landing after the sweep was analysed: in
  // Apply, in Verify and in the next sweep's AwaitAck. None may move the
  // session or change what it sends.
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(37, {{Kind::Power, 20}, {Kind::Capacity, 20}}, 0, &err));
  const auto a = run_power(s, 37, 0, 20, kWalls, 0);
  REQUIRE(s.state() == State::Apply);
  s.on_mav(power_done(37, 0), 50);
  CHECK(s.state() == State::Apply);
  const auto again = as<cal::ApplyCmd>(s.due_mav(240));
  REQUIRE(again.has_value());
  CHECK(again->seq == a.seq && again->walls == a.walls && again->legacy == a.legacy);

  s.on_mav(cal::Applied{37, 0, 20, true, ""}, 250);
  REQUIRE(s.state() == State::Verify);
  s.on_mav(power_done(37, 0), 260);
  CHECK(s.state() == State::Verify);
  CHECK(s.radio_silent(260));

  const uint64_t next = 250 + verify_duration_ms(7) + cfg.phase_slack_ms;
  const auto cmd = as<cal::SweepCmd>(s.due_mav(next));
  REQUIRE(cmd.has_value());
  CHECK(cmd->seq == 1);
  s.on_mav(power_done(37, 0), next + 10);
  CHECK(s.state() == State::AwaitAck);
  CHECK(s.snapshot().width[0].have_walls);
  CHECK(s.snapshot().width[0].walls[0].wall == 20);
}

TEST(a_stale_applied_for_the_power_seq_does_not_satisfy_the_capacity_apply) {
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(36, {{Kind::Power, 20}, {Kind::Capacity, 20}}, 0, &err));
  run_power(s, 36, 0, 20, kWalls, 0);
  s.on_mav(cal::Applied{36, 0, 20, true, ""}, 100);
  const uint64_t t = 100 + verify_duration_ms(7) + cfg.phase_slack_ms;
  const auto cmd = as<cal::SweepCmd>(s.due_mav(t));
  REQUIRE(cmd.has_value() && cmd->kind == Kind::Capacity && cmd->seq == 1);
  ack(s, 36, 1, t + 10);
  for (int k = 0; k < 100; ++k)
    s.on_cal_frame(0, stamp(Kind::Capacity, 0, 0, 20, kFlags, k), -60, true, 1500, t + 20);
  cal::SweepDone d;
  d.nonce = 36;
  d.seq = 1;
  d.offered.fill(100);
  d.blocked_pct.fill(90);
  s.on_mav(d, t + 30);
  const auto a = as<cal::ApplyCmd>(s.due_mav(t + 40));
  REQUIRE(a.has_value());
  CHECK(a->seq == 1 && a->bw == 20 && a->has_eff);
  // The drone's cached answer to the power apply (same nonce, same width)
  // arriving again must not count as the capacity apply's.
  s.on_mav(cal::Applied{36, 0, 20, true, ""}, t + 50);
  CHECK(s.state() == State::Apply);
  CHECK(as<cal::ApplyCmd>(s.due_mav(t + 240)).has_value());
  s.on_mav(cal::Applied{36, 1, 20, true, ""}, t + 250);
  CHECK(s.state() == State::Done);
}

TEST(a_crc_bad_frame_is_not_an_implicit_ack) {
  // maburgs keeps corrupted frames, and a flipped LDPC/STBC bit still
  // parses: such a frame must not set the run's flags.
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(37, {{Kind::Power, 20}}, 0, &err));
  REQUIRE(s.due_mav(0).has_value());
  s.on_cal_frame(0, stamp(Kind::Power, 0, kSweepLo, 20, cal::kFlagLdpc), -60, /*crc_ok=*/false,
                 kPowerAir, 50);
  CHECK(s.state() == State::AwaitAck);
  CHECK(as<cal::SweepCmd>(s.due_mav(200)).has_value());  // still commanding
  s.on_cal_frame(0, stamp(Kind::Power, 0, kSweepLo, 20, kFlags), -60, true, kPowerAir, 250);
  CHECK(s.state() == State::Sweep);
  CHECK(s.snapshot().flags == kFlags);
}

TEST(capacity_without_done_writes_no_eff) {
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(19, {{Kind::Capacity, 20}}, 0, &err));
  REQUIRE(s.due_mav(0).has_value());
  ack(s, 19, 0, 10, cal::kFlagStbc);
  for (int k = 0; k < 900; ++k)
    s.on_cal_frame(0, stamp(Kind::Capacity, 0, 0, 20, cal::kFlagStbc, k), -60, true, 1500, 100);
  const uint64_t close =
      10 + sweep_duration_ms(make_capacity_sweep(19, 0, 20)) + cfg.phase_slack_ms;
  for (uint64_t t = 200; t <= close + 2000; t += 100)
    CHECK(!as<cal::ApplyCmd>(s.due_mav(t)).has_value());
  CHECK(s.state() == State::Done);
  const auto w = s.snapshot().width[0];
  CHECK(w.eff_unknown);
  CHECK(!w.have_eff);
  for (size_t r = 0; r < 8; ++r) CHECK(w.eff_x1000[r] == 0);
}

TEST(legacy_comes_from_the_20_table_when_both_are_swept) {
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(20, {{Kind::Power, 40}, {Kind::Power, 20}}, 0, &err));
  const auto a20 = run_power(s, 20, 0, 20, kWalls, 0);
  CHECK(a20.legacy == 20);
  s.on_mav(cal::Applied{20, 0, 20, true, ""}, 100);
  const uint64_t t = 100 + verify_duration_ms(7) + cfg.phase_slack_ms;
  const std::array<int, 8> w40 = {15, 10, 5, 0, -5, -10, -20, -30};
  const auto a40 = run_power(s, 20, 1, 40, w40, t);
  CHECK(a40.bw == 40);
  CHECK(a40.walls[0] == 15);
  CHECK(a40.legacy == cal::kWallLeave);

  // No 20 MHz table in the run: the 40 MHz mcs0 wall is all there is.
  CalSession u(cfg);
  u.set_peer(true, true);
  REQUIRE(u.start(21, {{Kind::Power, 40}}, 0, &err));
  const auto b40 = run_power(u, 21, 0, 40, w40, 0);
  CHECK(b40.legacy == 15);
}

TEST(ack_timeout_fails_without_an_apply) {
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(22, {{Kind::Power, 20}}, 0, &err));
  int applies = 0, aborts = 0;
  for (uint64_t t = 0; t <= cfg.ack_timeout_ms + 1000; t += 50) {
    const auto m = s.due_mav(t);
    if (as<cal::ApplyCmd>(m)) ++applies;
    if (as<cal::Abort>(m)) ++aborts;
  }
  CHECK(s.state() == State::Failed);
  CHECK(s.snapshot().fail_reason.find("ack") != std::string::npos);
  CHECK(applies == 0);
  CHECK(aborts == 3);  // the drone may have heard a SweepCmd: close its run
  CHECK(!s.radio_silent(cfg.ack_timeout_ms + 1000));
}

TEST(refused_ack_fails_with_its_reason) {
  const std::pair<cal::AckResult, const char*> cases[] = {
      {cal::AckResult::AnchorUnreadable, "anchor_unreadable"},
      {cal::AckResult::Busy, "busy"},
      {cal::AckResult::Unsupported, "unsupported"}};
  for (const auto& [res, word] : cases) {
    CalSession s(CalSessionCfg{});
    s.set_peer(true, true);
    std::string err;
    REQUIRE(s.start(23, {{Kind::Power, 20}}, 0, &err));
    REQUIRE(s.due_mav(0).has_value());
    ack(s, 23, 0, 10, kFlags, res);
    CHECK(s.state() == State::Failed);
    CHECK(s.snapshot().fail_reason.find(word) != std::string::npos);
    CHECK(!s.radio_silent(10));
  }
}

TEST(apply_timeout_fails) {
  CalSessionCfg cfg;
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(24, {{Kind::Power, 20}}, 0, &err));
  run_power(s, 24, 0, 20, kWalls, 0);  // first ApplyCmd at t = 40
  int applies = 1;
  for (uint64_t t = 50; t < 40 + cfg.apply_timeout_ms; t += 50)
    if (as<cal::ApplyCmd>(s.due_mav(t))) ++applies;
  CHECK(s.state() == State::Apply);
  CHECK(applies >= 20);  // every 200 ms
  s.due_mav(40 + cfg.apply_timeout_ms);
  CHECK(s.state() == State::Failed);
  CHECK(s.snapshot().fail_reason.find("apply") != std::string::npos);
}

TEST(abort_emits_three_aborts_and_reopens_the_air) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(25, {{Kind::Power, 20}}, 0, &err));
  REQUIRE(s.due_mav(0).has_value());
  ack(s, 25, 0, 10);
  REQUIRE(s.radio_silent(100));
  s.abort("operator abort");
  CHECK(!s.radio_silent(100));
  CHECK(s.state() == State::Idle);
  CHECK(!s.running());
  const auto m = as<cal::Abort>(s.due_mav(110));
  REQUIRE(m.has_value());
  CHECK(m->nonce == 25);
  CHECK(!s.due_mav(120).has_value());
  CHECK(!s.due_mav(309).has_value());
  CHECK(as<cal::Abort>(s.due_mav(310)).has_value());
  CHECK(!s.due_mav(400).has_value());
  const auto last = as<cal::Abort>(s.due_mav(510));
  REQUIRE(last.has_value());
  CHECK(last->nonce == 25);
  CHECK(!s.due_mav(710).has_value());
  CHECK(!s.due_mav(1000).has_value());
  CHECK(s.snapshot().fail_reason == "operator abort");
  // A late frame or ack of the aborted run changes nothing.
  ack(s, 25, 0, 130);
  CHECK(s.state() == State::Idle);
}

TEST(picker_is_cancelled_on_pick_timeout_and_abort) {
  CalSessionCfg cfg;
  std::string err;
  FakePicker p;
  p.op_ch = 36;
  CalSession s(cfg, nullptr, &p);
  s.set_peer(true, true);
  REQUIRE(s.start(26, {{Kind::Power, 20}}, 0, &err));
  CHECK(!s.due_mav(cfg.pick_timeout_ms - 1).has_value());
  CHECK(p.cancels == 0);
  CHECK(as<cal::SweepCmd>(s.due_mav(cfg.pick_timeout_ms)).has_value());
  CHECK(p.cancels == 1);
  CHECK(s.snapshot().channel == 36);
  CHECK(s.snapshot().pick_note.find("stayed on 36") == 0);

  FakePicker q;
  CalSession t(cfg, nullptr, &q);
  t.set_peer(true, true);
  REQUIRE(t.start(27, {{Kind::Power, 20}}, 0, &err));
  t.abort("operator abort");
  CHECK(q.cancels == 1);
  CHECK(t.state() == State::Idle);
  CHECK(!t.due_mav(10).has_value());  // nothing reached the drone: no Abort
}

TEST(implicit_ack_from_the_first_matching_frame) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(28, {{Kind::Power, 20}}, 0, &err));
  REQUIRE(s.due_mav(0).has_value());
  // Neither a capacity frame nor a 40 MHz one stands in for the ack.
  s.on_cal_frame(0, stamp(Kind::Capacity, 0, 0, 20, cal::kFlagStbc), -60, true, 1500, 50);
  s.on_cal_frame(0, stamp(Kind::Power, 0, kSweepLo, 40, cal::kFlagStbc), -60, true, kPowerAir,
                 60);
  CHECK(s.state() == State::AwaitAck);
  s.on_cal_frame(0, stamp(Kind::Power, 0, kSweepLo, 20, cal::kFlagStbc), -60, true, kPowerAir,
                 100);
  CHECK(s.state() == State::Sweep);
  CHECK(s.radio_silent(100));
  CHECK(s.snapshot().flags == cal::kFlagStbc);  // adopted from the frame
  CHECK(s.cell_received(0, kSweepLo, 0) == 1);
  CHECK(!s.due_mav(300).has_value());  // no more SweepCmd repeats
  // The adopted flags now filter like an ack's.
  s.on_cal_frame(0, stamp(Kind::Power, 0, kSweepLo, 20, kFlags), -60, true, kPowerAir, 110);
  CHECK(s.cell_received(0, kSweepLo, 0) == 1);
}

TEST(verify_parks_match_the_drone_rule) {
  CalSessionCfg cfg;
  cfg.margin_db = 7.0;  // 28 steps: pushes a low wall past kRelMin
  CalSession s(cfg);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(29, {{Kind::Power, 20}}, 0, &err));
  const std::array<int, 8> walls = {kSweepLo, 30, kNoWall, kNoWall,
                                    kNoWall,  kNoWall, kNoWall, kNoWall};
  const auto a = run_power(s, 29, 0, 20, walls, 0);
  REQUIRE(a.walls[0] == kSweepLo);
  s.on_mav(cal::Applied{29, 0, 20, true, ""}, 100);
  REQUIRE(s.state() == State::Verify);
  const auto drone = mabur::park_rel_for(a.walls, cfg.margin_db);
  const auto w = s.snapshot().width[0];
  for (size_t r = 0; r < 8; ++r) CHECK(w.park[r] == drone[r]);
  CHECK(w.park[0] == mabur::rc::kRelMin);  // clamped, as the drone clamps
  CHECK(w.park[1] == 30 - 28);
  CHECK(w.park[2] == cal::kWallLeave);
  // The window covers exactly the drone's two verify cells.
  const uint64_t close = 100 + verify_duration_ms(verify_cells(drone).size()) + cfg.phase_slack_ms;
  CHECK(s.radio_silent(close - 1));
  CHECK(!s.radio_silent(close));
}

TEST(snapshot_tracks_sweep_and_cell_progress) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(30, {{Kind::Capacity, 20}, {Kind::Power, 20}}, 0, &err));
  REQUIRE(s.due_mav(0).has_value());
  ack(s, 30, 0, 10);
  auto snap = s.snapshot();
  CHECK(snap.state == State::Sweep);
  CHECK(snap.sweep_i == 0 && snap.sweep_n == 2);
  CHECK(snap.cur.kind == Kind::Power && snap.cur.bw == 20);
  CHECK(snap.cell_n == static_cast<uint32_t>(8 * kRows));
  CHECK(snap.cell_i == 0);
  CHECK(snap.flags == kFlags);
  s.on_cal_frame(0, stamp(Kind::Power, 1, kSweepLo, 20, kFlags), -60, true, kPowerAir, 20);
  CHECK(s.snapshot().cell_i == static_cast<uint32_t>(kRows + 1));
  s.on_cal_frame(0, stamp(Kind::Power, 1, kSweepLo + 1, 20, kFlags), -60, true, kPowerAir, 30);
  CHECK(s.snapshot().cell_i == static_cast<uint32_t>(kRows + 2));

  // Close the power sweep with nothing determined -> straight to sweep 1.
  s.on_mav(power_done(30, 0), 40);
  const auto cmd = as<cal::SweepCmd>(s.due_mav(50));
  REQUIRE(cmd.has_value());
  CHECK(cmd->kind == Kind::Capacity && cmd->seq == 1);
  ack(s, 30, 1, 60);
  snap = s.snapshot();
  CHECK(snap.sweep_i == 1);
  CHECK(snap.cur.kind == Kind::Capacity);
  CHECK(snap.cell_n == 8);
  s.on_cal_frame(0, stamp(Kind::Capacity, 3, 0, 20, kFlags), -60, true, 1500, 70);
  CHECK(s.snapshot().cell_i == 4);
}

TEST(tag_context_is_frozen_at_start) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true, mabur::rc::TagCtx{11, 22, 0});
  std::string err;
  REQUIRE(s.start(31, {{Kind::Power, 20}}, 0, &err));
  s.set_peer(true, true, mabur::rc::TagCtx{11, 33, 0});   // re-paired mid-run
  s.set_peer(false, true, mabur::rc::TagCtx{11, 33, 0});  // and lost again
  CHECK(s.tag_ctx().vrx_nonce == 11);
  CHECK(s.tag_ctx().vtx_nonce == 22);
  s.abort("test");
  s.set_peer(true, true, mabur::rc::TagCtx{11, 44, 0});
  REQUIRE(s.start(32, {{Kind::Power, 20}}, 10, &err));
  CHECK(s.tag_ctx().vtx_nonce == 44);

  // With a pick, nothing reaches the drone until the pick ends, so the
  // context is the one the link holds when the first SweepCmd is due.
  FakePicker p;
  p.best = 36;
  CalSession t(CalSessionCfg{}, nullptr, &p);
  t.set_peer(true, true, mabur::rc::TagCtx{1, 2, 0});
  REQUIRE(t.start(33, {{Kind::Power, 20}}, 0, &err));
  t.set_peer(true, true, mabur::rc::TagCtx{1, 3, 0});
  REQUIRE(as<cal::SweepCmd>(t.due_mav(10)).has_value());
  CHECK(t.tag_ctx().vtx_nonce == 3);
  t.set_peer(true, true, mabur::rc::TagCtx{1, 4, 0});
  CHECK(t.tag_ctx().vtx_nonce == 3);
}

TEST(cal_log_gets_sweep_cell_wall_cap_verify_and_applied_rows) {
  const std::string dir = fresh_dir("cal_session_log");
  CalLog log(dir);
  REQUIRE(log.ok());
  log.header();
  CalSessionCfg cfg;
  FakePicker p;
  p.m = CalPicker::Mode::Pinned;
  p.op_ch = 36;
  CalSession s(cfg, &log, &p);
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(77, {{Kind::Capacity, 20}, {Kind::Power, 20}}, 0, &err));
  run_power(s, 77, 0, 20, kWalls, 0);
  s.on_mav(cal::Applied{77, 0, 20, true, ""}, 100);
  const auto parks = mabur::park_rel_for(
      {20, 10, 5, 0, -5, -10, -20, cal::kWallLeave}, cfg.margin_db);
  for (int k = 0; k < 20; ++k)
    s.on_cal_frame(0, stamp(Kind::Verify, 0, parks[0], 20, kFlags, k), -60, true, kPowerAir, 200);
  uint64_t t = 100 + verify_duration_ms(7) + cfg.phase_slack_ms;
  const auto cmd = as<cal::SweepCmd>(s.due_mav(t));
  REQUIRE(cmd.has_value() && cmd->kind == Kind::Capacity);
  ack(s, 77, 1, t + 10);
  for (int k = 0; k < 100; ++k)
    s.on_cal_frame(0, stamp(Kind::Capacity, 0, 0, 20, kFlags, k), -60, true, 1500, t + 20);
  cal::SweepDone d;
  d.nonce = 77;
  d.seq = 1;
  d.offered.fill(100);
  d.blocked_pct.fill(90);
  s.on_mav(d, t + 30);
  REQUIRE(as<cal::ApplyCmd>(s.due_mav(t + 40)).has_value());
  s.on_mav(cal::Applied{77, 1, 20, true, ""}, t + 50);
  REQUIRE(s.state() == State::Done);

  // On disk the moment the run reaches Done, without closing the log.
  const auto lines = cal_log_lines(dir);
  std::vector<std::string> seq;
  for (const auto& l : lines) {
    const std::string tag = l.substr(0, l.find(' '));
    if (seq.empty() || seq.back() != tag) seq.push_back(tag);
  }
  const std::vector<std::string> want = {"callog", "R", "S", "C", "W", "A",
                                         "V",      "S", "P", "A"};
  CHECK(seq == want);
  REQUIRE(lines.size() > 3);
  CHECK(lines[1] == "R 77 1.00 36 pinned 1");
  CHECK(lines[2] == "S 0 power 20 20");
  size_t c = 0, w = 0, v = 0, pr = 0;
  for (const auto& l : lines) {
    if (l[0] == 'C') ++c;
    if (l[0] == 'W') ++w;
    if (l[0] == 'V') ++v;
    if (l[0] == 'P') ++pr;
    if (l[0] == 'S' && l.find("capacity") != std::string::npos)
      CHECK(l == "S 1 capacity 20 2000");
  }
  CHECK(c == static_cast<size_t>(8 * kRows));
  CHECK(w == 8);
  CHECK(v == 7);  // every parked rate, heard or not
  CHECK(pr == 8);
}

TEST(null_log_sink_is_inert) {
  CalSession s(CalSessionCfg{});
  s.set_peer(true, true);
  std::string err;
  REQUIRE(s.start(34, {{Kind::Power, 20}}, 0, &err));
  run_power(s, 34, 0, 20, kWalls, 0);
  s.on_mav(cal::Applied{34, 0, 20, true, ""}, 100);
  s.due_mav(100 + verify_duration_ms(7) + 1000);
  CHECK(s.state() == State::Done);
}

MTEST_MAIN
