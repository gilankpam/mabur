#include "cal_link.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "cal_sweep.h"

#include "mtest.h"
#include "scratch.h"

using namespace mabur;

TEST(cal_write_scales_eff_and_keeps_zero_as_leave) {
  cal::ApplyCmd a;
  a.nonce = 7;
  a.bw = 40;
  a.eff_x1000 = {930, 0, 840, 800, 760, 680, 630, 600};
  const CalWrite w = cal_write_for(a);
  CHECK(w.bw == 40);
  CHECK(w.eff[0] == 0.93 && w.eff[1] == 0.0 && w.eff[7] == 0.6);
}

namespace {
std::string slurp(const std::string& p) {
  std::ifstream f(p);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}
}  // namespace

// The run's applies the way the TX writer makes them: the ".pre-cal" backup
// holds the file as it was before the run, however many applies follow.
// Revert (the ledger not latching the backup on the first ok apply): the
// second apply overwrites ".pre-cal" with the mid-run file.
TEST(a_run_of_efficiency_applies_backs_up_once) {
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal_link_run.toml";
  std::remove((path + ".pre-cal").c_str());
  const std::string original =
      "[air_clock]\nshed_ms = 25\nefficiency_20 = [0.93, 0.88, 0.84, 0.8, 0.76, 0.76, 0.78, "
      "0.76]\n";
  { std::ofstream f(path); f << original; }
  CalApplyLedger ledger;
  for (uint8_t seq : {0, 1}) {
    cal::ApplyCmd a;
    a.nonce = 5;
    a.seq = seq;
    a.bw = seq == 0 ? 20 : 40;
    a.eff_x1000 = {800, 0, 0, 0, 0, 0, 0, 700};
    std::string err;
    const bool ok = apply_calibration(path, cal_write_for(a), ledger.want_backup(a.nonce), &err) ==
                    ApplyResult::Ok;
    CHECK(ok);
    ledger.record(cal::Applied{a.nonce, a.seq, a.bw, ok, ""});
  }
  CHECK(slurp(path + ".pre-cal") == original);
  const std::string now = slurp(path);
  CHECK(now.find("efficiency_20 = [0.8, 0.88,") != std::string::npos);
  CHECK(now.find("efficiency_40 = [0.8,") != std::string::npos);
}

TEST(ledger_caches_applied_per_nonce_and_seq) {
  CalApplyLedger l;
  CHECK(!l.cached(1, 0).has_value());
  CHECK(l.want_backup(1));

  l.record(cal::Applied{1, 0, 20, false, "walls out of range"});
  REQUIRE(l.cached(1, 0).has_value());
  CHECK(!l.cached(1, 0)->ok && l.cached(1, 0)->reason == "walls out of range");
  CHECK(l.cached(1, 0)->bw == 20);
  CHECK(!l.cached(1, 1).has_value());
  CHECK(l.want_backup(1));  // a failed apply wrote nothing to back up

  l.record(cal::Applied{1, 1, 40, true, ""});
  CHECK(l.cached(1, 1).has_value() && l.cached(1, 1)->ok);
  CHECK(!l.want_backup(1));  // the run's backup is taken

  // A later apply of the same seq replaces the cached answer.
  l.record(cal::Applied{1, 0, 20, true, ""});
  CHECK(l.cached(1, 0)->ok);

  // Another nonce: nothing cached for it, and it wants its own backup.
  CHECK(!l.cached(2, 0).has_value());
  CHECK(l.want_backup(2));
  l.record(cal::Applied{2, 0, 20, true, ""});
  CHECK(!l.cached(1, 1).has_value());  // the old run's answers are gone
  CHECK(!l.want_backup(2));
}

TEST(ledger_treats_a_new_seq_at_the_same_width_as_a_new_apply) {
  // A run may apply twice at one width (two capacity sweeps). Answering the
  // second from the first's cache would leave its table unwritten.
  CalApplyLedger l;
  l.record(cal::Applied{7, 0, 20, true, ""});
  REQUIRE(l.cached(7, 0).has_value());  // a repeat of the first apply
  CHECK(!l.cached(7, 1).has_value());   // the second apply at 20 is new
}

TEST(ldpc_mismatch_warns_per_flown_width) {
  RadioCfg r;
  r.power_mode = "offset";
  r.rate_walls_rel_20 = std::array<int, 8>{63, 63, 63, 63, 45, 23, 22, 21};
  r.cal_flags_20 = "ldpc+stbc";
  r.cal_flags_40 = "stbc";  // no 40 MHz table: never warned about
  r.ldpc = true;
  CHECK(ldpc_mismatch_warnings(r).empty());

  r.ldpc = false;
  auto w = ldpc_mismatch_warnings(r);
  REQUIRE(w.size() == 1);
  CHECK(w[0] ==
        "radio: 20 MHz walls were measured with ldpc on; radio.ldpc is off \xe2\x80\x94 re-run "
        "maburcal");

  r.rate_walls_rel_40 = std::array<int, 8>{63, 63, 63, 63, 45, 28, 28, 23};
  r.ldpc = true;
  r.width = 20;  // a 20 MHz radio never flies 40: its table is not warned about
  CHECK(ldpc_mismatch_warnings(r).empty());
  r.width = 40;
  w = ldpc_mismatch_warnings(r);
  REQUIRE(w.size() == 1);
  CHECK(w[0] ==
        "radio: 40 MHz walls were measured with ldpc off; radio.ldpc is on \xe2\x80\x94 re-run "
        "maburcal");

  r.power_mode = "none";  // nothing flown, nothing to warn about
  CHECK(ldpc_mismatch_warnings(r).empty());
}

TEST(missing_wall_notes_name_the_fallback_width) {
  RadioCfg r;
  r.power_mode = "offset";
  r.width = 20;
  r.rate_walls_rel_20 = std::array<int, 8>{63, 63, 63, 63, 45, 23, 22, 21};
  CHECK(missing_wall_notes(r).empty());  // a 20 MHz radio never flies 40
  r.width = 40;
  auto n = missing_wall_notes(r);
  REQUIRE(n.size() == 1);
  CHECK(n[0] == "radio: no 40 MHz walls, flying the 20 MHz table at 40");

  r.rate_walls_rel_20.reset();
  r.rate_walls_rel_40 = std::array<int, 8>{63, 63, 63, 63, 45, 28, 28, 23};
  n = missing_wall_notes(r);
  REQUIRE(n.size() == 1);
  // 40 MHz walls sit at or above the 20 MHz ones, so this direction
  // overdrives and the note says so.
  CHECK(n[0] ==
        "radio: no 20 MHz walls, flying the 40 MHz table at 20 "
        "\xe2\x80\x94 the overdriving direction, re-run maburcal at 20 MHz");

  r.rate_walls_rel_20 = std::array<int, 8>{63, 63, 63, 63, 45, 23, 22, 21};
  CHECK(missing_wall_notes(r).empty());

  r.rate_walls_rel_40.reset();
  r.power_mode = "none";
  CHECK(missing_wall_notes(r).empty());
}

namespace {
struct NullPower : CalSweep::PowerCtl {
  bool set_index_override(int) override { return true; }
  bool zero_rate_diffs() override { return true; }
  int read_anchor_idx() override { return anchor; }
  int anchor = 50;
};
struct NullWall : CalSweep::WallCtl {
  cal::AckResult check(uint8_t, uint8_t) override { return cal::AckResult::Accepted; }
  void start(uint8_t, uint8_t, uint64_t) override {}
  bool step(uint64_t) override { return true; }
  cal::SweepDone finish() override { return {}; }
  void cancel() override {}
};
cal::SweepCmd sweep(uint32_t nonce) {
  cal::SweepCmd c;
  c.nonce = nonce;
  c.kind = cal::Kind::Capacity;
  return c;
}
}  // namespace

// The restore is owed from the first device write of a sweep until no run
// is open, however briefly the run lived: a SWEEP and an ABORT drained in
// the same pass open and close a run before the owner looks at active().
TEST(restore_is_owed_even_for_a_run_opened_and_closed_in_one_pass) {
  NullPower pwr;
  NullWall wall;
  CalSweep s(CalSweepCfg{});
  CalRestoreOwed owed;
  CHECK(!owed.take_due(s.active()));  // nothing touched yet

  const auto c = sweep(7);
  REQUIRE(s.starts_sweep(c));
  owed.note_device_touched();
  s.on_cmd(c, 0, pwr, wall);
  s.on_abort(cal::Abort{7}, 1, pwr, wall);
  REQUIRE(!s.active());
  CHECK(owed.take_due(s.active()));
  CHECK(!owed.take_due(s.active()));  // once

  // A run still open owes its restore until it closes.
  const auto d = sweep(8);
  owed.note_device_touched();
  s.on_cmd(d, 10, pwr, wall);
  REQUIRE(s.active());
  CHECK(!owed.take_due(s.active()));
  s.on_abort(cal::Abort{8}, 11, pwr, wall);
  CHECK(owed.take_due(s.active()));

  // A refused sweep (anchor unreadable) never opens a run but still owes it.
  pwr.anchor = -1;
  owed.note_device_touched();
  s.on_cmd(sweep(9), 20, pwr, wall);
  REQUIRE(!s.active());
  CHECK(owed.take_due(s.active()));
}

MTEST_MAIN
