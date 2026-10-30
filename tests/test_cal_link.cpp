#include "cal_link.h"

#include "cal_sweep.h"

#include "mabur/cal_wire.h"
#include "mabur/rc_proto.h"
#include "mtest.h"
#include "power_plan.h"
#include "power_tables.h"

using namespace mabur;

namespace {

cal::ApplyCmd walls_apply(uint32_t nonce, uint8_t bw) {
  cal::ApplyCmd a;
  a.nonce = nonce;
  a.bw = bw;
  a.has_walls = true;
  a.walls = {63, 63, cal::kWallLeave, 50, 45, 23, 22, -60};
  a.legacy = 40;
  return a;
}

}  // namespace

TEST(cal_write_maps_leave_sentinels_and_scales_eff) {
  cal::ApplyCmd a = walls_apply(7, 40);
  a.has_eff = true;
  a.eff_x1000 = {930, 0, 840, 800, 760, 680, 630, 600};
  const CalWrite w = cal_write_for(a, cal::kFlagStbc);
  CHECK(w.bw == 40 && w.has_walls && w.has_eff);
  CHECK(w.walls[0] == 63 && w.walls[7] == -60);
  CHECK(w.walls[2] == rc::kWallUndetermined);
  CHECK(w.legacy_wall == 40);
  CHECK(w.eff[0] == 0.93 && w.eff[1] == 0.0 && w.eff[7] == 0.6);
  CHECK(w.cal_flags == "stbc");

  cal::ApplyCmd b;
  b.legacy = cal::kWallLeave;
  CHECK(cal_write_for(b, 0).legacy_wall == rc::kWallUndetermined);
}

TEST(cal_flags_name_inverts_cal_flags_mask) {
  for (const char* s : {"ldpc+stbc", "stbc", "ldpc", "none"}) {
    const auto mask = cal_flags_mask(s);
    REQUIRE(mask.has_value());
    CHECK(cal_flags_name(*mask) == s);
  }
}

TEST(verify_parks_are_wall_minus_margin_for_written_walls_only) {
  const cal::ApplyCmd a = walls_apply(7, 20);
  const auto p = verify_parks(a, 1.0);  // 4 steps
  CHECK(p[0] == 59 && p[1] == 59);
  CHECK(p[2] == cal::kWallLeave);
  CHECK(p[3] == 46 && p[5] == 19);
  CHECK(p[7] == -64);  // -60 - 4: on the stamp's floor
  CHECK(p[3] == 50 - margin_steps(1.0));

  // An odd margin rounds the same way make_power_plan does.
  CHECK(verify_parks(a, 0.6)[4] == 45 - margin_steps(0.6));

  cal::ApplyCmd eff_only;
  eff_only.has_eff = true;
  eff_only.walls = {10, 10, 10, 10, 10, 10, 10, 10};  // ignored without has_walls
  for (int v : verify_parks(eff_only, 1.0)) CHECK(v == cal::kWallLeave);
}

TEST(ledger_caches_applied_per_nonce_and_width) {
  CalApplyLedger l;
  CHECK(!l.cached(1, 20).has_value());
  CHECK(l.want_backup(1));

  l.record(cal::Applied{1, 20, false, "walls out of range"});
  REQUIRE(l.cached(1, 20).has_value());
  CHECK(!l.cached(1, 20)->ok && l.cached(1, 20)->reason == "walls out of range");
  CHECK(!l.cached(1, 40).has_value());
  CHECK(l.want_backup(1));  // a failed apply wrote nothing to back up

  l.record(cal::Applied{1, 40, true, ""});
  CHECK(l.cached(1, 40).has_value() && l.cached(1, 40)->ok);
  CHECK(!l.want_backup(1));  // the run's backup is taken

  // A later apply of the same width replaces the cached answer.
  l.record(cal::Applied{1, 20, true, ""});
  CHECK(l.cached(1, 20)->ok);

  // Another nonce: nothing cached for it, and it wants its own backup.
  CHECK(!l.cached(2, 20).has_value());
  CHECK(l.want_backup(2));
  l.record(cal::Applied{2, 20, true, ""});
  CHECK(!l.cached(1, 40).has_value());  // the old run's answers are gone
  CHECK(!l.want_backup(2));
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
  CHECK(n[0] == "radio: no 20 MHz walls, flying the 40 MHz table at 20");

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
cal::SweepCmd power_sweep(uint32_t nonce) {
  cal::SweepCmd c;
  c.nonce = nonce;
  c.kind = cal::Kind::Power;
  return c;
}
}  // namespace

// The restore is owed from the first device write of a sweep until no run
// is open, however briefly the run lived: a SWEEP and an ABORT drained in
// the same pass open and close a run before the owner looks at active().
TEST(restore_is_owed_even_for_a_run_opened_and_closed_in_one_pass) {
  NullPower pwr;
  CalSweep s(CalSweepCfg{});
  CalRestoreOwed owed;
  CHECK(!owed.take_due(s.active()));  // nothing touched yet

  const auto c = power_sweep(7);
  REQUIRE(s.starts_sweep(c));
  owed.note_device_touched();
  s.on_cmd(c, 0, pwr);
  s.on_abort(cal::Abort{7}, 1, pwr);
  REQUIRE(!s.active());
  CHECK(owed.take_due(s.active()));
  CHECK(!owed.take_due(s.active()));  // once

  // A run still open owes its restore until it closes.
  const auto d = power_sweep(8);
  owed.note_device_touched();
  s.on_cmd(d, 10, pwr);
  REQUIRE(s.active());
  CHECK(!owed.take_due(s.active()));
  s.on_abort(cal::Abort{8}, 11, pwr);
  CHECK(owed.take_due(s.active()));

  // A refused sweep (anchor unreadable) never opens a run but still owes it.
  pwr.anchor = -1;
  owed.note_device_touched();
  s.on_cmd(power_sweep(9), 20, pwr);
  REQUIRE(!s.active());
  CHECK(owed.take_due(s.active()));
}

MTEST_MAIN
