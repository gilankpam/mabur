// LivePowerTable: which width's power plan is live, and how calibration
// takes the table over and hands it back. CalEndEdge: the agent thread's
// calibration falling edge.
#include "live_power_table.h"

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "mtest.h"
#include "power_tables.h"

using namespace mabur;

namespace {

// Records every device call, and whether it ran inside the retune gate.
struct FakeDevice : PowerTableDevice {
  std::vector<std::string> calls;
  bool in_gate = false;
  bool ungated_call = false;
  bool refuse_diffs = false;
  std::optional<devourer::TxRateDiffsQdb> last_diffs;
  bool diffs_written = false;

  void gated(const std::function<void()>& f) override {
    calls.push_back("gate");
    in_gate = true;
    f();
    in_gate = false;
  }
  bool set_rate_diffs(const std::optional<devourer::TxRateDiffsQdb>& d) override {
    note(d ? "diffs mcs5=" + std::to_string(d->mcs[5]) : std::string("diffs efuse"));
    last_diffs = d;
    diffs_written = true;
    return !refuse_diffs;
  }
  void set_offset_qdb(int q) override { note("offset " + std::to_string(q)); }
  void set_index_override(int i) override { note("override " + std::to_string(i)); }
  bool set_ampdu(bool on) {
    note(on ? "ampdu on" : "ampdu off");
    return true;
  }
  void note(std::string s) {
    if (!in_gate) ungated_call = true;
    calls.push_back(std::move(s));
  }
  void clear() { calls.clear(); }
};

struct Rig {
  FakeDevice dev;
  std::atomic<bool> ready{true};
  std::atomic<bool> cal_active{false};
  std::atomic<uint32_t> cal_epoch{0};
  LivePowerTable t;
  Rig() {
    t.dev = &dev;
    t.ready = &ready;
    t.cal_active = &cal_active;
    t.cal_epoch = &cal_epoch;
  }
};

// Two tables that differ on mcs5, so which one went live is visible.
WidthPlans both() {
  RadioCfg r;
  r.rate_walls_rel_20 = std::array<int, 8>{63, 63, 63, 63, 45, 23, 22, 21};
  r.rate_walls_rel_40 = std::array<int, 8>{63, 63, 63, 63, 45, 28, 28, 23};
  return make_width_plans(r, 1.0, 53);
}
WidthPlans only20() {
  RadioCfg r;
  r.rate_walls_rel_20 = std::array<int, 8>{63, 63, 63, 63, 45, 23, 22, 21};
  return make_width_plans(r, 1.0, 53);
}
int mcs5(const WidthPlans& p, uint8_t bw) { return plan_for(p, bw)->mcs[5]; }
std::string diffs_of(const WidthPlans& p, uint8_t bw) {
  return "diffs mcs5=" + std::to_string(mcs5(p, bw));
}

using Calls = std::vector<std::string>;

}  // namespace

TEST(follow_programs_a_new_width_once) {
  Rig r;
  const WidthPlans p = both();
  REQUIRE(mcs5(p, 20) != mcs5(p, 40));
  REQUIRE(r.t.replace(p, 20, true, false));
  CHECK((r.dev.calls == Calls{"gate", diffs_of(p, 20), "offset 0"}));
  r.dev.clear();
  r.t.follow(20);  // already live
  CHECK(r.dev.calls.empty());
  r.t.follow(40);
  CHECK((r.dev.calls == Calls{"gate", diffs_of(p, 40)}));
  CHECK(!r.dev.ungated_call);
}

TEST(follow_is_a_no_op_while_calibrating) {
  Rig r;
  const WidthPlans p = both();
  REQUIRE(r.t.replace(p, 20, true, false));
  r.dev.clear();
  r.t.begin_cal();
  CHECK(r.cal_active.load());
  CHECK(r.cal_epoch.load() == 1);
  r.t.follow(40);
  r.t.follow(20);
  CHECK(r.dev.calls.empty());
}

TEST(follow_puts_back_a_table_calibration_changed) {
  // A sweep zeroes the diffs behind the table's back; once calibration lets
  // go, follow() rewrites the plan even at the width it already names.
  Rig r;
  const WidthPlans p = both();
  REQUIRE(r.t.replace(p, 20, true, false));
  r.t.begin_cal();
  r.cal_active = false;
  r.dev.clear();
  r.t.follow(20);
  CHECK((r.dev.calls == Calls{"gate", diffs_of(p, 20)}));
  r.dev.clear();
  r.t.follow(20);  // clean again: nothing more to write
  CHECK(r.dev.calls.empty());
}

TEST(follow_puts_back_the_efuse_table_with_no_plans) {
  // power_mode "none": never written, except to undo a calibration.
  Rig r;
  r.t.follow(20);
  CHECK(r.dev.calls.empty() || !r.dev.diffs_written);
  r.dev.clear();
  r.t.begin_cal();
  r.cal_active = false;
  r.t.follow(20);
  CHECK((r.dev.calls == Calls{"gate", "diffs efuse"}));
}

TEST(follow_waits_for_the_device) {
  Rig r;
  r.ready = false;
  r.t.set_plans(both());
  r.t.follow(20);
  CHECK(r.dev.calls.empty());
  r.ready = true;
  r.t.follow(20);
  CHECK(!r.dev.calls.empty());
}

TEST(restore_after_a_run_clears_the_override_in_the_same_gate_hold) {
  Rig r;
  const WidthPlans p = both();
  REQUIRE(r.t.replace(p, 40, true, false));
  r.t.begin_cal();
  r.cal_active = false;
  r.dev.clear();
  REQUIRE(r.t.replace(p, 40, /*zero_offset=*/true, /*clear_override=*/true));
  CHECK((r.dev.calls == Calls{"gate", diffs_of(p, 40), "offset 0", "override -1"}));
  CHECK(!r.dev.ungated_call);
  r.dev.clear();
  r.t.follow(40);  // the restore left it clean
  CHECK(r.dev.calls.empty());
}

TEST(restore_with_no_plans_writes_the_efuse_table_and_clears_the_override) {
  Rig r;
  r.t.begin_cal();
  r.cal_active = false;
  REQUIRE(r.t.replace(WidthPlans{}, 20, true, true));
  CHECK((r.dev.calls == Calls{"gate", "diffs efuse", "override -1"}));
}

TEST(program_for_cal_flies_the_fallback_or_the_efuse_table) {
  Rig r;
  // Only a 20 MHz table: a 40 MHz capacity sweep flies it, as production does.
  const WidthPlans p = only20();
  REQUIRE(r.t.replace(p, 20, true, false));
  r.t.begin_cal();
  r.dev.clear();
  REQUIRE(r.t.program_for_cal(40));
  CHECK((r.dev.calls == Calls{"gate", diffs_of(p, 20)}));
  // ...and calibration still owns the table: the run's end puts it back.
  r.cal_active = false;
  r.dev.clear();
  r.t.follow(20);
  CHECK((r.dev.calls == Calls{"gate", diffs_of(p, 20)}));

  // No plan for any width: the chip's own table, written even though the
  // table was never dirtied by a plan.
  Rig n;
  n.t.begin_cal();
  REQUIRE(n.t.program_for_cal(40));
  CHECK((n.dev.calls == Calls{"gate", "diffs efuse"}));
  // Whatever state the table is in: a capacity point flies what production
  // would, and with no plan that is the chip's own table.
  Rig m;
  REQUIRE(m.t.program_for_cal(20));
  CHECK((m.dev.calls == Calls{"gate", "diffs efuse"}));
}

TEST(replace_unless_cal_refuses_during_calibration) {
  Rig r;
  const WidthPlans p = both();
  REQUIRE(r.t.replace(p, 20, true, false));
  r.t.begin_cal();
  r.dev.clear();
  bool refused = false;
  CHECK(!r.t.replace_unless_cal(only20(), 20, &refused));
  CHECK(refused);
  CHECK(r.dev.calls.empty());
  // The refused plans were not taken either: the run's end restores the
  // plans it had.
  r.cal_active = false;
  r.t.follow(40);
  CHECK((r.dev.calls == Calls{"gate", diffs_of(p, 40)}));

  r.dev.clear();
  CHECK(r.t.replace_unless_cal(only20(), 20, &refused));
  CHECK(!refused);
  CHECK((r.dev.calls == Calls{"gate", diffs_of(only20(), 20)}));
}

TEST(a_refused_write_is_reported_and_not_retried) {
  Rig r;
  r.dev.refuse_diffs = true;
  CHECK(!r.t.replace(both(), 20, true, false));
  r.dev.clear();
  r.t.follow(20);
  CHECK(r.dev.calls.empty());
}

TEST(the_production_ampdu_write_is_refused_under_calibration) {
  Rig r;
  int written = 0;
  r.t.unless_cal([&] { written += r.dev.set_ampdu(true); });
  CHECK(written == 1);
  r.t.begin_cal();
  r.t.unless_cal([&] { written += r.dev.set_ampdu(true); });
  CHECK(written == 1);
  // Calibration's own write still goes through, under the same lock.
  r.t.with_lock([&] { written += r.dev.set_ampdu(false); });
  CHECK(written == 2);
  r.cal_active = false;
  r.t.unless_cal([&] { written += r.dev.set_ampdu(true); });
  CHECK(written == 3);
}

TEST(cal_end_edge_fires_once_per_run) {
  Rig r;
  CalEndEdge e;
  auto poll = [&] { return e.poll(r.cal_epoch.load(), r.cal_active.load()); };
  CHECK(!poll());
  r.t.begin_cal();
  CHECK(!poll());  // calibrating
  CHECK(!poll());
  r.cal_active = false;
  CHECK(poll());  // the falling edge
  CHECK(!poll());  // once
}

TEST(cal_end_edge_catches_a_run_too_short_to_see) {
  // A sweep refused, or a command the engine ignored: cal_active went up
  // and down between two polls. The epoch still moved, so the device
  // writes made for it are undone.
  Rig r;
  CalEndEdge e;
  auto poll = [&] { return e.poll(r.cal_epoch.load(), r.cal_active.load()); };
  CHECK(!poll());
  r.t.begin_cal();
  r.cal_active = false;
  CHECK(poll());
  CHECK(!poll());
  // Two short runs between polls are one edge.
  r.t.begin_cal();
  r.cal_active = false;
  r.t.begin_cal();
  r.cal_active = false;
  CHECK(poll());
  CHECK(!poll());
}

MTEST_MAIN
