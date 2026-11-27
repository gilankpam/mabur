// LivePowerTable: which (channel, width)'s power plan is live, and how
// calibration takes the table over and hands it back. CalEndEdge: the agent
// thread's calibration falling edge.
#include "live_power_table.h"

#include <unistd.h>

#include <array>
#include <cstdio>
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
    t.set_channel(144);
  }
  // The agent's retune: the table's lock, then (in maburd) the gate.
  void retune(uint8_t ch) {
    t.with_lock([&] { t.retuned_in_gate(ch); });
  }
};

PowerPlan plan(int mcs5) {
  return make_power_plan({63, 63, 63, 63, 45, mcs5, 22, 21}, 63, 1.0, 53);
}
// Two widths of channel 144 that differ on mcs5, so which one went live is
// visible.
ChannelPlans both() {
  ChannelPlans p;
  p.by[{144, 20}] = plan(23);
  p.by[{144, 40}] = plan(28);
  return p;
}
ChannelPlans only20() {
  ChannelPlans p;
  p.by[{144, 20}] = plan(23);
  return p;
}
// Two channels at 20 MHz that differ on mcs5; 64 and 40 not measured.
ChannelPlans two_channels() {
  ChannelPlans p;
  p.by[{144, 20}] = plan(23);
  p.by[{112, 20}] = plan(30);
  return p;
}
int mcs5(const ChannelPlans& p, uint8_t ch, uint8_t bw) {
  return plan_for(p, ch, bw)->mcs[5];
}
std::string diffs_of(const ChannelPlans& p, uint8_t bw, uint8_t ch = 144) {
  return "diffs mcs5=" + std::to_string(mcs5(p, ch, bw));
}

// What the table logs while `f` runs.
template <class F>
std::string stderr_of(F&& f) {
  std::fflush(stderr);
  FILE* tmp = std::tmpfile();
  const int saved = ::dup(2);
  ::dup2(::fileno(tmp), 2);
  f();
  std::fflush(stderr);
  ::dup2(saved, 2);
  ::close(saved);
  std::rewind(tmp);
  std::string out;
  for (int c; (c = std::fgetc(tmp)) != EOF;) out.push_back(static_cast<char>(c));
  std::fclose(tmp);
  return out;
}

using Calls = std::vector<std::string>;

}  // namespace

TEST(follow_programs_a_new_width_once) {
  Rig r;
  const ChannelPlans p = both();
  REQUIRE(mcs5(p, 144, 20) != mcs5(p, 144, 40));
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
  const ChannelPlans p = both();
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
  const ChannelPlans p = both();
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
  const ChannelPlans p = both();
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
  REQUIRE(r.t.replace(ChannelPlans{}, 20, true, true));
  CHECK((r.dev.calls == Calls{"gate", "diffs efuse", "override -1"}));
}

TEST(program_for_cal_flies_the_plan_or_the_efuse_table) {
  Rig r;
  // Only a 20 MHz table: a 20 MHz capacity sweep flies it, a 40 MHz one the
  // chip's own table, as production does.
  const ChannelPlans p = only20();
  REQUIRE(r.t.replace(p, 20, true, false));
  r.t.begin_cal();
  r.dev.clear();
  REQUIRE(r.t.program_for_cal(40));
  CHECK((r.dev.calls == Calls{"gate", "diffs efuse"}));
  r.dev.clear();
  REQUIRE(r.t.program_for_cal(20));
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
  const ChannelPlans p = both();
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

TEST(follows_the_channel_on_a_retune) {
  Rig r;
  const ChannelPlans p = two_channels();
  REQUIRE(mcs5(p, 144, 20) != mcs5(p, 112, 20));
  REQUIRE(r.t.replace(p, 20, true, false));
  CHECK(r.dev.last_diffs && r.dev.last_diffs->mcs[5] == mcs5(p, 144, 20));
  r.dev.clear();
  r.retune(112);
  // Written in the caller's gate hold: no second take of the gate.
  CHECK((r.dev.calls == Calls{diffs_of(p, 20, 112)}));
  CHECK(r.t.channel() == 112);
  r.dev.clear();
  r.t.follow(20);  // already live
  CHECK(r.dev.calls.empty());
}

TEST(a_hop_to_an_unmeasured_channel_restores_the_chip_table) {
  // Leaving 144's diffs on 64's anchor would fly walls nobody measured.
  Rig r;
  const ChannelPlans p = two_channels();
  REQUIRE(r.t.replace(p, 20, true, false));
  r.dev.clear();
  const std::string log = stderr_of([&] { r.retune(64); });
  CHECK((r.dev.calls == Calls{"diffs efuse"}));
  CHECK(log.find("ch 64 20 MHz: chip's own table") != std::string::npos);
  r.dev.clear();
  r.retune(40);  // also unmeasured: the chip is on its own table already
  CHECK(r.dev.calls.empty());
  // Once per channel and width: a second visit writes, but does not log.
  r.retune(144);
  r.dev.clear();
  const std::string again = stderr_of([&] { r.retune(64); });
  CHECK((r.dev.calls == Calls{"diffs efuse"}));
  CHECK(again.find("chip's own table") == std::string::npos);
}

TEST(a_hop_back_reprograms_the_plan) {
  Rig r;
  const ChannelPlans p = two_channels();
  REQUIRE(r.t.replace(p, 20, true, false));
  r.retune(64);
  r.dev.clear();
  r.retune(144);
  CHECK((r.dev.calls == Calls{diffs_of(p, 20, 144)}));
}

TEST(a_width_without_its_own_plan_flies_the_chip_table) {
  // A channel measured only at 20 MHz: at 40 it flies the chip's own table,
  // never the 20 MHz plan.
  Rig r;
  const ChannelPlans p = only20();
  REQUIRE(r.t.replace(p, 20, true, false));
  r.dev.clear();
  const std::string log = stderr_of([&] { r.t.follow(40); });
  CHECK((r.dev.calls == Calls{"gate", "diffs efuse"}));
  CHECK(log.find("ch 144 40 MHz: chip's own table") != std::string::npos);
}

TEST(a_retune_during_calibration_only_records) {
  Rig r;
  const ChannelPlans p = two_channels();
  REQUIRE(r.t.replace(p, 20, true, false));
  r.t.begin_cal();
  r.dev.clear();
  r.retune(112);
  CHECK(r.dev.calls.empty());
  CHECK(r.t.channel() == 112);
  // The run's end programs the recorded channel.
  r.cal_active = false;
  r.t.follow(20);
  CHECK((r.dev.calls == Calls{"gate", diffs_of(p, 20, 112)}));
}

TEST(a_retune_before_bring_up_only_records) {
  Rig r;
  ChannelPlans p = two_channels();
  p.by[{112, 40}] = plan(28);
  r.t.set_plans(p);  // plans held, but nothing programmed yet
  r.retune(112);
  CHECK(r.dev.calls.empty());
  CHECK(r.t.channel() == 112);
  REQUIRE(r.t.replace(p, 20, true, false));
  CHECK((r.dev.calls == Calls{"gate", diffs_of(p, 20, 112), "offset 0"}));
}

TEST(offset_with_an_empty_cache_flies_the_chip_table) {
  Rig r;
  CHECK(r.t.replace(ChannelPlans{}, 20, true, false));
  int own = 0;
  for (const auto& c : r.dev.calls) {
    own += c == "diffs efuse";
    CHECK(c.rfind("diffs mcs5", 0) != 0);
  }
  CHECK(own <= 1);
  r.dev.clear();
  r.t.follow(40);
  r.retune(112);
  CHECK(!r.dev.diffs_written);
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
