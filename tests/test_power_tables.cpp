#include "mtest.h"
#include "power_tables.h"
#include "config.h"

using namespace mabur;

namespace {
// A measured (144, 20) curve, anchor 39 (test_wall_curve.cpp): walls
// {63,63,63,63,44,26,24,20} with LDPC, mcs7 11 without.
const int k144_20[25] = {-306,-302,-296,-285,-241,-221,-200,-184,-168,-154,-144,-133,-124,
                         -116,-109,-103,-98,-94,-91,-89,-87,-86,-84,-83,-81};
WallCurve curve(const int (&v)[25], int anchor) {
  WallCurve c;
  c.anchor = anchor;
  for (int k = 0; k < 25; ++k) c.evm_db[k] = v[k] / 10.0f;
  return c;
}
WallCache cache144() {
  WallCache c;
  c.entries[{144, 20}] = curve(k144_20, 39);
  return c;
}
RadioCfg offset() {
  RadioCfg r;
  r.power_mode = "offset";
  return r;
}
}  // namespace

TEST(plans_come_from_the_cache_one_per_entry) {
  const ChannelPlans p = make_channel_plans(cache144(), offset(), 1.0, true);
  CHECK(p.by.size() == 1);
  REQUIRE(p.by.count({144, 20}) == 1);
  const PowerPlan& pl = p.by.at({144, 20});
  CHECK(pl.mcs[7] == 16);  // wall 20, margin 1.0 dB = 4 steps
  CHECK(pl.mcs[4] == 40);
  CHECK(pl.legacy == 59);  // legacy_wall_rel 63 - 4
}

TEST(ldpc_off_lowers_the_plans) {
  const ChannelPlans p = make_channel_plans(cache144(), offset(), 1.0, false);
  CHECK(p.by.at({144, 20}).mcs[7] == 7);
}

TEST(a_larger_margin_lowers_the_plans) {
  const ChannelPlans p = make_channel_plans(cache144(), offset(), 2.0, true);
  CHECK(p.by.at({144, 20}).mcs[7] == 12);
}

TEST(power_mode_none_has_no_plans) {
  RadioCfg r;
  r.power_mode = "none";
  CHECK(make_channel_plans(cache144(), r, 1.0, true).by.empty());
}

TEST(lookup_order) {
  ChannelPlans p;
  p.by[{144, 20}] = make_power_plan({63, 63, 63, 63, 45, 23, 22, 21}, 63, 1.0, 0);
  p.by[{112, 40}] = make_power_plan({63, 63, 63, 63, 45, 28, 28, 23}, 63, 1.0, 0);
  CHECK(plan_for(p, 144, 20).source == PlanSource::Own);
  CHECK((plan_for(p, 144, 20).plan == &p.by[{144, 20}]));
  CHECK((plan_for(p, 144, 40).source == PlanSource::OtherWidth &&
         plan_for(p, 144, 40).plan == &p.by[{144, 20}]));
  CHECK((plan_for(p, 112, 20).source == PlanSource::OtherWidth &&
         plan_for(p, 112, 20).plan == &p.by[{112, 40}]));
  CHECK(plan_for(p, 64, 20).source == PlanSource::None && plan_for(p, 64, 20).plan == nullptr);
}

MTEST_MAIN
