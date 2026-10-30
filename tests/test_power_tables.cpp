#include "mtest.h"
#include "power_tables.h"
#include "config.h"

using namespace mabur;

TEST(plan_for_missing_width_uses_the_other) {
  RadioCfg r;
  r.rate_walls_rel_20 = std::array<int, 8>{63, 63, 63, 63, 45, 23, 22, 21};
  // No rate_walls_rel_40.
  auto wp = make_width_plans(r, /*margin_db=*/1.0, /*anchor_idx=*/0);
  CHECK(wp.p20.has_value());
  CHECK(!wp.p40.has_value());
  const PowerPlan* p40 = plan_for(wp, 40);
  const PowerPlan* p20 = plan_for(wp, 20);
  REQUIRE(p40 != nullptr);
  REQUIRE(p20 != nullptr);
  CHECK(p40 == &*wp.p20);  // 40 falls back to the 20 plan
  CHECK(p20 == &*wp.p20);
}

TEST(each_width_plan_uses_its_own_table) {
  RadioCfg r;
  r.rate_walls_rel_20 = std::array<int, 8>{63, 63, 63, 63, 45, 23, 22, 21};
  r.rate_walls_rel_40 = std::array<int, 8>{63, 63, 63, 63, 45, 28, 28, 23};
  auto wp = make_width_plans(r, /*margin_db=*/1.0, /*anchor_idx=*/0);
  REQUIRE(wp.p20.has_value());
  REQUIRE(wp.p40.has_value());
  const PowerPlan* p20 = plan_for(wp, 20);
  const PowerPlan* p40 = plan_for(wp, 40);
  REQUIRE(p20 != nullptr);
  REQUIRE(p40 != nullptr);
  CHECK(p20->mcs[5] == 19);  // 23 - 4 (margin 1.0 -> m=4)
  CHECK(p40->mcs[5] == 24);  // 28 - 4
}

TEST(cal_flags_mask_parses_all_four) {
  auto both = cal_flags_mask("ldpc+stbc");
  REQUIRE(both.has_value());
  CHECK(*both == 3);
  auto stbc = cal_flags_mask("stbc");
  REQUIRE(stbc.has_value());
  CHECK(*stbc == 2);
  auto ldpc = cal_flags_mask("ldpc");
  REQUIRE(ldpc.has_value());
  CHECK(*ldpc == 1);
  auto none = cal_flags_mask("none");
  REQUIRE(none.has_value());
  CHECK(*none == 0);
  CHECK(!cal_flags_mask("bogus").has_value());
}

MTEST_MAIN
