#include "mtest.h"
#include "wall_curve.h"

using namespace mabur;

namespace {
WallCurve curve(const int (&v)[25], int anchor) {
  WallCurve c;
  c.anchor = anchor;
  for (int k = 0; k < 25; ++k) c.evm_db[k] = v[k] / 10.0f;
  return c;
}
const int k144_20[25] = {-306,-302,-296,-285,-241,-221,-200,-184,-168,-154,-144,-133,-124,
                         -116,-109,-103,-98,-94,-91,-89,-87,-86,-84,-83,-81};      // anchor 39
const int k40_40[25]  = {-319,-320,-322,-311,-305,-307,-308,-307,-305,-302,-292,-282,-268,
                         -243,-219,-199,-176,-161,-148,-134,-123,-112,-101,-95,-86};  // anchor 75
}  // namespace

TEST(measured_curves_give_these_walls) {
  CHECK(walls_from_curve(curve(k144_20, 39), true)  == (std::array<int,8>{63,63,63,63,44,26,24,20}));
  CHECK(walls_from_curve(curve(k144_20, 39), false) == (std::array<int,8>{63,63,63,34,26,15,14,11}));
  CHECK(walls_from_curve(curve(k40_40, 75), true)   == (std::array<int,8>{63,63,63,47,37,24,22,18}));
  CHECK(walls_from_curve(curve(k40_40, 75), false)  == (std::array<int,8>{63,63,63,30,24,14,13,10}));
}
TEST(the_search_starts_at_the_floor) {      // k40_40's floor is its third point, not its first
  CHECK(*curve_floor_db(curve(k40_40, 75)) == -32.2f);
}
TEST(points_without_a_value_are_bridged) {  // drop the point at index 56 of k144_20: mcs7 crosses between 52 and 60
  auto c = curve(k144_20, 39);  c.evm_db[7].reset();
  CHECK(walls_from_curve(c, true)[7] == 20);    // 52 + 8*(-17.0+20.0)/3.2 = 59.5 -> 20
}
TEST(walls_are_clamped_to_the_diff_range) {
  CHECK(walls_from_curve(curve(k144_20, 0), true)[7] == 59);     // 59.5 - 0
  CHECK(walls_from_curve(curve(k144_20, 0), true)[4] == 63);     // 83.4 - 0, clamped
  CHECK(walls_from_curve(curve(k144_20, 127), true)[7] == -64);  // 59.5 - 127, clamped
}
TEST(validity_rules) {
  CHECK(check_curve(curve(k144_20, 39)) == cal::WallResult::Ok);
  auto few = curve(k144_20, 39);  for (int k = 0; k < 6; ++k) few.evm_db[k + 10].reset();
  CHECK(check_curve(few) == cal::WallResult::TooFewPoints);        // 19 of 25
  auto shallow = curve(k144_20, 39);  for (auto& v : shallow.evm_db) if (*v < -25.9f) v = -25.9f;
  CHECK(check_curve(shallow) == cal::WallResult::FloorTooHigh);
  auto flat = curve(k144_20, 39);  for (auto& v : flat.evm_db) if (*v > -11.1f) v = -11.1f;
  CHECK(check_curve(flat) == cal::WallResult::NotSaturated);
  auto tail = curve(k144_20, 39);  tail.evm_db[24].reset();        // the last point WITH a value decides
  CHECK(check_curve(tail) == cal::WallResult::Ok);
}

MTEST_MAIN
