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
  CHECK(walls_from_curve(curve(k144_20, 39), true)  == (std::array<int,8>{63,63,63,63,36,26,24,20}));
  CHECK(walls_from_curve(curve(k144_20, 39), false) == (std::array<int,8>{63,63,63,34,21,15,14,11}));
  CHECK(walls_from_curve(curve(k40_40, 75), true)   == (std::array<int,8>{63,63,63,47,32,24,22,18}));
  CHECK(walls_from_curve(curve(k40_40, 75), false)  == (std::array<int,8>{63,63,63,30,19,14,13,10}));
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
  CHECK(walls_from_curve(curve(k144_20, 0), true)[4] == 63);     // 75.6 - 0, clamped
  CHECK(walls_from_curve(curve(k144_20, 127), true)[7] == -64);  // 59.5 - 127, clamped
}
TEST(validity_rules) {
  CHECK(check_curve(curve(k144_20, 39)) == cal::WallResult::Ok);   // last -8.1
  auto few = curve(k144_20, 39);  few.evm_db[10].reset();
  CHECK(check_curve(few) == cal::WallResult::TooFewPoints);
  auto flat = curve(k144_20, 39);  flat.evm_db[24] = -11.5f;
  CHECK(check_curve(flat) == cal::WallResult::NotSaturated);
  auto shallow = curve(k144_20, 39);  for (auto& v : shallow.evm_db) if (*v < -20.0f) v = -20.0f;
  CHECK(check_curve(shallow) == cal::WallResult::Ok);              // no floor rule
  auto fall = curve(k144_20, 39);  fall.evm_db[10] = -15.0f;  fall.evm_db[11] = -20.0f;
  CHECK(check_curve(fall) == cal::WallResult::Ok);                 // no fall-back rule
}

TEST(an_early_dip_before_the_floor_is_ignored) {  // fails if the search starts at index 0: mcs4 would cross at 30, not 46
  const int v[25] = {-300,-50,-100,-320,-200,-80,-80,-80,-80,-80,-80,-80,-80,
                     -80,-80,-80,-80,-80,-80,-80,-80,-80,-80,-80,-80};
  CHECK(walls_from_curve(curve(v, 0), true)[4] == 46);
}
TEST(only_the_first_crossing_counts) {  // fails if the search keeps going after the first crossing: mcs4 would be 41
  const int v[25] = {-300,-200,-90,-140,-80,-80,-80,-80,-80,-80,-80,-80,-80,
                     -80,-80,-80,-80,-80,-80,-80,-80,-80,-80,-80,-80};
  CHECK(walls_from_curve(curve(v, 0), true)[4] == 34);
}
TEST(a_threshold_under_the_curve_takes_the_lowest_index_of_its_minimum) {  // -16.5 from 28 to 120
  const int v[25] = {-165,-165,-165,-165,-165,-165,-165,-165,-165,-165,-165,-165,-165,
                     -165,-165,-165,-165,-165,-165,-165,-165,-165,-165,-165,-80};
  CHECK(walls_from_curve(curve(v, 100), true)[7] == -64);   // 28 - 100, clamped
  CHECK(walls_from_curve(curve(v, 0), true)[7] == 28);      // fails if it takes the plateau's top (120 -> 63)
}
TEST(a_curve_that_never_reaches_a_threshold_gives_63) {
  const int v[25] = {-200,-200,-200,-200,-200,-200,-200,-200,-200,-200,-200,-200,-200,
                     -200,-200,-200,-200,-200,-200,-200,-200,-200,-200,-200,-200};
  const auto w = walls_from_curve(curve(v, 60), true);
  for (int m = 3; m < 8; ++m) CHECK(w[m] == 63);
}
TEST(a_weak_park_curve_uses_thresholds_2_5_db_lower) {
  WallCurve c;  c.anchor = 60;
  for (int k = 0; k < 25; ++k) c.evm_db[k] = -30.0f + k;
  auto at = [&](bool weak, bool ldpc) {
    c.weak_park = weak;
    const auto w = walls_from_curve(c, ldpc);
    return std::array<int,5>{w[3], w[4], w[5], w[6], w[7]};
  };
  CHECK(at(false, true)  == (std::array<int,5>{52, 38, 28, 26, 20}));
  CHECK(at(true, true)   == (std::array<int,5>{42, 28, 18, 16, 10}));
  CHECK(at(true, false)  == (std::array<int,5>{26, 12, 2, 0, -6}));
}
TEST(a_threshold_under_every_point_takes_the_lowest_index_of_the_minimum) {
  WallCurve c;  c.anchor = 60;  c.weak_park = true;
  for (int k = 0; k < 19; ++k) c.evm_db[k] = -23.0f;
  const float rest[6] = {-20, -17, -14, -11, -8, -8};
  for (int k = 19; k < 25; ++k) c.evm_db[k] = rest[k - 19];
  const auto w = walls_from_curve(c, false);
  CHECK(w[7] == 28 - 60);   // mcs7 -23.5 is under the -23.0 plateau (28 to 100)
  CHECK(w[6] == 41);        // mcs6 -22.0 crosses at 100 + 4/3
}
TEST(validity_boundaries_are_inclusive) {
  auto last11 = curve(k144_20, 39);  for (auto& v : last11.evm_db) if (*v > -11.0f) v = -11.0f;
  CHECK(check_curve(last11) == cal::WallResult::Ok);               // fails if the last-point test is <= -11
}

MTEST_MAIN
