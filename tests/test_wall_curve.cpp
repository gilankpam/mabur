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
// Measured at 40 MHz while something else was on the air: two points jump up and the curve
// falls back 8.6 dB. Every other rule passes it, and its first -17 dB crossing is 19 index
// steps below the undisturbed one.
const int kDisturbed[25] = {-279,-292,-277,-201,-138,-224,-171,-189,-174,-157,-145,-123,-124,
                            -99,-108,-95,-84,-89,-84,-82,-79,-77,-75,-75,-73};
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
TEST(no_crossing_is_63) {  // floor above mcs7's threshold: nothing to cross, not a clamp (anchor 0 would give <63)
  const int v[25] = {-165,-165,-165,-165,-165,-165,-165,-165,-165,-165,-165,-165,-165,
                     -165,-165,-165,-165,-165,-165,-165,-165,-165,-165,-165,-80};
  CHECK(walls_from_curve(curve(v, 0), true)[7] == 63);
}
TEST(validity_boundaries_are_inclusive) {
  auto twenty = curve(k144_20, 39);  for (int k = 10; k < 15; ++k) twenty.evm_db[k].reset();
  CHECK(check_curve(twenty) == cal::WallResult::Ok);               // 20 of 25; fails if the count test is <= 20
  auto floor26 = curve(k144_20, 39);  for (auto& v : floor26.evm_db) if (*v < -26.0f) v = -26.0f;
  CHECK(check_curve(floor26) == cal::WallResult::Ok);              // fails if the floor test is >= -26
  auto last11 = curve(k144_20, 39);  for (auto& v : last11.evm_db) if (*v > -11.0f) v = -11.0f;
  CHECK(check_curve(last11) == cal::WallResult::Ok);               // fails if the last-point test is <= -11
}

TEST(a_curve_that_falls_back_is_disturbed) {
  CHECK(check_curve(curve(kDisturbed, 53)) == cal::WallResult::Disturbed);
}
TEST(a_fall_back_of_2_dB_is_tolerated_and_more_is_not) {
  auto c = curve(k144_20, 39);     // -16.8 at index 60, then -15.4
  c.evm_db[9] = -18.8f;
  CHECK(check_curve(c) == cal::WallResult::Ok);          // fails if the test is >= 2 dB
  c.evm_db[9] = -18.9f;
  CHECK(check_curve(c) == cal::WallResult::Disturbed);
}
TEST(scatter_under_the_lowest_threshold_is_not_a_disturbance) {  // the noise-limited floor moves by several dB
  auto c = curve(k144_20, 39);
  c.evm_db[1] = -22.0f;
  c.evm_db[2] = -32.0f;
  CHECK(check_curve(c) == cal::WallResult::Ok);
}

MTEST_MAIN
