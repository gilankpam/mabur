#include <cmath>
#include "mtest.h"
#include "vectors.h"
#include "energy.h"
using namespace maburgs;

TEST(gain_is_linear_qdb) {
  CHECK(maburgs::gain_db(0) == 0.0);
  CHECK(maburgs::gain_db(16) == 4.0);
  CHECK(maburgs::gain_db(-40) == -10.0);
}
TEST(min_offset_for_gain) {
  CHECK(maburgs::min_offset_qdb_for_gain(4.0) == 16);
  CHECK(maburgs::min_offset_qdb_for_gain(3.9) == 16);   // ceil
  CHECK(maburgs::min_offset_qdb_for_gain(-2.0) == -8);
}
TEST(pa_w_maps_offset_over_base) {
  CHECK(maburgs::pa_w(0, 53) == maburgs::pa_w_index(53));   // helper reading kPaW directly
  CHECK(maburgs::pa_w(20, 53) == maburgs::pa_w_index(63));  // clamps at table edge
}

TEST(energy_matches_python_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/energy.json");
  for (auto& c : j["cases"]) {
    TxPoint p{c["vht"], c["mcs"], c["bw"], c["sgi"], c["pwr_offset_qdb"],
             c["base_ref_idx"]};
    CHECK(phy_rate_eff_bps(p, c["payload"]) == c["eff_bps"].get<double>());
    CHECK(airtime_fraction(p, c["src"], c["ov"], c["payload"]) ==
          c["airtime"].get<double>());
    double eb = energy_per_delivered_bit(p, c["src"], c["ov"], c["payload"],
                                         c["p_deliver"]);
    if (c["e_bit"].is_null()) CHECK(std::isinf(eb));
    else CHECK(eb == c["e_bit"].get<double>());
  }
  for (auto& g : j["gain"])
    CHECK(min_offset_qdb_for_gain(g["need_db"]) == g["idx"].get<int>());
  for (auto& b : j["bw_noise"])
    CHECK(bw_noise_db(b["bw"]) == b["db"].get<double>());
}
MTEST_MAIN
