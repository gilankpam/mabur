#include <cmath>
#include "mtest.h"
#include "vectors.h"
#include "energy.h"
using namespace maburgs;

TEST(energy_matches_python_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/energy.json");
  for (auto& c : j["cases"]) {
    TxPoint p{c["vht"], c["mcs"], c["bw"], c["sgi"], c["txagc"]};
    CHECK(phy_rate_eff_bps(p, c["payload"]) == c["eff_bps"].get<double>());
    CHECK(airtime_fraction(p, c["src"], c["ov"], c["payload"]) ==
          c["airtime"].get<double>());
    double eb = energy_per_delivered_bit(p, c["src"], c["ov"], c["payload"],
                                         c["p_deliver"]);
    if (c["e_bit"].is_null()) CHECK(std::isinf(eb));
    else CHECK(eb == c["e_bit"].get<double>());
  }
  for (auto& g : j["gain"])
    CHECK(min_txagc_for_gain(g["need_db"]) == g["idx"].get<int>());
  for (auto& b : j["bw_noise"])
    CHECK(bw_noise_db(b["bw"]) == b["db"].get<double>());
}
MTEST_MAIN
