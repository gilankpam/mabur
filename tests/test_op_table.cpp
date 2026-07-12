#include <cmath>
#include "mtest.h"
#include "vectors.h"
#include "op_table.h"
using namespace maburgs;

TEST(snr_required_matches_python) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/optable.json");
  LinkTable lt;
  for (auto& c : j["snr_req"])
    CHECK(lt.snr_required(c["mcs"], c["ov"], c["target"]) == c["req"].get<double>());
}

TEST(build_rows_matches_python) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/optable.json");
  LinkTable lt;
  auto rows = build_link_rows(lt, 0.99, {0, 1, 2, 3, 4, 5, 6, 7},
                              {0.10, 0.25, 0.50, 0.75, 1.00}, 20, false, {}, false);
  REQUIRE(rows.size() == j["rows"].size());
  for (size_t i = 0; i < rows.size(); ++i) {
    CHECK(rows[i].mcs == j["rows"][i]["mcs"].get<int>());
    CHECK(rows[i].overhead == j["rows"][i]["ov"].get<double>());
    CHECK(rows[i].snr_req == j["rows"][i]["snr_req"].get<double>());
  }
}

TEST(resolve_matches_python) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/optable.json");
  LinkTable lt;
  for (auto& c : j["resolve"]) {
    LinkRow r{c["row"]["vht"], c["row"]["mcs"], c["row"]["bw"], c["row"]["sgi"],
              c["row"]["ov"], c["row"]["snr_req"]};
    auto op = resolve(r, c["pl"], lt, 1024, 4e6, 2.0);
    if (c["op"].is_null()) { CHECK(!op.has_value()); continue; }
    REQUIRE(op.has_value());
    CHECK(op->txagc == c["op"]["txagc"].get<int>());
    CHECK(op->p_deliver == c["op"]["p_deliver"].get<double>());
    if (c["op"]["e_bit"].is_null()) CHECK(std::isinf(op->e_bit));
    else CHECK(op->e_bit == c["op"]["e_bit"].get<double>());
  }
}
MTEST_MAIN
