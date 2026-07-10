#include "mtest.h"
#include "vectors.h"
#include "mabur/rs_encoder.h"
using namespace mabur;
static std::vector<std::string> run_stream(RsEncoder& e, const nlohmann::json& pkts) {
  std::vector<std::string> out;
  for (auto& ph : pkts) {
    auto p = mtest::unhex(ph.get<std::string>());
    for (auto& env : e.add_packet(p.data(), p.size())) out.push_back(mtest::hex(env));
  }
  return out;
}
TEST(rs_matches_python_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rs.json");
  for (auto& c : j["cases"]) {
    RsEncoder enc(RsConfig{c["k"], c["symbol_size"], c["overhead"]});
    auto stream = run_stream(enc, c["packets"]);
    REQUIRE(stream.size() == c["stream"].size());
    for (size_t i = 0; i < stream.size(); ++i) CHECK(stream[i] == c["stream"][i].get<std::string>());
    size_t fi = 0;
    for (auto& env : enc.flush()) CHECK(mtest::hex(env) == c["flush"][fi++].get<std::string>());
    CHECK(fi == c["flush"].size());
  }
}
TEST(rs_config_math) {
  RsConfig cfg1{8, 64, 0.25};
  RsConfig cfg2{8, 64, 1.0};
  CHECK(cfg1.repair_count() == 2);
  CHECK(cfg2.n() == 16);
  CHECK(cfg1.max_packet_size() == 62);
}
TEST(rs_set_overhead_next_block) {
  RsEncoder e(RsConfig{8, 64, 0.25});
  std::vector<uint8_t> p(62, 0xAB);
  e.set_overhead(1.0);
  std::vector<std::vector<uint8_t>> envs;
  while (envs.empty()) envs = e.add_packet(p.data(), p.size());
  CHECK(envs.size() == 16);          // k=8 + repair 8
  CHECK(envs[0][10] == 16);          // header n field
}
MTEST_MAIN
