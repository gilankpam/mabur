#include "mtest.h"
#include "vectors.h"
#include "mabur/rs_decoder.h"
#include "mabur/rs_encoder.h"
using namespace mabur;

TEST(decode_matches_python_scenarios) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rs_decode.json");
  for (auto& c : j["cases"]) {
    RsDecoder dec(RsConfig{c["k"], c["symbol_size"], 1.0});
    std::vector<std::string> out;
    for (auto& eh : c["envelopes"]) {
      auto e = mtest::unhex(eh.get<std::string>());
      for (auto& p : dec.add_symbol(e.data(), e.size(), 0)) out.push_back(mtest::hex(p));
    }
    REQUIRE(out.size() == c["packets"].size());
    for (size_t i = 0; i < out.size(); ++i) CHECK(out[i] == c["packets"][i].get<std::string>());
    CHECK(dec.blocks_decoded() == c["blocks_decoded"].get<uint64_t>());
    CHECK(dec.symbols_dropped_stale_block() == c["dropped_stale"].get<uint64_t>());
    CHECK(dec.symbols_dropped_bad_cfg() == c["dropped_bad_cfg"].get<uint64_t>());
  }
}

TEST(encode_decode_roundtrip_all_overheads) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rs.json");
  for (auto& c : j["cases"]) {
    RsDecoder dec(RsConfig{c["k"], c["symbol_size"], c["overhead"]});
    std::vector<std::string> out;
    auto feed = [&](const nlohmann::json& envs) {
      for (auto& eh : envs) {
        auto e = mtest::unhex(eh.get<std::string>());
        for (auto& p : dec.add_symbol(e.data(), e.size(), 0)) out.push_back(mtest::hex(p));
      }
    };
    feed(c["stream"]); feed(c["flush"]);
    REQUIRE(out.size() == c["packets"].size());
    for (size_t i = 0; i < out.size(); ++i) CHECK(out[i] == c["packets"][i].get<std::string>());
  }
}

TEST(expiry_counts_unrecoverable) {
  // One lone symbol -> block can never solve; expire it.
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rs_decode.json");
  auto e = mtest::unhex(j["cases"][0]["envelopes"][0].get<std::string>());
  RsDecoder dec(RsConfig{8, 64, 1.0});
  CHECK(dec.add_symbol(e.data(), e.size(), 1000).empty());
  CHECK(dec.in_flight_blocks() == 1);
  CHECK(dec.expire_blocks_older_than(2000, 2500) == 0);   // not old enough
  CHECK(dec.expire_blocks_older_than(2000, 3001) == 1);
  CHECK(dec.blocks_unrecoverable() == 1);
  CHECK(dec.in_flight_blocks() == 0);
}
MTEST_MAIN
