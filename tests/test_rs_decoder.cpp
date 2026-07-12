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

TEST(reject_nonzero_version_byte) {
  // Nonzero version byte (env[2]) must be rejected like bad magic,
  // with no counter change. Matches Python stream_fec_rs.py _unpack_header.
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/rs_decode.json");
  auto e = mtest::unhex(j["cases"][0]["envelopes"][0].get<std::string>());
  // Sanity: envelope must start with kRsMagic (0xF540 little-endian = "40f5")
  CHECK(e[0] == 0x40 && e[1] == 0xF5);
  CHECK(e[2] == 0x00);  // version is 0 in golden vector

  // Flip version to 1, keeping everything else the same.
  e[2] = 0x01;

  RsDecoder dec(RsConfig{8, 64, 1.0});
  uint64_t prev_symbols_in = dec.symbols_in();
  uint64_t prev_dropped = dec.symbols_dropped_bad_cfg();

  // add_symbol should return empty, with no counter increments.
  auto result = dec.add_symbol(e.data(), e.size(), 0);
  CHECK(result.empty());
  CHECK(dec.symbols_in() == prev_symbols_in);  // no increment
  CHECK(dec.symbols_dropped_bad_cfg() == prev_dropped);  // no increment
}
MTEST_MAIN
