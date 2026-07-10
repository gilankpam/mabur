#include "mtest.h"
#include "vectors.h"
#include "mabur/frag.h"
using namespace mabur;

TEST(fragmenter_matches_python_vectors) {
  auto j = mtest::load_json(std::string(MABUR_VECTOR_DIR) + "/frag.json");
  Fragmenter frag;
  for (auto& c : j["cases"]) {
    auto in = mtest::unhex(c["in"].get<std::string>());
    int usable = c["usable"].get<int>();
    auto out = frag.fragment(in.data(), in.size(), usable);
    REQUIRE(out.size() == c["out"].size());
    for (size_t i = 0; i < out.size(); ++i)
      CHECK(mtest::hex(out[i]) == c["out"][i].get<std::string>());
  }
}

TEST(fragmenter_empty_input_yields_one_empty_chunk) {
  Fragmenter frag;
  auto out = frag.fragment(nullptr, 0, 58);
  REQUIRE(out.size() == 1);
  // header <u16 seq LE, u8 idx, u8 count> + 0 payload bytes -> 4-byte chunk.
  CHECK(out[0].size() == 4);
  CHECK(out[0][2] == 0);  // idx
  CHECK(out[0][3] == 1);  // count
}

TEST(fragmenter_seq_wraps_u16) {
  Fragmenter frag;
  std::vector<uint8_t> p = {0xAB};
  std::vector<uint8_t> out;
  for (int i = 0; i < 65536; ++i) out = frag.fragment(p.data(), p.size(), 58)[0];
  // After 65536 calls (seq 0..65535 consumed), the next call's seq should be 0 again.
  auto out2 = frag.fragment(p.data(), p.size(), 58);
  uint16_t seq = static_cast<uint16_t>(out2[0][0]) | (static_cast<uint16_t>(out2[0][1]) << 8);
  CHECK(seq == 0);
}

MTEST_MAIN
