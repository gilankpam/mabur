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

TEST(wide_fragmenter_six_byte_header) {
  Fragmenter frag(/*wide=*/true);
  std::vector<uint8_t> p(100, 0xAA);
  auto out = frag.fragment(p.data(), p.size(), 40);
  REQUIRE(out.size() == 3);  // 40+40+20
  // header <u16 seq, u16 idx, u16 count> LE
  CHECK(out[0].size() == 6 + 40);
  CHECK(out[2].size() == 6 + 20);
  for (size_t i = 0; i < out.size(); ++i) {
    uint16_t idx = static_cast<uint16_t>(out[i][2] | (out[i][3] << 8));
    uint16_t count = static_cast<uint16_t>(out[i][4] | (out[i][5] << 8));
    CHECK(idx == i);
    CHECK(count == 3);
  }
}

TEST(wide_fragmenter_exceeds_255_fragments) {
  // The reason wide mode exists: a 100 KB IDR frame at usable=158 needs
  // 633 fragments — impossible in the u8-idx narrow format.
  Fragmenter frag(/*wide=*/true);
  std::vector<uint8_t> p(100 * 1024, 0x42);
  auto out = frag.fragment(p.data(), p.size(), 158);
  REQUIRE(out.size() == (p.size() + 157) / 158);
  REQUIRE(out.size() > 255);
  uint16_t count = static_cast<uint16_t>(out[0][4] | (out[0][5] << 8));
  CHECK(count == out.size());
}

TEST(wide_fragmenter_empty_input_yields_one_empty_chunk) {
  Fragmenter frag(/*wide=*/true);
  auto out = frag.fragment(nullptr, 0, 58);
  REQUIRE(out.size() == 1);
  CHECK(out[0].size() == 6);
}

MTEST_MAIN
