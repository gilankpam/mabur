#include "mtest.h"
#include "mabur/frag.h"
using namespace mabur;

// One format only since the pre-frame-shm video path was deleted: the 6-byte
// <u16 seq, u16 idx, u16 count> header. The 4-byte <HBB> format (and its
// frag.json pin against devourer's svc_uep_fec) went with that path.
TEST(fragmenter_emits_six_byte_header) {
  Fragmenter frag;
  std::vector<uint8_t> p(100, 0xAA);
  auto out = frag.fragment(p.data(), p.size(), 40);
  REQUIRE(out.size() == 3);  // 40+40+20
  CHECK(out[0].size() == 6 + 40);
  CHECK(out[2].size() == 6 + 20);
  for (size_t i = 0; i < out.size(); ++i) {
    uint16_t idx = static_cast<uint16_t>(out[i][2] | (out[i][3] << 8));
    uint16_t count = static_cast<uint16_t>(out[i][4] | (out[i][5] << 8));
    CHECK(idx == i);
    CHECK(count == 3);
  }
}

TEST(fragmenter_exceeds_255_fragments) {
  // The reason the wide format exists: a 100 KB IDR frame at usable=158 needs
  // 633 fragments — impossible in the u8-idx format it replaced.
  Fragmenter frag;
  std::vector<uint8_t> p(100 * 1024, 0x42);
  auto out = frag.fragment(p.data(), p.size(), 158);
  REQUIRE(out.size() == (p.size() + 157) / 158);
  REQUIRE(out.size() > 255);
  uint16_t count = static_cast<uint16_t>(out[0][4] | (out[0][5] << 8));
  CHECK(count == out.size());
}

TEST(fragmenter_empty_input_yields_one_empty_chunk) {
  Fragmenter frag;
  auto out = frag.fragment(nullptr, 0, 58);
  REQUIRE(out.size() == 1);
  CHECK(out[0].size() == 6);
  uint16_t idx = static_cast<uint16_t>(out[0][2] | (out[0][3] << 8));
  uint16_t count = static_cast<uint16_t>(out[0][4] | (out[0][5] << 8));
  CHECK(idx == 0);
  CHECK(count == 1);
}

TEST(fragmenter_seq_wraps_u16) {
  Fragmenter frag;
  std::vector<uint8_t> p = {0xAB};
  for (int i = 0; i < 65536; ++i) frag.fragment(p.data(), p.size(), 58);
  // After 65536 calls (seq 0..65535 consumed), the next call's seq is 0 again.
  auto out = frag.fragment(p.data(), p.size(), 58);
  uint16_t seq = static_cast<uint16_t>(out[0][0]) | (static_cast<uint16_t>(out[0][1]) << 8);
  CHECK(seq == 0);
}

MTEST_MAIN
