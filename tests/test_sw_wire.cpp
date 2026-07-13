#include <cstdint>
#include <vector>

#include "mabur/sw_wire.h"
#include "mtest.h"

using namespace mabur;

TEST(header_roundtrip_source) {
  sw::SwHeader h;
  h.repair = false; h.symbol_size = 164; h.seq = 0xDEADBEEF;
  h.window_len = 0; h.repair_key = 0;
  std::vector<uint8_t> buf;
  sw::pack_header(buf, h);
  CHECK(buf.size() == sw::kSwHeaderLen);
  sw::SwHeader g;
  CHECK(sw::parse_header(buf.data(), buf.size(), &g));
  CHECK(!g.repair);
  CHECK(g.symbol_size == 164);
  CHECK(g.seq == 0xDEADBEEF);
  CHECK(g.window_len == 0);
  CHECK(g.repair_key == 0);
}

TEST(header_roundtrip_repair) {
  sw::SwHeader h;
  h.repair = true; h.symbol_size = 64; h.seq = 1000;
  h.window_len = 128; h.repair_key = 42;
  std::vector<uint8_t> buf;
  sw::pack_header(buf, h);
  sw::SwHeader g;
  CHECK(sw::parse_header(buf.data(), buf.size(), &g));
  CHECK(g.repair);
  CHECK(g.window_len == 128);
  CHECK(g.repair_key == 42);
}

TEST(header_rejects_garbage) {
  sw::SwHeader h;
  h.repair = true; h.symbol_size = 64; h.seq = 7; h.window_len = 3; h.repair_key = 9;
  std::vector<uint8_t> buf;
  sw::pack_header(buf, h);
  sw::SwHeader g;
  CHECK(!sw::parse_header(buf.data(), 13, &g));            // short
  auto bad_magic = buf; bad_magic[0] ^= 0xFF;
  CHECK(!sw::parse_header(bad_magic.data(), bad_magic.size(), &g));
  auto bad_flags = buf; bad_flags[2] = 0x81;               // unknown flag bit
  CHECK(!sw::parse_header(bad_flags.data(), bad_flags.size(), &g));
  auto zero_wl = buf; zero_wl[9] = 0;                      // repair with wl 0
  CHECK(!sw::parse_header(zero_wl.data(), zero_wl.size(), &g));
  // source with nonzero repair fields
  sw::SwHeader s; s.repair = false; s.symbol_size = 64; s.seq = 1;
  s.window_len = 0; s.repair_key = 0;
  std::vector<uint8_t> sb; sw::pack_header(sb, s);
  auto src_wl = sb; src_wl[9] = 1;
  CHECK(!sw::parse_header(src_wl.data(), src_wl.size(), &g));
}

TEST(coeffs_deterministic_nonzero_and_key_sensitive) {
  uint8_t a[255], b[255], c[255];
  sw::repair_coeffs(7, 255, a);
  sw::repair_coeffs(7, 255, b);
  sw::repair_coeffs(8, 255, c);
  int diff = 0;
  for (int i = 0; i < 255; ++i) {
    CHECK(a[i] >= 1);
    CHECK(a[i] == b[i]);
    if (a[i] != c[i]) ++diff;
  }
  CHECK(diff > 200);  // different keys give (almost entirely) different vectors
}

MTEST_MAIN
