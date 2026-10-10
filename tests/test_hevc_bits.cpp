#include <vector>
#include "mabur/hevc_bits.h"
#include "mtest.h"
using namespace mabur::hevc;

TEST(escape_and_unescape_round_trip) {
  const std::vector<uint8_t> rbsp = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x03, 0x80};
  const auto nal = escape(rbsp.data(), rbsp.size());
  const std::vector<uint8_t> expect = {0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x01,
                                       0x00, 0x00, 0x03, 0x03, 0x80};
  CHECK(nal == expect);
  CHECK(unescape(nal.data(), nal.size()) == rbsp);
}

TEST(reader_reads_fixed_and_exp_golomb) {
  // 101 | 1 | 010 | 011 | 00100 | 00101 -> u(3)=5, ue=0, ue=1, ue=2, ue=3, se(ue=4)=-2
  const std::vector<uint8_t> b = {0xB4, 0xC8, 0x50};
  BitReader r(b.data(), b.size());
  CHECK(r.u(3) == 5);
  CHECK(r.ue() == 0);
  CHECK(r.ue() == 1);
  CHECK(r.ue() == 2);
  CHECK(r.ue() == 3);
  CHECK(r.se() == -2);
  CHECK(!r.bad());
  r.u(16);
  CHECK(r.bad());
}

TEST(writer_matches_reader_and_copies_bit_ranges) {
  BitWriter w;
  w.u(5, 3); w.ue(0); w.ue(1); w.ue(2); w.ue(3); w.ue(4);
  CHECK(w.bits() == 3 + 1 + 3 + 3 + 5 + 5);
  BitWriter c;
  c.copy(w.bytes().data(), 3, w.bits());  // everything after u(3)
  c.align_zero();
  BitReader r(c.bytes().data(), c.bytes().size());
  CHECK(r.ue() == 0); CHECK(r.ue() == 1); CHECK(r.ue() == 2);
  CHECK(r.ue() == 3); CHECK(r.ue() == 4);
  CHECK(c.bits() % 8 == 0);
}

TEST(ceil_log2_values) {
  CHECK(ceil_log2(0) == 0); CHECK(ceil_log2(1) == 0); CHECK(ceil_log2(2) == 1);
  CHECK(ceil_log2(3) == 2); CHECK(ceil_log2(510) == 9); CHECK(ceil_log2(512) == 9);
}

MTEST_MAIN
