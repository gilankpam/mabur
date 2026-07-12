#include <cstdint>
#include <vector>

#include "mtest.h"
#include "rtp_reorder.h"

using maburgs::RtpReorder;

namespace {
std::vector<uint8_t> rtp(uint16_t seq) {
  std::vector<uint8_t> p(12, 0);
  p[0] = 0x80;
  p[2] = static_cast<uint8_t>(seq >> 8);
  p[3] = static_cast<uint8_t>(seq & 0xFF);
  return p;
}
uint16_t seq_of(const std::vector<uint8_t>& p) {
  return static_cast<uint16_t>((p[2] << 8) | p[3]);
}
}  // namespace

TEST(in_order_passthrough_is_immediate) {
  std::vector<uint16_t> out;
  RtpReorder r([&](const std::vector<uint8_t>& p) { out.push_back(seq_of(p)); });
  for (uint16_t s = 10; s < 15; ++s) r.push(rtp(s), 1000 + s);
  REQUIRE(out.size() == 5);
  for (size_t i = 0; i < 5; ++i) CHECK(out[i] == 10 + i);
}

TEST(reordered_packets_emit_sorted) {
  std::vector<uint16_t> out;
  RtpReorder r([&](const std::vector<uint8_t>& p) { out.push_back(seq_of(p)); });
  r.push(rtp(10), 1000);
  r.push(rtp(12), 1001);  // held: 11 missing
  r.push(rtp(13), 1002);  // held
  CHECK(out.size() == 1);
  r.push(rtp(11), 1003);  // fills the gap -> 11,12,13 flush
  REQUIRE(out.size() == 4);
  CHECK(out[1] == 11);
  CHECK(out[2] == 12);
  CHECK(out[3] == 13);
}

TEST(gap_skips_after_hold_and_counts) {
  std::vector<uint16_t> out;
  RtpReorder r([&](const std::vector<uint8_t>& p) { out.push_back(seq_of(p)); },
               /*hold_ms=*/100);
  r.push(rtp(10), 1000);
  r.push(rtp(13), 1001);  // 11,12 missing
  CHECK(out.size() == 1);
  r.poll(1050);
  CHECK(out.size() == 1);  // still holding
  r.poll(1101);            // hold expired -> skip 11,12; emit 13
  REQUIRE(out.size() == 2);
  CHECK(out[1] == 13);
  CHECK(r.skipped() == 2);
  // The skipped seq arriving later is dropped, not re-emitted out of order.
  r.push(rtp(11), 1102);
  CHECK(out.size() == 2);
  CHECK(r.late_dropped() == 1);
}

TEST(seq_wraparound_stays_ordered) {
  std::vector<uint16_t> out;
  RtpReorder r([&](const std::vector<uint8_t>& p) { out.push_back(seq_of(p)); });
  r.push(rtp(65534), 1000);
  r.push(rtp(0), 1001);      // held (65535 missing)
  r.push(rtp(65535), 1002);  // fills -> 65535, 0 flush
  REQUIRE(out.size() == 3);
  CHECK(out[0] == 65534);
  CHECK(out[1] == 65535);
  CHECK(out[2] == 0);
}

MTEST_MAIN
