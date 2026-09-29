// relay_ring.h: the page's core<->Worker SPSC ring. The goldens here are
// repeated byte for byte in web/tests/relay_ring.test.mjs (the JS side).
#include <cstring>
#include "mtest.h"
#include "relay_ring.h"
using namespace webgs;

namespace {
struct Small {
  uint32_t head = 0, tail = 0, drops = 0;
  uint8_t data[32] = {};
  Ring r() { return Ring{&head, &tail, data, 32, &drops}; }
};
std::vector<uint8_t> bytes(const char* s) { return std::vector<uint8_t>(s, s + std::strlen(s)); }
}  // namespace

TEST(write_read_golden) {
  Small s;
  auto a = bytes("ABCDE");
  REQUIRE(ring_write(s.r(), a.data(), 5));
  const uint8_t want[12] = {5, 0, 0, 0, 'A', 'B', 'C', 'D', 'E', 0, 0, 0};
  CHECK(std::memcmp(s.data, want, 12) == 0);
  CHECK(s.head == 12);
  std::vector<uint8_t> out;
  REQUIRE(ring_read(s.r(), out));
  CHECK(out == a && s.tail == 12);
  CHECK(!ring_read(s.r(), out));
}

TEST(wrap_marker_golden) {
  Small s;
  auto a = bytes("012345678");   // 9 B -> 16 B record
  auto b = bytes("wxyz!");       // 5 B -> 12 B record
  auto x = bytes("abcde");       // 5 B -> 12 B record
  std::vector<uint8_t> out;
  REQUIRE(ring_write(s.r(), a.data(), 9));
  REQUIRE(ring_read(s.r(), out));          // head 16, tail 16
  REQUIRE(ring_write(s.r(), b.data(), 5));
  REQUIRE(ring_read(s.r(), out));          // head 28, tail 28
  REQUIRE(ring_write(s.r(), x.data(), 5)); // 4 B left at the end < 12: marker, then offset 0
  const uint8_t marker[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  CHECK(std::memcmp(s.data + 28, marker, 4) == 0);
  // Only the header + real payload are pinned; the 3 pad bytes at data[9..11]
  // are never touched by this write (they're leftover from `a`'s own payload,
  // which reached that offset) and are not part of what ring_read exposes.
  const uint8_t rec[9] = {5, 0, 0, 0, 'a', 'b', 'c', 'd', 'e'};
  CHECK(std::memcmp(s.data, rec, 9) == 0);
  CHECK(s.head == 44);
  REQUIRE(ring_read(s.r(), out));
  CHECK(out == x && s.tail == 44);
}

TEST(ring_full_drops_newest_and_recovers) {
  Small s;
  auto m = bytes("0123456789");            // 16 B record
  std::vector<uint8_t> out;
  REQUIRE(ring_write(s.r(), m.data(), 10));
  REQUIRE(ring_write(s.r(), m.data(), 10));
  CHECK(!ring_write(s.r(), m.data(), 10)); // full
  CHECK(s.drops == 1);
  REQUIRE(ring_read(s.r(), out));
  REQUIRE(ring_write(s.r(), m.data(), 10));
  CHECK(!ring_write(s.r(), m.data(), 17)); // > cap/2 is never accepted
  CHECK(s.drops == 2);
}

TEST(real_layout) {
  std::vector<uint8_t> mem(kRingBytes + 4);
  uint8_t* base = mem.data();
  Ring rx = rx_ring(base), tx = tx_ring(base);
  CHECK(rx.data == base + kRingHdr && rx.cap == kRxCap);
  CHECK(tx.data == base + kRingHdr + kRxCap && tx.cap == kTxCap);
  CHECK(reinterpret_cast<uint8_t*>(tx.head) == base + 8);
  CHECK(reinterpret_cast<uint8_t*>(tx.drops) == base + 24);
}

MTEST_MAIN
