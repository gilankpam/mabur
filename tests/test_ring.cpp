#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>

#include "mtest.h"
#include "ring_source.h"

using namespace mabur;

namespace {

// Builds a packet of `len` bytes patterned as an increasing byte sequence
// seeded by `seed`, so a reader can verify content (not just length).
void make_packet(uint8_t* buf, size_t len, uint8_t seed) {
  for (size_t i = 0; i < len; ++i) buf[i] = static_cast<uint8_t>(seed + i);
}

bool packet_matches(const uint8_t* buf, size_t len, uint8_t seed) {
  for (size_t i = 0; i < len; ++i) {
    if (buf[i] != static_cast<uint8_t>(seed + i)) return false;
  }
  return true;
}

std::string unique_ring_name(const char* suffix) {
  return "mabur_test_ring_" + std::to_string(getpid()) + "_" + suffix;
}

}  // namespace

TEST(reads_five_patterned_packets_then_times_out) {
  std::string name = unique_ring_name("basic");
  venc_ring_t* producer = venc_ring_create(name.c_str(), 64, 2048);
  REQUIRE(producer != nullptr);

  RingSource src(name);
  // First read() call performs the (lazy) initial attach, which snaps
  // read_idx to the current (empty) write head. Do this before writing
  // anything so the 5 packets below are the very first thing the
  // consumer's read_idx can observe — otherwise, per the documented
  // "initial attach also snaps to head" semantics, packets written
  // before the first attach would be skipped as stale backlog.
  {
    uint8_t buf[2048];
    int n = src.read(buf, sizeof(buf), 50);
    CHECK(n == 0);
  }
  CHECK(src.attached());

  for (int i = 0; i < 5; ++i) {
    uint8_t pkt[64];
    size_t len = 32 + i;
    make_packet(pkt, len, static_cast<uint8_t>(i * 10));
    REQUIRE(venc_ring_write(producer, pkt, static_cast<uint16_t>(len), nullptr,
                             0) == 0);
  }

  for (int i = 0; i < 5; ++i) {
    uint8_t buf[2048];
    int n = src.read(buf, sizeof(buf), 500);
    CHECK(n == static_cast<int>(32 + i));
    if (n > 0) {
      CHECK(packet_matches(buf, static_cast<size_t>(n),
                            static_cast<uint8_t>(i * 10)));
    }
  }
  CHECK(src.attached());

  uint8_t buf[2048];
  int n = src.read(buf, sizeof(buf), 50);
  CHECK(n == 0);
  CHECK(src.reattach_count() == 0u);

  venc_ring_destroy(producer);
}

TEST(producer_restart_triggers_reattach_and_serves_new_packets) {
  std::string name = unique_ring_name("restart");
  venc_ring_t* producer = venc_ring_create(name.c_str(), 64, 2048);
  REQUIRE(producer != nullptr);

  RingSource src(name);

  // Prime the initial attach (before writing anything — the initial
  // attach also snaps read_idx to the current head, so a packet written
  // beforehand would be treated as stale backlog and skipped) so the
  // reattach path (not the cold-start path) is what fires on restart.
  {
    uint8_t buf[2048];
    int n = src.read(buf, sizeof(buf), 50);
    CHECK(n == 0);
  }
  CHECK(src.attached());
  CHECK(src.reattach_count() == 0u);

  {
    uint8_t pkt[16];
    make_packet(pkt, sizeof(pkt), 1);
    REQUIRE(venc_ring_write(producer, pkt, sizeof(pkt), nullptr, 0) == 0);
    uint8_t buf[2048];
    int n = src.read(buf, sizeof(buf), 500);
    CHECK(n == 16);
  }
  CHECK(src.attached());
  CHECK(src.reattach_count() == 0u);

  // Restart the producer: destroy (unlinks) then create again — new
  // inode, new epoch. Write a packet the old attach never saw so it
  // would sit stale in the old (now orphaned) mapping if reattach didn't
  // happen.
  venc_ring_destroy(producer);
  producer = venc_ring_create(name.c_str(), 64, 2048);
  REQUIRE(producer != nullptr);

  // One read to let RingSource notice the restart (timeout path re-stats
  // and detaches the stale handle).
  {
    uint8_t buf[2048];
    int n = src.read(buf, sizeof(buf), 50);
    CHECK(n == 0);
  }
  CHECK(src.reattach_count() == 1u);
  CHECK(!src.attached());  // detached; the actual reattach is lazy

  // A second read is what actually performs the reattach (attach is lazy,
  // triggered by the next read() call) and snaps read_idx to the new
  // ring's current (still-empty) write head.
  {
    uint8_t buf[2048];
    int n = src.read(buf, sizeof(buf), 50);
    CHECK(n == 0);  // nothing written yet on the new ring
  }
  CHECK(src.attached());
  CHECK(src.reattach_count() == 1u);

  // Now write the 2 packets the reattached consumer should read — since
  // attach snaps read_idx to the current (empty) write head, packets
  // written after reattach are exactly what should be delivered.
  for (int i = 0; i < 2; ++i) {
    uint8_t pkt[24];
    make_packet(pkt, sizeof(pkt), static_cast<uint8_t>(100 + i));
    REQUIRE(venc_ring_write(producer, pkt, sizeof(pkt), nullptr, 0) == 0);
  }

  for (int i = 0; i < 2; ++i) {
    uint8_t buf[2048];
    int n = src.read(buf, sizeof(buf), 500);
    CHECK(n == 24);
    if (n > 0) {
      CHECK(packet_matches(buf, static_cast<size_t>(n),
                            static_cast<uint8_t>(100 + i)));
    }
  }
  CHECK(src.reattach_count() == 1u);

  {
    uint8_t buf[2048];
    int n = src.read(buf, sizeof(buf), 50);
    CHECK(n == 0);
  }

  venc_ring_destroy(producer);
}

TEST(cold_start_before_ring_exists_then_ring_appears) {
  std::string name = unique_ring_name("coldstart");

  // Short backoff so this test doesn't burn the full production 1 s
  // retry interval; documented via the RingSource constructor's
  // attach_backoff_ms parameter.
  RingSource src(name, /*attach_backoff_ms=*/50);

  auto start = std::chrono::steady_clock::now();
  uint8_t buf[2048];
  int n = src.read(buf, sizeof(buf), 50);
  auto elapsed = std::chrono::steady_clock::now() - start;
  CHECK(n == 0);
  CHECK(!src.attached());
  CHECK(elapsed < std::chrono::milliseconds(500));

  venc_ring_t* producer = venc_ring_create(name.c_str(), 64, 2048);
  REQUIRE(producer != nullptr);

  // Poll (bounded) until RingSource performs its lazy attach — this may
  // take up to one backoff cycle (~50 ms here). Attach snaps read_idx to
  // the current (still-empty) write head, so only packets written after
  // this point should be delivered.
  auto poll_start = std::chrono::steady_clock::now();
  while (!src.attached() && std::chrono::steady_clock::now() - poll_start <
                                 std::chrono::milliseconds(1000)) {
    src.read(buf, sizeof(buf), 100);
  }
  CHECK(src.attached());

  uint8_t pkt[8];
  make_packet(pkt, sizeof(pkt), 7);
  REQUIRE(venc_ring_write(producer, pkt, sizeof(pkt), nullptr, 0) == 0);

  int got = src.read(buf, sizeof(buf), 500);
  CHECK(got == 8);
  if (got > 0) CHECK(packet_matches(buf, static_cast<size_t>(got), 7));
  CHECK(src.attached());

  venc_ring_destroy(producer);
}

TEST(timeout_zero_is_nonblocking_poll) {
  std::string name = unique_ring_name("timeout_zero");
  venc_ring_t* producer = venc_ring_create(name.c_str(), 64, 2048);
  REQUIRE(producer != nullptr);

  RingSource src(name);

  // Prime the initial attach (before writing anything).
  {
    uint8_t buf[2048];
    int n = src.read(buf, sizeof(buf), 50);
    CHECK(n == 0);
  }
  CHECK(src.attached());

  // Test 1: empty ring + timeout=0 should return 0 immediately (not block).
  auto start = std::chrono::steady_clock::now();
  uint8_t buf[2048];
  int n = src.read(buf, sizeof(buf), 0);
  auto elapsed = std::chrono::steady_clock::now() - start;
  CHECK(n == 0);
  CHECK(elapsed < std::chrono::milliseconds(100));

  // Test 2: write one packet, then timeout=0 should return it immediately.
  uint8_t pkt[32];
  make_packet(pkt, sizeof(pkt), 42);
  REQUIRE(venc_ring_write(producer, pkt, sizeof(pkt), nullptr, 0) == 0);

  start = std::chrono::steady_clock::now();
  n = src.read(buf, sizeof(buf), 0);
  elapsed = std::chrono::steady_clock::now() - start;
  CHECK(n == 32);
  CHECK(packet_matches(buf, static_cast<size_t>(n), 42));
  CHECK(elapsed < std::chrono::milliseconds(100));

  venc_ring_destroy(producer);
}

TEST(timeout_zero_unattached_no_sleep) {
  std::string name = unique_ring_name("timeout_zero_unattached");

  // Short backoff so the test doesn't burn time.
  RingSource src(name, /*attach_backoff_ms=*/1000);

  // First read() on non-existent ring: tries attach (fails), returns 0.
  // Now in backoff window (can't try attach again for 1000 ms).
  auto start = std::chrono::steady_clock::now();
  uint8_t buf[2048];
  int n = src.read(buf, sizeof(buf), 0);
  auto elapsed = std::chrono::steady_clock::now() - start;
  CHECK(n == 0);
  CHECK(!src.attached());
  CHECK(elapsed < std::chrono::milliseconds(100));

  // Second read() with timeout=0 while still in backoff window:
  // should return 0 immediately without sleeping.
  start = std::chrono::steady_clock::now();
  n = src.read(buf, sizeof(buf), 0);
  elapsed = std::chrono::steady_clock::now() - start;
  CHECK(n == 0);
  CHECK(!src.attached());
  CHECK(elapsed < std::chrono::milliseconds(100));
}

MTEST_MAIN
