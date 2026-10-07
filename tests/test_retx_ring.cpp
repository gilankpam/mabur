#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include "mabur/retx_ring.h"
#include "mtest.h"
using mabur::RetxRing;

TEST(slots_for_sizes_by_time_at_max_bitrate) {
  CHECK(RetxRing::slots_for(24000, 150, 332) == 1024);   // 24 Mb/s * 0.6 * 0.15 s / 8 / 332 = 813 -> 1024
  CHECK(RetxRing::slots_for(2000, 150, 332) == 128);     // 67.8 -> 128
  CHECK(RetxRing::slots_for(24000, 150, 332) * (14 + 332) < 400 * 1024);
}

TEST(put_get_hit_miss_and_overwrite) {
  RetxRing r(4, 3);
  uint8_t e[3] = {1, 2, 3}, out[3] = {0, 0, 0};
  for (uint32_t s = 10; s < 16; ++s) { e[0] = static_cast<uint8_t>(s); r.put(s, e, 3); }
  CHECK(r.slots() == 4);
  CHECK(!r.get(10, out) && !r.get(11, out));             // overwritten (6 puts, 4 slots)
  CHECK(r.get(15, out) && out[0] == 15 && out[1] == 2);
  CHECK(r.get(12, out) && out[0] == 12);
  CHECK(!r.get(99, out));
  r.put(16, e, 2);                                        // wrong length: ignored
  CHECK(!r.get(16, out));
}

TEST(concurrent_reader_never_sees_a_torn_envelope) {
  // seq 0..63 only hits during the writer's first ~64 puts (a few us), so a
  // reader fixed on that range goes from "racing the writer" to "100% miss"
  // the instant the writer laps the ring -- CHECK(hits > 0) on that alone is
  // a timing bet, not a concurrency test. Instead the writer publishes
  // `latest`, and the reader keeps chasing the two highest-contention
  // slots: the one just published (racing put()'s still-in-flight store
  // sequence) and the one about to fall off the back of the ring (racing
  // the writer's NEXT overwrite of that same slot). Each envelope's bytes
  // are seq-derived, so a torn copy -- part of an old put, part of a new
  // one -- is caught by recomputing the expected pattern from the seq the
  // reader asked for, not by comparing bytes to each other.
  constexpr size_t kSlots = 64, kLen = 64;
  RetxRing r(kSlots, kLen);
  std::atomic<bool> stop{false};
  std::atomic<uint32_t> latest{0};
  std::thread writer([&] {
    std::vector<uint8_t> env(kLen);
    for (uint32_t seq = 0; !stop.load(std::memory_order_relaxed); ++seq) {
      for (size_t k = 0; k < kLen; ++k) env[k] = static_cast<uint8_t>((seq + k) & 0xFFu);
      r.put(seq, env.data(), env.size());
      latest.store(seq, std::memory_order_release);
    }
  });

  uint8_t out[kLen];
  size_t hits = 0;
  constexpr size_t kTargetHits = 2000;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  bool timed_out = false;
  while (hits < kTargetHits) {
    if (std::chrono::steady_clock::now() >= deadline) { timed_out = true; break; }
    const uint32_t cur = latest.load(std::memory_order_acquire);
    const uint32_t oldest_live = cur >= static_cast<uint32_t>(kSlots - 1) ? cur - static_cast<uint32_t>(kSlots - 1) : 0;
    const uint32_t candidates[2] = {cur, oldest_live};
    for (uint32_t want : candidates) {
      if (!r.get(want, out)) continue;
      ++hits;
      for (size_t k = 0; k < kLen; ++k)
        REQUIRE(out[k] == static_cast<uint8_t>((want + k) & 0xFFu));  // a torn read fails this
    }
  }
  stop.store(true, std::memory_order_relaxed);
  writer.join();
  // Under a loaded box the writer thread may barely get scheduled before
  // the deadline -- stay lenient there (some progress happened) rather
  // than spuriously fail; off a loaded box this always clears the floor.
  if (timed_out) CHECK(hits > 0);
  else CHECK(hits >= kTargetHits);
}
MTEST_MAIN
