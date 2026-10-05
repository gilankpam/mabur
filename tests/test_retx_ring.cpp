#include <algorithm>
#include <atomic>
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
  RetxRing r(64, 64);
  std::atomic<bool> stop{false};
  std::thread writer([&] {
    std::vector<uint8_t> env(64);
    for (uint32_t s = 0; !stop.load(); ++s) { std::fill(env.begin(), env.end(), static_cast<uint8_t>(s)); r.put(s, env.data(), env.size()); }
  });
  uint8_t out[64];
  size_t hits = 0;
  for (int i = 0; i < 200000; ++i) {
    const uint32_t want = static_cast<uint32_t>(i % 64);
    if (!r.get(want, out)) continue;
    ++hits;
    for (int k = 1; k < 64; ++k) REQUIRE(out[k] == out[0]);   // a torn read would mix two seqs
  }
  stop = true; writer.join();
  CHECK(hits > 0);
}
MTEST_MAIN
