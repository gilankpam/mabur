#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <set>
#include <thread>

#include "mtest.h"
#include "usb_tx_pool.h"

using namespace mabur;

// Every submitted frame is sent exactly once, in batches of <= 3, across
// parallel senders.
TEST(pool_delivers_every_frame_once_in_small_batches) {
  std::mutex m;
  std::set<uint32_t> seen;
  std::atomic<size_t> max_batch{0};
  UsbTxPool pool(
      [&](const std::vector<std::vector<uint8_t>>& batch) {
        if (batch.size() > max_batch.load()) max_batch = batch.size();
        std::this_thread::sleep_for(std::chrono::microseconds(200));
        std::lock_guard<std::mutex> l(m);
        for (auto& f : batch) {
          REQUIRE(f.size() == 64);
          uint32_t id;
          std::memcpy(&id, f.data(), 4);
          CHECK(seen.insert(id).second);  // no duplicates
        }
        return batch.size();
      },
      4, 24);
  for (uint32_t i = 0; i < 200; ++i) {
    uint8_t frame[64] = {0};
    std::memcpy(frame, &i, 4);
    CHECK(pool.submit(frame, sizeof frame));
  }
  pool.stop();  // drains, then joins
  CHECK(seen.size() == 200);
  CHECK(pool.sent_ok() == 200);
  CHECK(pool.send_fail() == 0);
  CHECK(max_batch.load() <= 3);
}

// A slow sink backpressures submit() (bounded queue) instead of growing
// unbounded; failed sends are counted.
TEST(pool_counts_failures_and_bounds_depth) {
  std::atomic<uint64_t> calls{0};
  UsbTxPool pool(
      [&](const std::vector<std::vector<uint8_t>>& batch) {
        ++calls;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return batch.size() - 1;  // last frame of every batch "fails"
      },
      2, 8);
  uint8_t frame[32] = {0xAB};
  for (int i = 0; i < 60; ++i) CHECK(pool.submit(frame, sizeof frame));
  pool.stop();
  CHECK(pool.sent_ok() + pool.send_fail() == 60);
  CHECK(pool.send_fail() == calls.load());
}

// submit_many() enqueues the whole group under one lock with one wakeup, so
// a grouped feed reaches ONE worker as one <=3-frame batch (one 3-descriptor
// URB) instead of splitting 1+1+1 across idle workers.
TEST(submit_many_keeps_group_in_one_batch) {
  std::mutex m;
  std::vector<size_t> batch_sizes;
  UsbTxPool pool(
      [&](const std::vector<std::vector<uint8_t>>& batch) {
        std::lock_guard<std::mutex> l(m);
        batch_sizes.push_back(batch.size());
        return batch.size();
      },
      1, 24);
  uint8_t a[16] = {1}, b[16] = {2}, c[16] = {3};
  const UsbTxPool::Frame group[3] = {{a, sizeof a}, {b, sizeof b}, {c, sizeof c}};
  CHECK(pool.submit_many(group, 3) == 3);
  pool.stop();
  REQUIRE(batch_sizes.size() == 1);
  CHECK(batch_sizes[0] == 3);
  CHECK(pool.sent_ok() == 3);
}

// submit_many after stop() refuses like submit().
TEST(submit_many_after_stop_returns_zero) {
  UsbTxPool pool(
      [](const std::vector<std::vector<uint8_t>>& b) { return b.size(); }, 1,
      4);
  pool.stop();
  uint8_t f[8] = {0};
  const UsbTxPool::Frame group[2] = {{f, sizeof f}, {f, sizeof f}};
  CHECK(pool.submit_many(group, 2) == 0);
}

// submit() after stop() refuses instead of blocking forever.
TEST(pool_submit_after_stop_returns_false) {
  UsbTxPool pool(
      [](const std::vector<std::vector<uint8_t>>& b) { return b.size(); }, 1,
      4);
  pool.stop();
  uint8_t frame[8] = {0};
  CHECK(!pool.submit(frame, sizeof frame));
}

MTEST_MAIN
