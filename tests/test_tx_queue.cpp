#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include "mtest.h"
#include "tx_queue.h"

using namespace mabur;

namespace {
UepBody body(uint8_t tag) { return UepBody{0, std::vector<uint8_t>(8, tag)}; }
}  // namespace

TEST(fifo_order_and_batch_limit) {
  TxQueue q(8);
  for (uint8_t i = 0; i < 5; ++i) q.push(body(i));
  std::vector<UepBody> out;
  CHECK(q.pop_batch(out, 3, 0) == 3);
  CHECK(q.pop_batch(out, 8, 0) == 2);
  REQUIRE(out.size() == 5);
  for (uint8_t i = 0; i < 5; ++i) CHECK(out[i].body[0] == i);
  CHECK(q.depth() == 0);
  CHECK(q.dropped() == 0);
}

TEST(overflow_drops_oldest) {
  TxQueue q(3);
  for (uint8_t i = 0; i < 5; ++i) q.push(body(i));  // 0,1 evicted
  CHECK(q.dropped() == 2);
  std::vector<UepBody> out;
  CHECK(q.pop_batch(out, 8, 0) == 3);
  CHECK(out[0].body[0] == 2);
  CHECK(out[2].body[0] == 4);
}

// feed_batch: with set_batch(G), a blocked pop_batch is not woken until G
// un-notified bodies accumulate — the designed feed grouping that lets URBs
// fill (3 descriptors) and A-MPDU aggregates form, instead of the per-body
// trickle. Bodies are never withheld from an awake consumer (a timeout pop
// still drains partial groups); only the WAKEUP is grouped.
TEST(batched_push_defers_wakeup_until_group) {
  TxQueue q(8);
  q.set_batch(3);
  std::atomic<bool> done{false};
  std::vector<UepBody> out;
  std::thread waiter([&] {
    q.pop_batch(out, 8, 2000);
    done = true;
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(30));  // waiter blocks
  q.push(body(0));
  q.push(body(1));
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  CHECK(!done.load());  // 2 of 3: no wakeup yet
  q.push(body(2));  // group complete
  waiter.join();
  REQUIRE(out.size() == 3);
  for (uint8_t i = 0; i < 3; ++i) CHECK(out[i].body[0] == i);
}

// flush() releases a partial group immediately — called at AU end so a
// frame's tail bodies never wait on the next frame's production.
TEST(flush_wakes_partial_group) {
  TxQueue q(8);
  q.set_batch(4);
  std::atomic<bool> done{false};
  std::vector<UepBody> out;
  std::thread waiter([&] {
    q.pop_batch(out, 8, 2000);
    done = true;
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  q.push(body(7));
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  CHECK(!done.load());
  q.flush();
  waiter.join();
  REQUIRE(out.size() == 1);
  CHECK(out[0].body[0] == 7);
}

// Default (no set_batch) keeps the streaming shape: every push wakes.
TEST(default_batch_wakes_per_push) {
  TxQueue q(8);
  std::vector<UepBody> out;
  std::thread waiter([&] { q.pop_batch(out, 8, 2000); });
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  q.push(body(9));
  waiter.join();  // must return promptly on the single push
  REQUIRE(out.size() == 1);
  CHECK(out[0].body[0] == 9);
}

TEST(close_wakes_and_rejects) {
  TxQueue q(4);
  q.close();
  q.push(body(1));  // rejected after close
  std::vector<UepBody> out;
  CHECK(q.pop_batch(out, 4, 50) == 0);  // returns promptly, not 50ms-hang-then-item
  CHECK(q.depth() == 0);
}

MTEST_MAIN
