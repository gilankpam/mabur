#include <cstdint>
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

TEST(close_wakes_and_rejects) {
  TxQueue q(4);
  q.close();
  q.push(body(1));  // rejected after close
  std::vector<UepBody> out;
  CHECK(q.pop_batch(out, 4, 50) == 0);  // returns promptly, not 50ms-hang-then-item
  CHECK(q.depth() == 0);
}

MTEST_MAIN
