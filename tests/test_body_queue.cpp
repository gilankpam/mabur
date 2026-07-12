#include <thread>
#include "mtest.h"
#include "body_queue.h"
using namespace maburgs;

static mabur::node::RxBody mk(uint16_t seq) {
  mabur::node::RxBody m;
  m.mac_seq = seq;
  return m;
}

TEST(push_drain_ordering) {
  BodyQueue q;
  q.push(mk(1)); q.push(mk(2)); q.push(mk(3));
  std::vector<mabur::node::RxBody> out;
  CHECK(q.drain(out, 0) == 3);
  REQUIRE(out.size() == 3);
  CHECK(out[0].mac_seq == 1);
  CHECK(out[2].mac_seq == 3);
}

TEST(drain_blocks_until_push) {
  BodyQueue q;
  std::thread t([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    q.push(mk(9));
  });
  std::vector<mabur::node::RxBody> out;
  CHECK(q.drain(out, 1000) == 1);
  CHECK(out[0].mac_seq == 9);
  t.join();
}

TEST(overflow_drops_newest_and_counts) {
  BodyQueue q;
  for (int i = 0; i < 9000; ++i) q.push(mk(static_cast<uint16_t>(i)));
  CHECK(q.dropped() == 9000 - 8192);
  std::vector<mabur::node::RxBody> out;
  CHECK(q.drain(out, 0) == 8192);
}

TEST(close_wakes_consumer) {
  BodyQueue q;
  std::thread t([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    q.close();
  });
  std::vector<mabur::node::RxBody> out;
  CHECK(q.drain(out, 5000) == 0);
  CHECK(q.closed());
  t.join();
}
MTEST_MAIN
