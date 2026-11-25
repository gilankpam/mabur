// SendTimer: the longest radio send per watchdog interval, including a send
// that never returns.
#include "mtest.h"
#include "send_timer.h"
using namespace mabur;

TEST(idle_reads_zero)               { SendTimer t; CHECK(t.take_longest_us(5000) == 0); }
TEST(finished_send_is_reported_once) {
  SendTimer t; int s = t.begin(1000); t.end(s, 46000);
  CHECK(t.take_longest_us(50000) == 45000);
  CHECK(t.take_longest_us(60000) == 0);
}
TEST(in_progress_send_reports_its_age) {
  SendTimer t; int s = t.begin(1000);
  CHECK(t.take_longest_us(31000) == 30000);
  CHECK(t.take_longest_us(61000) == 60000);
  t.end(s, 70000);
  CHECK(t.take_longest_us(80000) == 69000);
  CHECK(t.take_longest_us(90000) == 0);
}
TEST(longest_of_concurrent_sends) {
  SendTimer t; int a = t.begin(1000); int b = t.begin(2000);
  t.end(b, 4000);                                   // 2 ms
  CHECK(t.take_longest_us(11000) == 10000);         // a still running
  t.end(a, 12000);
}
TEST(slots_exhausted_is_harmless) {
  SendTimer t; int s[SendTimer::kSlots];
  for (int& x : s) { x = t.begin(1000); CHECK(x >= 0); }
  CHECK(t.begin(1000) == -1);
  t.end(-1, 2000);
  for (int x : s) t.end(x, 2000);
  CHECK(t.begin(3000) >= 0);                        // slots are reusable
}

MTEST_MAIN
