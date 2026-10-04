// UsbLate (gs/src/usb_late.h): p99/max of host-vs-TSF lateness above the
// window's minimum, TSF wrap handled, window reset on take().
#include "mtest.h"
#include "usb_late.h"
using namespace maburgs;

TEST(p99_and_max_above_min_then_reset) {
  UsbLate u;
  for (int i = 0; i < 100; ++i) u.add(/*host*/ 1000 + i, /*tsf*/ 1000);   // offsets 0..99
  int64_t p99 = -1, mx = -1;
  u.take(p99, mx);
  CHECK(p99 == 99);
  CHECK(mx == 99);
  u.take(p99, mx);           // window reset
  CHECK(p99 == 0 && mx == 0);
}

TEST(tsf_wrap_keeps_offsets_continuous) {
  UsbLate u;
  u.add(/*host*/ 5'000'000'000LL, 0xFFFFFF00u);
  u.add(5'000'000'000LL + 0x200, 0x100u);     // tsf wrapped by 0x200 us; host moved 0x200 us -> same offset
  int64_t p99 = -1, mx = -1;
  u.take(p99, mx);
  CHECK(p99 == 0 && mx == 0);
}
MTEST_MAIN
