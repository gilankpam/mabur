#include "scout_pick.h"
#include "mtest.h"
using namespace maburgs;

TEST(boot_scout_is_last_scout_capable_card) {
  CHECK(pick_boot_scout({true}) == 0);                 // one USB card: itself
  CHECK(pick_boot_scout({true, true}) == 1);           // two USB: the spare (last)
  CHECK(pick_boot_scout({true, false}) == 0);          // USB + relay: the USB card
  CHECK(pick_boot_scout({true, true, false}) == 1);    // two USB + relay
  CHECK(pick_boot_scout({false}) == -1);               // relay only: nobody
  CHECK(pick_boot_scout({}) == -1);
}

TEST(inflight_scout_is_last_scout_capable_non_tx_card) {
  CHECK(pick_inflight_scout({true, true}, 0) == 1);
  CHECK(pick_inflight_scout({true, true}, 1) == 0);
  CHECK(pick_inflight_scout({true, false}, 1) == 0);   // relay transmits: USB card dwells
  CHECK(pick_inflight_scout({true, false}, 0) == -1);  // USB transmits: nothing can dwell
  CHECK(pick_inflight_scout({true, true, false}, 0) == 1);
  CHECK(pick_inflight_scout({true}, 0) == -1);         // one card never dwells
}
MTEST_MAIN
