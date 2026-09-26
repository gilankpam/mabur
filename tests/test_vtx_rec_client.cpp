#include "mtest.h"
#include "rec_control.h"
#include "vtx_rec_client.h"

// Round trip over real loopback sockets on a test port (not 8401, so a
// running maburgs on the dev box cannot interfere).
TEST(client_sends_on_change_and_every_period) {
  constexpr int kPort = 18401;
  maburgs::RecControl ctl;
  REQUIRE(ctl.open(kPort));
  maburplay::VtxRecClient cli;
  REQUIRE(cli.open(kPort));
  cli.tick(true, 0);
  CHECK(cli.sent() == 1);
  ctl.poll();
  CHECK(ctl.wire() == (mabur::rc::kRecKnown | mabur::rc::kRecOn));
  cli.tick(true, 500);                 // same wish inside the period: quiet
  CHECK(cli.sent() == 1);
  cli.tick(true, 1000);                // period elapsed: re-send
  CHECK(cli.sent() == 2);
  cli.tick(false, 1001);               // change: immediate
  CHECK(cli.sent() == 3);
  ctl.poll();
  CHECK(ctl.wire() == mabur::rc::kRecKnown);
}

MTEST_MAIN
