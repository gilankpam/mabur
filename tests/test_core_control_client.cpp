#include "mtest.h"
#include "rec_control.h"
#include "core_control_client.h"

// Round trip over real loopback sockets on a test port (not 8401, so a
// running maburgs on the dev box cannot interfere).
TEST(client_sends_on_change_and_every_period) {
  constexpr int kPort = 18401;
  maburgs::RecControl ctl;
  REQUIRE(ctl.open(kPort));
  maburplay::CoreControlClient cli;
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

TEST(client_send_line_reaches_the_refresh_queue) {
  constexpr int kPort = 18402;
  maburgs::RecControl ctl;
  REQUIRE(ctl.open(kPort));
  maburplay::CoreControlClient cli;
  REQUIRE(cli.open(kPort));
  cli.send_line("refresh start");
  CHECK(cli.sent() == 1);
  ctl.poll();
  const auto reasons = ctl.take_refresh();
  REQUIRE(reasons.size() == 1);
  CHECK(reasons[0] == "start");
  CHECK(ctl.wire() == 0);  // a refresh line never touches the VTX wish
}

MTEST_MAIN
