#include "drone_restart.h"
#include "mtest.h"

// The drone's T_TELEM tlm_seq restarts from 0 at every maburd start. A large
// backwards step against the last accepted value is the only signal the GS
// has that the peer rebooted (DiscAck carries no boot id), and it is what
// rotates the debug-log session into a new flight directory.

TEST(first_sample_never_fires) {
  maburgs::DroneRestartDetector d;
  CHECK(!d.on_telem(0, 1000.0));
  CHECK(!d.on_telem(5, 2000.0));
}

TEST(monotonic_and_small_reorder_do_not_fire) {
  maburgs::DroneRestartDetector d;
  d.on_telem(500, 0.0);
  CHECK(!d.on_telem(501, 1000.0));
  CHECK(!d.on_telem(498, 1100.0));   // reorder/late frame: tolerated
  CHECK(!d.on_telem(401, 1200.0));   // exactly the threshold: still tolerated
}

TEST(large_backwards_step_fires_once) {
  maburgs::DroneRestartDetector d;
  d.on_telem(5000, 0.0);
  CHECK(d.on_telem(3, 1000.0));       // drone rebooted: 5000 -> 3
  CHECK(!d.on_telem(4, 2000.0));      // climbing again
}

TEST(wrap_at_65535_does_not_fire) {
  maburgs::DroneRestartDetector d;
  d.on_telem(65530, 0.0);
  CHECK(!d.on_telem(2, 1000.0));
}

TEST(holdoff_suppresses_a_second_fire_within_10s) {
  maburgs::DroneRestartDetector d;
  d.on_telem(5000, 0.0);
  CHECK(d.on_telem(1, 1000.0));
  d.on_telem(4000, 2000.0);           // garbage / late frame from the old run
  CHECK(!d.on_telem(2, 3000.0));      // 2 s after the first fire: held off
  d.on_telem(4000, 12000.0);
  CHECK(d.on_telem(3, 12001.0));      // 11 s after: fires again
}

MTEST_MAIN
