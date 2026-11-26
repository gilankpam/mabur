#include "../drone/src/mav_uplink_gate.h"
#include "mtest.h"

using mabur::MavUplinkGate;
using mabur::kMavUplinkMinMs;

TEST(heartbeat_once_per_second_when_linked) {
  MavUplinkGate g;
  CHECK(g.heartbeat_due(1000, true, false));
  CHECK(!g.heartbeat_due(1999, true, false));
  CHECK(g.heartbeat_due(2000, true, false));
}

TEST(no_heartbeat_unlinked_or_sweeping) {
  MavUplinkGate g;
  CHECK(!g.heartbeat_due(5000, false, false));
  CHECK(!g.heartbeat_due(5000, true, true));
  CHECK(g.heartbeat_due(5000, true, false));  // refusals did not consume the slot
}

TEST(uplink_paced_when_linked) {
  MavUplinkGate g;
  CHECK(g.uplink_due(1000, true, false, true, false, 1));
  CHECK(!g.uplink_due(1000 + kMavUplinkMinMs - 1, true, false, true, false, 1));
  CHECK(g.uplink_due(1000 + kMavUplinkMinMs, true, false, true, false, 1));
}

TEST(uplink_unpaced_on_update_pair_only) {
  MavUplinkGate g;
  CHECK(g.uplink_due(1000, false, true, true, false, 1));
  CHECK(g.uplink_due(1000, false, true, true, false, 1));
  CHECK(!g.uplink_due(1000, false, false, true, false, 1));  // nobody listening
}

TEST(uplink_needs_pair_and_packets) {
  MavUplinkGate g;
  CHECK(!g.uplink_due(1000, true, false, false, false, 1));  // no pair
  CHECK(!g.uplink_due(1000, true, false, true, false, 0));   // nothing queued
  CHECK(!g.uplink_due(1000, true, false, true, true, 1));    // sweeping
  CHECK(g.uplink_due(1000, true, false, true, false, 1));
}

TEST(exit_immediate_when_nothing_queued) {
  MavUplinkGate g;
  CHECK(!g.exit_now(100, false, 0));
  CHECK(g.exit_now(100, true, 0));
}

TEST(exit_waits_for_replies_at_most_grace) {
  MavUplinkGate g;
  CHECK(!g.exit_now(100, true, 3));
  CHECK(!g.exit_now(1099, true, 2));
  CHECK(g.exit_now(1100, true, 2));
}

MTEST_MAIN
