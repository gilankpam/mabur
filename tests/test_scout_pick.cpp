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
TEST(burst_card_prefers_non_tx_scout_capable_then_tx_scout_capable) {
  CHECK(pick_burst_card({true, true}, 0) == 1);        // two USB: the non-TX one (unchanged)
  CHECK(pick_burst_card({true, true}, 1) == 0);
  CHECK(pick_burst_card({true}, 0) == 0);              // one card: itself (unchanged)
  CHECK(pick_burst_card({true, false}, 1) == 0);       // relay transmits: USB card bursts
  CHECK(pick_burst_card({true, false}, 0) == 0);       // USB transmits: USB card bursts anyway (never the relay)
  CHECK(pick_burst_card({true, true, false}, 0) == 1);
  CHECK(pick_burst_card({false}, 0) == -1);            // nothing can scout: skip
  CHECK(pick_burst_card({false, false}, 1) == -1);
}
TEST(scan_disc_targets_never_rely_on_the_relay_alone) {
  using V = std::vector<int>;
  // two USB: the non-scout card always; the scout card too while it bursts
  CHECK(scan_disc_targets(2, 2, 1, false, {true, true}) == V{0});
  CHECK(scan_disc_targets(2, 2, 1, true, {true, true}) == (V{0, 1}));
  CHECK(scan_disc_targets(2, 3, 1, false, {true, true, true}) == (V{0, 2}));   // + ready relay
  CHECK(scan_disc_targets(2, 3, 1, false, {true, true, false}) == V{0});       // relay not ready
  // one USB: the sole card only while it beacons (op window / burst)
  CHECK(scan_disc_targets(1, 2, 0, true, {true, true}) == (V{0, 1}));
  CHECK(scan_disc_targets(1, 2, 0, false, {true, true}) == V{1});
  CHECK(scan_disc_targets(1, 2, 0, false, {true, false}) == V{});
  CHECK(scan_disc_targets(1, 1, 0, true, {true}) == V{0});
  CHECK(scan_disc_targets(1, 1, 0, false, {true}) == V{});
  CHECK(scan_disc_targets(0, 0, -1, true, {}) == V{});
}

TEST(hop_lead_is_first_ready_non_tx_card) {
  CHECK(pick_hop_lead({true, true}, 0) == 1);
  CHECK(pick_hop_lead({true, false}, 0) == -1);       // the non-TX card (relay/dead USB) is down
  CHECK(pick_hop_lead({false, true}, 1) == -1);
  CHECK(pick_hop_lead({true, true, true}, 1) == 0);
  CHECK(pick_hop_lead({true}, 0) == -1);              // one card: the one-card hop path
}

TEST(scan_disc_targets_relay_only_roster_uses_scout_relay_while_beaconing) {
  // n_usb 0, two relays, scout = relay 1 (the last)
  std::vector<bool> ready{true, true};
  auto t = scan_disc_targets(0, 2, 1, /*scout_beaconing=*/true, ready);
  CHECK(t.size() == 2);                       // relay 0 (ready) + the scout relay (beaconing)
  t = scan_disc_targets(0, 2, 1, false, ready);
  REQUIRE(t.size() == 1);
  CHECK(t[0] == 0);                           // the scout relay off a burst is silent
  ready = {false, false};
  t = scan_disc_targets(0, 2, 1, true, ready);
  CHECK(t.empty());                           // not ready: no DISC leaves at all
}
MTEST_MAIN
