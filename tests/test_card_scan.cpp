// Startup card scan: the policy half (no USB). The bus half -- opening a
// device and asking the chip what it is -- lives in card_scan.cpp behind
// the Enumerate callback these tests substitute.
#include <cstdint>
#include <vector>

#include "card_scan.h"
#include "mtest.h"

namespace {

// A fake clock the fake sleep advances: the settle loop's timing is then
// exact and the test runs in microseconds instead of seconds.
struct FakeClock {
  uint64_t ms = 0;
  maburgs::NowMs now() { return [this] { return ms; }; }
  maburgs::SleepMs sleep() { return [this](int d) { ms += static_cast<uint64_t>(d); }; }
};

maburgs::ScannedCard card(uint8_t bus, std::vector<uint8_t> path) {
  maburgs::ScannedCard c;
  c.bus = bus;
  c.port_path = std::move(path);
  c.usb_vid = 0x0bda;
  c.usb_pid = 0xa81a;
  return c;
}

maburgs::ScanPolicy fast_policy() {
  maburgs::ScanPolicy p;
  p.poll_ms = 500;
  p.settle_ms = 2000;
  p.timeout_ms = 15000;
  p.max_cards = 4;
  return p;
}

}  // namespace

// The boot race this exists for: USB enumeration is still in flight when
// maburgs starts, so the first poll sees one card and the second sees both.
// Returning after the first poll would give a permanently one-card GS.
TEST(scan_waits_for_the_count_to_stop_growing) {
  FakeClock clk;
  int polls = 0;
  auto enumerate = [&polls]() -> std::vector<maburgs::ScannedCard> {
    if (++polls == 1) return {card(2, {1})};
    return {card(2, {1}), card(2, {2})};
  };
  const auto found = maburgs::scan_until_settled(fast_policy(), enumerate,
                                                 clk.now(), clk.sleep());
  CHECK(found.size() == 2);
  // Settled only after settle_ms of an unchanged count, not on first sight.
  CHECK(clk.ms >= 2000);
}

// Nothing on the bus yet: keep polling rather than concluding "no radio".
TEST(scan_waits_for_the_first_card_to_appear) {
  FakeClock clk;
  int polls = 0;
  auto enumerate = [&polls]() -> std::vector<maburgs::ScannedCard> {
    if (++polls < 6) return {};
    return {card(2, {1})};
  };
  const auto found = maburgs::scan_until_settled(fast_policy(), enumerate,
                                                 clk.now(), clk.sleep());
  CHECK(found.size() == 1);
}

// An empty bus must end the wait eventually: main.cpp turns this into a
// exit(1) so the S96maburgs wrapper respawns instead of running blind.
TEST(scan_gives_up_on_an_empty_bus_after_the_timeout) {
  FakeClock clk;
  auto enumerate = []() -> std::vector<maburgs::ScannedCard> { return {}; };
  const auto found = maburgs::scan_until_settled(fast_policy(), enumerate,
                                                 clk.now(), clk.sleep());
  CHECK(found.empty());
  CHECK(clk.ms >= 15000);
}

// card_id must mean the same physical port across reboots, so the order is
// (bus, port path) -- not libusb enumeration order, which is arrival order.
TEST(scan_orders_cards_by_bus_then_port_path) {
  FakeClock clk;
  auto enumerate = []() -> std::vector<maburgs::ScannedCard> {
    return {card(2, {4}), card(1, {2, 1}), card(2, {3}), card(1, {2})};
  };
  const auto found = maburgs::scan_until_settled(fast_policy(), enumerate,
                                                 clk.now(), clk.sleep());
  CHECK(found.size() == 4);
  CHECK(found[0].bus == 1 && found[0].port_path == std::vector<uint8_t>{2});
  CHECK(found[1].bus == 1 && found[1].port_path == std::vector<uint8_t>({2, 1}));
  CHECK(found[2].bus == 2 && found[2].port_path == std::vector<uint8_t>{3});
  CHECK(found[3].bus == 2 && found[3].port_path == std::vector<uint8_t>{4});
}

// The player OSD draws at most kMaxCards rows; more than that is a wiring
// mistake, not a link to receive on.
TEST(scan_clamps_to_max_cards) {
  FakeClock clk;
  auto enumerate = []() -> std::vector<maburgs::ScannedCard> {
    return {card(1, {1}), card(1, {2}), card(1, {3}),
            card(1, {4}), card(1, {5}), card(1, {6})};
  };
  const auto found = maburgs::scan_until_settled(fast_policy(), enumerate,
                                                 clk.now(), clk.sleep());
  CHECK(found.size() == 4);
  CHECK(found[3].port_path == std::vector<uint8_t>{4});
}

// tx_card is an index into a list that, under auto-scan, is only known after
// the scan. A pin that outruns the hardware is a missing antenna, not a
// config error -- refusing to start would cost the whole link because one
// card did not enumerate, while TxSelector already handles a pinned card
// that dies by falling through to auto-selection.
TEST(a_tx_card_pin_beyond_the_found_cards_falls_back_to_auto) {
  CHECK(maburgs::effective_tx_card(-1, 2) == -1);  // auto stays auto
  CHECK(maburgs::effective_tx_card(1, 2) == 1);    // in range, honoured
  CHECK(maburgs::effective_tx_card(0, 1) == 0);
  CHECK(maburgs::effective_tx_card(1, 1) == -1);   // second card missing
  CHECK(maburgs::effective_tx_card(3, 0) == -1);
}

MTEST_MAIN
