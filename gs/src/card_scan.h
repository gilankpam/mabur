#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

struct libusb_context;
struct libusb_device;

// Startup card discovery. maburgs receives on every supported card it can
// find; which cards those are is a hardware fact, not a config fact, unless
// [[radio.cards]] pins a list (see config.h).
//
// Split in two on purpose: scan_until_settled() is the timing policy and
// takes its bus view through a callback, so the boot-race behaviour is
// testable without USB; enumerate_supported_cards() is the bus half, which
// opens each Realtek device and asks the chip what it is.
namespace maburgs {

// One discovered card. The identity is (bus, port path) -- the physical
// port -- not the enumeration index, so card 0 is the same antenna across
// reboots and across a card that dies and re-enumerates at a new address.
struct ScannedCard {
  uint8_t bus = 0;
  std::vector<uint8_t> port_path;
  uint16_t usb_vid = 0;
  uint16_t usb_pid = 0;
};

struct ScanPolicy {
  int poll_ms = 500;
  int settle_ms = 2000;    // count must hold this long before it is believed
  int timeout_ms = 15000;  // then give up with whatever is visible
  int max_cards = 4;       // the player OSD draws at most this many rows
};

using Enumerate = std::function<std::vector<ScannedCard>()>;
using NowMs = std::function<uint64_t()>;
using SleepMs = std::function<void(int)>;

// Polls `enumerate` until the card count has been unchanged for settle_ms
// (and is non-zero), or timeout_ms elapses. Result is sorted by (bus, port
// path) and clamped to max_cards. Empty means nothing was found in time --
// the caller exits so the wrapper can respawn.
std::vector<ScannedCard> scan_until_settled(const ScanPolicy& policy,
                                            const Enumerate& enumerate,
                                            const NowMs& now_ms,
                                            const SleepMs& sleep_ms);

// Bus half (card_scan_usb.cpp, links libusb + devourer). Every device the
// chip itself claims as a supported radio, identified by physical port.
std::vector<ScannedCard> enumerate_supported_cards(libusb_context* ctx);

// True when `dev` is plugged into the same physical port as `card`. This is
// what makes a card that dies and re-enumerates come back as the same
// card_id instead of shifting every later card up one.
bool device_at_port(libusb_device* dev, const ScannedCard& card);

// "2-1.4" -- the bus-port path, for logs.
std::string port_name(const ScannedCard& c);

// Resolve a configured radio.tx_card against the cards actually present.
// Returns -1 (auto-select) when the pin names a card the scan did not find:
// a missing antenna must not cost the whole uplink, and TxSelector already
// treats a pinned card that dies the same way.
inline int effective_tx_card(int configured, int n_cards) {
  return configured >= 0 && configured < n_cards ? configured : -1;
}

}  // namespace maburgs
