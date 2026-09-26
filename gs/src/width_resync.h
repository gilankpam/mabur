#pragma once
// Per-card RX width as desired state (docs/bw40.md "Boot scan"). The boot
// scout card InitWrites at 20 MHz and its only 20 -> radio.width switch is
// ChannelScout::run()'s retune_width at the freeze. Two paths skip that:
// the scout card dying mid-scan (the revive reopens it), and the first
// DISC_ACK freezing the plan before the scout thread ever started. Either
// leaves the card at 20 -- a one-card GS capped at the 20 rungs, a two-card
// GS without diversity on the 40 rungs. The core loop therefore checks,
// every tick and for every card, whether a one-shot set_width is due;
// these two predicates are that check, kept pure for the test.
#include <cstdint>

namespace maburgs {

// The fix-up may touch cards only once no boot scout thread owns one:
// joined (after run(), or never started) AND the scan is over (frozen).
// No scout at all (radio.scan.enable off) is open from the start.
inline bool width_resync_open(bool scout_joined, bool has_scout, bool scout_frozen) {
  return scout_joined && (!has_scout || scout_frozen);
}

struct WidthCard {
  bool ready = false;   // RadioFrontend::ready()
  uint8_t width = 20;   // RadioFrontend::width()
  bool tried = false;   // a fix-up already ran on this bring-up
  bool busy = false;    // the in-flight scout thread has it off on a dwell
};

// Once per bring-up (tried resets when the card is reopened): a failed
// set_width is logged, not retried every tick.
inline bool needs_width_fix(const WidthCard& c, uint8_t want_mhz) {
  return c.ready && !c.tried && !c.busy && c.width != want_mhz;
}

}  // namespace maburgs
