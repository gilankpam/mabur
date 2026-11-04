#pragma once
// Per-card RX width as desired state (docs/bw40.md "Boot scan dwells once
// per pair"). Every card, the boot scout card included, InitWrites at
// radio.width and the scout never changes a card's width. A card that still
// reads another width -- a revive that came up at the wrong width -- would
// leave a one-card GS capped at the 20 rungs, a two-card GS without
// diversity on the 40 rungs. The core loop therefore checks, every tick and
// for every card, whether a one-shot set_width is due, once no scout thread
// can own a card (including one that never started because the first
// DISC_ACK froze the plan first); these two predicates are that check,
// kept pure for the test.
#include <cstdint>

namespace maburgs {

// The fix-up may touch cards only once no boot scout thread owns one:
// joined (after run(), or never started) AND the scan is over (frozen).
// No scout at all (pinned, or no scout-capable card) is open from the start.
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
