#pragma once
// Per-card RX width as desired state (docs/bw40.md "Boot scan dwells once
// per pair"). Every card, the boot scout card included, InitWrites at the
// receiver width and the scout never changes a card's width. A card that
// still reads another width -- a revive that came up at the wrong width, or
// any card after a live width flip (ChannelCore::set_width) -- would
// leave a one-card GS capped at the 20 rungs, a two-card GS without
// diversity on the 40 rungs. The core loop therefore checks, every tick and
// for every card, whether a one-shot set_width is due; these two
// predicates are that check, kept pure for the test.
#include <cstdint>

namespace maburgs {

// The fix-up is closed while a scan holds the scout card: the boot pick is
// open (the scout measures the set, and a flip would change what it
// ranks), or a calibration measure round runs. No scout at all (no
// scout-capable card) is open from the start. Outside those, a scout that
// only searches (unlinked) keeps its own card -- that card is skipped
// (WidthCard::scouting) and every other card is fixed.
inline bool width_resync_open(bool has_scout, bool pick_open, bool round_open) {
  return !has_scout || (!pick_open && !round_open);
}

struct WidthCard {
  bool ready = false;     // RadioFrontend::ready()
  uint8_t width = 20;     // RadioFrontend::width()
  bool tried = false;     // a fix-up already ran on this bring-up
  bool busy = false;      // the in-flight scout thread has it off on a dwell
  bool scouting = false;  // the boot scout has it (searching): fixed once the search lets go
};

// Once per bring-up and width (tried resets when the card is reopened, when
// the boot scout lets go of it, or when the width flips): a failed
// set_width is logged, not retried every tick.
inline bool needs_width_fix(const WidthCard& c, uint8_t want_mhz) {
  return c.ready && !c.tried && !c.busy && !c.scouting && c.width != want_mhz;
}

}  // namespace maburgs
