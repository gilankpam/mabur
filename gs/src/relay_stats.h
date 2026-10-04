#pragma once
#include <cstdint>
namespace maburgs {
// The relay card's own state for the sideport (cards[i].relay). Filled by
// RemoteCard::relay_stats(); USB cards return nullopt.
struct RelayStatsIn {
  uint8_t state = 0, ch = 0, sec = 0;  // last STATUS: 0 tuned, 1 retuning, 2 failed, 3 refused
  bool owned = false;                  // owned_and_tuned()
  uint64_t frames = 0, gaps = 0;       // FRAMEs seen; relay->GS seq gaps (not air loss)
  uint32_t your_drops = 0, tx = 0, tx_fail = 0, tx_refused = 0;
  uint32_t reconnects = 0;             // client restarts (refused) + reopen after lost
  bool you_own = false;                // last STATUS you_own (this client holds the relay)
  uint64_t rx_drops = 0, tx_drops = 0; // the transport's own drops (the browser ring); 0 over UDP
};
}  // namespace maburgs
