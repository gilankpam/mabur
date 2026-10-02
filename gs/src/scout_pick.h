#pragma once
// Which card scouts (spec 2026-10-02-maburgs-remote-card §3). Pure, like
// width_resync.h. The relay (RemoteCard) has no FA/CCA/NHM reads and so is
// never a scout; USB cards come first in the roster, so "last capable" is
// the spare USB card -- the same choice as the old n_cards - 1 rule on an
// all-USB GS.
#include <vector>

namespace maburgs {

inline int pick_boot_scout(const std::vector<bool>& can_scout) {
  for (int i = static_cast<int>(can_scout.size()) - 1; i >= 0; --i)
    if (can_scout[static_cast<size_t>(i)]) return i;
  return -1;
}

// The in-flight scout dwells on a card that is NOT transmitting (an
// off-channel TX card would lose every RCF). -1 = skip this period.
inline int pick_inflight_scout(const std::vector<bool>& can_scout, int tx) {
  if (can_scout.size() < 2) return -1;
  for (int i = static_cast<int>(can_scout.size()) - 1; i >= 0; --i)
    if (i != tx && can_scout[static_cast<size_t>(i)]) return i;
  return -1;
}

}  // namespace maburgs
