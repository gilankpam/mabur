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

// scout_pick.h — the card for the hop freshness burst: a scout-capable card
// that is not transmitting; else the last scout-capable card even if it IS
// the TX card (the burst only runs once the verdict has fired, i.e. the link
// is already impaired -- the same acceptance a one-card GS already has);
// -1 when no card can scout at all (the burst is skipped).
inline int pick_burst_card(const std::vector<bool>& can_scout, int tx) {
  const int non_tx = pick_inflight_scout(can_scout, tx);
  if (non_tx >= 0) return non_tx;
  return pick_boot_scout(can_scout);   // last scout-capable card, TX or not; -1 if none
}

// DISC beacon targets during the boot scan. The scout card beacons only
// while it is home (one-card mode: it interleaves home windows); a USB home
// card (two-card mode: first USB card != scout) beacons always; every relay
// that is ready() beacons as well -- but a relay is never the ONLY path to
// rendezvous, because a CPE that is still booting, unplugged or owned by
// another client would otherwise mean no DISC ever leaves the GS.
// `ready` is per card; `n_usb` USB cards come first in the roster.
inline std::vector<int> scan_disc_targets(int n_usb, int n_cards, int scout_card,
                                          bool scout_at_home,
                                          const std::vector<bool>& ready) {
  std::vector<int> out;
  if (n_cards <= 0) return out;
  if (n_usb >= 2) {
    out.push_back(scout_card == 0 ? 1 : 0);   // the first USB card != scout
  } else if (n_usb == 1 && scout_at_home) {
    out.push_back(scout_card);
  }
  for (int i = n_usb; i < n_cards && i < static_cast<int>(ready.size()); ++i)
    if (ready[static_cast<size_t>(i)]) out.push_back(i);
  return out;
}

// The in-flight hop lead: the first ready() card that is not transmitting,
// any type (a relay leads via TUNE by design); -1 when none is ready -- the
// caller then runs the one-card hop path (n_cards 1) instead of leading on a
// dead card.
inline int pick_hop_lead(const std::vector<bool>& ready, int tx) {
  for (int i = 0; i < static_cast<int>(ready.size()); ++i)
    if (i != tx && ready[static_cast<size_t>(i)]) return i;
  return -1;
}

}  // namespace maburgs
