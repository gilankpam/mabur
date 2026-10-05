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

// The card for the hop freshness burst (spec 2026-10-05 §4), first match:
// a scout-capable (USB) card that is not transmitting; a sweep-capable
// (relay) card that is not transmitting -- the USB TX card keeps the link
// while the relay sweeps; the scout-capable TX card (the one-card GS's
// acceptance: the burst only runs once the link is already impaired); the
// sweep-capable TX card (a relay-only GS). -1 = nothing can burst.
// can_scout and can_sweep are per card and must be the same size (the
// roster); can_sweep is the caller's per-tick "can take a SCAN now" (v4
// relay AND ready()), so a dead relay never takes the burst. A short
// can_sweep reads as false past its end.
inline int pick_burst_card(const std::vector<bool>& can_scout, const std::vector<bool>& can_sweep, int tx) {
  const int n = static_cast<int>(can_scout.size());
  auto last = [&](const std::vector<bool>& cap, bool non_tx) {
    for (int i = n - 1; i >= 0; --i)
      if (i < static_cast<int>(cap.size()) && cap[static_cast<size_t>(i)] && (!non_tx || i != tx)) return i;
    return -1;
  };
  if (const int c = last(can_scout, true); c >= 0) return c;
  if (const int c = last(can_sweep, true); c >= 0) return c;
  if (const int c = last(can_scout, false); c >= 0) return c;
  return last(can_sweep, false);
}

// DISC targets while the scout owns a card (spec 2026-10-03 §4.2).
// Two USB: the link card (always) + the scout card while it bursts DISC on a member.
// One USB: the scout card while beaconing (op window or search burst).
// Plus every relay that is ready() — but a relay is never the ONLY path to rendezvous,
// because a CPE that is still booting, unplugged or owned by another client would
// otherwise mean no DISC ever leaves the GS.
// `ready` is per card; `n_usb` USB cards come first in the roster.
inline std::vector<int> scan_disc_targets(int n_usb, int n_cards, int scout_card,
                                          bool scout_beaconing,
                                          const std::vector<bool>& ready) {
  std::vector<int> out;
  if (n_cards <= 0) return out;
  if (n_usb >= 2) {
    out.push_back(scout_card == 0 ? 1 : 0);          // the link card, on op, always
    if (scout_beaconing) out.push_back(scout_card);  // the search burst on a member
  } else if (n_usb == 1 && scout_beaconing) {
    out.push_back(scout_card);
  }
  // Every ready relay. With no USB card the scout IS a relay (search-only,
  // spec 2026-10-04 §3.7): it joins only while beaconing, like a USB scout.
  for (int i = n_usb; i < n_cards && i < static_cast<int>(ready.size()); ++i) {
    if (!ready[static_cast<size_t>(i)]) continue;
    if (n_usb == 0 && i == scout_card) { if (scout_beaconing) out.push_back(i); continue; }
    out.push_back(i);
  }
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
