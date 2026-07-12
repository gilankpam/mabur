#include "tx_selector.h"

namespace maburgs {

TxSelector::TxSelector(TxSelectorCfg cfg, int n_cards)
    : cfg_(cfg), n_cards_(n_cards) {
  if (cfg_.pinned >= 0 && cfg_.pinned < n_cards_) selected_ = cfg_.pinned;
}

bool TxSelector::dead(const CardSnapshot& c, uint64_t now_us) const {
  if (!c.alive) return true;
  return now_us - c.last_frame_us >
         static_cast<uint64_t>(cfg_.card_dead_ms) * 1000;
}

int TxSelector::best_alive(const std::vector<CardSnapshot>& cards,
                           uint64_t now_us) const {
  int best = -1;
  for (int i = 0; i < n_cards_ && i < static_cast<int>(cards.size()); ++i) {
    if (dead(cards[static_cast<size_t>(i)], now_us)) continue;
    if (best < 0 ||
        cards[static_cast<size_t>(i)].snr_ema > cards[static_cast<size_t>(best)].snr_ema)
      best = i;
  }
  return best < 0 ? selected_ : best;  // all dead: keep current (TX anyway)
}

int TxSelector::update(const std::vector<CardSnapshot>& cards, uint64_t now_us) {
  if (cfg_.pinned >= 0 && cfg_.pinned < static_cast<int>(cards.size())) {
    if (!dead(cards[static_cast<size_t>(cfg_.pinned)], now_us)) {
      selected_ = cfg_.pinned;
      return selected_;
    }
    // pinned card dead: fall through to auto-selection until it returns
  }
  const CardSnapshot& cur = cards[static_cast<size_t>(selected_)];
  if (dead(cur, now_us)) {
    const int next = best_alive(cards, now_us);
    if (next != selected_) { selected_ = next; ++switches_; }
    challenger_ = -1;
    return selected_;
  }
  const int cand = best_alive(cards, now_us);
  if (cand != selected_ &&
      cards[static_cast<size_t>(cand)].snr_ema >=
          cur.snr_ema + cfg_.switch_margin_db) {
    if (challenger_ != cand) {
      challenger_ = cand;
      challenge_since_us_ = now_us;
    } else if (now_us - challenge_since_us_ >=
               static_cast<uint64_t>(cfg_.hold_ms) * 1000) {
      selected_ = cand;
      ++switches_;
      challenger_ = -1;
    }
  } else {
    challenger_ = -1;
  }
  return selected_;
}

int TxSelector::selected() const { return selected_; }
uint64_t TxSelector::switches() const { return switches_; }

}  // namespace maburgs
