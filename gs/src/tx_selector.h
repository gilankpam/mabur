#pragma once
#include <cstdint>
#include <vector>

namespace maburgs {

struct CardSnapshot {
  bool alive = false;
  double snr_ema = 0.0;
  double rssi_b_ema = 0.0;
  uint64_t last_frame_us = 0;
};

struct TxSelectorCfg {
  int pinned = -1;
  double switch_margin_db = 3.0;
  int hold_ms = 2000;
  int card_dead_ms = 1500;
};

class TxSelector {
 public:
  TxSelector(TxSelectorCfg cfg, int n_cards);
  int update(const std::vector<CardSnapshot>& cards, uint64_t now_us);
  int selected() const;
  uint64_t switches() const;

 private:
  TxSelectorCfg cfg_;
  int n_cards_;
  int selected_ = 0;
  int challenger_ = -1;
  uint64_t challenge_since_us_ = 0;
  uint64_t switches_ = 0;

  bool dead(const CardSnapshot& c, uint64_t now_us) const;
  int best_alive(const std::vector<CardSnapshot>& cards,
                 uint64_t now_us) const;
};

}  // namespace maburgs
