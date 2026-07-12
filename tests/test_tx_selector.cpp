#include "mtest.h"
#include "tx_selector.h"
using namespace maburgs;

static CardSnapshot snap(double snr, uint64_t last_us) {
  return CardSnapshot{true, snr, 40.0, last_us};
}

TEST(hysteresis_switch_needs_sustained_margin) {
  TxSelector sel(TxSelectorCfg{-1, 3.0, 2000, 1500}, 2);
  std::vector<CardSnapshot> cards = {snap(20.0, 0), snap(21.0, 0)};
  CHECK(sel.update(cards, 0) == 0);              // start on card 0
  // +1 dB advantage: never switches.
  for (uint64_t t = 0; t < 5'000'000; t += 100'000) {
    cards[0].last_frame_us = cards[1].last_frame_us = t;
    CHECK(sel.update(cards, t) == 0);
  }
  // +4 dB advantage must persist hold_ms before the switch.
  cards[1].snr_ema = 24.0;
  uint64_t t = 5'000'000;
  CHECK(sel.update(cards, t) == 0);              // challenge starts
  cards[0].last_frame_us = cards[1].last_frame_us = t + 1'000'000;
  CHECK(sel.update(cards, t + 1'000'000) == 0);  // 1 s < hold
  cards[0].last_frame_us = cards[1].last_frame_us = t + 2'100'000;
  CHECK(sel.update(cards, t + 2'100'000) == 1);  // 2.1 s > hold -> switch
  CHECK(sel.switches() == 1);
}

TEST(challenge_resets_if_margin_drops) {
  TxSelector sel(TxSelectorCfg{-1, 3.0, 2000, 1500}, 2);
  std::vector<CardSnapshot> cards = {snap(20.0, 0), snap(24.0, 0)};
  sel.update(cards, 0);                          // challenge starts at t=0
  cards[1].snr_ema = 21.0;                       // margin lost
  cards[0].last_frame_us = cards[1].last_frame_us = 1'000'000;
  CHECK(sel.update(cards, 1'000'000) == 0);
  cards[1].snr_ema = 24.0;                       // margin back: clock restarts
  cards[0].last_frame_us = cards[1].last_frame_us = 2'000'000;
  CHECK(sel.update(cards, 2'000'000) == 0);
  cards[0].last_frame_us = cards[1].last_frame_us = 3'500'000;
  CHECK(sel.update(cards, 3'500'000) == 0);      // only 1.5 s since restart
}

TEST(dead_card_switches_immediately) {
  TxSelector sel(TxSelectorCfg{-1, 3.0, 2000, 1500}, 2);
  std::vector<CardSnapshot> cards = {snap(25.0, 0), snap(20.0, 0)};
  CHECK(sel.update(cards, 0) == 0);
  cards[1].last_frame_us = 2'000'000;            // card 0 silent for 2 s
  CHECK(sel.update(cards, 2'000'000) == 1);
  cards[0].alive = false;                        // and if it's down, stay away
  CHECK(sel.update(cards, 2'100'000) == 1);
}

TEST(pin_overrides_unless_dead) {
  TxSelector sel(TxSelectorCfg{1, 3.0, 2000, 1500}, 2);
  std::vector<CardSnapshot> cards = {snap(30.0, 0), snap(10.0, 0)};
  CHECK(sel.update(cards, 0) == 1);              // pinned wins despite worse SNR
  cards[1].alive = false;
  CHECK(sel.update(cards, 100'000) == 0);        // pinned dead -> best alive
}
MTEST_MAIN
