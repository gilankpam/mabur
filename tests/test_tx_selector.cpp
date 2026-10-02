#include "mtest.h"
#include "tx_selector.h"
using namespace maburgs;

// RSSI raw = dBm + 110: 50 = -60 dBm.
static CardSnapshot snap(double rssi_raw, uint64_t last_us) { return CardSnapshot{true, rssi_raw, last_us}; }

TEST(hysteresis_switch_needs_sustained_margin) {
  TxSelector sel(TxSelectorCfg{-1, 3.0, 2000, 1500}, 2);
  std::vector<CardSnapshot> cards = {snap(50.0, 0), snap(51.0, 0)};
  CHECK(sel.update(cards, 0) == 0);
  for (uint64_t t = 0; t < 5'000'000; t += 100'000) {
    cards[0].last_frame_us = cards[1].last_frame_us = t;
    CHECK(sel.update(cards, t) == 0);               // +1 dB: never
  }
  cards[1].rssi_ema = 54.0;                         // +4 dB
  uint64_t t = 5'000'000;
  CHECK(sel.update(cards, t) == 0);
  cards[0].last_frame_us = cards[1].last_frame_us = t + 1'000'000;
  CHECK(sel.update(cards, t + 1'000'000) == 0);
  cards[0].last_frame_us = cards[1].last_frame_us = t + 2'100'000;
  CHECK(sel.update(cards, t + 2'100'000) == 1);
  CHECK(sel.switches() == 1);
}

TEST(equal_rssi_never_switches) {
  TxSelector sel(TxSelectorCfg{-1, 3.0, 2000, 1500}, 2);
  std::vector<CardSnapshot> cards = {snap(48.0, 0), snap(48.0, 0)};
  for (uint64_t t = 0; t < 10'000'000; t += 100'000) {
    cards[0].last_frame_us = cards[1].last_frame_us = t;
    cards[1].rssi_ema = 48.0 + ((t / 100'000) % 2 ? 2.9 : -2.9);   // jitter inside the margin
    CHECK(sel.update(cards, t) == 0);
  }
  CHECK(sel.switches() == 0);
}

TEST(challenge_resets_if_margin_drops) {
  TxSelector sel(TxSelectorCfg{-1, 3.0, 2000, 1500}, 2);
  std::vector<CardSnapshot> cards = {snap(50.0, 0), snap(54.0, 0)};
  sel.update(cards, 0);
  cards[1].rssi_ema = 51.0;
  cards[0].last_frame_us = cards[1].last_frame_us = 1'000'000;
  CHECK(sel.update(cards, 1'000'000) == 0);
  cards[1].rssi_ema = 54.0;
  cards[0].last_frame_us = cards[1].last_frame_us = 2'000'000;
  CHECK(sel.update(cards, 2'000'000) == 0);
  cards[0].last_frame_us = cards[1].last_frame_us = 3'500'000;
  CHECK(sel.update(cards, 3'500'000) == 0);
}

TEST(dead_card_switches_immediately) {
  TxSelector sel(TxSelectorCfg{-1, 3.0, 2000, 1500}, 2);
  std::vector<CardSnapshot> cards = {snap(55.0, 0), snap(50.0, 0)};
  CHECK(sel.update(cards, 0) == 0);
  cards[1].last_frame_us = 2'000'000;
  CHECK(sel.update(cards, 2'000'000) == 1);
  cards[0].alive = false;
  CHECK(sel.update(cards, 2'100'000) == 1);
}

TEST(pin_overrides_unless_dead) {
  TxSelector sel(TxSelectorCfg{1, 3.0, 2000, 1500}, 2);
  std::vector<CardSnapshot> cards = {snap(60.0, 0), snap(40.0, 0)};
  CHECK(sel.update(cards, 0) == 1);
  cards[1].alive = false;
  CHECK(sel.update(cards, 100'000) == 0);
}

TEST(relay_with_high_synthetic_snr_but_lower_rssi_loses) {
  // The relay's SNR is RSSI+95 and would have won an SNR argmax; it does
  // not reach the selector at all now -- only RSSI does.
  TxSelector sel(TxSelectorCfg{-1, 3.0, 2000, 1500}, 2);
  std::vector<CardSnapshot> cards = {snap(55.0, 0), snap(45.0, 0)};   // -55 dBm USB vs -65 dBm relay
  for (uint64_t t = 0; t < 5'000'000; t += 100'000) {
    cards[0].last_frame_us = cards[1].last_frame_us = t;
    CHECK(sel.update(cards, t) == 0);
  }
}
MTEST_MAIN
