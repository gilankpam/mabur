#include "rf_labels.h"
#include "mtest.h"

using namespace maburgs;

// The RF staleness gate (spec 2026-08-14 fade-demote §3). CardTrack's EMAs
// freeze at their last value when frames stop arriving — which is exactly
// what a deep fade does — so a card that received nothing this feedback
// window must not supply the RF labels the predictive fade trigger reads.

TEST(first_window_counts_as_fresh) {
  // prev_frames is zero-initialised at daemon start, so on the very first
  // feedback window "fresh" degenerates to "has ever received an s1 frame".
  // That is the right answer: with no previous window, ever == this window.
  const std::vector<CardLabelInput> cards{{true, 1, 0, 20.0}};
  CHECK(select_label_card(cards) == 0);
}

TEST(zero_frame_window_is_stale) {
  // Counter did not advance: every label on this card is a frozen EMA.
  const std::vector<CardLabelInput> cards{{true, 5, 5, 40.0}};
  CHECK(select_label_card(cards) == -1);
}

TEST(no_card_selected_when_none_has_ema) {
  const std::vector<CardLabelInput> cards{{false, 0, 0, 0.0},
                                            {false, 0, 0, 0.0}};
  CHECK(select_label_card(cards) == -1);
}

TEST(no_card_selected_when_all_stale) {
  // Every card silent this window -> no labels at all. The caller NaNs all
  // three, which leaves the fade trigger inert.
  const std::vector<CardLabelInput> cards{{true, 9, 9, 40.0},
                                            {true, 3, 3, 30.0}};
  CHECK(select_label_card(cards) == -1);
}

TEST(no_card_selected_when_no_cards) {
  const std::vector<CardLabelInput> cards{};
  CHECK(select_label_card(cards) == -1);
}

TEST(strongest_snr_wins_among_fresh_cards) {
  // With freshness equal, the selector is still the plain argmax it always
  // was (snr_ema is raw half-dB; ordering is scale-invariant).
  const std::vector<CardLabelInput> cards{{true, 4, 1, 20.0},
                                            {true, 7, 2, 44.0},
                                            {true, 9, 3, 31.0}};
  CHECK(select_label_card(cards) == 1);
}

TEST(wedged_card_never_outranks_a_live_sibling) {
  // THE fix-1 case. Card 0's front-end is wedged: its s1 EMA froze at a high
  // value and never decays, so a freshness-blind argmax picks it forever and
  // (with the gate below it) NaNs the labels permanently — losing the live
  // sibling's real measurement and silently disabling the fade trigger on a
  // multi-card GS.
  const std::vector<CardLabelInput> cards{{true, 5, 5, 60.0},   // frozen high
                                            {true, 12, 7, 20.0}};  // live, weaker
  CHECK(select_label_card(cards) == 1);
}

TEST(freshness_baseline_is_per_card_not_a_scalar) {
  // Both cards report the SAME cumulative counter; only their per-card
  // baselines differ. No single scalar baseline can tell these apart, so this
  // pins prev_frames as a per-card snapshot.
  const std::vector<CardLabelInput> cards{{true, 100, 100, 55.0},  // stale
                                            {true, 100, 40, 18.0}};  // fresh
  CHECK(select_label_card(cards) == 1);
}

TEST(selection_follows_freshness_across_windows) {
  // Window A: both live, card 0 stronger -> card 0. Window B: card 0 goes
  // silent (counter frozen at its window-A value, EMA frozen high) while card
  // 1 keeps receiving -> selection must move to card 1, not stick.
  const std::vector<CardLabelInput> win_a{{true, 30, 10, 50.0},
                                            {true, 28, 9, 22.0}};
  CHECK(select_label_card(win_a) == 0);
  const std::vector<CardLabelInput> win_b{{true, 30, 30, 50.0},
                                            {true, 46, 28, 22.0}};
  CHECK(select_label_card(win_b) == 1);
}

TEST(card_without_ema_is_never_selected) {
  // has_ema guards snr_ema's validity: a card that has never folded an s1
  // frame has a meaningless 0.0 EMA that must not enter the argmax.
  const std::vector<CardLabelInput> cards{{false, 8, 2, 99.0},
                                            {true, 5, 4, 12.0}};
  CHECK(select_label_card(cards) == 1);
}

TEST(selection_is_unchanged_by_the_pooled_input) {
  // The selector is pure and input-shaped: moving the caller from cls[S1]
  // to rf_pool changes what fills CardLabelInput, not how it chooses. This
  // pins that contract so a future change to the pool cannot silently
  // alter selection semantics.
  const std::vector<CardLabelInput> cards{{true, 40, 10, 22.0},   // fresh, weaker
                                          {true, 90, 60, 31.0}};  // fresh, stronger
  CHECK(select_label_card(cards) == 1);
  // Same SNR ordering, but the stronger card went silent this window.
  const std::vector<CardLabelInput> stale{{true, 40, 10, 22.0},
                                          {true, 90, 90, 31.0}};
  CHECK(select_label_card(stale) == 0);
}

MTEST_MAIN
