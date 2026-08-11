// The two pieces of the record button that have no device in them. The
// ioctl shell around them (RecButton) is deliberately thin and is covered
// by hardware acceptance instead.
#include "mtest.h"
#include "rec_button.h"

using maburplay::ButtonDebounce;
using maburplay::match_pin_name;

// --- pin name matching -------------------------------------------------
// The Radxa ZERO 3's device tree names header lines "PIN_7".."PIN_40".
// Other boards use "GPIO<n>" or a bare number. The whole string must be
// consumed, or PIN_320 would answer to pin 32.

TEST(pin_names_match_the_three_accepted_forms) {
  CHECK(match_pin_name("PIN_32", 32));
  CHECK(match_pin_name("GPIO32", 32));
  CHECK(match_pin_name("32", 32));
  CHECK(match_pin_name("PIN_7", 7));
}

TEST(pin_names_reject_prefixes_suffixes_and_junk) {
  CHECK(!match_pin_name("PIN_320", 32));   // longer number, same prefix
  CHECK(!match_pin_name("PIN_3", 32));     // shorter number
  CHECK(!match_pin_name("PIN_32x", 32));   // trailing junk
  CHECK(!match_pin_name("PIN_ 32", 32));   // strtol would skip the space
  CHECK(!match_pin_name("PIN_+32", 32));   // strtol would accept the sign
  CHECK(!match_pin_name("board-antenna", 32));
  CHECK(!match_pin_name("", 32));
  CHECK(!match_pin_name(nullptr, 32));
}

// --- debounce ----------------------------------------------------------
// feed() takes an ALREADY polarity-corrected level: the kernel inverts for
// us when the line is requested ACTIVE_LOW, so true means pressed.

TEST(a_button_held_at_startup_does_not_fire) {
  ButtonDebounce d;
  CHECK(!d.feed(true, 0));      // first call only seeds the baseline
  CHECK(!d.feed(true, 100));
  CHECK(!d.feed(true, 5000));
}

TEST(a_press_fires_once_and_a_hold_never_repeats) {
  ButtonDebounce d;
  CHECK(!d.feed(false, 0));     // seed: released
  CHECK(d.feed(true, 100));     // press
  for (uint64_t t = 102; t < 5000; t += 2) CHECK(!d.feed(true, t));
}

TEST(a_bounce_burst_yields_exactly_one_press) {
  ButtonDebounce d;
  CHECK(!d.feed(false, 0));
  int presses = 0;
  // 0->1 at 100, then contact chatter well inside the 50 ms window.
  if (d.feed(true, 100)) ++presses;
  if (d.feed(false, 105)) ++presses;
  if (d.feed(true, 110)) ++presses;
  if (d.feed(false, 118)) ++presses;
  if (d.feed(true, 130)) ++presses;
  if (d.feed(true, 200)) ++presses;   // settled
  CHECK(presses == 1);
}

TEST(release_then_press_fires_a_second_time) {
  ButtonDebounce d;
  CHECK(!d.feed(false, 0));
  CHECK(d.feed(true, 100));
  CHECK(!d.feed(false, 400));   // a release is accepted but never a press
  CHECK(d.feed(true, 800));
}

TEST(a_release_inside_the_window_is_not_accepted) {
  ButtonDebounce d;
  CHECK(!d.feed(false, 0));
  CHECK(d.feed(true, 100));
  CHECK(!d.feed(false, 120));   // rejected: 20 ms < 50 ms
  // Because the release was rejected the level is still "pressed", so the
  // line returning high is not a new press.
  CHECK(!d.feed(true, 140));
  CHECK(!d.feed(true, 300));
}

MTEST_MAIN
