// KeyTimer turns a debounced, polarity-corrected level per button into the
// menu's three gestures (tap / hold / repeat); ButtonBank wires the five
// GPIO lines to it. No device here -- the ioctl shell is RecButton's, and
// that stays covered by rec_button's own tests.
#include <string>

#include "button_bank.h"
#include "key_timer.h"
#include "mtest.h"
#include "player_config.h"

using maburplay::Button;
using maburplay::ButtonBank;
using maburplay::ButtonDebounce;
using maburplay::InputCfg;
using maburplay::KeyKind;
using maburplay::KeyTimer;

TEST(a_spike_shorter_than_the_debounce_never_reaches_the_timer) {
  ButtonDebounce d; d.feed(false, 0);
  d.feed(true, 10); d.feed(false, 30); d.feed(false, 100);
  CHECK(d.level() == false);
}

TEST(right_tap_fires_on_release_before_the_hold_time) {
  KeyTimer t; t.feed(Button::kRight, false, 0);
  CHECK(!t.feed(Button::kRight, true, 100));
  auto e = t.feed(Button::kRight, false, 400);
  REQUIRE(e); CHECK(e->button == Button::kRight && e->kind == KeyKind::kTap);
}

TEST(a_held_right_fires_one_hold_and_nothing_on_release) {
  KeyTimer t; t.feed(Button::kRight, false, 0); t.feed(Button::kRight, true, 100);
  CHECK(!t.feed(Button::kRight, true, 699));
  auto e = t.feed(Button::kRight, true, 700); REQUIRE(e); CHECK(e->kind == KeyKind::kHold);
  CHECK(!t.feed(Button::kRight, true, 900));
  CHECK(!t.feed(Button::kRight, false, 1000));
}

TEST(up_taps_on_press_and_repeats_only_when_asked) {
  KeyTimer t; t.feed(Button::kUp, false, 0);
  auto e = t.feed(Button::kUp, true, 10); REQUIRE(e); CHECK(e->kind == KeyKind::kTap);
  CHECK(!t.feed(Button::kUp, true, 600));          // no repeat without set_repeat
  t.feed(Button::kUp, false, 700);
  t.set_repeat(true);
  t.feed(Button::kUp, true, 1000);                   // tap
  CHECK(!t.feed(Button::kUp, true, 1399));
  auto r = t.feed(Button::kUp, true, 1400); REQUIRE(r); CHECK(r->kind == KeyKind::kRepeat);
  CHECK(!t.feed(Button::kUp, true, 1450));
  r = t.feed(Button::kUp, true, 1500); REQUIRE(r); CHECK(r->kind == KeyKind::kRepeat);
}

TEST(the_first_sample_only_seeds) {
  KeyTimer t; CHECK(!t.feed(Button::kSel, true, 0)); CHECK(!t.feed(Button::kSel, true, 700));
}

TEST(bank_refuses_partial_wiring_naming_the_missing_button) {
  InputCfg in; in.up.configured = in.down.configured = in.left.configured = true;
  in.up.pin = in.down.pin = in.left.pin = 1;
  ButtonBank b; std::string err;
  CHECK(!b.open(in, &err)); CHECK(err.find("input.right") != std::string::npos);
}

MTEST_MAIN
