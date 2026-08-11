#ifndef MABUR_PLAYER_REC_BUTTON_H_
#define MABUR_PLAYER_REC_BUTTON_H_

#include <cstdint>
#include <string>

namespace maburplay {

// True when `name` is a GPIO line name for header pin `pin`. Accepts
// "PIN_<n>", "GPIO<n>" and a bare "<n>", and nothing else: the whole
// string must be consumed and the number must start with a digit, so
// "PIN_320" never answers to pin 32 and strtol's leading-space/sign
// tolerance cannot leak in.
bool match_pin_name(const char* name, int pin);

// Edge-triggered press detector. `level` is already polarity-corrected --
// the kernel inverts for us when the line is requested ACTIVE_LOW -- so
// true means pressed.
//
// Deliberately simple: a transition is accepted only if it is at least
// kDebounceMs after the last ACCEPTED transition, and a press is reported
// only on the accepted false->true edge. A held button therefore fires
// once and never repeats. The known limit is a switch whose bounce lasts
// longer than kDebounceMs, which can read as press-release-press; 50 ms
// is comfortably past the settling time of the tactile switches this is
// built for, and is the value PixelPilot_rk uses for the same job.
class ButtonDebounce {
 public:
  static constexpr uint64_t kDebounceMs = 50;

  // Returns true exactly once per accepted press. The FIRST call only
  // seeds the baseline and never fires, so a button held down (or a line
  // that idles pressed because the wiring is inverted) cannot toggle the
  // DVR at startup.
  bool feed(bool level, uint64_t now_ms);

 private:
  bool level_ = false;
  uint64_t last_change_ms_ = 0;
  bool seeded_ = false;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_REC_BUTTON_H_
