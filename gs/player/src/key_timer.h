#ifndef MABUR_PLAYER_KEY_TIMER_H_
#define MABUR_PLAYER_KEY_TIMER_H_

#include <array>
#include <cstdint>
#include <optional>

namespace maburplay {

// The menu shell's five-way pad.
enum class Button : uint8_t { kUp, kDown, kLeft, kRight, kSel, kCount };

// What a line produced this feed(): kTap on a short press, kHold once a
// press has run past kHoldMs, kRepeat on an auto-repeat tick.
enum class KeyKind : uint8_t { kTap, kHold, kRepeat };

struct KeyEvent {
  Button button;
  KeyKind kind;
};

// Turns a per-button debounced, polarity-corrected level (ButtonDebounce
// already strips the chatter) into the menu's gestures.
//
// Up/Down/Sel are tap buttons: a press edge is the event, and Up/Down
// additionally auto-repeat while held once set_repeat(true) is asked for
// (a number picker's +/-). Left/Right are hold buttons: a short press taps
// on release, a press held past kHoldMs fires one kHold at that instant
// and nothing on release -- opening a group or committing a pick must
// happen exactly once, not again when the finger comes off.
class KeyTimer {
 public:
  static constexpr uint64_t kHoldMs = 600;
  static constexpr uint64_t kRepeatDelayMs = 400;
  static constexpr uint64_t kRepeatMs = 100;

  // The first feed() for any given button only seeds its level -- a
  // button already held when the player starts (or a line idling at the
  // "pressed" polarity because the wiring is inverted) must not fire on
  // sight, the same rule ButtonDebounce's first sample follows.
  std::optional<KeyEvent> feed(Button b, bool level, uint64_t now_ms);

  void set_repeat(bool on) { repeat_ = on; }

 private:
  struct State {
    bool seeded = false;
    bool level = false;
    uint64_t press_ms = 0;
    bool hold_fired = false;
    uint64_t next_repeat_ms = 0;
  };

  std::array<State, static_cast<size_t>(Button::kCount)> state_{};
  bool repeat_ = false;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_KEY_TIMER_H_
