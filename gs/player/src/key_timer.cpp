#include "key_timer.h"

namespace maburplay {
namespace {

bool is_hold_button(Button b) { return b == Button::kLeft || b == Button::kRight; }

}  // namespace

std::optional<KeyEvent> KeyTimer::feed(Button b, bool level, uint64_t now_ms) {
  State& s = state_[static_cast<size_t>(b)];

  if (!s.seeded) {
    s.seeded = true;
    s.level = level;
    return std::nullopt;
  }

  if (level == s.level) {
    if (!level) return std::nullopt;  // still released: nothing to do

    if (is_hold_button(b)) {
      if (!s.hold_fired && now_ms - s.press_ms >= kHoldMs) {
        s.hold_fired = true;
        return KeyEvent{b, KeyKind::kHold};
      }
      return std::nullopt;
    }

    // Up/Down repeat while held; Sel never repeats.
    if (repeat_ && b != Button::kSel && now_ms >= s.next_repeat_ms) {
      s.next_repeat_ms = now_ms + kRepeatMs;
      return KeyEvent{b, KeyKind::kRepeat};
    }
    return std::nullopt;
  }

  // An edge: level just changed.
  s.level = level;
  if (level) {
    // Press edge.
    s.press_ms = now_ms;
    s.hold_fired = false;
    s.next_repeat_ms = now_ms + kRepeatDelayMs;
    if (!is_hold_button(b)) return KeyEvent{b, KeyKind::kTap};
    return std::nullopt;
  }

  // Release edge: a hold button taps only if it never crossed kHoldMs.
  if (is_hold_button(b) && !s.hold_fired && now_ms - s.press_ms < kHoldMs) {
    return KeyEvent{b, KeyKind::kTap};
  }
  return std::nullopt;
}

}  // namespace maburplay
