#include "rec_button.h"

#include <cctype>
#include <cstdlib>
#include <cstring>

namespace maburplay {
namespace {

// The tail of a line name must be exactly the decimal pin number: no
// sign, no leading space, no trailing characters. strtol alone accepts
// all three, which is why the first character is checked by hand.
bool tail_is_pin(const char* s, int pin) {
  if (!s || !std::isdigit(static_cast<unsigned char>(s[0]))) return false;
  char* end = nullptr;
  const long v = std::strtol(s, &end, 10);
  return end && *end == '\0' && v == static_cast<long>(pin);
}

}  // namespace

bool match_pin_name(const char* name, int pin) {
  if (!name || !*name) return false;
  if (std::strncmp(name, "PIN_", 4) == 0) return tail_is_pin(name + 4, pin);
  if (std::strncmp(name, "GPIO", 4) == 0) return tail_is_pin(name + 4, pin);
  return tail_is_pin(name, pin);
}

bool ButtonDebounce::feed(bool level, uint64_t now_ms) {
  if (!seeded_) {
    seeded_ = true;
    level_ = level;
    last_change_ms_ = now_ms;
    return false;
  }
  if (level == level_) return false;
  if (now_ms - last_change_ms_ < kDebounceMs) return false;  // bounce
  level_ = level;
  last_change_ms_ = now_ms;
  return level;  // a press, never a release
}

}  // namespace maburplay
