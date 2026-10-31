#ifndef MABUR_PLAYER_BUTTON_BANK_H_
#define MABUR_PLAYER_BUTTON_BANK_H_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "key_timer.h"
#include "player_config.h"
#include "rec_button.h"

namespace maburplay {

// The menu shell's five GPIO lines (up/down/left/right/sel), each a
// RecButton of its own, feeding one shared KeyTimer. open() requires
// menu_buttons_wired(in) -- the same minimum the menu needs to navigate --
// and opens whichever of the five are configured under the consumer name
// "maburplay_menu", distinct from the rec button's "maburplay_rec" so a
// busy-line error says which button it is.
class ButtonBank {
 public:
  bool open(const InputCfg& in, std::string* err);
  bool is_open() const { return open_; }
  bool has_sel() const { return sel_open_; }

  // Reads every open line once (RecButton::read_level) and feeds the
  // timer; returns whatever gestures that produced, in button order.
  std::vector<KeyEvent> poll(uint64_t now_ms);

  void set_repeat(bool on) { timer_.set_repeat(on); }

 private:
  static constexpr size_t kCount = static_cast<size_t>(Button::kCount);

  std::array<RecButton, kCount> line_;
  std::array<bool, kCount> line_open_{};
  KeyTimer timer_;
  bool open_ = false;
  bool sel_open_ = false;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_BUTTON_BANK_H_
