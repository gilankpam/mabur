#include "button_bank.h"

namespace maburplay {

bool ButtonBank::open(const InputCfg& in, std::string* err) {
  if (!menu_buttons_wired(in)) {
    // menu_buttons_wired's own order: up, down, left, right. sel is
    // optional and never the reason open() refuses.
    if (!in.up.configured) *err = "input.up: not configured";
    else if (!in.down.configured) *err = "input.down: not configured";
    else if (!in.left.configured) *err = "input.left: not configured";
    else *err = "input.right: not configured";
    return false;
  }

  const struct { Button button; const ButtonCfg& cfg; } wires[] = {
      {Button::kUp, in.up},
      {Button::kDown, in.down},
      {Button::kLeft, in.left},
      {Button::kRight, in.right},
      {Button::kSel, in.sel},
  };
  for (const auto& w : wires) {
    if (!w.cfg.configured) continue;  // only sel may be left unwired here
    RecButtonCfg rbc;
    rbc.pin = w.cfg.pin;
    rbc.active_low = w.cfg.active_low;
    rbc.bias = w.cfg.bias;
    const size_t i = static_cast<size_t>(w.button);
    if (!line_[i].open(rbc, err, "maburplay_menu")) return false;
    line_open_[i] = true;
    if (w.button == Button::kSel) sel_open_ = true;
  }

  open_ = true;
  return true;
}

std::vector<KeyEvent> ButtonBank::poll(uint64_t now_ms) {
  std::vector<KeyEvent> out;
  if (!open_) return out;
  for (size_t i = 0; i < kCount; ++i) {
    if (!line_open_[i]) continue;
    const std::optional<bool> lvl = line_[i].read_level(now_ms);
    if (!lvl) continue;  // the line closed itself after repeated ioctl errors
    if (auto ev = timer_.feed(static_cast<Button>(i), *lvl, now_ms)) {
      out.push_back(*ev);
    }
  }
  return out;
}

}  // namespace maburplay
