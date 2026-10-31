#ifndef MABUR_PLAYER_MENU_VIEW_FEED_H_
#define MABUR_PLAYER_MENU_VIEW_FEED_H_

#include <cstdint>
#include <functional>
#include <utility>

#include "menu_model.h"

namespace maburplay {

// Paces MenuModel::view() for the presentation loop and keeps the result
// owed to the screen until a composition takes it.
//
// view() rebuilds the menu tree, so it is recomputed only when the
// session's generation moved (a key, a reply, a row's progress) or
// kPeriodMs has passed -- often enough for the time-driven changes nothing
// signals (a toast or status expiring, the link pill, the banner). A
// recomputed view equal to the last one owes nothing; a different one is
// owed until composed() -- a change made while the OSD buffers are busy
// must still reach the screen.
class MenuViewFeed {
 public:
  static constexpr uint64_t kPeriodMs = 33;

  // The view to hand the composer this iteration, or null when it has
  // nothing new.
  const MenuView* next(uint64_t now_ms, uint64_t generation,
                       const std::function<MenuView(uint64_t)>& compute) {
    if (!computed_ || generation != generation_ || now_ms - last_ms_ >= kPeriodMs) {
      computed_ = true;
      generation_ = generation;
      last_ms_ = now_ms;
      MenuView v = compute(now_ms);
      if (!(v == view_)) {
        view_ = std::move(v);
        owed_ = true;
      }
    }
    return owed_ ? &view_ : nullptr;
  }

  // A composition took the view.
  void composed() { owed_ = false; }
  bool owed() const { return owed_; }
  const MenuView& view() const { return view_; }

 private:
  MenuView view_;
  bool computed_ = false;
  bool owed_ = false;
  uint64_t generation_ = 0;
  uint64_t last_ms_ = 0;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_MENU_VIEW_FEED_H_
