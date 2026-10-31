#include "menu_session.h"

#include <algorithm>
#include <utility>

namespace maburplay {
namespace {

bool touches(const std::vector<SetStep>& steps, Owner o) {
  return std::any_of(steps.begin(), steps.end(), [o](const SetStep& s) { return s.owner == o; });
}

}  // namespace

MenuSession::MenuSession(MenuModel& model, MenuApplier& applier, GsViewSource& gs, GsLiveView& view)
    : model_(model), applier_(applier), gs_(gs), view_(view) {
  applier_.on_ok = [this](Owner o) {
    if (o == Owner::kGs) refresh_pending_ = true;
  };
}

std::optional<bool> MenuSession::key(const KeyEvent& e, uint64_t now_ms) {
  const bool was_open = model_.is_open();
  MenuModel::Effect eff = model_.handle(e, now_ms);
  if (!was_open && model_.is_open()) {
    refresh_pending_ = true;
    if (log) log("maburplay: menu: open");
  } else if (was_open && !model_.is_open()) {
    if (log) log("maburplay: menu: close");
  }
  if (!eff.steps.empty()) {
    if (refresh_in_flight_ && touches(eff.steps, Owner::kGs)) {
      // The client is the refresh's until its reply lands.
      if (held_) model_.toast("One GS change at a time", now_ms);
      else held_ = Held{eff.row_id, std::move(eff.steps)};
    } else {
      start(eff.row_id, std::move(eff.steps), now_ms);
    }
  }
  if (eff.repeat_on) return true;
  if (eff.repeat_off) return false;
  return std::nullopt;
}

void MenuSession::start(const std::string& row_id, std::vector<SetStep> steps, uint64_t now_ms) {
  const bool drone = touches(steps, Owner::kDrone);
  if (!applier_.start(row_id, std::move(steps), now_ms))
    model_.toast(drone ? "One drone change at a time" : "One GS change at a time", now_ms);
}

void MenuSession::tick(uint64_t now_ms) {
  if (refresh_in_flight_) {
    if (std::optional<LiveReply> r = gs_.poll(now_ms)) {
      refresh_in_flight_ = false;
      // A refused or unanswered get leaves the view as it was; the next
      // open or GS set asks again.
      if (r->ok) parse_get_all("ok\n" + r->text, &view_, now_ms);
      if (held_) {
        Held h = std::move(*held_);
        held_.reset();
        start(h.row_id, std::move(h.steps), now_ms);
      }
    }
  }
  applier_.tick(now_ms);
  if (refresh_pending_ && !refresh_in_flight_ && !applier_.running() && gs_.idle()) {
    refresh_pending_ = false;
    std::string why;
    if (gs_.send_get_all(now_ms, &why)) refresh_in_flight_ = true;
  }
}

}  // namespace maburplay
