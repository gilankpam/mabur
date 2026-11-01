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
    ++generation_;
    if (o == Owner::kGs) refresh_pending_ = true;
  };
}

std::optional<bool> MenuSession::key(const KeyEvent& e, uint64_t now_ms) {
  ++generation_;
  model_.set_gs_known(view_.have);
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
      // The client is the refresh's until its reply lands. The row reads
      // "applying" meanwhile, and its owners stay locked.
      if (held_) {
        model_.toast("One GS change at a time", now_ms);
      } else {
        model_.apply_started(eff.row_id, static_cast<int>(eff.steps.size()));
        held_ = Held{eff.row_id, std::move(eff.steps)};
      }
    } else {
      start(eff.row_id, std::move(eff.steps), /*shown=*/false, now_ms);
    }
  }
  if (eff.repeat_on) return true;
  if (eff.repeat_off) return false;
  return std::nullopt;
}

void MenuSession::start(const std::string& row_id, std::vector<SetStep> steps, bool shown,
                        uint64_t now_ms) {
  const bool drone = touches(steps, Owner::kDrone);
  if (applier_.start(row_id, std::move(steps), now_ms)) return;
  const char* why = drone ? "One drone change at a time" : "One GS change at a time";
  model_.toast(why, now_ms);
  // A held row is already shown applying: settle it, or its locks stay.
  if (shown) model_.apply_done(row_id, false, "busy", now_ms);
}

void MenuSession::tick(uint64_t now_ms) {
  if (refresh_in_flight_) {
    if (std::optional<LiveReply> r = gs_.poll(now_ms)) {
      ++generation_;
      refresh_in_flight_ = false;
      // A refused or unanswered get leaves the view as it was.
      if (r->ok) parse_get_all("ok\n" + r->text, &view_, now_ms);
      if (held_) {
        Held h = std::move(*held_);
        held_.reset();
        // maburgs did not answer the get: the held row's set would only
        // wait out the same timeout again, so it ends here.
        if (!r->ok && r->text == "no answer")
          model_.apply_done(h.row_id, false, "no answer", now_ms);
        else
          start(h.row_id, std::move(h.steps), /*shown=*/true, now_ms);
      }
    }
  }
  const bool was_running = applier_.running();
  applier_.tick(now_ms);
  if (applier_.running() != was_running) ++generation_;
  // Nothing read yet while the menu is open: ask again every kRetryMs.
  if (model_.is_open() && !view_.have && !refresh_in_flight_ &&
      now_ms - last_get_ms_ >= kRetryMs)
    refresh_pending_ = true;
  if (refresh_pending_ && !refresh_in_flight_ && !applier_.running() && gs_.idle()) {
    refresh_pending_ = false;
    last_get_ms_ = now_ms;
    std::string why;
    if (gs_.send_get_all(now_ms, &why)) refresh_in_flight_ = true;
  }
}

}  // namespace maburplay
