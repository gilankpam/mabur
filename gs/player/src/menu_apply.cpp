#include "menu_apply.h"

namespace maburplay {

bool PlayerLiveSetter::submit(const std::string& key, const std::string& value, uint64_t,
                              std::string* why) {
  return ctl_.submit_local(applier_, key, value, why);
}

std::optional<LiveReply> PlayerLiveSetter::poll(uint64_t) {
  std::optional<PlayerApplyResult> r = ctl_.take_local_result();
  if (!r) return std::nullopt;
  return LiveReply{r->ok, r->reason};
}

bool GsLiveSetter::submit(const std::string& key, const std::string& value, uint64_t now_ms,
                          std::string* why) {
  return client_.send_set(key, value, now_ms, why);
}

std::optional<LiveReply> GsLiveSetter::poll(uint64_t now_ms) { return client_.poll(now_ms); }

MenuApplier::MenuApplier(MenuModel& model, LiveSetter& player, LiveSetter& gs)
    : model_(model), player_(player), gs_(gs) {}

LiveSetter& MenuApplier::setter_for(Owner o) { return o == Owner::kGs ? gs_ : player_; }

bool MenuApplier::start(const std::string& row_id, std::vector<SetStep> steps, uint64_t now_ms) {
  for (const SetStep& s : steps)
    if (s.owner == Owner::kDrone) return false;
  row_id_ = row_id;
  steps_ = std::move(steps);
  step_i_ = 0;
  running_ = true;
  model_.apply_started(row_id_, static_cast<int>(steps_.size()));
  if (steps_.empty()) {
    finish(true, "", now_ms);
    return true;
  }
  submit_current(now_ms);
  return true;
}

void MenuApplier::submit_current(uint64_t now_ms) {
  model_.apply_step(row_id_, static_cast<int>(step_i_) + 1);
  const SetStep& s = steps_[step_i_];
  std::string why;
  if (!setter_for(s.owner).submit(s.key, s.value, now_ms, &why)) finish(false, why, now_ms);
}

void MenuApplier::tick(uint64_t now_ms) {
  if (!running_) return;
  const Owner owner = steps_[step_i_].owner;
  std::optional<LiveReply> reply = setter_for(owner).poll(now_ms);
  if (!reply) return;
  if (!reply->ok) {
    finish(false, reply->text, now_ms);
    return;
  }
  ++step_i_;
  if (on_ok) on_ok(owner);
  if (step_i_ >= steps_.size()) {
    finish(true, "", now_ms);
    return;
  }
  submit_current(now_ms);
}

void MenuApplier::finish(bool ok, const std::string& reason, uint64_t now_ms) {
  running_ = false;
  model_.apply_done(row_id_, ok, reason, now_ms);
}

}  // namespace maburplay
