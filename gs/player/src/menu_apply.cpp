#include <cstdlib>

#include "menu_apply.h"

namespace maburplay {

namespace {

// strtol with no leftover text, same rule gs_live_view.cpp's parse_int
// uses to read a sideport value back.
bool parse_int(const std::string& s, int* out) {
  if (s.empty()) return false;
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (end != s.c_str() + s.size()) return false;
  *out = static_cast<int>(v);
  return true;
}

}  // namespace

bool PlayerLiveSetter::submit(const std::string& key, const std::string& value, uint64_t,
                              std::string* why) {
  return ctl_.submit_local(applier_, key, value, why);
}

std::optional<LiveReply> PlayerLiveSetter::poll(uint64_t) {
  std::optional<PlayerApplyResult> r = ctl_.take_local_result();
  if (!r) return std::nullopt;
  return LiveReply{r->ok, r->reason};
}

void PinSettleWatch::begin(int ch, const GsSnapshot* before) {
  active_ = true;
  ch_ = ch;
  before_seq_ = before ? before->scan_relocate_seq : std::nullopt;
}

std::optional<LiveReply> PinSettleWatch::check(const DroneSnapshotView& v) {
  if (!active_) return std::nullopt;
  if (v.stale || !v.snap) {
    active_ = false;
    return LiveReply{false, "link lost"};
  }
  const GsSnapshot& s = *v.snap;
  // The old outcome, not this submit's (a snapshot with no seq at all
  // carries no outcome either).
  if (!s.scan_relocate_seq || s.scan_relocate_seq == before_seq_) return std::nullopt;

  if (!s.scan_relocate || s.scan_relocate->ch != ch_) {
    active_ = false;
    return LiveReply{true, ""};
  }
  const ScanRelocate& r = *s.scan_relocate;
  if (r.state == "landed") {
    active_ = false;
    return LiveReply{true, ""};
  }
  if (r.state == "failed") {
    active_ = false;
    const std::string link = s.channel ? std::to_string(*s.channel) : "—";
    return LiveReply{false, "Pinned here, link still on " + link + " — " + r.why};
  }
  // state == "pending": settled ok once the link isn't there to watch
  // the relocate land on; still in flight while session holds.
  if (!s.session) {
    active_ = false;
    return LiveReply{true, ""};
  }
  return std::nullopt;
}

bool GsLiveSetter::submit(const std::string& key, const std::string& value, uint64_t now_ms,
                          std::string* why) {
  if (!client_.send_set(key, value, now_ms, why)) return false;
  pin_pending_ = false;
  before_.reset();
  int ch = 0;
  if (view_ && key == "radio.channel" && value != "auto" && parse_int(value, &ch)) {
    pin_ch_ = ch;
    pin_pending_ = true;
    const DroneSnapshotView view = view_();
    before_ = view.snap ? std::optional<GsSnapshot>(*view.snap) : std::nullopt;
  }
  return true;
}

std::optional<LiveReply> GsLiveSetter::poll(uint64_t now_ms) {
  if (std::optional<LiveReply> r = client_.poll(now_ms)) {
    if (!r->ok || !pin_pending_) return r;
    watch_.begin(pin_ch_, before_ ? &*before_ : nullptr);
    armed_ms_ = now_ms;
  }
  if (!watch_.active()) return std::nullopt;
  if (now_ms - armed_ms_ >= kPinSettleMs) {
    watch_ = PinSettleWatch{};
    return LiveReply{false, "no answer"};
  }
  return watch_.check(view_ ? view_() : DroneSnapshotView{});
}

void DroneSetWatch::begin(const std::string& param_id, const GsSnapshot* before) {
  active_ = true;
  param_id_ = param_id;
  seen_in_flight_ = false;
  before_age_ms_.reset();
  before_result_ms_.reset();
  if (!before) return;
  if (const VtxRow* row = vtx_row(*before, param_id.c_str())) {
    before_age_ms_ = row->age_ms;
    if (row->age_ms && before->t_ms)
      before_result_ms_ = *before->t_ms - static_cast<uint64_t>(*row->age_ms);
  }
}

std::optional<LiveReply> DroneSetWatch::check(const DroneSnapshotView& v) {
  if (!active_) return std::nullopt;
  if (v.stale || !v.snap || !v.snap->vtx || !v.snap->vtx->linked) {
    active_ = false;
    return LiveReply{false, "link lost"};
  }
  const VtxRow* row = vtx_row(*v.snap, param_id_.c_str());
  if (!row) return std::nullopt;  // never reported yet: keep waiting
  if (!vtx_terminal(row->st)) {
    if (row->st == "sending" || row->st == "in_progress") seen_in_flight_ = true;
    return std::nullopt;
  }

  bool settles = seen_in_flight_;
  if (!settles && before_result_ms_ && v.snap->t_ms && row->age_ms) {
    const uint64_t result_ms = *v.snap->t_ms - static_cast<uint64_t>(*row->age_ms);
    settles = result_ms > *before_result_ms_;
  } else if (!settles && !before_age_ms_) {
    settles = true;  // the row had never answered before this watch began
  } else if (!settles) {
    // Fallback for a peer that never exported t_ms on one side or the
    // other: the only signal left is the raw age getting smaller.
    settles = row->age_ms && *row->age_ms < *before_age_ms_;
  }
  if (!settles) return std::nullopt;

  active_ = false;
  if (row->st == "accepted") return LiveReply{true, ""};
  if (row->st == "timeout") return LiveReply{false, "no answer"};
  return LiveReply{false, row->status.empty() ? "refused" : row->status};
}

bool DroneLiveSetter::submit(const std::string& param_id, const std::string& value,
                             uint64_t now_ms, std::string* why) {
  if (!client_.send_line("vtx set " + param_id + " " + value, now_ms, why)) return false;
  param_id_ = param_id;
  const DroneSnapshotView view = view_ ? view_() : DroneSnapshotView{};
  before_ = view.snap ? std::optional<GsSnapshot>(*view.snap) : std::nullopt;
  return true;
}

std::optional<LiveReply> DroneLiveSetter::poll(uint64_t now_ms) {
  if (std::optional<LiveReply> r = client_.poll(now_ms)) {
    if (!r->ok) return LiveReply{false, r->text};
    watch_.begin(param_id_, before_ ? &*before_ : nullptr);
    armed_ms_ = now_ms;
  }
  if (!watch_.active()) return std::nullopt;
  if (now_ms - armed_ms_ >= kDroneSettleMs) {
    watch_ = DroneSetWatch{};
    return LiveReply{false, "no answer"};
  }
  return watch_.check(view_ ? view_() : DroneSnapshotView{});
}

MenuApplier::MenuApplier(MenuModel& model, LiveSetter& player, LiveSetter& gs, LiveSetter& drone)
    : model_(model), player_(player), gs_(gs), drone_(drone) {}

LiveSetter& MenuApplier::setter_for(Owner o) {
  switch (o) {
    case Owner::kGs: return gs_;
    case Owner::kDrone: return drone_;
    default: return player_;
  }
}

bool MenuApplier::start(const std::string& row_id, std::vector<SetStep> steps, uint64_t now_ms) {
  if (running_) return false;  // one row at a time; no per-owner runners
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
