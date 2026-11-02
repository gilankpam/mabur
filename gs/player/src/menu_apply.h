#ifndef MABUR_PLAYER_MENU_APPLY_H_
#define MABUR_PLAYER_MENU_APPLY_H_
// Runs one menu row's SetSteps (menu_model.h) against the player,
// maburgs and the drone: each step goes to its owner's LiveSetter in
// turn, and the model hears apply_started/apply_step/apply_done as it
// goes.
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "gs_snapshot.h"
#include "live_udp_client.h"
#include "menu_model.h"
#include "player_config_apply.h"

namespace maburplay {

// One owner's side of a set: the player's own live-config engine, or a
// UDP round trip to maburgs. submit() starts at most one set at a time;
// poll() hands out its reply once, in the same shape either side's
// listener gives: ok with "" for a plain success, or !ok with the bare
// reason ("busy", "no answer", or whatever the applier itself refused
// with) -- the model, not this, dresses that up for the operator.
class LiveSetter {
 public:
  virtual ~LiveSetter() = default;
  virtual bool submit(const std::string& key, const std::string& value, uint64_t now_ms,
                      std::string* why) = 0;
  virtual std::optional<LiveReply> poll(uint64_t now_ms) = 0;
};

// The player's own live-config engine, applied in-process through
// LiveConfigControl::submit_local -- never through the UDP listener
// maburplay itself owns, which would otherwise fight the menu for the
// one pending slot.
class PlayerLiveSetter final : public LiveSetter {
 public:
  PlayerLiveSetter(PlayerConfigControl& ctl, PlayerConfigApplier& applier)
      : ctl_(ctl), applier_(applier) {}
  bool submit(const std::string& key, const std::string& value, uint64_t now_ms,
             std::string* why) override;
  // Reads take_local_result(): the caller's tick must run ctl.poll(applier)
  // itself before calling this, same as the housekeeping loop already does.
  std::optional<LiveReply> poll(uint64_t now_ms) override;

 private:
  PlayerConfigControl& ctl_;
  PlayerConfigApplier& applier_;
};

// maburgs over its loopback live-config port (8402), one UDP exchange
// per step.
class GsLiveSetter final : public LiveSetter {
 public:
  explicit GsLiveSetter(LiveUdpClient& client) : client_(client) {}
  bool submit(const std::string& key, const std::string& value, uint64_t now_ms,
             std::string* why) override;
  std::optional<LiveReply> poll(uint64_t now_ms) override;

 private:
  LiveUdpClient& client_;
};

// What the drone setter watches: the sideport as the player holds it
// right now. `snap` is null before the first datagram ever arrives;
// `stale` mirrors the player's own staleness read on that snapshot
// (gs_source.h's stale()), taken at the same instant.
struct DroneSnapshotView {
  const GsSnapshot* snap = nullptr;
  bool stale = true;
};

// The settle rule on its own, testable without a socket: a drone set's
// `vtx set` reply only says the line reached maburgs, never the outcome
// -- that is the first later sideport snapshot whose row (a) is terminal
// and (b) carries a result fresher than the one seen just before the
// submit. "Fresher" is not simply "a smaller age_ms": if the previous
// result was itself very recent (say 50 ms old at submit) a genuinely
// new result can still read an age bigger than that (150 ms, a datagram
// later) while being the later event. So the comparison is on each
// side's absolute result time (the snapshot's own t_ms minus the row's
// age_ms) rather than on the raw ages -- with age_ms < age_ms as the
// fallback for a peer too old to export t_ms. A row seen `sending` or
// `in_progress` at any point after begin() always settles on its next
// terminal state, regardless of the age arithmetic: having watched the
// row leave idle removes any doubt about which submit it is answering.
class DroneSetWatch {
 public:
  // Remembers, from `before` (null if there is none, or the row has
  // never been seen), the row's own last result time and raw age --
  // nullopt either way when the row has no age yet (it then settles on
  // its very first terminal state, see check()).
  void begin(const std::string& param_id, const GsSnapshot* before);
  // nullopt while waiting; {true,""} on accepted; {false, status|"refused"}
  // on failed/unsupported; {false,"no answer"} on timeout; {false,"link
  // lost"} when stale or !vtx->linked (this also ends the watch).
  std::optional<LiveReply> check(const DroneSnapshotView& v);
  bool active() const { return active_; }

 private:
  bool active_ = false;
  std::string param_id_;
  bool seen_in_flight_ = false;
  std::optional<int64_t> before_age_ms_;
  std::optional<uint64_t> before_result_ms_;  // before.t_ms - before_age_ms_, when both known
};

// A drone-owned step over 8402's `vtx set`, settled on the sideport
// (DroneSetWatch above) rather than on the verb's own reply, which only
// says the line was sent.
class DroneLiveSetter final : public LiveSetter {
 public:
  // kDroneSettleMs is a backstop only: the client's own reply deadline
  // (LiveUdpClient::kReplyTimeoutMs) always ends the "was it sent" half
  // sooner, so this is purely how long the watch will wait on the
  // sideport for a result that never shows up.
  static constexpr uint64_t kDroneSettleMs = 30000;

  DroneLiveSetter(LiveUdpClient& client, std::function<DroneSnapshotView()> view)
      : client_(client), view_(std::move(view)) {}
  bool submit(const std::string& param_id, const std::string& value, uint64_t now_ms,
             std::string* why) override;
  std::optional<LiveReply> poll(uint64_t now_ms) override;

 private:
  LiveUdpClient& client_;
  std::function<DroneSnapshotView()> view_;
  DroneSetWatch watch_;
  std::string param_id_;
  std::optional<GsSnapshot> before_;
  uint64_t armed_ms_ = 0;
};

// Runs one row's steps, in order, one at a time: a later step is never
// sent after an earlier one answers err or times out, and only one row
// runs at a time (no per-owner runners). The model hears apply_started
// once, apply_step before each submit, and apply_done with the stopping
// step's bare reason.
class MenuApplier {
 public:
  MenuApplier(MenuModel& model, LiveSetter& player, LiveSetter& gs, LiveSetter& drone);

  // False, nothing started and the model untouched, while another row is
  // still running(). An empty step list (an action with nothing to send)
  // settles ok at once.
  bool start(const std::string& row_id, std::vector<SetStep> steps, uint64_t now_ms);
  // Every tick while running(): polls the current step's setter and
  // either advances to the next step or settles the row.
  void tick(uint64_t now_ms);
  bool running() const { return running_; }

  // Fires after each step that answers ok, with that step's owner -- main
  // uses it to refresh the GS view once a GS set has landed.
  std::function<void(Owner)> on_ok;

 private:
  LiveSetter& setter_for(Owner o);
  void submit_current(uint64_t now_ms);
  void finish(bool ok, const std::string& reason, uint64_t now_ms);

  MenuModel& model_;
  LiveSetter& player_;
  LiveSetter& gs_;
  LiveSetter& drone_;
  std::string row_id_;
  std::vector<SetStep> steps_;
  size_t step_i_ = 0;
  bool running_ = false;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_MENU_APPLY_H_
