#ifndef MABUR_PLAYER_MENU_APPLY_H_
#define MABUR_PLAYER_MENU_APPLY_H_
// Runs one menu row's SetSteps (menu_model.h) against the player and
// maburgs: each step goes to its owner's LiveSetter in turn, and the
// model hears apply_started/apply_step/apply_done as it goes. A row
// carrying a kDrone step has nowhere to go yet, so MenuApplier::start
// refuses it outright.
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

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

// Runs one row's steps, in order, one at a time: a later step is never
// sent after an earlier one answers err or times out, and only one row
// runs at a time (no per-owner runners). The model hears apply_started
// once, apply_step before each submit, and apply_done with the stopping
// step's bare reason.
class MenuApplier {
 public:
  MenuApplier(MenuModel& model, LiveSetter& player, LiveSetter& gs);

  // False, nothing started and the model untouched, while another row is
  // still running() or for a row carrying a kDrone step -- no setter for
  // it in this build. An empty step list (an action with nothing to send)
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
  std::string row_id_;
  std::vector<SetStep> steps_;
  size_t step_i_ = 0;
  bool running_ = false;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_MENU_APPLY_H_
