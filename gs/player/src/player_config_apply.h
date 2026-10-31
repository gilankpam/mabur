#ifndef MABUR_PLAYER_CONFIG_APPLY_H_
#define MABUR_PLAYER_CONFIG_APPLY_H_
// maburplay live config: a closed table of keys an operator may change
// while the player runs, the loopback `set`/`get` listener that takes them
// (127.0.0.1:8403), and the applier that puts one set into the running
// player and then into /etc/maburplay.toml.
//
// The pipeline -- re-read, patch, load the candidate, apply, publish, the
// worker/owner split and its write lock -- is mabur::LiveApplyEngine
// (mabur/live_apply.h), shared with maburgs. What is maburplay's own: the
// table below, the candidate check (the boot loader, then
// unsupported_on_build against this binary's decoders, so the Orange Pi
// refuses a burned DVR exactly as its boot would), and the apply, which
// runs on the housekeeping loop through PlayerApplyHooks by
// player_apply_rule.
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "mabur/live_apply.h"
#include "player_config.h"

namespace maburplay {

enum class PlayerLiveType : uint8_t { Bool, Int, Str };

// One live key: `key` is the dotted name the listener takes, `section` and
// `name` where its line sits in the file (section "" = above every header).
struct PlayerLiveKey {
  const char* key;
  const char* section;
  const char* name;
  PlayerLiveType type;
};

// The closed live table, in `get` order: screen_mode, colortrans.enable,
// osd.enable, osd.gs.enable, dvr.target, dvr.mode, dvr.burned.bitrate_kbps,
// dvr.burned.every, dvr.burned.size.
extern const std::vector<PlayerLiveKey> kPlayerLiveKeys;
const PlayerLiveKey* player_live_key(const std::string& dotted);  // nullptr = not live

// The live value of one key as the file would write it: strings quoted,
// dvr.burned.size as "WxH" or "" for the decoded size.
std::string render_player_live_value(const Config& c, const PlayerLiveKey& k);

// How a changed key reaches the running player.
enum class PlayerApplyRule : uint8_t {
  // Recreate the presenter on the new config: screen_mode; colortrans.enable
  // (the presenter's LUT/CTM setters are call-once per presenter, so a
  // toggle rebuilds it); an osd switch that changes whether any OSD is
  // wanted, which adds or drops the OSD plane.
  Presenter,
  // An osd switch that leaves "any OSD wanted" unchanged: that overlay's
  // source and layer open or close in place.
  OsdLayers,
  // dvr.*: read when the next recording starts; nothing to do now.
  NextStart,
};
PlayerApplyRule player_apply_rule(const PlayerLiveKey& k, const Config& before,
                                  const Config& after);

// What a recording started under `d` records. Read from the live config at
// each record start and held until that recording stops: a dvr.* set
// during a recording takes effect at the next start, never on the running
// one.
struct RecPlan {
  bool gs = true;       // this GS's DVR: dvr.target "gs" or "both"
  bool vtx = false;     // the drone's SD card: "vtx" or "both"
  bool burned = false;  // the GS DVR re-encodes with the OSD burnt in
  bool operator==(const RecPlan& o) const {
    return gs == o.gs && vtx == o.vtx && burned == o.burned;
  }
};
RecPlan rec_plan(const DvrCfg& d);

// What an apply does to the running player. Every hook runs on the owning
// (housekeeping) thread and is handed the live config with only the set
// key changed. All three must be set (the applier's constructor throws
// std::invalid_argument otherwise).
struct PlayerApplyHooks {
  std::function<bool(const Config&, std::string* err)> reacquire_display;  // Presenter
  std::function<bool(const Config&, std::string* err)> osd_layers;         // OsdLayers
  std::function<void(const Config&)> next_start;                           // NextStart
};

using PlayerApplyResult = mabur::LiveApplyResult;

// submit / poll / busy / live / get / get_all: see mabur::LiveApplyEngine.
// The owning thread must never take mabur::lock_config_writes() (see
// mabur/live_apply.h). A refused hook (a mode the display does not offer)
// publishes nothing and leaves live() unchanged.
class PlayerConfigApplier : public mabur::LiveApplyEngine<Config, PlayerLiveKey> {
 public:
  // `live` is the config the player runs (the one it booted with); `caps`
  // the decoders this binary carries. `log`, when set, receives one
  // "maburplay: cfg set ..." line per set, plus a line for each revert.
  PlayerConfigApplier(std::string cfg_path, Config live, BuildCaps caps, PlayerApplyHooks hooks,
                      std::function<void(const std::string&)> log);
};

// The loopback listener (mabur::LiveConfigControl), on 127.0.0.1 ONLY.
using PlayerConfigControl = mabur::LiveConfigControl<PlayerConfigApplier>;

constexpr int kPlayerConfigControlPort = 8403;

}  // namespace maburplay

#endif  // MABUR_PLAYER_CONFIG_APPLY_H_
