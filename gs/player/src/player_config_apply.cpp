#include "player_config_apply.h"

#include <cstring>
#include <memory>
#include <utility>

#include "mabur/config_write.h"

namespace maburplay {

const std::vector<PlayerLiveKey> kPlayerLiveKeys = {
    {"screen_mode", "", "screen_mode", PlayerLiveType::Str},
    {"colortrans.enable", "colortrans", "enable", PlayerLiveType::Bool},
    {"osd.enable", "osd", "enable", PlayerLiveType::Bool},
    {"osd.gs.enable", "osd.gs", "enable", PlayerLiveType::Bool},
    {"dvr.target", "dvr", "target", PlayerLiveType::Str},
    {"dvr.mode", "dvr", "mode", PlayerLiveType::Str},
    {"dvr.burned.bitrate_kbps", "dvr.burned", "bitrate_kbps", PlayerLiveType::Int},
    {"dvr.burned.every", "dvr.burned", "every", PlayerLiveType::Int},
    {"dvr.burned.size", "dvr.burned", "size", PlayerLiveType::Str},
};

namespace {

bool is(const PlayerLiveKey& k, const char* dotted) { return std::strcmp(k.key, dotted) == 0; }

// The loader has already checked every live string against its own
// grammar, none of which admits a quote or a backslash.
std::string quoted(const std::string& s) { return "\"" + s + "\""; }

std::string render_bool(bool b) { return b ? "true" : "false"; }

// Copies the one live field `k` names from `src` into `dst`.
void copy_live_field(const PlayerLiveKey& k, const Config& src, Config* dst) {
  if (is(k, "screen_mode")) dst->screen_mode = src.screen_mode;
  else if (is(k, "colortrans.enable")) dst->colortrans.enable = src.colortrans.enable;
  else if (is(k, "osd.enable")) dst->osd.enable = src.osd.enable;
  else if (is(k, "osd.gs.enable")) dst->osd.gs.enable = src.osd.gs.enable;
  else if (is(k, "dvr.target")) dst->dvr.target = src.dvr.target;
  else if (is(k, "dvr.mode")) dst->dvr.mode = src.dvr.mode;
  else if (is(k, "dvr.burned.bitrate_kbps")) dst->dvr.burned.bitrate_kbps = src.dvr.burned.bitrate_kbps;
  else if (is(k, "dvr.burned.every")) dst->dvr.burned.every = src.dvr.burned.every;
  else if (is(k, "dvr.burned.size")) {
    dst->dvr.burned.size_w = src.dvr.burned.size_w;
    dst->dvr.burned.size_h = src.dvr.burned.size_h;
  }
}

bool patch_live_key(std::string* text, const PlayerLiveKey& k, const std::string& value,
                    std::string* why) {
  if (value.find_first_of("\r\n") != std::string::npos) {
    *why = std::string(k.key) + ": value spans lines";
    return false;
  }
  if (!mabur::patch_toml_key(text, k.section, k.name, value)) {
    *why = std::string(k.key) + ": not set in the config file";
    return false;
  }
  return true;
}

bool apply_to_player(const PlayerApplyHooks& h, const PlayerLiveKey& k, const Config& from,
                     const Config& to, std::string* reason) {
  std::string err;
  switch (player_apply_rule(k, from, to)) {
    case PlayerApplyRule::Presenter:
      if (h.reacquire_display(to, &err)) return true;
      *reason = err.empty() ? "display reacquire failed" : err;
      return false;
    case PlayerApplyRule::OsdLayers:
      if (h.osd_layers(to, &err)) return true;
      *reason = err.empty() ? "osd layer refused" : err;
      return false;
    case PlayerApplyRule::NextStart:
      h.next_start(to);
      return true;
  }
  *reason = std::string(k.key) + ": no apply for this key";
  return false;
}

mabur::LiveApplySpec<Config, PlayerLiveKey> player_spec(BuildCaps caps, PlayerApplyHooks hooks) {
  const auto h = std::make_shared<const PlayerApplyHooks>(std::move(hooks));
  mabur::LiveApplySpec<Config, PlayerLiveKey> s;
  s.daemon = "maburplay";
  s.thread_name = "mbr-plcfgset";
  s.table = &kPlayerLiveKeys;
  s.render = render_player_live_value;
  s.patch = patch_live_key;
  s.load = [](const std::string& path) { return load_config(path); };
  s.refuse_config = [caps](const Config& c) { return unsupported_on_build(c, caps); };
  s.copy_field = copy_live_field;
  s.apply = [h](const PlayerLiveKey& k, const Config& from, const Config& to, std::string* reason) {
    return apply_to_player(*h, k, from, to, reason);
  };
  return s;
}

}  // namespace

const PlayerLiveKey* player_live_key(const std::string& dotted) {
  for (const PlayerLiveKey& k : kPlayerLiveKeys)
    if (dotted == k.key) return &k;
  return nullptr;
}

std::string render_player_live_value(const Config& c, const PlayerLiveKey& k) {
  if (is(k, "screen_mode")) return quoted(c.screen_mode);
  if (is(k, "colortrans.enable")) return render_bool(c.colortrans.enable);
  if (is(k, "osd.enable")) return render_bool(c.osd.enable);
  if (is(k, "osd.gs.enable")) return render_bool(c.osd.gs.enable);
  if (is(k, "dvr.target")) return quoted(c.dvr.target);
  if (is(k, "dvr.mode")) return quoted(c.dvr.mode);
  if (is(k, "dvr.burned.bitrate_kbps")) return std::to_string(c.dvr.burned.bitrate_kbps);
  if (is(k, "dvr.burned.every")) return std::to_string(c.dvr.burned.every);
  if (is(k, "dvr.burned.size")) {
    if (c.dvr.burned.size_w == 0 && c.dvr.burned.size_h == 0) return quoted("");
    return quoted(std::to_string(c.dvr.burned.size_w) + "x" + std::to_string(c.dvr.burned.size_h));
  }
  return "";
}

PlayerApplyRule player_apply_rule(const PlayerLiveKey& k, const Config& before,
                                  const Config& after) {
  if (is(k, "osd.enable") || is(k, "osd.gs.enable")) {
    const bool want_before = before.osd.enable || before.osd.gs.enable;
    const bool want_after = after.osd.enable || after.osd.gs.enable;
    return want_before != want_after ? PlayerApplyRule::Presenter : PlayerApplyRule::OsdLayers;
  }
  if (is(k, "screen_mode") || is(k, "colortrans.enable")) return PlayerApplyRule::Presenter;
  return PlayerApplyRule::NextStart;
}

PlayerConfigApplier::PlayerConfigApplier(std::string cfg_path, Config live, BuildCaps caps,
                                         PlayerApplyHooks hooks,
                                         std::function<void(const std::string&)> log)
    : LiveApplyEngine(std::move(cfg_path), std::move(live), player_spec(caps, std::move(hooks)),
                      std::move(log)) {}

}  // namespace maburplay
