#pragma once
// maburgs live config: a closed table of keys an operator may change while
// the daemon runs, the loopback `set`/`get` listener that takes them, and
// the applier that puts one set into the running process and then into
// /etc/maburgs.toml.
//
// The pipeline -- re-read, patch, load the candidate, apply, publish, the
// worker/owner split and its write lock -- is mabur::LiveApplyEngine
// (mabur/live_apply.h), shared with maburplay. What is maburgs' own: the
// table below, the ladder's inline form, the refusal of a ladder rung the
// loader would drop above link.max_mcs, and the apply, which runs on the
// thread that owns ChannelCore and the VrxController (the core loop)
// through GsApplyHooks. radio.channel is the exception to apply-first: the
// pin is an order to the drone, placed only once the file holds it. So a
// pin is checked against the core's refusals at submit, before any file
// work; an order still refused after the publish (a hop started between
// the check and the order) publishes the previous text again. "auto" is
// never refused.
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "config.h"
#include "mabur/live_apply.h"

namespace maburgs {

enum class LiveType : uint8_t { Bool, Int, Real, Str, Ladder, Channel };

// One live key: `key` is the dotted name the listener takes, `section` and
// `name` where its line sits in the file.
struct LiveKey {
  const char* key;
  const char* section;
  const char* name;
  LiveType type;
};

// The closed live table, in `get` order.
extern const std::vector<LiveKey> kGsLiveKeys;
const LiveKey* gs_live_key(const std::string& dotted);  // nullptr = not live

// The live value of one key as the file would write it: strings quoted, a
// pinned channel as its number and "auto" quoted, reals with a decimal
// point, the ladder inline as
// `[{mcs = 0, bw = 40, overhead_base = 0.5, overhead_enh = 0.25}, ...]`.
std::string render_live_value(const Config& c, const LiveKey& k);

// Patches `text` for one key. Single-line keys go through patch_toml_key;
// the ladder takes the inline form render_live_value prints, refuses
// anything that is not a non-empty array of tables with exactly mcs, bw,
// overhead_base and overhead_enh, and replaces the [[link.ladder]] blocks.
// False with *why, `*text` unchanged, on a refusal. The value's types and
// ranges are the loader's to check, not this function's.
bool patch_live_key(std::string* text, const LiveKey& k, const std::string& value,
                    std::string* why);

// What an apply does to the running process. Every hook runs on the
// owning thread. All must be set (the applier's constructor throws
// std::invalid_argument otherwise).
struct GsApplyHooks {
  std::function<std::optional<std::string>(uint8_t ch)> set_pin;  // ChannelCore::set_pin
  std::function<void()> set_auto;                                 // ChannelCore::set_auto
  std::function<void(const Config&)> set_static;                  // VrxController::set_pin from link.static_*
  std::function<void(const Config&)> replace_ladder;              // ChannelCore::replace_ladder + log re-header
  std::function<void(bool on)> nack;                              // the software NACK on/off
  std::function<bool(bool on, std::string* err)> debug_log;       // open/close the session + logs
  std::function<void(bool on, const Config&)> msp;                // build/drop the MSP sink
  // ChannelCore::pin_refusal: why a pin to this member would be refused
  // now, asked at submit so a refused pin never reaches the file.
  std::function<std::optional<std::string>(uint8_t ch)> pin_refusal;
};

using GsApplyResult = mabur::LiveApplyResult;

// submit / poll / busy / live / get / get_all: see mabur::LiveApplyEngine.
// The owning thread is the core loop, which must never take
// mabur::lock_config_writes() (see mabur/live_apply.h).
class GsConfigApplier : public mabur::LiveApplyEngine<Config, LiveKey> {
 public:
  // `live` is the config the process runs (the one it booted with). `log`,
  // when set, receives one "maburgs: cfg set ..." line per set, plus a line
  // for each revert.
  GsConfigApplier(std::string cfg_path, Config live, GsApplyHooks hooks,
                  std::function<void(const std::string&)> log);
};

// The loopback listener (mabur::LiveConfigControl), on 127.0.0.1 ONLY.
using GsConfigControl = mabur::LiveConfigControl<GsConfigApplier>;

constexpr int kGsConfigControlPort = 8402;

}  // namespace maburgs
