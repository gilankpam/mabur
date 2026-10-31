#pragma once
// maburgs live config: a closed table of keys an operator may change while
// the daemon runs, the loopback `set`/`get` listener that takes them, and
// the applier that puts one set into the running process and then into
// /etc/maburgs.toml.
//
// The GS twin of drone/src/config_apply.h, with the same order of
// operations and the same writer:
//
//   re-read the file -> patch the key's line(s) -> load the result with the
//   boot loader (the candidate) -> apply the candidate to the process ->
//   publish
//
// with one split the drone does not need: the file work runs on a worker
// thread, while the apply runs on the thread that owns ChannelCore and the
// VrxController (the core loop), through GsApplyHooks. The owning thread
// never blocks on file I/O or on the worker: poll() picks up whatever the
// worker finished and hands it the next step. The worker holds
// lock_config_writes() from the re-read through the publish, so a
// calibration publish can neither slip between them nor be overwritten by
// a stale read.
//
// The file is published only after the process accepted the candidate: a
// refused or failed apply leaves the file byte-identical and writes no
// ".pre-live" backup. A publish that fails after a successful apply
// re-applies the previous config, putting the process back in line with
// the unchanged file. radio.channel is the exception to apply-first: the
// pin is an order to the drone, placed only once the file holds it; an
// order refused after the publish (a hop started in between) publishes the
// previous text again.
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <sys/socket.h>
#include <string>
#include <thread>
#include <vector>

#include "config.h"

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
// owning thread.
struct GsApplyHooks {
  std::function<std::optional<std::string>(uint8_t ch)> set_pin;  // ChannelCore::set_pin
  std::function<void()> set_auto;                                 // ChannelCore::set_auto
  std::function<void(const Config&)> set_static;                  // VrxController::set_pin from link.static_*
  std::function<void(const Config&)> replace_ladder;              // VrxController::replace_ladder + log re-header
  std::function<void(bool on)> nack;                              // the software NACK on/off
  std::function<bool(bool on, std::string* err)> debug_log;       // open/close the session + logs
  std::function<void(bool on, const Config&)> msp;                // build/drop the MSP sink
  std::function<bool()> hop_or_cal_busy;                          // precondition for radio.channel
};

struct GsApplyResult {
  bool ok;
  std::string reason;  // empty on ok
  std::string key;
  uint64_t ms;         // submit to result
};

class GsConfigApplier {
 public:
  // `live` is the config the process runs (the one it booted with). `log`,
  // when set, receives one "maburgs: cfg set ..." line per set, plus a line
  // for each revert.
  GsConfigApplier(std::string cfg_path, Config live, GsApplyHooks hooks,
                  std::function<void(const std::string&)> log);
  ~GsConfigApplier();  // lets the worker finish its step, then joins it
  GsConfigApplier(const GsConfigApplier&) = delete;
  GsConfigApplier& operator=(const GsConfigApplier&) = delete;

  // Owning thread. Starts applying `value` (TOML text, a string carrying
  // its own quotes) to `key`. False with *why, nothing started, for a key
  // that is not live ("not live: <key>"), while a set is in flight
  // ("busy"), or for a value that is empty or spans lines. Two outcomes
  // are settled here with no file work, their result handed out by the
  // next poll(): radio.channel while a hop or calibration runs (refused),
  // and a value equal to the live one (ok). A set is in flight from an
  // accepted submit until poll() hands out its result.
  bool submit(const std::string& key, const std::string& value, std::string* why);
  // Owning thread, every tick: applies a candidate the worker has ready,
  // hands an applied candidate to the publish, re-applies the previous
  // config when a publish failed, and returns true once with the result.
  bool poll(GsApplyResult* out);
  bool busy() const;                // owning thread
  const Config& live() const;       // owning thread
  // Owning thread: the live value, or "" with *why ("not live: <key>").
  std::string get(const std::string& key, std::string* why) const;
  std::string get_all() const;      // "<key> = <value>\n" per live key, in table order

 private:
  // The worker's position in one set. The owner moves it on from the
  // states marked (owner); the worker from the rest.
  enum class Phase {
    Idle,
    Reading,       // worker: re-read, patch, load the candidate
    Refused,       // (owner) the worker refused: no candidate
    Ready,         // (owner) candidate loaded, waiting for the apply
    Publishing,    // worker
    Published,     // (owner) publish outcome in pub_ok_/pub_err_
    Restoring,     // worker: publishing the previous text again
    Restored,      // (owner)
  };
  // What the owner tells the worker holding the write lock to do next.
  enum class Step { None, Publish, Restore, Release };

  struct Job {
    const LiveKey* key = nullptr;
    std::string value;
    bool write_backup = false;
  };

  void run();
  void work(const Job& job);
  // Waits (worker, mu_ held via `l`) for the owner's next step.
  Step await_step(std::unique_lock<std::mutex>& l);
  void tell(Step s, Phase next);
  bool apply_to_process(const LiveKey& k, const Config& c, std::string* reason);
  void finish(bool ok, const std::string& reason, GsApplyResult* out);
  void log(const std::string& line) const;

  const std::string cfg_path_;
  Config live_;
  GsApplyHooks hooks_;
  std::function<void(const std::string&)> log_;

  // Owning thread only.
  bool in_flight_ = false;
  const LiveKey* cur_key_ = nullptr;
  std::string cur_value_;
  std::chrono::steady_clock::time_point t0_{};
  std::optional<GsApplyResult> settled_;  // a result submit() decided
  Config cand_;                           // the applied candidate, until it is live
  std::string restore_reason_;            // a set_pin refusal after the publish
  bool backup_written_ = false;

  // Shared with the worker, under mu_.
  mutable std::mutex mu_;
  std::condition_variable cv_;
  std::thread thread_;
  Job job_;
  bool has_job_ = false;
  bool stop_ = false;
  Phase phase_ = Phase::Idle;
  Step step_ = Step::None;
  std::string refusal_;     // Refused
  Config ready_cand_;       // Ready
  bool pub_ok_ = false;     // Published
  std::string pub_err_;     // Published
};

// Loopback-only UDP listener for `set` / `get`, CalControl-shaped. Binds
// 127.0.0.1 ONLY: a set changes the running link and rewrites its config,
// so whoever sends one must be logged into the GS itself.
//
// Replies: "ok", "ok <value>", "ok\n<key> = <value>\n..." (a bare `get`),
// "err <reason>" ("err busy", "err not live: <key>", the usage line).
class GsConfigControl {
 public:
  GsConfigControl() = default;
  ~GsConfigControl();
  GsConfigControl(const GsConfigControl&) = delete;
  GsConfigControl& operator=(const GsConfigControl&) = delete;

  bool open(int port);  // 127.0.0.1 only
  bool ok() const { return fd_ >= 0; }

  // Owning thread, every tick: drains datagrams (each answered at once
  // unless handle() set *pending, whose sender address is kept), then
  // calls a.poll() and, when a result arrives, answers the kept sender
  // with "ok" / "err <reason>". One pending sender at a time: the
  // applier's busy refusal keeps a second `set` from queueing. The
  // cfg-set log line is the applier's.
  void poll(GsConfigApplier& a);

  // One line in, one reply out; "" with *pending = true when a `set` was
  // accepted and its answer comes from a later poll(). Static and I/O-free
  // so the command language is testable without a socket.
  //   set <key> = <value>
  //   get [<key>]
  static std::string handle(const std::string& line, GsConfigApplier& a, bool* pending);

 private:
  int fd_ = -1;
  bool has_pending_ = false;
  sockaddr_in pending_from_{};
  socklen_t pending_len_ = 0;
};

constexpr int kGsConfigControlPort = 8402;

}  // namespace maburgs
