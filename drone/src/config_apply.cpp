#include "config_apply.h"

#include <pthread.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <exception>
#include <string>
#include <system_error>
#include <utility>

#include "cal_link.h"
#include "config_write.h"
#include "mabur/config_params.h"

namespace mabur {

namespace {

// cfgparam::Apply and ConfigApplyPath are the same three values under
// different names (the table is shared code with no MAVLink/ConfigApplyPath
// dependency); this converts one to the other.
ConfigApplyPath from_cfgparam(cfgparam::Apply a) {
  switch (a) {
    case cfgparam::Apply::Knob: return ConfigApplyPath::Knob;
    case cfgparam::Apply::Reprogram: return ConfigApplyPath::Reprogram;
    case cfgparam::Apply::Rebuild: return ConfigApplyPath::Rebuild;
  }
  return ConfigApplyPath::Rebuild;
}

const char* path_name(ConfigApplyPath p) {
  switch (p) {
    case ConfigApplyPath::Knob: return "knob";
    case ConfigApplyPath::Reprogram: return "reprogram";
    case ConfigApplyPath::Rebuild: return "rebuild";
  }
  return "?";
}

bool is(const cfgparam::Row& row, const char* section, const char* key) {
  return std::strcmp(row.section, section) == 0 && std::strcmp(row.key, key) == 0;
}

}  // namespace

ConfigApplyPath apply_path_for(size_t row) {
  return row < cfgparam::kCount ? from_cfgparam(cfgparam::kTable[row].apply) : ConfigApplyPath::Rebuild;
}

ConfigApplier::ConfigApplier(std::string cfg_path, Config live, ConfigApplyHooks hooks,
                             std::function<void(const std::string&)> log)
    : cfg_path_(std::move(cfg_path)),
      live_(std::move(live)),
      hooks_(std::move(hooks)),
      log_(std::move(log)) {}

ConfigApplier::~ConfigApplier() { shutdown(); }

void ConfigApplier::shutdown() {
  {
    std::lock_guard<std::mutex> l(mu_);
    stop_ = true;
  }
  cv_.notify_all();
  // submit() refuses once stop_ is set, so thread_ no longer changes.
  if (thread_.joinable()) thread_.join();
}

void ConfigApplier::log(const std::string& line) const {
  if (log_) log_(line);
}

bool ConfigApplier::submit(size_t row, const mavparam::Value& want, std::string* why) {
  auto refuse = [&](const char* text) {
    if (why) *why = text;
    return false;
  };
  if (row >= cfgparam::kCount) return refuse("no such parameter");
  std::lock_guard<std::mutex> l(mu_);
  if (stop_) return refuse("shutting down");
  if (busy_.load()) return refuse("apply in progress");
  if (hooks_.calibration_active && hooks_.calibration_active())
    return refuse("calibration in progress");
  // The thread first, so a refusal for a thread that cannot start leaves
  // no job behind; it waits on mu_, held here, until the job is in place.
  if (!thread_.joinable()) {
    try {
      thread_ = std::thread([this] { run(); });
    } catch (const std::system_error&) {
      return refuse("apply thread failed to start");
    }
  }
  job_ = Job{row, want};
  has_job_ = true;
  busy_.store(true);
  cv_.notify_one();
  return true;
}

bool ConfigApplier::take_result(ConfigApplyResult* out) {
  std::lock_guard<std::mutex> l(mu_);
  if (!has_result_) return false;
  *out = std::move(result_);
  has_result_ = false;
  busy_.store(false);
  return true;
}

bool ConfigApplier::busy() const { return busy_.load(); }

const Config& ConfigApplier::live() const { return live_; }

void ConfigApplier::note_file_rewritten_externally() { refresh_live_.store(true); }

ConfigApplierStats ConfigApplier::stats() const {
  ConfigApplierStats s;
  s.ok = ok_.load();
  s.failed = failed_.load();
  s.rebuilds = rebuilds_.load();
  return s;
}

void ConfigApplier::run() {
  pthread_setname_np(pthread_self(), "mbr-apply");
  std::unique_lock<std::mutex> l(mu_);
  for (;;) {
    cv_.wait(l, [this] { return stop_ || has_job_; });
    // A job accepted before shutdown() is applied first: its submit was
    // answered IN_PROGRESS and is owed a result.
    if (!has_job_) return;
    const Job job = std::move(job_);
    has_job_ = false;
    l.unlock();
    ConfigApplyResult r = apply(job);
    l.lock();
    result_ = std::move(r);
    has_result_ = true;
  }
}

ConfigApplyResult ConfigApplier::apply(const Job& job) {
  const auto t0 = std::chrono::steady_clock::now();
  const cfgparam::Row& row = cfgparam::kTable[job.row];
  const std::string field = std::string(row.section) + "." + row.key;
  ConfigApplyResult r{false, "", apply_path_for(job.row), 0};
  std::string followed;
  try {
    // Held from the re-read through the publish: a calibration publish
    // between them would be overwritten by this one's stale text.
    auto lock = lock_config_writes();
    r.ok = apply_locked(job, field, &r.reason, &followed);
  } catch (const std::exception& e) {
    r.ok = false;
    r.reason = std::string("apply threw: ") + e.what();
  }
  r.ms = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now() - t0)
                                   .count());
  ++(r.ok ? ok_ : failed_);
  log("apply " + std::string(row.id) + " " + path_name(r.path) + " " +
      (r.ok ? std::string("ok") : "failed: " + r.reason) +
      (followed.empty() ? "" : " +" + followed) + " " + std::to_string(r.ms) + " ms");
  // The ldpc flag the link now flies may no longer be the one a flown wall
  // table was measured under: accepted all the same (the operator may be
  // about to re-calibrate), with the boot warning's text sent back.
  if (r.ok && is(row, "radio", "ldpc")) {
    r.notices = ldpc_mismatch_warnings(live_.radio);
    for (const auto& n : r.notices) log(n);
  }
  return r;
}

bool ConfigApplier::apply_locked(const Job& job, const std::string& field, std::string* reason,
                                 std::string* followed) {
  const cfgparam::Row& row = cfgparam::kTable[job.row];
  std::string text;
  if (!read_text_file(cfg_path_, &text)) {
    *reason = "config file unreadable";
    return false;
  }
  // Calibration published new walls and applied them itself: the file is
  // what runs now, so a revert below must go back to it.
  if (refresh_live_.exchange(false)) {
    try {
      live_ = load_config(cfg_path_);
    } catch (const std::exception& e) {
      refresh_live_.store(true);
      *reason = "config does not load: " + loader_reason(e.what());
      return false;
    }
  }
  // patch_row's record.size follow reads the config this text loads as --
  // the source handle_set checked the same set against -- never live_,
  // which an ssh edit the daemon has not reloaded leaves behind the file.
  Config on_disk;
  try {
    on_disk = load_config(cfg_path_);
  } catch (const std::exception& e) {
    *reason = "config does not load: " + loader_reason(e.what());
    return false;
  }
  std::string patched = text;
  if (!mavparam::patch_row(&patched, job.row, job.value, on_disk, followed, reason)) return false;
  // The candidate is the patched text as the boot loader reads it: the
  // process is never handed a config maburd would refuse to start from.
  const std::string tmp = cfg_path_ + ".new";
  Config cand;
  if (!write_file_fsync(tmp, patched)) {
    ::unlink(tmp.c_str());
    *reason = "failed to write " + tmp;
    return false;
  }
  try {
    cand = load_config(tmp);
  } catch (const std::exception& e) {
    ::unlink(tmp.c_str());
    *reason = loader_reason(e.what());
    return false;
  }
  // Gone before the apply: a failed apply leaves no trace, and publish_config
  // writes its own copy.
  ::unlink(tmp.c_str());

  if (!dispatch(job.row, field, cand, reason)) return false;

  std::string err;
  const PublishResult pr = publish_config(cfg_path_, patched, "pre-param", &err);
  if (pr != PublishResult::Ok) {
    // The file still holds the previous value: put the process back on it.
    log("apply " + std::string(row.id) + ": publish failed, reverting");
    std::string revert_reason;
    if (!dispatch(job.row, field, live_, &revert_reason))
      log("apply " + std::string(row.id) + ": revert failed: " + revert_reason);
    *reason = "publish failed: " + (pr == PublishResult::ReparseFailed ? loader_reason(err) : err);
    return false;
  }
  live_ = std::move(cand);
  return true;
}

bool ConfigApplier::dispatch(size_t row_idx, const std::string& field, const Config& cand,
                             std::string* reason) {
  const cfgparam::Row& row = cfgparam::kTable[row_idx];
  switch (apply_path_for(row_idx)) {
    case ConfigApplyPath::Knob:
      hooks_.set_knobs(cand);
      return true;
    case ConfigApplyPath::Reprogram:
      if (is(row, "radio", "wall_margin_db")) {
        std::string err;
        if (hooks_.power_plan(cand, &err)) return true;
        *reason = "power plan refused: " + err;
        return false;
      }
      if (is(row, "radio", "ldpc")) {
        hooks_.ldpc(cand.radio.ldpc);
        return true;
      }
      if (is(row, "msp", "enable")) {
        hooks_.msp(cand.msp.enable, cand);
        return true;
      }
      *reason = field + ": no reprogram for this key";
      return false;
    case ConfigApplyPath::Rebuild:
      return rebuild(field, cand, reason);
  }
  *reason = field + ": no apply path";
  return false;
}

// The failure ladder. Each rung logs one line; after the first rung the
// process runs `live_` again (or exits), never a half-applied candidate.
bool ConfigApplier::rebuild(const std::string& field, const Config& cand, std::string* reason) {
  const std::string tag = "rebuild " + field + ": ";
  auto restart = [this](const Config& c) {
    ++rebuilds_;
    return hooks_.pipeline_restart(c);
  };

  hooks_.recorder_stop();
  hooks_.record_configure(cand);
  if (restart(cand)) {
    hooks_.after_rebuild(cand);
    log(tag + "new pipeline started");
    return true;
  }
  log(tag + "new pipeline refused");

  hooks_.record_configure(live_);
  if (restart(live_)) {
    hooks_.after_rebuild(live_);
    log(tag + "previous pipeline restored");
    *reason = "pipeline refused " + field + ": restored previous";
    return false;
  }
  log(tag + "previous pipeline refused");

  if (hooks_.pipeline_cold_restart(live_)) {
    hooks_.after_rebuild(live_);
    log(tag + "previous pipeline restored by a cold restart");
    *reason = "pipeline refused " + field + ": restored previous (cold)";
    return false;
  }
  log(tag + "cold restart failed");

  hooks_.request_exit();
  log(tag + "pipeline lost, requesting exit");
  *reason = "pipeline lost: exiting";
  return false;
}

}  // namespace mabur
