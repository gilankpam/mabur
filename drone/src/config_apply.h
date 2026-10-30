#pragma once
// Applies one MAVLink parameter set to the running maburd, then publishes
// it to /etc/mabur.toml.
//
// The running daemon takes each set before the file records it.
// ConfigApplier runs each set on its own thread (a rebuild stops the encoder for about half a
// second, too long for the MAVLink loop that submits it) in the order
//
//   re-read the file -> patch the row's line(s) -> load the result with the
//   boot loader (the candidate) -> apply the candidate to the process -> publish
//
// holding lock_config_writes() from the re-read through the publish, so a
// calibration publish can neither slip between them nor be overwritten by
// a stale read. The file is published only after the process accepted the
// candidate: a refused or failed apply leaves the file byte-identical and
// writes no ".pre-param" backup, so the file never claims a value the
// running daemon does not have. A publish that fails after a successful
// apply runs the same path again with the previous config, putting the
// process back in line with the unchanged file.
//
// Each parameter takes one of three paths (apply_path_for):
//   Knob       a value RcAgent reads live (set_knobs)
//   Reprogram  a radio or peripheral setting changed in place: the TX-power
//              plan for the wall margin, the radiotap LDPC flag, the MSP
//              thread
//   Rebuild    an encoder pipeline setting: the recorder stops, the
//              pipeline restarts on the candidate. If the new pipeline is
//              refused the ladder restarts the previous one, then tries a
//              cold restart of it, and as a last resort asks the daemon to
//              exit so its wrapper respawns it from the unchanged file.
//
// Everything that touches the process goes through ConfigApplyHooks, so the
// host tests drive the applier with fakes.
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "config.h"
#include "mav_params.h"

namespace mabur {

enum class ConfigApplyPath : uint8_t { Knob, Reprogram, Rebuild };

// The path for cfgparam::kTable row `row`. Rebuild for a row past the table.
ConfigApplyPath apply_path_for(size_t row);

struct ConfigApplyHooks {
  std::function<void(const Config&)> set_knobs;                     // RcAgent's live knobs
  std::function<bool(const Config&, std::string* err)> power_plan;  // wall margin reprogram
  std::function<void(bool ldpc)> ldpc;                              // radiotaps + the agent
  std::function<void(bool enable, const Config&)> msp;              // start/stop the MSP thread
  std::function<void()> recorder_stop;                              // onboard recorder off before a rebuild
  std::function<void(const Config&)> record_configure;              // recorder channel + size
  std::function<bool(const Config&)> pipeline_restart;              // pipeline-only restart
  std::function<bool(const Config&)> pipeline_cold_restart;         // full stop + start
  std::function<void(const Config&)> after_rebuild;                 // slice geometry + the agent
  std::function<void()> request_exit;                               // the ladder's last rung
  std::function<bool()> calibration_active;
};

struct ConfigApplyResult {
  bool ok;
  std::string reason;  // empty on ok
  ConfigApplyPath path;
  uint64_t ms;         // apply thread time, lock wait through publish
  // Warnings an accepted apply leaves the operator, one STATUSTEXT each:
  // an accepted radio.ldpc set that leaves a flown wall table measured
  // under the other ldpc flag (cal_link.h ldpc_mismatch_warnings).
  std::vector<std::string> notices;
};

struct ConfigApplierStats {
  uint64_t ok = 0, failed = 0, rebuilds = 0;  // rebuilds: every pipeline_restart call
};

class ConfigApplier {
 public:
  // `live` is the config the process is running (the one it booted with).
  // `log`, when set, receives one line per apply and one per ladder rung.
  ConfigApplier(std::string cfg_path, Config live, ConfigApplyHooks hooks,
                std::function<void(const std::string&)> log);
  ~ConfigApplier();  // shutdown()
  ConfigApplier(const ConfigApplier&) = delete;
  ConfigApplier& operator=(const ConfigApplier&) = delete;

  // Starts applying `rendered_value` (TOML text, a string carrying its own
  // quotes) to row `row`. Returns false with *why set, and changes nothing,
  // when an apply is in progress ("apply in progress"), calibration is
  // running ("calibration in progress") or the row does not exist. One
  // apply is in flight from an accepted submit until take_result hands its
  // result over, so a result is never overwritten by the next submit.
  bool submit(size_t row, const mavparam::Value& want, std::string* why);
  // Non-blocking: the finished apply's result, once. The MAV loop polls it.
  bool take_result(ConfigApplyResult* out);
  bool busy() const;
  // The config the process runs. Written only by the apply thread; read it
  // from that thread or after take_result returned true.
  const Config& live() const;
  // Calibration rewrote the file and applied its walls itself: the next
  // apply takes `live` from the file it re-reads, so a revert restores the
  // calibrated values rather than the ones this applier last saw.
  void note_file_rewritten_externally();
  ConfigApplierStats stats() const;
  // Teardown: a job accepted before this call is still applied (its result
  // stays takeable), then the thread is joined; from then on every submit
  // is refused ("shutting down"). Idempotent; call it from one thread.
  void shutdown();

 private:
  struct Job {
    size_t row = 0;
    mavparam::Value value{cfgparam::Type::Bool};
  };

  void run();
  ConfigApplyResult apply(const Job& job);
  // `followed`, non-null, is left empty unless mavparam::patch_row set it
  // ("record.size": VENC_MODE crossed an aspect change with the recorder
  // pinned to an explicit size) -- apply() appends it to the log line.
  bool apply_locked(const Job& job, const std::string& field, std::string* reason,
                    std::string* followed);
  bool dispatch(size_t row, const std::string& field, const Config& cand, std::string* reason);
  bool rebuild(const std::string& field, const Config& cand, std::string* reason);
  void log(const std::string& line) const;

  const std::string cfg_path_;
  Config live_;
  ConfigApplyHooks hooks_;
  std::function<void(const std::string&)> log_;

  mutable std::mutex mu_;  // job_, has_job_, result_, has_result_, stop_, thread_
  std::condition_variable cv_;
  std::thread thread_;
  Job job_;
  bool has_job_ = false;
  ConfigApplyResult result_{};
  bool has_result_ = false;
  bool stop_ = false;

  std::atomic<bool> busy_{false};
  std::atomic<bool> refresh_live_{false};
  std::atomic<uint64_t> ok_{0}, failed_{0}, rebuilds_{0};
};

}  // namespace mabur
