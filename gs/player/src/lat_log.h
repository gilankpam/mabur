#ifndef MABUR_PLAYER_LAT_LOG_H_
#define MABUR_PLAYER_LAT_LOG_H_

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

namespace maburplay {

// Written by maburgs' DebugSession; maburplay only ever reads it. Declared
// here rather than shared from gs/src because the player does not link
// mabur_gs_core -- the two constants must stay equal.
inline constexpr const char* kSessionMarker = "/tmp/mabur-session";

// Persists a player record into the CURRENT debug session, so it survives
// power-off (/tmp is tmpfs; the flight-0035 tail was lost exactly that
// way). Two instances exist: the 1 Hz `lat:` line (lat.log, "# latlog 2")
// and the per-frame rows behind it (frame.log, "# framelog 2", batched:
// each second's rows are written together at flush()).
//
// maburplay holds no logging config: it follows the marker maburgs writes.
// No marker means no session means no file -- which is how one knob in
// maburgs.toml turns the whole GS's debug logging off.
//
// Format: the marker line, then "<mono_us> <payload>" per write, appended.
// There is no `# sync` clock bridge: maburgs and maburplay both stamp
// CLOCK_MONOTONIC on the same box, so every debug-log file shares one clock.
//
// Never blocks, never throws, never spams: a missing or unusable session is
// re-checked at most every 5 s (kRecheckUs) and the lat: line always still
// goes to stderr via the caller.
class LatLog {
 public:
  explicit LatLog(const char* marker_path = kSessionMarker,
                  const char* file_name = "lat.log",
                  const char* header = "# latlog 2", bool batched = false)
      : marker_(marker_path), file_name_(file_name), header_(header),
        buf_(batched ? new char[kBatchBytes] : nullptr) {}
  ~LatLog();
  LatLog(const LatLog&) = delete;
  LatLog& operator=(const LatLog&) = delete;

  // Line-buffered by default: each write() reaches the file at once. A
  // batched instance holds its writes in a kBatchBytes buffer until
  // flush(), so a burst of rows costs one syscall instead of one per row.
  void write(uint64_t mono_us, const char* payload);
  void flush();
  const std::string& path() const { return path_; }

 private:
  void reopen_(uint64_t mono_us);

  const char* marker_;
  const char* file_name_;
  const char* header_;
  static constexpr size_t kBatchBytes = 64 * 1024;  // > 1 s of frame.log rows
  std::unique_ptr<char[]> buf_;  // batched only: the stream's own buffer
  std::FILE* f_ = nullptr;
  std::string path_;
  std::string dir_;
  uint64_t last_check_us_ = 0;
  bool checked_ = false;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_LAT_LOG_H_
