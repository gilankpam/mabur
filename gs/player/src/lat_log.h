#ifndef MABUR_PLAYER_LAT_LOG_H_
#define MABUR_PLAYER_LAT_LOG_H_

#include <cstdint>
#include <cstdio>
#include <string>

namespace maburplay {

// Written by maburgs' DebugSession; maburplay only ever reads it. Declared
// here rather than shared from gs/src because the player does not link
// mabur_gs_core -- the two constants must stay equal.
inline constexpr const char* kSessionMarker = "/tmp/mabur-session";

// Persists the player's 1 Hz `lat:` line into the CURRENT debug session, so
// the tail latency segments survive power-off (/tmp is tmpfs; the flight-0035
// tail was lost exactly that way).
//
// maburplay holds no logging config: it follows the marker maburgs writes.
// No marker means no session means no file -- which is how one knob in
// maburgs.json turns the whole GS's debug logging off.
//
// Format: "# latlog 2" then "<mono_us> <payload>" per write, appended. There
// is no `# sync` clock bridge any more: maburgs and maburplay both stamp
// CLOCK_MONOTONIC on the same box, so every debug-log file shares one clock.
//
// Never blocks, never throws, never spams: a missing or unusable session is
// re-checked at most every 30 s and the line always still goes to stderr via
// the caller.
class LatLog {
 public:
  explicit LatLog(const char* marker_path = kSessionMarker)
      : marker_(marker_path) {}
  ~LatLog();
  LatLog(const LatLog&) = delete;
  LatLog& operator=(const LatLog&) = delete;

  void write(uint64_t mono_us, const char* payload);
  const std::string& path() const { return path_; }

 private:
  void reopen_(uint64_t mono_us);

  const char* marker_;
  std::FILE* f_ = nullptr;
  std::string path_;
  std::string dir_;
  uint64_t last_check_us_ = 0;
  bool checked_ = false;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_LAT_LOG_H_
