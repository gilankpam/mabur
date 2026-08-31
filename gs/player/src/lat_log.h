#ifndef MABUR_PLAYER_LAT_LOG_H_
#define MABUR_PLAYER_LAT_LOG_H_

#include <cstdint>
#include <cstdio>
#include <string>

namespace maburplay {

// Persists the player's 1 Hz `lat:` line to <dir>/lat-NNNN.log so the
// tail latency segments survive power-off (/tmp is tmpfs; the flight-0035
// tail was lost exactly this way). Never blocks, never throws, never
// spams: a failed open retries at most every 30 s and the line always
// still goes to stderr via the caller.
//
// Format: "# latlog 1" then "# sync <mono_us> <wall_us>" (clock bridge to
// the flight jsonl), then "<mono_us> <payload>" per write. Index scan
// mirrors gs/src/ctl_log.cpp; no date suffix (RTC is bogus at boot).
class LatLog {
 public:
  explicit LatLog(std::string dir) : dir_(std::move(dir)) {}
  ~LatLog();
  LatLog(const LatLog&) = delete;
  LatLog& operator=(const LatLog&) = delete;

  void write(uint64_t mono_us, uint64_t wall_us, const char* payload);
  const std::string& path() const { return path_; }

 private:
  void try_open(uint64_t mono_us, uint64_t wall_us);

  const std::string dir_;
  std::FILE* f_ = nullptr;
  std::string path_;
  uint64_t last_attempt_us_ = 0;
  bool attempted_ = false;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_LAT_LOG_H_
