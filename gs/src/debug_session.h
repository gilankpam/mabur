#pragma once

#include <string>

namespace maburgs {

// Where the current session's directory path is recorded. tmpfs on both the
// drone-image and buildroot GS, which is the point: the marker self-expires
// at reboot (a new session) but survives a 2 s wrapper respawn (the same
// session, appended to). The GS RTC restarts at the same bogus epoch every
// boot, so no wall-clock stamp could tell two sessions apart.
inline constexpr const char* kSessionMarker = "/tmp/mabur-session";

// Resolves which session directory this run's debug logs belong to.
//
// enable=false is not merely "do nothing": it REMOVES the marker, which is
// what makes debug_log.enable one knob for the whole GS -- maburplay follows
// the marker and stops writing too.
//
// Every failure (root missing/unwritable, mkdir failure, marker unwritable)
// is non-fatal: ok() reads false, the reason goes to stderr, and the caller
// simply opens no logs.
class DebugSession {
 public:
  DebugSession(const std::string& root, bool enable,
               const char* marker_path = kSessionMarker);

  DebugSession(const DebugSession&) = delete;
  DebugSession& operator=(const DebugSession&) = delete;

  bool ok() const { return ok_; }
  const std::string& dir() const { return dir_; }  // e.g. /media/dvr/log/0042
  int index() const { return index_; }
  // True when this run adopted a session a previous run created (a respawn
  // inside one boot). Callers open their files in APPEND mode either way;
  // this is for the stderr line only.
  bool rejoined() const { return rejoined_; }

 private:
  bool adopt_marker_(const char* marker_path);
  bool allocate_(const std::string& root, const char* marker_path);

  std::string dir_;
  int index_ = 0;
  bool ok_ = false;
  bool rejoined_ = false;
};

}  // namespace maburgs
