#pragma once
#include <cstdint>
#include <string>

#include "config.h"

namespace mabur {

// Minimal blocking HTTP/1.0 client for talking to the Waybeam VTX's local
// control API. Each call's individual phases (connect, send, recv) are bounded
// to 200 ms each; aggregate worst case is ~600 ms, so it can never stall the
// agent thread it runs on. Never throws: all failure modes (connect refused,
// timeout, non-200 status, short read) are reported via the boolean return
// value and tallied in failures(). Errors are logged to stderr, rate-limited
// to at most one line per 5 seconds.
//
// Not thread-safe: intended for exclusive use by a single (agent) thread.
class WaybeamClient {
 public:
  explicit WaybeamClient(const WaybeamCfg& cfg);

  // GET /api/v1/set?<key>=<value> HTTP/1.0 — returns true iff the response
  // status line contains "200". `key`/`value` are assumed already
  // URL-safe (numeric/short tokens) and are not escaped.
  bool set_param(const std::string& key, const std::string& value);

  // GET <cfg.idr_path> HTTP/1.0 — returns true iff the response status
  // line contains "200".
  bool request_idr();

  // Cumulative count of failed requests (connect failure, timeout,
  // non-200 status, etc.) since construction.
  uint64_t failures() const;

 private:
  bool do_get(const std::string& path);

  WaybeamCfg cfg_;
  uint64_t failures_ = 0;
};

}  // namespace mabur
