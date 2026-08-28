#pragma once
#include <cstdint>
#include <string>

namespace mabur {

// B6 deletes this: WaybeamClient is going away with the rest of the HTTP
// control-plane (venc is in-process now). This trimmed shim replaces the
// WaybeamCfg that used to live in Config — Task B5 dropped the "waybeam"
// config section entirely (host/port/idr_path are no longer JSON-parsed
// anywhere), so main.cpp constructs one of these with hardcoded defaults
// purely to keep this class's constructor callable until B6 removes it.
struct WaybeamCfg {
  std::string host = "127.0.0.1";
  int port = 80;
  std::string idr_path = "/request/idr";  // waybeam IDR route (bench-confirmed: GET -> {"ok":true,"data":{"idr":true}})
};

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

  // GET /api/v1/get?<key> HTTP/1.0 — returns true iff the response status
  // line contains "200", with the raw response body (everything after the
  // header/blank-line separator) copied into `body_out` on success.
  // `body_out` is left untouched on failure.
  bool get_param(const std::string& key, std::string& body_out);

  // Cumulative count of failed requests (connect failure, timeout,
  // non-200 status, etc.) since construction.
  uint64_t failures() const;

 private:
  bool do_get(const std::string& path);

  WaybeamCfg cfg_;
  uint64_t failures_ = 0;
  std::string last_body_;
};

}  // namespace mabur
