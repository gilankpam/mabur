// drone/src/debug_http.h — thin localhost debug endpoint (spec
// 2026-08-28-venc-foldin, Task B7). Three routes: GET /snapshot.jpg,
// GET /venc (stats), POST /venc/set?k=v (whitelisted bitrate/qp_delta/
// roi_qp). Global namespace on purpose — this header is exercised by a
// pure host-tested parse function with no socket dependency, matching the
// task brief's test code verbatim.
#pragma once
#include <string>

// Parsed request line. Deliberately dumb: no method beyond the three
// routes below is understood, and everything else collapses to BAD so the
// caller can 404 it without special-casing.
struct DebugReq {
  enum { SNAPSHOT, STATS, SET, BAD } kind;
  std::string key;
  long val = 0;
};

// Pure: parses the HTTP request line only (e.g. "GET /venc HTTP/1.0"),
// no socket I/O. Host-testable without a listener.
DebugReq debug_http_parse(const std::string& request_line);

// Starts the debug endpoint on a detached thread, bound to
// 127.0.0.1:port. Bind/listen failure logs to stderr and returns without
// starting a serving loop -- never fatal, matches the "keep it simple"
// brief. snapshot_quality is cfg.venc.core.snapshot_quality, forwarded
// unmodified into the SNAPSHOT route so it can tell "subsystem disabled"
// (quality == 0) apart from a capture failure (quality > 0, verb still
// returns -1). Safe to call on host/dry-run builds too: without
// MABUR_HAVE_VENC every route just answers "disabled".
//
// Shutdown: the thread is detached and never joined. main() never waits
// on it, so a stuck accept() cannot block process exit; the OS reclaims
// the socket and thread at process teardown. No stop flag is needed
// because nothing ever needs to observe "has it stopped yet".
void debug_http_start(int port, int snapshot_quality);
