// drone/src/debug_http.cpp — see debug_http.h for the route/shutdown
// contract. Two independent halves: debug_http_parse is pure (host-tested,
// no socket touched) and the accept loop below it is the ~80-line thread
// the brief asks for, with every venc call gated on MABUR_HAVE_VENC so the
// host build still compiles and serves (uniformly "disabled") responses.
#include "debug_http.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#ifdef MABUR_HAVE_VENC
#include "venc_core.h"
#endif

namespace {

// strtol with a full-consumption check: "nope" and "4000x" are both
// rejected, "4000" is not. Empty input is rejected too (strtol would
// happily parse it as 0 with end == s.c_str()).
bool parse_long_strict(const std::string& s, long* out) {
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  long v = std::strtol(s.c_str(), &end, 10);
  if (end != s.c_str() + s.size()) return false;
  if (errno == ERANGE) return false;
  *out = v;
  return true;
}

bool key_whitelisted(const std::string& k) {
  return k == "bitrate" || k == "qp_delta" || k == "roi_qp";
}

}  // namespace

DebugReq debug_http_parse(const std::string& request_line) {
  DebugReq req;
  req.kind = DebugReq::BAD;

  size_t sp1 = request_line.find(' ');
  if (sp1 == std::string::npos) return req;
  std::string method = request_line.substr(0, sp1);
  size_t sp2 = request_line.find(' ', sp1 + 1);
  if (sp2 == std::string::npos) return req;
  std::string target = request_line.substr(sp1 + 1, sp2 - sp1 - 1);

  if (method == "GET" && target == "/snapshot.jpg") {
    req.kind = DebugReq::SNAPSHOT;
    return req;
  }
  if (method == "GET" && target == "/venc") {
    req.kind = DebugReq::STATS;
    return req;
  }
  static const std::string kSetPrefix = "/venc/set?";
  if (method == "POST" && target.compare(0, kSetPrefix.size(), kSetPrefix) == 0) {
    std::string query = target.substr(kSetPrefix.size());
    size_t eq = query.find('=');
    if (eq == std::string::npos) return req;
    std::string key = query.substr(0, eq);
    std::string val_str = query.substr(eq + 1);
    if (!key_whitelisted(key)) return req;
    long val = 0;
    if (!parse_long_strict(val_str, &val)) return req;
    req.kind = DebugReq::SET;
    req.key = key;
    req.val = val;
    return req;
  }
  return req;
}

namespace {

void send_all(int fd, const char* data, size_t len) {
  size_t off = 0;
  while (off < len) {
    ssize_t n = ::send(fd, data + off, len - off, 0);
    if (n <= 0) return;  // peer gone; nothing more to do
    off += static_cast<size_t>(n);
  }
}

void send_http(int fd, const char* status, const char* content_type,
               const char* body, size_t body_len) {
  char header[192];
  int hn = std::snprintf(header, sizeof(header),
                          "HTTP/1.0 %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\n"
                          "Connection: close\r\n\r\n",
                          status, content_type, body_len);
  if (hn > 0) send_all(fd, header, static_cast<size_t>(hn));
  send_all(fd, body, body_len);
}

void send_json(int fd, const char* status, const std::string& body) {
  send_http(fd, status, "application/json", body.data(), body.size());
}

// GET /snapshot.jpg. Controller carry: venc_snapshot_jpeg returns -1 both
// when the JPEG subsystem was never built (snapshot_quality == 0 at boot)
// and on a genuine capture failure -- those need to read differently to an
// operator, so the disabled case is checked BEFORE calling the verb rather
// than inferred from its return value.
void handle_snapshot(int fd, int snapshot_quality) {
#ifdef MABUR_HAVE_VENC
  if (snapshot_quality <= 0) {
    send_json(fd, "503 Service Unavailable", "{\"error\":\"disabled\"}\n");
    return;
  }
  uint8_t* jpeg = nullptr;
  size_t jpeg_len = 0;
  if (venc_snapshot_jpeg(&jpeg, &jpeg_len, snapshot_quality) != 0) {
    send_json(fd, "500 Internal Server Error", "{\"error\":\"capture_failed\"}\n");
    return;
  }
  send_http(fd, "200 OK", "image/jpeg", reinterpret_cast<const char*>(jpeg), jpeg_len);
  std::free(jpeg);
#else
  (void)snapshot_quality;
  send_json(fd, "503 Service Unavailable", "{\"error\":\"disabled\"}\n");
#endif
}

// GET /venc. cur_bitrate_kbps is the REQUESTED rate (VencStats' own
// comment), not a readback of what the encoder actually programmed, so the
// JSON key says "req_" to avoid implying otherwise (controller carry).
void handle_stats(int fd) {
#ifdef MABUR_HAVE_VENC
  VencStats vs{};
  venc_get_stats(&vs);
  char body[192];
  int bn = std::snprintf(
      body, sizeof(body),
      "{\"req_bitrate_kbps\":%d,\"ring_fill_pct\":%u,\"full_drops\":%llu,\"frames\":%u}\n",
      vs.cur_bitrate_kbps, static_cast<unsigned>(vs.ring_fill_pct),
      static_cast<unsigned long long>(vs.full_drops),
      static_cast<unsigned>(vs.frames_encoded));
  send_json(fd, "200 OK", std::string(body, bn > 0 ? static_cast<size_t>(bn) : 0));
#else
  send_json(fd, "503 Service Unavailable", "{\"error\":\"disabled\"}\n");
#endif
}

// POST /venc/set?k=v. Nothing here is persisted to config, but an override
// is NOT transient: RcAgent only calls set_bitrate_kbps()/set_roi_qp() when
// its computed value CHANGES (run_bitrate_policy's changed/decrease gate),
// so on a parked link an override survives until the next commanded-rate
// change -- a rung transition, a failsafe/LINKED entry, or an ROI threshold
// crossing. Measured on hardware 2026-08-29 (B9 gate 5): a bitrate set here
// held for 20 s+ with the ladder parked. That is exactly what makes the
// endpoint useful for bench experiments, and exactly why it must not be
// left set: nothing re-asserts the policy value on a timer.
// Controller carry: report {"ok":false} on a failed verb rather than a
// blind {"ok":true} -- B6 made the three verbs return real status.
void handle_set(int fd, const DebugReq& req) {
#ifdef MABUR_HAVE_VENC
  bool ok = false;
  int v = static_cast<int>(req.val);
  if (req.key == "bitrate") {
    ok = venc_set_bitrate_kbps(v) == 0;
  } else if (req.key == "qp_delta") {
    ok = venc_set_qp_delta(v) == 0;
  } else if (req.key == "roi_qp") {
    ok = venc_set_roi_qp(v) == 0;
  }
  send_json(fd, "200 OK", ok ? "{\"ok\":true}\n" : "{\"ok\":false}\n");
#else
  (void)req;
  send_json(fd, "503 Service Unavailable", "{\"error\":\"disabled\"}\n");
#endif
}

void handle_conn(int fd, int snapshot_quality) {
  char buf[512];
  ssize_t n = ::recv(fd, buf, sizeof(buf) - 1, 0);
  if (n <= 0) return;
  buf[n] = '\0';

  std::string request(buf, static_cast<size_t>(n));
  size_t eol = request.find("\r\n");
  std::string line = eol == std::string::npos ? request : request.substr(0, eol);

  DebugReq req = debug_http_parse(line);
  switch (req.kind) {
    case DebugReq::SNAPSHOT:
      handle_snapshot(fd, snapshot_quality);
      break;
    case DebugReq::STATS:
      handle_stats(fd);
      break;
    case DebugReq::SET:
      handle_set(fd, req);
      break;
    case DebugReq::BAD:
    default:
      send_json(fd, "404 Not Found", "{\"error\":\"not_found\"}\n");
      break;
  }
}

void serve(int port, int snapshot_quality) {
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    std::fprintf(stderr, "debug_http: socket() failed: %s -- endpoint disabled\n",
                 std::strerror(errno));
    return;
  }
  int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(static_cast<uint16_t>(port));

  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::fprintf(stderr, "debug_http: bind(127.0.0.1:%d) failed: %s -- endpoint disabled\n",
                 port, std::strerror(errno));
    ::close(fd);
    return;
  }
  if (::listen(fd, 4) != 0) {
    std::fprintf(stderr, "debug_http: listen() failed: %s -- endpoint disabled\n",
                 std::strerror(errno));
    ::close(fd);
    return;
  }

  std::fprintf(stderr, "debug_http: listening on 127.0.0.1:%d\n", port);
  // Single-threaded accept loop by design (thin debug endpoint, not a real
  // server): one client at a time is fine for an operator poking curl at
  // it, but that also means one client that connects and never sends
  // (aborted nc, half-open probe) would otherwise wedge recv() forever and
  // starve every other client indefinitely. A couple of seconds of
  // SO_RCVTIMEO/SO_SNDTIMEO on the accepted fd bounds that: a stalled
  // reader or a stalled peer on the response write both fail fast and
  // return control to accept() instead of hanging the whole endpoint.
  for (;;) {
    int cfd = ::accept(fd, nullptr, nullptr);
    if (cfd < 0) continue;  // EINTR or a transient accept error; keep serving
    timeval tv{};
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    ::setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    handle_conn(cfd, snapshot_quality);
    ::close(cfd);
  }
}

}  // namespace

void debug_http_start(int port, int snapshot_quality) {
  // Detached, never joined -- see debug_http.h for why that is the whole
  // shutdown design. bind/listen failures are handled inside serve() (log
  // + return), so the detached thread just exits quietly in that case.
  std::thread(serve, port, snapshot_quality).detach();
}
