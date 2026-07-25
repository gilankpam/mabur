#include "waybeam_client.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>

namespace mabur {

namespace {

constexpr int kTimeoutMs = 200;

// Logs `msg` to stderr, but at most once per 5 seconds, to avoid flooding
// logs when the link/VTX is down for an extended period. Not thread-safe
// (matches the class's agent-thread-only contract).
void log_rate_limited(const std::string& msg) {
  using clock = std::chrono::steady_clock;
  static clock::time_point last_log{};
  auto now = clock::now();
  if (last_log.time_since_epoch().count() == 0 ||
      now - last_log >= std::chrono::seconds(5)) {
    std::fprintf(stderr, "waybeam_client: %s\n", msg.c_str());
    last_log = now;
  }
}

// Connects to host:port with a bounded timeout (200 ms), returning a
// connected fd on success or -1 on any failure (timeout, refused, resolution
// failure). Never throws. Closes any intermediate fd on failure paths.
int connect_with_timeout(const std::string& host, int port, int timeout_ms) {
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) return -1;

  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    close(fd);
    return -1;
  }

  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    close(fd);
    return -1;
  }

  int rc = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  if (rc == 0) {
    // Connected immediately (e.g. loopback). Restore blocking mode.
    fcntl(fd, F_SETFL, flags);
    return fd;
  }
  if (errno != EINPROGRESS) {
    close(fd);
    return -1;
  }

  pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLOUT;
  pfd.revents = 0;
  rc = poll(&pfd, 1, timeout_ms);
  if (rc <= 0) {
    // rc == 0: timeout. rc < 0: poll error. Either way, bail.
    close(fd);
    return -1;
  }

  int so_error = 0;
  socklen_t len = sizeof(so_error);
  if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0 ||
      so_error != 0) {
    close(fd);
    return -1;
  }

  // Restore blocking mode; reads/writes below rely on SO_RCVTIMEO instead
  // of O_NONBLOCK for their bounding.
  fcntl(fd, F_SETFL, flags);
  return fd;
}

}  // namespace

WaybeamClient::WaybeamClient(const WaybeamCfg& cfg) : cfg_(cfg) {}

uint64_t WaybeamClient::failures() const { return failures_; }

bool WaybeamClient::set_param(const std::string& key,
                               const std::string& value) {
  return do_get("/api/v1/set?" + key + "=" + value);
}

bool WaybeamClient::request_idr() { return do_get(cfg_.idr_path); }

bool WaybeamClient::get_param(const std::string& key, std::string& body_out) {
  bool ok = do_get("/api/v1/get?" + key);
  if (ok) body_out = last_body_;
  return ok;
}

bool WaybeamClient::do_get(const std::string& path) {
  int fd = connect_with_timeout(cfg_.host, cfg_.port, kTimeoutMs);
  if (fd < 0) {
    ++failures_;
    log_rate_limited("connect failed to " + cfg_.host + ":" +
                      std::to_string(cfg_.port));
    return false;
  }

  // Bound send and recv to 200 ms each; aggregate worst case (connect +
  // send + recv) is ~600 ms.
  timeval tv;
  tv.tv_sec = kTimeoutMs / 1000;
  tv.tv_usec = (kTimeoutMs % 1000) * 1000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  std::string request = "GET " + path + " HTTP/1.0\r\n\r\n";
  ssize_t sent = send(fd, request.data(), request.size(), 0);
  if (sent < 0 || static_cast<size_t>(sent) != request.size()) {
    close(fd);
    ++failures_;
    log_rate_limited("write failed/timed out to " + cfg_.host + ":" +
                      std::to_string(cfg_.port));
    return false;
  }

  // Read to EOF (HTTP/1.0: the server closes when done), not just once — a
  // server that writes headers and body in separate segments would otherwise
  // hand us headers with an empty body, and get_param() would report success
  // with no value (rig 2026-07-25: maburd's waybeam cross-check claimed
  // `FATAL MISMATCH ... (got: )` against a correctly configured waybeam).
  // Capped so a chatty or wedged server cannot grow this unboundedly; replies
  // of interest are a few hundred bytes.
  constexpr size_t kMaxResponse = 8192;
  std::string response;
  char buf[512];
  while (response.size() < kMaxResponse) {
    ssize_t n = recv(fd, buf, sizeof(buf), 0);
    if (n <= 0) break;
    response.append(buf, static_cast<size_t>(n));
  }
  close(fd);
  if (response.empty()) {
    ++failures_;
    log_rate_limited("read failed/timed out from " + cfg_.host + ":" +
                      std::to_string(cfg_.port));
    return false;
  }
  auto eol = response.find('\n');
  std::string status_line =
      eol == std::string::npos ? response : response.substr(0, eol);
  bool ok = status_line.find("200") != std::string::npos;
  if (!ok) {
    ++failures_;
    log_rate_limited("non-200 response from " + cfg_.host + ":" +
                      std::to_string(cfg_.port) + ": " + status_line);
  }

  // Capture the body (everything after the header/blank-line separator) for
  // get_param(). Tolerates either CRLFCRLF or bare-LFLF separators.
  auto body_pos = response.find("\r\n\r\n");
  size_t sep_len = 4;
  if (body_pos == std::string::npos) {
    body_pos = response.find("\n\n");
    sep_len = 2;
  }
  last_body_ = body_pos == std::string::npos
                   ? std::string()
                   : response.substr(body_pos + sep_len);

  return ok;
}

}  // namespace mabur
