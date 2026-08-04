#include "gs_source.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace maburplay {

GsSource::~GsSource() {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

bool GsSource::open(int port, std::string* err) {
  fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd_ < 0) {
    if (err) *err = std::string("gs osd: socket: ") + std::strerror(errno);
    return false;
  }
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons((uint16_t)port);
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(fd_, (sockaddr*)&a, sizeof(a)) != 0) {
    if (err)
      *err = "gs osd: cannot bind 127.0.0.1:" + std::to_string(port) + ": " +
             std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  socklen_t len = sizeof(a);
  if (::getsockname(fd_, (sockaddr*)&a, &len) == 0) port_ = ntohs(a.sin_port);
  opened_ = true;
  return true;
}

bool GsSource::feed_open() {
  opened_ = true;
  port_ = 0;
  return true;
}

bool GsSource::feed(const uint8_t* p, size_t n, uint64_t now_ms) {
  ++datagrams_;
  GsSnapshot s;
  if (!parse_gs_snapshot((const char*)p, n, &s)) {
    ++parse_errors_;
    return false;
  }
  snap_ = std::move(s);
  ++snapshots_;
  last_ok_ms_ = now_ms;
  return true;
}

bool GsSource::poll(uint64_t now_ms) {
  if (fd_ < 0) return false;
  bool fresh = false;
  for (;;) {
    const ssize_t n = ::recv(fd_, buf_.data(), buf_.size(), 0);
    if (n <= 0) break;  // EAGAIN or an empty datagram: nothing more to drain
    // Every datagram is parsed, not just the last: parse_errors_ must count
    // honestly, and a malformed tail datagram must not discard a good one
    // that arrived ahead of it in the same drain.
    if (feed(buf_.data(), (size_t)n, now_ms)) fresh = true;
  }
  return fresh;
}

bool GsSource::stale(uint64_t now_ms) const {
  if (stale_ms_ <= 0 || snapshots_ == 0) return false;
  return now_ms - last_ok_ms_ >= (uint64_t)stale_ms_;
}

}  // namespace maburplay
