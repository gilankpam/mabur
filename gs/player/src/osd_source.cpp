#include "osd_source.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <vector>

namespace maburplay {

OsdSource::~OsdSource() {
  if (fd_ >= 0) ::close(fd_);
}

bool OsdSource::open(int port, std::string* err) {
  fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd_ < 0) {
    if (err) *err = std::string("socket: ") + std::strerror(errno);
    return false;
  }
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons((uint16_t)port);
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(fd_, (sockaddr*)&a, sizeof(a)) != 0) {
    if (err) *err = "bind 127.0.0.1:" + std::to_string(port) + ": " + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  socklen_t len = sizeof(a);
  if (::getsockname(fd_, (sockaddr*)&a, &len) == 0) port_ = ntohs(a.sin_port);
  opened_ = true;
  return true;
}

bool OsdSource::feed_open() {
  opened_ = true;
  return true;
}

bool OsdSource::consume_(const uint8_t* p, size_t n, uint64_t now_ms) {
  last_rx_ms_ = now_ms ? now_ms : 1;
  for (const mabur::MspMessage& m : parser_.feed(p, n)) {
    if (screen_.apply(m)) {
      complete_ = true;
      ++screens_;
    }
  }
  return complete_;
}

bool OsdSource::gate_(uint64_t now_ms) {
  if (!complete_) return false;
  if (rendered_once_ && now_ms - last_render_ms_ < (uint64_t)min_interval_ms_) return false;
  complete_ = false;
  last_render_ms_ = now_ms;
  rendered_once_ = true;
  return true;
}

bool OsdSource::poll(uint64_t now_ms) {
  if (!opened_) return false;
  if (fd_ >= 0) {
    // 64 KiB: one datagram is one decoded MSP snapshot from MspSink, which
    // is bounded by the sliding-window packet size, well under this.
    static std::vector<uint8_t> buf(65536);
    for (;;) {
      const ssize_t n = ::recv(fd_, buf.data(), buf.size(), 0);
      if (n <= 0) break;
      ++datagrams_;
      consume_(buf.data(), (size_t)n, now_ms);
    }
  }
  return gate_(now_ms);
}

bool OsdSource::feed(const uint8_t* p, size_t n, uint64_t now_ms) {
  if (!opened_) return false;
  ++datagrams_;
  consume_(p, n, now_ms);
  return gate_(now_ms);
}

bool OsdSource::stale(uint64_t now_ms) const {
  if (stale_ms_ <= 0 || last_rx_ms_ == 0) return false;
  return now_ms - last_rx_ms_ >= (uint64_t)stale_ms_;
}

}  // namespace maburplay
