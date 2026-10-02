#include "relay_transport.h"

#ifndef __EMSCRIPTEN__
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>

namespace maburgs {
namespace {
class UdpTransport final : public RelayTransport {
 public:
  explicit UdpTransport(int fd) : fd_(fd) {}
  ~UdpTransport() override { close(); }
  bool send(const uint8_t* p, size_t n) override {
    const int fd = fd_.load();
    return fd >= 0 && ::send(fd, p, n, MSG_DONTWAIT) == static_cast<ssize_t>(n);
  }
  int recv(uint8_t* buf, size_t cap, int timeout_ms) override {
    const int fd = fd_.load();
    if (closed_.load() || fd < 0) return -1;
    pollfd pf{fd, POLLIN, 0};
    const int r = ::poll(&pf, 1, timeout_ms);
    if (closed_.load()) return -1;
    if (r <= 0) return 0;
    // ECONNREFUSED (ICMP port unreachable) is not fatal: the relay may be
    // restarting. RelayClient's lost() decides.
    const ssize_t n = ::recv(fd, buf, cap, MSG_DONTWAIT);
    return n > 0 ? static_cast<int>(n) : 0;
  }
  void close() override {
    if (closed_.exchange(true)) return;
    // -1 first: no send()/recv() after close() reaches a closed (or, once
    // the number is reused, someone else's) descriptor.
    const int fd = fd_.exchange(-1);
    if (fd < 0) return;
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }

 private:
  std::atomic<int> fd_;
  std::atomic<bool> closed_{false};
};
}  // namespace

std::unique_ptr<RelayTransport> open_udp_transport(const std::string& host_port, std::string& err) {
  const auto colon = host_port.rfind(':');
  const std::string host = colon == std::string::npos ? host_port : host_port.substr(0, colon);
  const std::string port = colon == std::string::npos ? "8310" : host_port.substr(colon + 1);
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  addrinfo* ai = nullptr;
  if (getaddrinfo(host.c_str(), port.c_str(), &hints, &ai) != 0 || !ai) {
    err = "cannot resolve " + host_port;
    return nullptr;
  }
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  const bool ok = fd >= 0 && ::connect(fd, ai->ai_addr, ai->ai_addrlen) == 0;
  freeaddrinfo(ai);
  if (!ok) {
    if (fd >= 0) ::close(fd);
    err = std::string("cannot open UDP to ") + host_port + ": " + std::strerror(errno);
    return nullptr;
  }
  return std::make_unique<UdpTransport>(fd);
}
}  // namespace maburgs
#endif
