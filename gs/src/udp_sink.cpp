#include "udp_sink.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace maburgs {

UdpSink::UdpSink(const std::string& host, int port) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) return;
  const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (fd < 0) return;
  if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd);
    return;
  }
  fd_ = fd;
}

UdpSink::~UdpSink() {
  if (fd_ >= 0) close(fd_);
}

bool UdpSink::send(const uint8_t* data, size_t len) {
  if (fd_ < 0 || ::send(fd_, data, len, 0) != static_cast<ssize_t>(len)) {
    ++failed_;
    return false;
  }
  ++sent_;
  bytes_ += len;
  return true;
}

}  // namespace maburgs
