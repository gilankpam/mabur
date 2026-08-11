#include "player_feedback.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace maburgs {

PlayerFeedback::~PlayerFeedback() {
  if (fd_ >= 0) close(fd_);
}

bool PlayerFeedback::open(int port, std::string* err) {
  const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    if (err) *err = std::string("socket: ") + std::strerror(errno);
    return false;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  // Loopback only: this is a control input, never reachable off-box.
  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    if (err) *err = std::string("bind: ") + std::strerror(errno);
    close(fd);
    return false;
  }
  sockaddr_in bound{};
  socklen_t blen = sizeof(bound);
  if (getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &blen) == 0)
    port_ = ntohs(bound.sin_port);
  fd_ = fd;
  return true;
}

bool PlayerFeedback::poll(uint64_t now_ms) {
  if (fd_ < 0) return false;
  char buf[512];
  bool got = false;
  mabur::playerfb::Msg newest;
  for (;;) {
    const ssize_t n = recv(fd_, buf, sizeof(buf), 0);
    if (n < 0) break;  // EAGAIN -- genuinely drained
    ++datagrams_;
    // recv() == 0 is an empty datagram for UDP, not EOF: it cannot parse,
    // but it must not end the drain or the datagrams behind it are stranded.
    mabur::playerfb::Msg m;
    if (n > 0 && mabur::playerfb::parse(buf, static_cast<size_t>(n), &m)) {
      newest = m;
      got = true;
    } else {
      ++malformed_;
    }
  }
  if (!got) return false;
  // A backlog collapses to the newest sample: the level is now, not a queue.
  msg_ = newest;
  ++msgs_;
  last_rx_ms_ = now_ms;
  const bool edge = newest.idr && !want_;
  want_ = newest.idr;
  return edge;
}

void PlayerFeedback::expire(uint64_t now_ms, int stale_ms) {
  if (stale_ms <= 0 || !msgs_) return;
  if (now_ms - last_rx_ms_ >= static_cast<uint64_t>(stale_ms)) want_ = false;
}

}  // namespace maburgs
