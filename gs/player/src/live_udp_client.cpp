#include "live_udp_client.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace maburplay {

namespace {
// A reply is never close to this: the biggest one, a bare `get`'s full
// table dump, is well under it on both daemons' live-key tables.
constexpr size_t kMaxReply = 8192;
}  // namespace

LiveUdpClient::~LiveUdpClient() {
  if (fd_ >= 0) ::close(fd_);
}

bool LiveUdpClient::open(int port, std::string* err) {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  in_flight_ = false;
  port_ = port;
  return connect_fresh(err);
}

bool LiveUdpClient::connect_fresh(std::string* err) {
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    if (err) *err = std::string("socket: ") + std::strerror(errno);
    return false;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port_));
  if (::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1) {
    ::close(fd);
    if (err) *err = "bad loopback address";
    return false;
  }
  // connect() on a datagram socket just fixes the peer and picks a fresh
  // ephemeral source port; it sends nothing and never fails for "no
  // listener there yet" the way a stream socket would.
  if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    const int e = errno;
    ::close(fd);
    if (err) *err = std::string("connect: ") + std::strerror(e);
    return false;
  }
  fd_ = fd;
  return true;
}

bool LiveUdpClient::send(const std::string& line, uint64_t now_ms, std::string* why) {
  if (in_flight_) {
    if (why) *why = "busy";
    return false;
  }
  if (fd_ < 0) {
    if (why) *why = "not open";
    return false;
  }
  const ssize_t n = ::send(fd_, line.data(), line.size(), 0);
  if (n < 0 || static_cast<size_t>(n) != line.size()) {
    if (why) *why = std::string("send: ") + std::strerror(errno);
    return false;
  }
  in_flight_ = true;
  sent_ms_ = now_ms;
  return true;
}

bool LiveUdpClient::send_set(const std::string& key, const std::string& value, uint64_t now_ms,
                             std::string* why) {
  return send("set " + key + " = " + value, now_ms, why);
}

bool LiveUdpClient::send_get(const std::string& key, uint64_t now_ms, std::string* why) {
  return send(key.empty() ? "get" : "get " + key, now_ms, why);
}

bool LiveUdpClient::send_line(const std::string& line, uint64_t now_ms, std::string* why) {
  return send(line, now_ms, why);
}

std::optional<LiveReply> LiveUdpClient::poll(uint64_t now_ms) {
  if (!in_flight_) return std::nullopt;
  if (fd_ >= 0) {
    char buf[kMaxReply];
    const ssize_t n = ::recv(fd_, buf, sizeof(buf), 0);
    if (n >= 0) {
      in_flight_ = false;
      const std::string text(buf, static_cast<size_t>(n));
      // "ok", "ok <value>" or "ok\n<dump>" -- index 2 is always the one
      // separator char (space or newline) when there is a payload.
      if (text.rfind("ok", 0) == 0) return LiveReply{true, text.size() > 2 ? text.substr(3) : ""};
      if (text.rfind("err ", 0) == 0) return LiveReply{false, text.substr(4)};
      return LiveReply{false, "bad reply"};
    }
  }
  if (now_ms - sent_ms_ >= kReplyTimeoutMs) {
    in_flight_ = false;
    // A reply that only shows up after this must never be read as the
    // next exchange's: drop the socket it would arrive on and open a
    // fresh one (a new ephemeral source port) before saying so.
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    std::string ignored;
    connect_fresh(&ignored);
    return LiveReply{false, "no answer"};
  }
  return std::nullopt;
}

}  // namespace maburplay
