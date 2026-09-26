#pragma once
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <string>

#include "mabur/rc_proto.h"

namespace maburgs {

// maburplay's VtxRecClient sends here. Next to CalControl's 8400; the 830x
// block belongs to the stats sideport and the OSD feed.
constexpr int kRecControlPort = 8401;

// Loopback-only UDP listener for the record button's VTX wish
// (spec 2026-09-26-vtx-recorder). Commands: "vtx_rec on" / "vtx_rec off".
// The wish is HELD until the next command -- no decay: a crashed player
// must never stop the onboard recording by silence. It starts UNKNOWN
// (wire byte 0), so a maburgs restart does not stop it either; the player
// re-sends its wish every second. Non-blocking; polled from the core loop.
class RecControl {
 public:
  ~RecControl() { if (fd_ >= 0) close(fd_); }
  RecControl() = default;
  RecControl(const RecControl&) = delete;
  RecControl& operator=(const RecControl&) = delete;

  bool ok() const { return fd_ >= 0; }

  bool open(int port) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1) return false;
    const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) return false;
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      close(fd);
      return false;
    }
    fd_ = fd;
    return true;
  }

  // Drains every pending datagram. True iff the wish changed.
  bool poll() {
    if (fd_ < 0) return false;
    bool changed = false;
    char buf[64];
    for (;;) {
      const ssize_t n = recv(fd_, buf, sizeof(buf) - 1, 0);
      if (n < 0) break;
      buf[n] = '\0';
      if (apply(buf)) {
        changed = true;
        std::fprintf(stderr, "maburgs: VTX record wish -> %s\n", on_ ? "on" : "off");
        std::fflush(stderr);
      }
    }
    return changed;
  }

  // Parses one command; true iff the wish changed. I/O-free for tests.
  bool apply(const std::string& line) {
    std::string s = line;
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    bool on;
    if (s == "vtx_rec on") on = true;
    else if (s == "vtx_rec off") on = false;
    else return false;
    const bool changed = !known_ || on != on_;
    known_ = true;
    on_ = on;
    return changed;
  }

  // The RCF byte: 0 until the first command, then kRecKnown | (kRecOn if on).
  uint8_t wire() const {
    if (!known_) return 0;
    return static_cast<uint8_t>(mabur::rc::kRecKnown | (on_ ? mabur::rc::kRecOn : 0));
  }

 private:
  int fd_ = -1;
  bool known_ = false;
  bool on_ = false;
};

}  // namespace maburgs
