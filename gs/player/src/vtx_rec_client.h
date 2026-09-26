#pragma once
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>

namespace maburplay {

// Sends the record button's VTX wish to maburgs' RecControl
// (gs/src/rec_control.h) on 127.0.0.1. Fire-and-forget UDP: tick() sends on
// every change and re-sends the current wish every kPeriodMs, so a lost
// datagram or a restarted maburgs converges within a second. Always ticked,
// whatever dvr.target says: with the VTX not a target the wish is "off".
class VtxRecClient {
 public:
  static constexpr uint64_t kPeriodMs = 1000;

  ~VtxRecClient() { if (fd_ >= 0) close(fd_); }
  VtxRecClient() = default;
  VtxRecClient(const VtxRecClient&) = delete;
  VtxRecClient& operator=(const VtxRecClient&) = delete;

  bool open(int port) {
    fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd_ < 0) return false;
    addr_ = sockaddr_in{};
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(static_cast<uint16_t>(port));
    return inet_pton(AF_INET, "127.0.0.1", &addr_.sin_addr) == 1;
  }

  void tick(bool on, uint64_t now_ms) {
    if (fd_ < 0) return;
    if (have_sent_ && on == last_on_ && now_ms - last_ms_ < kPeriodMs) return;
    const char* msg = on ? "vtx_rec on" : "vtx_rec off";
    (void)sendto(fd_, msg, std::strlen(msg), 0, reinterpret_cast<const sockaddr*>(&addr_),
                 sizeof(addr_));
    have_sent_ = true;
    last_on_ = on;
    last_ms_ = now_ms;
    ++sent_;
  }

  uint64_t sent() const { return sent_; }

 private:
  int fd_ = -1;
  sockaddr_in addr_{};
  bool have_sent_ = false;
  bool last_on_ = false;
  uint64_t last_ms_ = 0;
  uint64_t sent_ = 0;
};

}  // namespace maburplay
