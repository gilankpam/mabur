#pragma once
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>

namespace maburplay {

// maburplay's line to maburgs' RecControl (gs/src/rec_control.h) on
// 127.0.0.1: fire-and-forget UDP. tick() carries the record button's VTX
// wish (sent on change and re-sent every kPeriodMs, so a lost datagram or a
// restarted maburgs converges within a second). send_line() carries one-shot
// reports ("refresh <reason>": the picture needs a key frame); loopback
// does not lose datagrams, and the core keeps asking the drone until an
// IRAP lands, so one send is enough.
class CoreControlClient {
 public:
  static constexpr uint64_t kPeriodMs = 1000;

  ~CoreControlClient() { if (fd_ >= 0) close(fd_); }
  CoreControlClient() = default;
  CoreControlClient(const CoreControlClient&) = delete;
  CoreControlClient& operator=(const CoreControlClient&) = delete;

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

  void send_line(const char* line) {
    if (fd_ < 0) return;
    (void)sendto(fd_, line, std::strlen(line), 0, reinterpret_cast<const sockaddr*>(&addr_),
                 sizeof(addr_));
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
