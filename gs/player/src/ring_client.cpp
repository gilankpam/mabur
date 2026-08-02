#include "ring_client.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

#include "mabur/frame_wire.h"

namespace maburplay {
namespace {

uint64_t now_ms() {
  struct timespec ts;
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000ull +
         static_cast<uint64_t>(ts.tv_nsec) / 1000000ull;
}

constexpr uint64_t kDoorReconnectBackoffMs = 1000;

}  // namespace

RingClient::RingClient(Cfg cfg, Sink sink)
    : cfg_(std::move(cfg)), sink_(std::move(sink)) {}

RingClient::~RingClient() { drop_door_(); }

bool RingClient::open() { return reader_.open(cfg_.ring_path); }

size_t RingClient::drain_ring_() {
  size_t n = 0;
  maburgs::AuRecordMeta m;
  std::vector<uint8_t> au;
  for (;;) {
    const auto res = reader_.next(&m, &au);
    if (res == maburgs::AuRingReader::Res::kNone) break;
    if (res == maburgs::AuRingReader::Res::kResync) {
      // Reader-observed discontinuity (epoch restart, lap overrun beyond
      // repair, or reopen recovery). meta/au are not populated on this
      // return; the flag rides forward to whatever AU is next delivered.
      pending_flush_ = true;
      continue;
    }
    // kOk: a real record. It may still be dropped whole by policy below,
    // in which case flush carries forward exactly as for kResync.
    const bool complete = (m.flags & maburgs::kRecFlagComplete) != 0;
    if (m.flags & mabur::framewire::kFlagDiscont) pending_flush_ = true;
    if (m.sid == 3 && !complete) {
      ++dropped_enhance_incomplete_;
      continue;
    }
    if (!complete) ++truncated_base_;  // base AU: delivered anyway, just counted
    AuEvent ev{m, std::move(au), pending_flush_};
    pending_flush_ = false;
    ++delivered_;
    ++n;
    sink_(std::move(ev));
    au.clear();  // au may be left moved-from; next() always assign()s it fresh
  }
  return n;
}

bool RingClient::oneshot_drain() {
  drain_ring_();
  return !reader_.dead();
}

size_t RingClient::pump(int timeout_ms) {
  service_door_(timeout_ms);
  return drain_ring_();
}

void RingClient::maybe_connect_door_() {
  if (door_fd_ >= 0) return;
  if (cfg_.socket.empty()) return;
  const uint64_t now = now_ms();
  if (door_last_attempt_ms_ != 0 &&
      now - door_last_attempt_ms_ < kDoorReconnectBackoffMs)
    return;
  door_last_attempt_ms_ = now;

  sockaddr_un a{};
  if (cfg_.socket.size() >= sizeof(a.sun_path)) return;
  const int fd = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
  if (fd < 0) return;
  a.sun_family = AF_UNIX;
  std::strncpy(a.sun_path, cfg_.socket.c_str(), sizeof(a.sun_path) - 1);
  if (::connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) {
    // Covers both "no socket present" (ENOENT) and "listener not up yet"
    // (ECONNREFUSED) alike: doorbell absence/loss is never fatal, just a
    // reason to fall back to the plain timeout-sleep wait in service_door_.
    ::close(fd);
    return;
  }
  door_fd_ = fd;
  door_hello_ok_ = false;
}

void RingClient::drop_door_() {
  if (door_fd_ >= 0) ::close(door_fd_);
  door_fd_ = -1;
  door_hello_ok_ = false;
}

void RingClient::door_mismatch_(const char* why) {
  if (!door_mismatch_logged_) {
    std::fprintf(stderr, "ring_client: doorbell hello mismatch on %s: %s\n",
                 cfg_.socket.c_str(), why);
    door_mismatch_logged_ = true;
  }
  drop_door_();
  door_last_attempt_ms_ = now_ms();  // retry no sooner than 1s from the drop
}

void RingClient::handle_door_datagram_(const uint8_t* buf, ssize_t n) {
  if (door_hello_ok_) return;  // already validated: this is a notify wakeup, discard
  if (n != 16) {
    door_mismatch_("bad hello size");
    return;
  }
  uint32_t magic, ver, sb, sc;
  std::memcpy(&magic, buf, 4);
  std::memcpy(&ver, buf + 4, 4);
  std::memcpy(&sb, buf + 8, 4);
  std::memcpy(&sc, buf + 12, 4);
  const auto g = reader_.geom();
  if (magic != maburgs::kAuRingMagic || ver != maburgs::kAuRingVersion ||
      sb != g.slot_bytes || sc != g.slot_count) {
    door_mismatch_("magic/version/geometry");
    return;
  }
  door_hello_ok_ = true;
  door_mismatch_logged_ = false;  // a later independent mismatch may log again
}

void RingClient::service_door_(int timeout_ms) {
  maybe_connect_door_();
  if (door_fd_ < 0) {
    // No doorbell: correctness never depends on it, just fall back to a
    // plain sleep so pump() has the same wait-then-drain cadence.
    if (timeout_ms > 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
    return;
  }
  pollfd pfd{door_fd_, POLLIN, 0};
  const int pr = ::poll(&pfd, 1, timeout_ms);
  if (pr <= 0) return;  // timeout or interrupted: ring still gets drained by the caller
  if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
    drop_door_();
    door_last_attempt_ms_ = now_ms();
    return;
  }
  if (!(pfd.revents & POLLIN)) return;
  // Drain every queued datagram (hello and/or notify wakeups) before
  // returning to the ring read — they carry no data of their own.
  for (;;) {
    uint8_t buf[64];
    const ssize_t r = ::recv(door_fd_, buf, sizeof(buf), MSG_DONTWAIT);
    if (r < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      drop_door_();
      door_last_attempt_ms_ = now_ms();
      break;
    }
    if (r == 0) {  // EOF: server closed/replaced the connection
      drop_door_();
      door_last_attempt_ms_ = now_ms();
      break;
    }
    handle_door_datagram_(buf, r);
    if (door_fd_ < 0) break;  // dropped by a mismatch above
  }
}

}  // namespace maburplay
