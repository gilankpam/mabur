#include "display_watch.h"

#include <linux/netlink.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace maburplay {

namespace {

bool field_is(const char* f, size_t n, const char* want) {
  const size_t w = std::strlen(want);
  return n == w && std::memcmp(f, want, w) == 0;
}

}  // namespace

bool is_drm_hotplug_uevent(const char* buf, size_t len) {
  bool drm = false;
  bool hotplug = false;
  size_t i = 0;
  while (i < len) {
    const char* f = buf + i;
    const size_t n = strnlen(f, len - i);
    if (field_is(f, n, "SUBSYSTEM=drm")) drm = true;
    if (field_is(f, n, "HOTPLUG=1")) hotplug = true;
    i += n + 1;
  }
  return drm && hotplug;
}

DisplayWatch::DisplayWatch(bool uevents) : uevents_(uevents) {}

void DisplayWatch::on_acquired(uint64_t now_ms) {
  acquired_ = true;
  acquired_ms_ = now_ms;
  next_poll_ms_ = now_ms + kPollMs;
  check_pending_ = false;
  armed_ = false;
  last_connected_ = true;
  last_activity_ms_ = now_ms;
}

bool DisplayWatch::due(uint64_t now_ms, bool uevent) {
  if (!acquired_) return false;
  bool read = false;
  if (uevent) {
    last_activity_ms_ = now_ms;
    read = true;
  }
  if (check_pending_ && now_ms >= check_at_ms_) {
    check_pending_ = false;
    read = true;
  }
  if (!uevents_ && now_ms >= next_poll_ms_) {
    next_poll_ms_ = now_ms + kPollMs;
    read = true;
  }
  return read;
}

DisplayWatch::Action DisplayWatch::on_status(uint64_t now_ms, bool connected) {
  const bool was_connected = last_connected_;
  last_connected_ = connected;
  // Blind while the guard or a cap window runs.
  const uint64_t blind_end = std::max(acquired_ms_ + kGuardMs, capped_until_ms_);
  if (now_ms < blind_end) {
    // Possibly the acquire's own modeset making the sink pulse HPD: not an
    // unplug. Look again once the blind spell is over, in case it was.
    if (!connected) {
      check_pending_ = true;
      check_at_ms_ = blind_end;
    }
    return Action::kNone;
  }
  if (!connected) {
    last_activity_ms_ = now_ms;
    if (armed_) return Action::kNone;
    armed_ = true;
    return Action::kUnplugged;
  }
  if (!armed_) return Action::kNone;
  // The reconnect itself starts the settle; so does every uevent after it.
  if (!was_connected) last_activity_ms_ = std::max(last_activity_ms_, now_ms);
  if (now_ms - last_activity_ms_ < kSettleMs) {
    check_pending_ = true;
    check_at_ms_ = last_activity_ms_ + kSettleMs;
    return Action::kNone;
  }
  armed_ = false;
  check_pending_ = false;
  const uint64_t oldest = rebuild_ms_[rebuild_next_];
  if (rebuild_count_ == kMaxRebuilds && now_ms - oldest < kCapWindowMs) {
    capped_until_ms_ = oldest + kCapWindowMs;
    return Action::kCapped;
  }
  rebuild_ms_[rebuild_next_] = now_ms;
  rebuild_next_ = (rebuild_next_ + 1) % kMaxRebuilds;
  if (rebuild_count_ < kMaxRebuilds) ++rebuild_count_;
  return Action::kRebuild;
}

HotplugSocket::~HotplugSocket() {
  if (fd_ >= 0) ::close(fd_);
}

bool HotplugSocket::open() {
  if (fd_ >= 0) return true;
  const int fd = ::socket(AF_NETLINK, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC,
                          NETLINK_KOBJECT_UEVENT);
  if (fd < 0) return false;
  sockaddr_nl a{};
  a.nl_family = AF_NETLINK;
  a.nl_groups = 1;  // the kernel's own uevents, not udev's re-broadcast
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&a), sizeof(a)) != 0) {
    const int bind_errno = errno;  // the caller logs it; close() may clobber it
    ::close(fd);
    errno = bind_errno;
    return false;
  }
  fd_ = fd;
  return true;
}

bool HotplugSocket::drain() {
  if (fd_ < 0) return false;
  bool hit = false;
  char buf[8192];
  for (;;) {
    sockaddr_nl from{};
    socklen_t from_len = sizeof(from);
    const ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), MSG_DONTWAIT,
                                 reinterpret_cast<sockaddr*>(&from), &from_len);
    if (n < 0) {
      if (errno == EINTR) continue;
      // Overflowed: uevents were dropped, and one of them may have been
      // a hotplug. A spurious status read costs nothing.
      if (errno == ENOBUFS) {
        hit = true;
        continue;
      }
      break;  // EAGAIN: drained
    }
    if (from.nl_pid != 0) continue;  // only the kernel speaks on this group
    if (is_drm_hotplug_uevent(buf, static_cast<size_t>(n))) hit = true;
  }
  return hit;
}

}  // namespace maburplay
