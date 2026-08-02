#include "au_doorbell.h"

#include <cerrno>
#include <cstring>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace maburgs {

AuDoorbell::~AuDoorbell() {
  drop_client_();
  if (listen_ >= 0) ::close(listen_);
}

bool AuDoorbell::open(const std::string& path, AuRingGeom geom) {
  geom_ = geom;
  listen_ = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
  if (listen_ < 0) return false;
  sockaddr_un a{};
  a.sun_family = AF_UNIX;
  if (path.size() >= sizeof(a.sun_path)) return false;
  std::strncpy(a.sun_path, path.c_str(), sizeof(a.sun_path) - 1);
  ::unlink(path.c_str());
  if (::bind(listen_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0 ||
      ::listen(listen_, 1) != 0) {
    ::close(listen_);
    listen_ = -1;
    return false;
  }
  return true;
}

void AuDoorbell::drop_client_() {
  if (client_ >= 0) ::close(client_);
  client_ = -1;
}

void AuDoorbell::poll() {
  if (listen_ < 0) return;
  // Reap a dead client: a zero-byte read on SEQPACKET means EOF.
  if (client_ >= 0) {
    uint8_t b;
    const ssize_t r = ::recv(client_, &b, 1, MSG_DONTWAIT);
    if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
      drop_client_();
  }
  const int c = ::accept4(listen_, nullptr, nullptr, SOCK_NONBLOCK);
  if (c < 0) return;
  drop_client_();  // newest consumer wins
  client_ = c;
  uint8_t hello[16];
  std::memcpy(hello, &kAuRingMagic, 4);
  std::memcpy(hello + 4, &kAuRingVersion, 4);
  std::memcpy(hello + 8, &geom_.slot_bytes, 4);
  std::memcpy(hello + 12, &geom_.slot_count, 4);
  if (::send(client_, hello, sizeof(hello), MSG_NOSIGNAL | MSG_DONTWAIT) < 0 &&
      errno != EAGAIN && errno != EWOULDBLOCK)
    drop_client_();
}

void AuDoorbell::notify(uint64_t rec_no) {
  if (client_ < 0) return;
  if (::send(client_, &rec_no, sizeof(rec_no), MSG_NOSIGNAL | MSG_DONTWAIT) < 0 &&
      errno != EAGAIN && errno != EWOULDBLOCK)
    drop_client_();
}

}  // namespace maburgs
