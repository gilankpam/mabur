#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <memory>
#include <vector>
#include "mtest.h"
#include "udp_sink.h"
using namespace maburgs;

TEST(sends_datagrams_to_local_receiver) {
  int rx = socket(AF_INET, SOCK_DGRAM, 0);
  REQUIRE(rx >= 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  REQUIRE(bind(rx, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
  socklen_t alen = sizeof(addr);
  REQUIRE(getsockname(rx, reinterpret_cast<sockaddr*>(&addr), &alen) == 0);

  UdpSink sink("127.0.0.1", ntohs(addr.sin_port));
  REQUIRE(sink.ok());
  const uint8_t pkt[5] = {1, 2, 3, 4, 5};
  CHECK(sink.send(pkt, sizeof(pkt)));
  uint8_t buf[16];
  ssize_t n = recv(rx, buf, sizeof(buf), 0);
  CHECK(n == 5);
  CHECK(std::memcmp(buf, pkt, 5) == 0);
  CHECK(sink.sent() == 1);
  close(rx);
}

TEST(bad_address_fails_soft) {
  UdpSink sink("not-an-ip", 5600);
  CHECK(!sink.ok());
  const uint8_t pkt[1] = {0};
  CHECK(!sink.send(pkt, 1));
  CHECK(sink.failed() == 1);
}

namespace {
// Binds an ephemeral UDP loopback socket and returns its fd/port. A short
// receive timeout is set so a fan-out regression (a destination that
// silently gets nothing) fails the CHECK below instead of hanging recv()
// forever and wedging the whole test binary.
int bind_ephemeral(uint16_t* port_out) {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return fd;
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd);
    return -1;
  }
  timeval tv{};
  tv.tv_sec = 2;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  socklen_t alen = sizeof(addr);
  getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &alen);
  *port_out = ntohs(addr.sin_port);
  return fd;
}
}  // namespace

// Proves the actual fan-out semantics (main.cpp's send lambda): every sink
// in the vector gets the SAME buffer, not just the first one. This is the
// property that distinguishes real fan-out from a stub that silently drops
// everything after out[0] -- config parsing alone can't catch that.
//
// Mirrors main.cpp's container choice exactly: UdpSink has a user-declared
// destructor and a deleted copy constructor with no declared move
// constructor, so std::vector<UdpSink> growth (even bare reserve() on an
// empty vector) fails to compile -- the standard library instantiates a
// copy/move path unconditionally, not only when a reallocation actually
// occurs at runtime. vector<unique_ptr<UdpSink>> sidesteps that: the vector
// moves pointers, never UdpSink objects.
TEST(fanout_delivers_to_every_destination) {
  uint16_t port_a = 0, port_b = 0;
  int rx_a = bind_ephemeral(&port_a);
  int rx_b = bind_ephemeral(&port_b);
  REQUIRE(rx_a >= 0);
  REQUIRE(rx_b >= 0);

  std::vector<std::unique_ptr<UdpSink>> sinks;
  sinks.reserve(2);
  sinks.push_back(std::make_unique<UdpSink>("127.0.0.1", port_a));
  sinks.push_back(std::make_unique<UdpSink>("127.0.0.1", port_b));
  REQUIRE(sinks[0]->ok());
  REQUIRE(sinks[1]->ok());

  const uint8_t pkt[4] = {0xDE, 0xAD, 0xBE, 0xEF};
  bool any = false;
  for (auto& s : sinks)
    if (s->send(pkt, sizeof(pkt))) any = true;
  CHECK(any);

  uint8_t buf_a[16], buf_b[16];
  ssize_t n_a = recv(rx_a, buf_a, sizeof(buf_a), 0);
  ssize_t n_b = recv(rx_b, buf_b, sizeof(buf_b), 0);
  CHECK(n_a == 4);
  CHECK(n_b == 4);
  CHECK(std::memcmp(buf_a, pkt, 4) == 0);
  CHECK(std::memcmp(buf_b, pkt, 4) == 0);  // both received -- not just sinks[0]
  close(rx_a);
  close(rx_b);
}

// One dead destination must not blind the live ones: the fan-out loop must
// not short-circuit or throw on the first failing sink.
TEST(fanout_dead_destination_does_not_block_live_one) {
  uint16_t port_b = 0;
  int rx_b = bind_ephemeral(&port_b);
  REQUIRE(rx_b >= 0);

  std::vector<std::unique_ptr<UdpSink>> sinks;
  sinks.reserve(2);
  sinks.push_back(std::make_unique<UdpSink>("not-an-ip", 12345));  // fails to resolve -> ok() == false
  sinks.push_back(std::make_unique<UdpSink>("127.0.0.1", port_b));
  CHECK(!sinks[0]->ok());
  CHECK(sinks[1]->ok());

  const uint8_t pkt[3] = {7, 8, 9};
  bool any = false;
  for (auto& s : sinks)
    if (s->send(pkt, sizeof(pkt))) any = true;
  CHECK(any);  // the live sink still got it despite the dead one failing first
  CHECK(sinks[0]->failed() == 1);

  uint8_t buf_b[16];
  ssize_t n_b = recv(rx_b, buf_b, sizeof(buf_b), 0);
  CHECK(n_b == 3);
  CHECK(std::memcmp(buf_b, pkt, 3) == 0);
  close(rx_b);
}

TEST(bytes_accumulates_on_success_only) {
  int rx = socket(AF_INET, SOCK_DGRAM, 0);
  REQUIRE(rx >= 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  REQUIRE(bind(rx, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
  socklen_t alen = sizeof(addr);
  REQUIRE(getsockname(rx, reinterpret_cast<sockaddr*>(&addr), &alen) == 0);
  UdpSink sink("127.0.0.1", ntohs(addr.sin_port));
  const uint8_t pkt[7] = {0};
  CHECK(sink.send(pkt, 7));
  CHECK(sink.send(pkt, 3));
  CHECK(sink.bytes() == 10);
  close(rx);

  UdpSink dead("not-an-ip", 5600);
  CHECK(!dead.send(pkt, 7));
  CHECK(dead.bytes() == 0);  // failed sends don't count
}
MTEST_MAIN
