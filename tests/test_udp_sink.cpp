#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
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
