#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <string>
#include "mabur/player_fb.h"
#include "mtest.h"
#include "player_feedback.h"

namespace {
// Sends a raw datagram to 127.0.0.1:port. Returns true on success.
bool send_to(int port, const std::string& s) {
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return false;
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(static_cast<uint16_t>(port));
  inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
  const ssize_t n = sendto(fd, s.data(), s.size(), 0,
                           reinterpret_cast<sockaddr*>(&a), sizeof(a));
  close(fd);
  return n == static_cast<ssize_t>(s.size());
}

std::string dgram(bool idr, const char* reason, uint64_t joins) {
  mabur::playerfb::Msg m;
  m.idr = idr;
  m.reason = std::string(reason) == "join" ? 2 : 1;
  m.joins = joins;
  char buf[192];
  const size_t n = mabur::playerfb::format(m, buf, sizeof(buf));
  return std::string(buf, n);
}
}  // namespace

TEST(open_binds_an_ephemeral_port) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  CHECK(fb.ok());
  CHECK(fb.port() > 0);
  CHECK(!fb.have_any());
}

TEST(level_edge_is_reported_once) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(true, "join", 1)));
  CHECK(fb.poll(1000) == true);   // clear -> set edge
  CHECK(fb.want());
  REQUIRE(send_to(fb.port(), dgram(true, "join", 2)));
  CHECK(fb.poll(1500) == false);  // still set: NOT a new episode
  CHECK(fb.want());
  CHECK(fb.msg().joins == 2);
}

TEST(level_clears_and_can_re_edge) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(true, "flush", 0)));
  CHECK(fb.poll(1000));
  REQUIRE(send_to(fb.port(), dgram(false, "flush", 0)));
  CHECK(!fb.poll(1100));
  CHECK(!fb.want());
  REQUIRE(send_to(fb.port(), dgram(true, "flush", 0)));
  CHECK(fb.poll(1200));  // new episode
}

TEST(backlog_collapses_to_the_newest) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(true, "join", 1)));
  REQUIRE(send_to(fb.port(), dgram(true, "join", 9)));
  CHECK(fb.poll(1000));
  CHECK(fb.msg().joins == 9);
  CHECK(fb.datagrams() == 2);
}

TEST(malformed_is_counted_and_ignored) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), "garbage not a datagram"));
  CHECK(!fb.poll(1000));
  CHECK(fb.malformed() == 1);
  CHECK(!fb.have_any());
  CHECK(!fb.want());
}

TEST(silence_expires_the_level_without_a_new_edge) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), dgram(true, "flush", 0)));
  CHECK(fb.poll(1000));
  CHECK(fb.want());
  fb.expire(5000, 3000);  // 4 s of silence, stale_ms 3000
  CHECK(!fb.want());
  CHECK(fb.age_ms(5000) == 4000);
}

TEST(zero_length_datagram_does_not_stall_the_drain) {
  maburgs::PlayerFeedback fb;
  std::string err;
  REQUIRE(fb.open(0, &err));
  REQUIRE(send_to(fb.port(), ""));  // zero-length datagram, arrives first
  REQUIRE(send_to(fb.port(), dgram(true, "join", 1)));
  CHECK(fb.poll(1000));  // the valid datagram behind it must still be seen
  CHECK(fb.want());
  CHECK(fb.msg().joins == 1);
  CHECK(fb.malformed() == 1);
  CHECK(fb.datagrams() == 2);
}

MTEST_MAIN
