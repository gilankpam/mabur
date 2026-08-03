#include "mtest.h"
#include "osd_source.h"
#include "mabur/msp_dp.h"
#include <arpa/inet.h>
#include <dirent.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <vector>
using namespace maburplay;

static std::vector<uint8_t> snapshot(int row, int col, const std::string& text) {
  std::vector<uint8_t> s;
  std::vector<uint8_t> clr = {2};
  mabur::msp_append_message(s, 182, clr.data(), clr.size());
  std::vector<uint8_t> ds = {3, (uint8_t)row, (uint8_t)col, 0};
  for (char c : text) ds.push_back((uint8_t)c);
  mabur::msp_append_message(s, 182, ds.data(), ds.size());
  std::vector<uint8_t> scr = {4};
  mabur::msp_append_message(s, 182, scr.data(), scr.size());
  return s;
}

static void send_to(int port, const std::vector<uint8_t>& b) {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  REQUIRE(fd >= 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons((uint16_t)port);
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  REQUIRE(::sendto(fd, b.data(), b.size(), 0, (sockaddr*)&a, sizeof(a)) == (ssize_t)b.size());
  ::close(fd);
}

TEST(datagram_becomes_a_complete_screen) {
  OsdSource src;
  std::string err;
  REQUIRE(src.open(0, &err));
  CHECK(src.port() > 0);

  CHECK(!src.poll(1000));  // nothing sent yet
  send_to(src.port(), snapshot(2, 3, "AB"));
  // The datagram is on the loopback queue; poll until it lands (bounded).
  bool ready = false;
  for (int i = 0; i < 100 && !ready; ++i) ready = src.poll(1000 + i);
  CHECK(ready);
  CHECK(src.datagrams() == 1);
  CHECK(src.screens() == 1);
  CHECK(src.screen().cell(2, 3) == (uint16_t)'A');
  CHECK(src.screen().cell(2, 4) == (uint16_t)'B');
}

// Number of open descriptors held by this process. Counting /proc/self/fd is
// what makes the leak observable: open(0, ...) always hands back a fresh
// ephemeral socket, so "it still works afterwards" passes just as happily
// with the close() removed (mutation-verified) -- only the fd count moves.
static int open_fd_count() {
  DIR* d = ::opendir("/proc/self/fd");
  if (!d) return -1;
  int n = 0;
  while (::readdir(d)) ++n;
  ::closedir(d);
  return n;  // includes . / .. / the dirfd itself: constant offsets, we diff
}

TEST(reopen_does_not_leak_the_previous_socket_and_still_works) {
  OsdSource src;
  std::string err;
  REQUIRE(src.open(0, &err));
  CHECK(src.port() > 0);

  // Re-opening on the same instance must CLOSE the previous fd, not leak it.
  // Eight reopens: with the close in place the count is unchanged; without
  // it the delta is exactly the number of reopens.
  const int before = open_fd_count();
  REQUIRE(before > 0);  // /proc must be mounted for this to mean anything
  for (int i = 0; i < 8; ++i) REQUIRE(src.open(0, &err));
  const int after = open_fd_count();
  CHECK(after - before == 0);

  // ...and the instance is still functional on its latest socket.
  CHECK(src.port() > 0);
  send_to(src.port(), snapshot(1, 1, "Z"));
  bool ready = false;
  for (int i = 0; i < 100 && !ready; ++i) ready = src.poll(1000 + i);
  CHECK(ready);
  CHECK(src.screen().cell(1, 1) == (uint16_t)'Z');
}

TEST(rate_limit_suppresses_a_second_render_inside_the_window) {
  OsdSource src;
  REQUIRE(src.feed_open());  // socket-free mode for deterministic timing
  src.set_min_interval_ms(30);
  const std::vector<uint8_t> s = snapshot(0, 0, "X");
  CHECK(src.feed(s.data(), s.size(), 1000));
  CHECK(!src.feed(s.data(), s.size(), 1010));  // 10 ms later: suppressed
  CHECK(src.feed(s.data(), s.size(), 1040));   // 40 ms later: allowed
  CHECK(src.screens() == 3);                   // all three parsed, two rendered
}

TEST(staleness_reports_only_after_traffic_then_silence) {
  OsdSource src;
  REQUIRE(src.feed_open());
  src.set_stale_ms(2000);
  CHECK(!src.stale(5000));  // never received anything: nothing to blank
  const std::vector<uint8_t> s = snapshot(0, 0, "X");
  src.feed(s.data(), s.size(), 1000);
  CHECK(!src.stale(2999));
  CHECK(src.stale(3000));
  src.feed(s.data(), s.size(), 3100);
  CHECK(!src.stale(3200));
}

TEST(stale_ms_zero_disables_blanking) {
  OsdSource src;
  REQUIRE(src.feed_open());
  src.set_stale_ms(0);
  const std::vector<uint8_t> s = snapshot(0, 0, "X");
  src.feed(s.data(), s.size(), 1000);
  CHECK(!src.stale(1000000));
}

TEST(partial_snapshot_does_not_render_until_draw_screen) {
  OsdSource src;
  REQUIRE(src.feed_open());
  std::vector<uint8_t> partial;
  std::vector<uint8_t> clr = {2};
  mabur::msp_append_message(partial, 182, clr.data(), clr.size());
  CHECK(!src.feed(partial.data(), partial.size(), 1000));
  std::vector<uint8_t> scr;
  std::vector<uint8_t> ds = {4};
  mabur::msp_append_message(scr, 182, ds.data(), ds.size());
  CHECK(src.feed(scr.data(), scr.size(), 1100));
}

MTEST_MAIN
