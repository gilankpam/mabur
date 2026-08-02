#include <cstring>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "au_doorbell.h"
#include "mtest.h"

namespace {
int connect_client(const std::string& path) {
  int fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
  REQUIRE(fd >= 0);
  sockaddr_un a{};
  a.sun_family = AF_UNIX;
  std::strncpy(a.sun_path, path.c_str(), sizeof(a.sun_path) - 1);
  REQUIRE(connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0);
  return fd;
}
}  // namespace

TEST(hello_then_notify) {
  const std::string path = "/tmp/test_au_doorbell_" + std::to_string(getpid());
  maburgs::AuDoorbell db;
  REQUIRE(db.open(path, {4096, 8}));
  CHECK(!db.client_connected());

  int c = connect_client(path);
  db.poll();  // accepts + sends hello
  CHECK(db.client_connected());
  uint8_t hello[16];
  REQUIRE(recv(c, hello, sizeof(hello), 0) == 16);
  uint32_t magic, ver, sb, sc;
  std::memcpy(&magic, hello, 4);
  std::memcpy(&ver, hello + 4, 4);
  std::memcpy(&sb, hello + 8, 4);
  std::memcpy(&sc, hello + 12, 4);
  CHECK(magic == maburgs::kAuRingMagic);
  CHECK(ver == maburgs::kAuRingVersion);
  CHECK(sb == 4096u);
  CHECK(sc == 8u);

  db.notify(42);
  uint64_t rec = 0;
  REQUIRE(recv(c, &rec, sizeof(rec), 0) == 8);
  CHECK(rec == 42u);

  close(c);
  db.notify(43);  // dead client must not crash or block
  db.poll();      // reaps the dead client
  CHECK(!db.client_connected());

  int c2 = connect_client(path);  // reconnect works
  db.poll();
  CHECK(db.client_connected());
  REQUIRE(recv(c2, hello, sizeof(hello), 0) == 16);
  close(c2);
  unlink(path.c_str());
}

TEST(notify_without_client_is_noop) {
  const std::string path = "/tmp/test_au_doorbell2_" + std::to_string(getpid());
  maburgs::AuDoorbell db;
  REQUIRE(db.open(path, {4096, 8}));
  db.notify(1);  // no client: no-op, no crash
  unlink(path.c_str());
}

TEST(open_rejects_overlong_path) {
  maburgs::AuDoorbell db;
  const std::string long_path(200, 'x');  // > sizeof(sun_path)
  CHECK(!db.open(long_path, {4096, 8}));
  db.poll();        // must be a no-op, not accept4 on an unbound socket
  db.notify(1);     // no-op
  CHECK(!db.client_connected());
}

MTEST_MAIN
