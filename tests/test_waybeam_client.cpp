#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "mtest.h"
#include "waybeam_client.h"
#include "config.h"
using namespace mabur;

namespace {

// Binds and starts listening on a TCP socket at 127.0.0.1:0 (ephemeral
// port), returning the fd plus the resolved port via out-param. Listening
// starts here (not on the server thread) so a client connecting right
// after this returns is queued by the kernel even before the server
// thread's accept() runs — avoiding a listen-vs-connect race.
int bind_listener(uint16_t* port_out) {
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  REQUIRE(fd >= 0);
  int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  addr.sin_port = 0;
  REQUIRE(bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
  REQUIRE(listen(fd, 1) == 0);
  socklen_t len = sizeof(addr);
  REQUIRE(getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) == 0);
  *port_out = ntohs(addr.sin_port);
  return fd;
}

// Runs a one-shot listener: accepts a single connection, reads the request
// line, writes `reply`, then closes. Captures the request line into
// `*request_line_out`. Meant to be run on a background thread. The fd must
// already be listening (see bind_listener).
void serve_one(int listen_fd, const std::string& reply,
               std::string* request_line_out) {
  int conn = accept(listen_fd, nullptr, nullptr);
  if (conn < 0) {
    close(listen_fd);
    return;
  }
  char buf[4096];
  ssize_t n = read(conn, buf, sizeof(buf) - 1);
  if (n > 0) {
    buf[n] = '\0';
    std::string request(buf);
    auto pos = request.find("\r\n");
    if (pos == std::string::npos) pos = request.find('\n');
    *request_line_out = request.substr(0, pos);
  }
  (void)write(conn, reply.data(), reply.size());
  close(conn);
  close(listen_fd);
}

// Silent server that accepts, reads the request, but never sends a reply.
// Meant to be run on a background thread. The fd must already be listening.
void serve_silent(int listen_fd) {
  int conn = accept(listen_fd, nullptr, nullptr);
  if (conn < 0) {
    close(listen_fd);
    return;
  }
  char buf[4096];
  (void)read(conn, buf, sizeof(buf) - 1);  // Read request but don't reply
  // Sleep long enough to trigger the recv timeout, then close.
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  close(conn);
  close(listen_fd);
}

}  // namespace

TEST(set_param_200_reply_returns_true_and_sends_expected_request_line) {
  uint16_t port;
  int listen_fd = bind_listener(&port);
  std::string request_line;
  std::thread server(serve_one, listen_fd, "HTTP/1.0 200 OK\r\n\r\n",
                      &request_line);

  WaybeamCfg cfg;
  cfg.host = "127.0.0.1";
  cfg.port = port;
  WaybeamClient client(cfg);
  bool ok = client.set_param("video0.bitrate", "4096");

  server.join();
  CHECK(ok);
  CHECK(request_line == "GET /api/v1/set?video0.bitrate=4096 HTTP/1.0");
  CHECK(client.failures() == 0);
}

TEST(set_param_500_reply_returns_false) {
  uint16_t port;
  int listen_fd = bind_listener(&port);
  std::string request_line;
  std::thread server(serve_one, listen_fd,
                      "HTTP/1.0 500 Internal Server Error\r\n\r\n",
                      &request_line);

  WaybeamCfg cfg;
  cfg.host = "127.0.0.1";
  cfg.port = port;
  WaybeamClient client(cfg);
  bool ok = client.set_param("video0.bitrate", "4096");

  server.join();
  CHECK(!ok);
}

TEST(set_param_closed_port_fails_fast_and_counts_failure) {
  uint16_t port;
  int probe_fd = bind_listener(&port);
  close(probe_fd);  // port is now guaranteed closed (nothing listening)

  WaybeamCfg cfg;
  cfg.host = "127.0.0.1";
  cfg.port = port;
  WaybeamClient client(cfg);

  auto start = std::chrono::steady_clock::now();
  bool ok = client.set_param("video0.bitrate", "4096");
  auto elapsed = std::chrono::steady_clock::now() - start;

  CHECK(!ok);
  CHECK(client.failures() == 1);
  CHECK(elapsed < std::chrono::milliseconds(500));
}

TEST(request_idr_uses_cfg_idr_path) {
  uint16_t port;
  int listen_fd = bind_listener(&port);
  std::string request_line;
  std::thread server(serve_one, listen_fd, "HTTP/1.0 200 OK\r\n\r\n",
                      &request_line);

  WaybeamCfg cfg;
  cfg.host = "127.0.0.1";
  cfg.port = port;
  cfg.idr_path = "/api/v1/idr";
  WaybeamClient client(cfg);
  bool ok = client.request_idr();

  server.join();
  CHECK(ok);
  CHECK(request_line == "GET /api/v1/idr HTTP/1.0");
}

TEST(get_param_returns_body) {
  uint16_t port;
  int listen_fd = bind_listener(&port);
  std::string request_line;
  std::thread server(
      serve_one, listen_fd,
      "HTTP/1.0 200 OK\r\n\r\n{\"ok\":true,\"value\":\"frame-shm://mabur_f\"}",
      &request_line);

  WaybeamCfg cfg;
  cfg.host = "127.0.0.1";
  cfg.port = port;
  WaybeamClient client(cfg);
  std::string body;
  bool ok = client.get_param("outgoing.server", body);

  server.join();
  CHECK(ok);
  CHECK(request_line == "GET /api/v1/get?outgoing.server HTTP/1.0");
  CHECK(body.find("frame-shm://") != std::string::npos);
}

TEST(recv_timeout_silent_server_returns_false) {
  uint16_t port;
  int listen_fd = bind_listener(&port);
  std::thread server(serve_silent, listen_fd);

  WaybeamCfg cfg;
  cfg.host = "127.0.0.1";
  cfg.port = port;
  WaybeamClient client(cfg);

  auto start = std::chrono::steady_clock::now();
  bool ok = client.set_param("video0.bitrate", "4096");
  auto elapsed = std::chrono::steady_clock::now() - start;

  server.join();
  CHECK(!ok);
  CHECK(client.failures() == 1);
  // Should timeout waiting for recv, well under 500 ms.
  CHECK(elapsed < std::chrono::milliseconds(500));
}

MTEST_MAIN
