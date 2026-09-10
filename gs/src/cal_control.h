#pragma once
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <random>
#include <string>
#include <vector>

#include "cal_session.h"

namespace maburgs {

// Loopback-only UDP command listener that lets an operator ssh'd into the
// GS box start, inspect, or abort a TX-power calibration run.
//
// Binds 127.0.0.1 ONLY. The GS answers on 10.18.0.1, and a calibration
// sweep silences the GS's own transmit path for whole phases at a time
// (CalSession::radio_silent) -- something the drone or a bench laptop must
// never be able to trigger by accident or on purpose. The loopback bind is
// what makes this a deliberate trigger: you have to be logged into the GS
// itself to reach port 8400.
//
// Unlike loss_control.h's LossControl, this is compiled into every prod
// build -- calibration is a real operator workflow, not a bench-only
// scaffold -- so there is no MABUR_LOSS_SIM-style CMake guard here.
//
// Non-blocking; poll() is called from the core loop next to the other
// per-tick polls, so command handling runs on the thread that owns the
// CalSession. No locking.
class CalControl {
 public:
  ~CalControl() { if (fd_ >= 0) close(fd_); }
  CalControl() = default;
  CalControl(const CalControl&) = delete;
  CalControl& operator=(const CalControl&) = delete;

  bool ok() const { return fd_ >= 0; }

  static constexpr const char* bound_address() { return "127.0.0.1"; }

  bool open(int port) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, bound_address(), &addr.sin_addr) != 1) return false;
    const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) return false;
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      close(fd);
      return false;
    }
    fd_ = fd;
    return true;
  }

  // Drains every pending datagram, applies it, and replies to the sender.
  void poll(CalSession& s) {
    if (fd_ < 0) return;
    char buf[256];
    for (;;) {
      sockaddr_in from{};
      socklen_t flen = sizeof(from);
      const ssize_t n = recvfrom(fd_, buf, sizeof(buf) - 1, 0,
                                 reinterpret_cast<sockaddr*>(&from), &flen);
      if (n < 0) return;    // EAGAIN/error when drained -- a genuine 0-length
                            // datagram must still be answered, not treated
                            // as end-of-drain (that would leave any
                            // datagrams still queued behind it unanswered
                            // until the next tick).
      buf[n] = '\0';
      std::string reply;
      const bool changed = apply(buf, s, &reply);
      sendto(fd_, reply.data(), reply.size(), 0,
             reinterpret_cast<sockaddr*>(&from), flen);
      if (changed) {
        // Loud, on every accepted start/abort: /tmp/maburgs.log is the
        // post-mortem surface, and "why did the link go quiet for 30s"
        // must be answerable from the log without asking the operator.
        std::fprintf(stderr, "maburgs: CAL %s -> %s\n", buf, reply.c_str());
        std::fflush(stderr);
      }
    }
  }

  // Parses one command and mutates `s`. Returns true iff state changed.
  // `*reply` always gets the datagram to send back. Static and I/O-free so
  // the command language is unit-testable without a socket.
  //
  // Command language:
  //   start [margin=<db>]   begin a session (refuses if not linked / no
  //                         CAP_CALIBRATE / already running)
  //   status                one-line progress, never mutates
  //   abort                 abandon the running session
  static bool apply(const std::string& line, CalSession& s,
                    std::string* reply) {
    const std::vector<std::string> tok = split(line);
    if (tok.empty()) { *reply = "err empty"; return false; }

    if (tok[0] == "status") {
      *reply = "ok " + s.progress();
      return false;
    }

    if (tok[0] == "abort") {
      s.abort("operator abort");
      *reply = "ok aborted";
      return true;
    }

    if (tok[0] == "start") {
      double margin = -1.0;
      for (size_t i = 1; i < tok.size(); ++i) {
        double v = 0.0;
        if (kv(tok[i], "margin=", &v)) { margin = v; continue; }
        *reply = "err bad token: " + tok[i];
        return false;
      }
      if (margin >= 0.0) s.set_margin_db(margin);

      // A fresh nonce per start means a stale ack from a session the
      // operator already aborted can never be mistaken for this one's --
      // random_device rather than rand() to match how the rest of the GS
      // mints session/nonce identifiers (see the stats-sideport session id
      // in main.cpp).
      static std::random_device rd;
      const uint32_t nonce = rd();
      std::string err;
      if (!s.start(/*vtx_id=*/0, nonce, now_ms(), &err)) {
        *reply = "err " + err;
        return false;
      }
      *reply = "ok started";
      return true;
    }

    *reply = "err want start [margin=<db>] | status | abort";
    return false;
  }

 private:
  static uint64_t now_ms() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000 +
           static_cast<uint64_t>(ts.tv_nsec) / 1000000;
  }

  static std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < s.size()) {
      while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
      const size_t start = i;
      while (i < s.size() && !std::isspace(static_cast<unsigned char>(s[i]))) ++i;
      if (i > start) out.push_back(s.substr(start, i - start));
    }
    return out;
  }

  // Strict numeric parse: "margin=abc" must be rejected, not silently read
  // as 0.
  static bool kv(const std::string& tok, const char* key, double* out) {
    const size_t klen = std::strlen(key);
    if (tok.size() <= klen || tok.compare(0, klen, key) != 0) return false;
    const std::string val = tok.substr(klen);
    char* end = nullptr;
    const double v = std::strtod(val.c_str(), &end);
    if (end == val.c_str() || *end != '\0') return false;
    *out = v;
    return true;
  }

  int fd_ = -1;
};

}  // namespace maburgs
