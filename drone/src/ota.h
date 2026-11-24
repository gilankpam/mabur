#pragma once
// Drone side of the over-the-air update: a MAVLink FTP write subset that can
// only ever touch <dir>/maburd.
#include <cstdint>
#include <string>
#include <vector>

#include "mabur/ota_mav.h"

namespace mabur {

struct OtaEnv {
  std::string dir = "/tmp/mabur-up";
  ota::Ident ident;       // release, board, chip
  bool trial = false;     // MABUR_TRIAL=1 in main
  uint64_t start_ms = 0;  // process start, steady clock
};

class OtaServer {
 public:
  explicit OtaServer(OtaEnv env);
  ~OtaServer();
  OtaServer(const OtaServer&) = delete;
  OtaServer& operator=(const OtaServer&) = delete;

  // One inbound uplink-lane message; returns the replies to queue (0 or 1).
  std::vector<ota::OtaMsg> on_msg(const ota::OtaMsg& m, uint64_t now_ms);
  bool exit_requested() const { return exit_; }
  void set_chip(std::string chip) { env_.ident.chip = std::move(chip); }

 private:
  ota::Ftp ftp(const ota::Ftp& req);
  ota::Status status(uint64_t now_ms) const;
  ota::Status update(const ota::UpdateCmd& u, uint64_t now_ms);
  bool trial_running() const;
  void close_fd();

  OtaEnv env_;
  int fd_ = -1;
  bool exit_ = false;
  // A trial binary's trial ends for good at the first result it sees: Create
  // deletes result to take the next upload, and that must not restart it.
  mutable bool trial_over_ = false;
};

}  // namespace mabur
