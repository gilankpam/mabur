#include "ota.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace mabur {
namespace {
enum : uint8_t { kTerminate = 1, kReset = 2, kCreate = 6, kWrite = 7, kOpenWO = 11, kTruncate = 12,
                 kAck = 128, kNak = 129 };
enum : uint8_t { kFail = 1, kFailErrno = 2, kBadSize = 3, kBadSession = 4, kUnknown = 7,
                 kProtected = 9, kNotFound = 10 };
}  // namespace

OtaServer::OtaServer(OtaEnv env) : env_(std::move(env)) {}
OtaServer::~OtaServer() { close_fd(); }

void OtaServer::close_fd() {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

bool OtaServer::trial_running() const {
  return env_.trial && ::access((env_.dir + "/result").c_str(), F_OK) != 0;
}

std::vector<ota::OtaMsg> OtaServer::on_msg(const ota::OtaMsg& m, uint64_t) {
  if (const auto* f = std::get_if<ota::Ftp>(&m)) return {ftp(*f)};
  return {};
}

ota::Ftp OtaServer::ftp(const ota::Ftp& req) {
  ota::Ftp r;
  r.seq = uint16_t(req.seq + 1);
  r.session = req.session;
  r.req_opcode = req.opcode;
  r.opcode = kAck;
  auto nak = [&](uint8_t code) {
    r.opcode = kNak;
    r.data = {code};
    r.size = 1;
    return r;
  };

  // Path requests: only the single leaf file, so nothing can escape the dir.
  std::string path;
  if (req.opcode == kCreate || req.opcode == kOpenWO || req.opcode == kTruncate) {
    size_t n = std::min<size_t>(req.size, req.data.size());
    path.assign(req.data.begin(), req.data.begin() + n);
    path.resize(strnlen(path.c_str(), path.size()));
    if (path != "maburd" && path != "/maburd") return nak(kProtected);
    if (trial_running()) return nak(kFail);
    ::mkdir(env_.dir.c_str(), 0755);
  }
  const std::string file = env_.dir + "/maburd";

  switch (req.opcode) {
    case kCreate:
    case kOpenWO: {
      close_fd();
      int flags = O_WRONLY | O_CREAT | (req.opcode == kCreate ? O_TRUNC : 0);
      fd_ = ::open(file.c_str(), flags | O_CLOEXEC, 0755);
      if (fd_ < 0) return nak(kFailErrno);
      ::fchmod(fd_, 0755);
      if (req.opcode == kCreate) ::unlink((env_.dir + "/result").c_str());
      r.session = 0;
      return r;
    }
    case kWrite: {
      if (fd_ < 0 || req.session != 0) return nak(kBadSession);
      size_t n = std::min<size_t>(req.size, req.data.size());
      if (n && ::pwrite(fd_, req.data.data(), n, off_t(req.offset)) != ssize_t(n)) return nak(kFailErrno);
      return r;
    }
    case kTruncate: {
      int fd = ::open(file.c_str(), O_WRONLY | O_CLOEXEC);
      if (fd < 0) return nak(errno == ENOENT ? kNotFound : kFailErrno);
      int rc = ::ftruncate(fd, off_t(req.offset));
      ::close(fd);
      return rc ? nak(kFailErrno) : r;
    }
    case kTerminate:
    case kReset:
      close_fd();
      return r;
    default:
      return nak(kUnknown);
  }
}

}  // namespace mabur
