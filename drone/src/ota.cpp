#include "ota.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>

#include "mabur/sha256.h"

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

std::vector<ota::OtaMsg> OtaServer::on_msg(const ota::OtaMsg& m, uint64_t now_ms) {
  if (const auto* f = std::get_if<ota::Ftp>(&m)) return {ftp(*f)};
  if (std::holds_alternative<ota::IdentReq>(m)) return {env_.ident};
  if (const auto* u = std::get_if<ota::UpdateCmd>(&m)) return {update(*u, now_ms)};
  return {};
}

namespace {
bool has(const std::string& p) { return ::access(p.c_str(), F_OK) == 0; }

std::string slurp(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
}  // namespace

bool OtaServer::trial_running() const {
  if (env_.trial && has(env_.dir + "/result")) trial_over_ = true;
  return env_.trial && !trial_over_;
}

ota::Status OtaServer::status(uint64_t now_ms) const {
  std::istringstream in(slurp(env_.dir + "/result"));
  std::string word, why;
  in >> word >> why;
  if (word == "committed") return {ota::State::kCommitted, ota::Reason::kNone, 0};
  if (word == "rolled_back" || word == "refused") {
    auto r = ota::reason_from_name(why);
    if (r && *r != ota::Reason::kNone)
      return {word == "refused" ? ota::State::kRefused : ota::State::kRolledBack, *r, 0};
  }
  if (trial_running()) {
    int64_t left = 40 - int64_t(now_ms - env_.start_ms) / 1000;
    return {ota::State::kTrial, ota::Reason::kNone, uint8_t(std::clamp<int64_t>(left, 1, 40))};
  }
  if (has(env_.dir + "/maburd")) return {ota::State::kReceiving, ota::Reason::kNone, 0};
  return {};
}

ota::Status OtaServer::update(const ota::UpdateCmd& u, uint64_t now_ms) {
  const std::string bin = env_.dir + "/maburd", go = env_.dir + "/go";
  auto refuse = [](ota::Reason r) { return ota::Status{ota::State::kRefused, r, 0}; };
  switch (u.cmd) {
    case ota::Cmd::kStart: {
      if (trial_running()) {  // a START resent after the old maburd's reply was lost
        auto h = sha256_file(bin);
        return h && *h == u.sha256 ? status(now_ms) : refuse(ota::Reason::kBusy);
      }
      if (has(go))  // a resent START after a lost reply gets the same answer
        return slurp(go) == sha256_hex(u.sha256) + "\n" ? ota::Status{ota::State::kTrial, ota::Reason::kNone, 40}
                                                             : refuse(ota::Reason::kBusy);
      if (!has(bin)) return refuse(ota::Reason::kNoFile);
      auto h = sha256_file(bin);
      if (!h || *h != u.sha256) return refuse(ota::Reason::kBadHash);
      close_fd();
      const std::string tmp = go + ".new";
      {
        std::ofstream f(tmp, std::ios::trunc);
        f << sha256_hex(*h) << "\n";
        if (!f.flush()) { ::unlink(tmp.c_str()); return refuse(ota::Reason::kBadHash); }
      }
      if (::rename(tmp.c_str(), go.c_str()) != 0) { ::unlink(tmp.c_str()); return refuse(ota::Reason::kBadHash); }
      exit_ = true;
      return {ota::State::kTrial, ota::Reason::kNone, 40};
    }
    case ota::Cmd::kAbort:
      if (trial_running()) {
        ::unlink((env_.dir + "/proven").c_str());
        exit_ = true;
      } else {
        close_fd();
        ::unlink(bin.c_str());
      }
      break;
    default:
      break;
  }
  return status(now_ms);
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
    // After START the file is hash-verified and waiting for the loop: frozen.
    if (trial_running() || exit_ || has(env_.dir + "/go")) return nak(kFail);
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
      if (exit_) return nak(kFail);
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
