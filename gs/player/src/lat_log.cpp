#include "lat_log.h"

#include <fstream>

namespace maburplay {

namespace {
constexpr uint64_t kRecheckUs = 5'000'000;  // a session rotation (drone restart) lands here within 5 s

std::string read_marker(const char* path) {
  std::ifstream f(path);
  if (!f.good()) return {};
  std::string s;
  std::getline(f, s);
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' '))
    s.pop_back();
  return s;
}
}  // namespace

LatLog::~LatLog() {
  if (f_) std::fclose(f_);
}

void LatLog::reopen_(uint64_t mono_us) {
  last_check_us_ = mono_us;
  checked_ = true;
  const std::string dir = read_marker(marker_);
  if (dir == dir_ && f_) return;  // same session, already open
  if (dir.empty()) {
    if (f_) {
      std::fclose(f_);
      f_ = nullptr;
      path_.clear();
      dir_.clear();
    }
    return;
  }
  const std::string p = dir + "/" + file_name_;
  std::FILE* f = std::fopen(p.c_str(), "a");
  // A new session that can't be opened leaves any still-open previous
  // session's handle alone (deliberate: keep logging into the old
  // directory rather than go dark over a bad new target).
  if (!f) return;
  // The old session closes (flushing whatever it holds) before the new one
  // is set up: a batched instance hands the new stream the same buffer.
  if (f_) std::fclose(f_);
  if (buf_)
    setvbuf(f, buf_.get(), _IOFBF, kBatchBytes);
  else
    setvbuf(f, nullptr, _IOLBF, 0);
  std::fprintf(f, "%s\n", header_);
  f_ = f;
  path_ = p;
  dir_ = dir;
}

void LatLog::flush() {
  if (f_) std::fflush(f_);
}

void LatLog::write(uint64_t mono_us, const char* payload) {
  // Gate on elapsed time alone -- not on whether f_ is currently open --
  // so a missing or unusable session backs off for kRecheckUs instead of
  // re-attempting read_marker()/fopen() on every single write(). A stamp
  // behind the last check (frame.log's rows carry their flip time) has
  // not elapsed anything.
  if (!checked_ ||
      (mono_us >= last_check_us_ && mono_us - last_check_us_ >= kRecheckUs))
    reopen_(mono_us);
  if (!f_) return;
  std::fprintf(f_, "%llu %s\n", static_cast<unsigned long long>(mono_us),
               payload);
}

}  // namespace maburplay
