#include "lat_log.h"

#include <fstream>

namespace maburplay {

namespace {
constexpr uint64_t kRecheckUs = 30'000'000;

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
  const std::string p = dir + "/lat.log";
  std::FILE* f = std::fopen(p.c_str(), "a");
  if (!f) return;
  setvbuf(f, nullptr, _IOLBF, 0);
  std::fprintf(f, "# latlog 2\n");
  if (f_) std::fclose(f_);
  f_ = f;
  path_ = p;
  dir_ = dir;
}

void LatLog::write(uint64_t mono_us, const char* payload) {
  if (!f_ || (checked_ && mono_us - last_check_us_ >= kRecheckUs))
    reopen_(mono_us);
  if (!f_) return;
  std::fprintf(f_, "%llu %s\n", static_cast<unsigned long long>(mono_us),
               payload);
}

}  // namespace maburplay
