#include "lat_log.h"

#include <dirent.h>

#include <algorithm>
#include <cstdio>

namespace maburplay {

namespace {
constexpr uint64_t kRetryUs = 30'000'000;
}  // namespace

LatLog::~LatLog() {
  if (f_) std::fclose(f_);
}

void LatLog::try_open(uint64_t mono_us, uint64_t wall_us) {
  last_attempt_us_ = mono_us;
  attempted_ = true;
  DIR* d = opendir(dir_.c_str());
  if (!d) return;
  int next_idx = 0;
  struct dirent* ent;
  while ((ent = readdir(d)) != nullptr) {
    int idx = -1;
    if (std::sscanf(ent->d_name, "lat-%d", &idx) == 1 && idx >= 0)
      next_idx = std::max(next_idx, idx + 1);
  }
  closedir(d);
  char fname[32];
  std::snprintf(fname, sizeof(fname), "lat-%04d.log", next_idx);
  const std::string p = dir_ + "/" + fname;
  std::FILE* f = std::fopen(p.c_str(), "w");
  if (!f) return;
  setvbuf(f, nullptr, _IOLBF, 0);
  std::fprintf(f, "# latlog 1\n# sync %llu %llu\n",
               static_cast<unsigned long long>(mono_us),
               static_cast<unsigned long long>(wall_us));
  f_ = f;
  path_ = p;
}

void LatLog::write(uint64_t mono_us, uint64_t wall_us, const char* payload) {
  if (dir_.empty()) return;
  if (!f_) {
    if (attempted_ && mono_us - last_attempt_us_ < kRetryUs) return;
    try_open(mono_us, wall_us);
    if (!f_) return;
  }
  std::fprintf(f_, "%llu %s\n", static_cast<unsigned long long>(mono_us),
               payload);
}

}  // namespace maburplay
