#include "ctl_log.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <ctime>

#include <dirent.h>

namespace maburgs {

namespace {
// Mirrors StatsExporter's clamp_util() (Task 8): u3/u_pred are ratios that
// carry a 1e9 zero-guard sentinel when the divisor budget is 0 (see
// LadderController's u3_/u_pred computations in ladder_controller.cpp). A
// degenerate config must not put that raw magnitude in the log.
double clamp_util(double u) { return std::min(u, 1e3); }
}  // namespace

CtlLog::CtlLog(const std::string& dir, const std::string& header_info) {
  DIR* d = opendir(dir.c_str());
  if (!d) {
    std::fprintf(stderr, "ctl-log: opendir '%s' failed: %s\n", dir.c_str(),
                 std::strerror(errno));
    return;
  }
  // Per-boot index: one past the highest existing ctl-NNNN* file (0 on an
  // empty/fresh dir). Malformed/foreign entries are silently ignored --
  // this is a best-effort scan of a directory maburgs does not own
  // exclusively (the DVR SD card also holds video and stats files).
  int next_idx = 0;
  struct dirent* ent;
  while ((ent = readdir(d)) != nullptr) {
    int idx = -1;
    if (std::sscanf(ent->d_name, "ctl-%d", &idx) == 1 && idx >= 0)
      next_idx = std::max(next_idx, idx + 1);
  }
  closedir(d);

  // Date suffix is cosmetic only (RTC is wrong at boot) -- matches the
  // statsrec.py file-naming convention. localtime_r failure (e.g. a
  // corrupt/unset TZ) must not be fatal for a log whose header promises
  // never to crash over logging -- fall back to a fixed placeholder date;
  // the per-boot index in the filename is what actually matters.
  std::time_t now = std::time(nullptr);
  std::tm tmv{};
  char date[16] = {};
  if (::localtime_r(&now, &tmv)) {
    std::strftime(date, sizeof(date), "%Y%m%d", &tmv);
  } else {
    std::snprintf(date, sizeof(date), "00000000");
  }

  char fname[64];
  std::snprintf(fname, sizeof(fname), "ctl-%04d_%s.log", next_idx, date);
  path_ = dir + "/" + fname;

  f_ = std::fopen(path_.c_str(), "w");
  if (!f_) {
    std::fprintf(stderr, "ctl-log: fopen '%s' failed: %s\n", path_.c_str(),
                 std::strerror(errno));
    return;
  }
  // Line-buffered: a power cut loses at most the current line.
  std::setvbuf(f_, nullptr, _IOLBF, 0);
  std::fprintf(f_, "ctllog 1 %s\n", header_info.c_str());
}

CtlLog::~CtlLog() {
  if (f_) std::fclose(f_);
}

void CtlLog::sample(double t_ms, int rung, double u, double snr_db,
                     double resid, double u3, double resid3) {
  if (!f_) return;
  std::fprintf(f_, "S %.0f %d %.4f %.1f %.4f %.4f %.4f\n", t_ms, rung, u,
               snr_db, resid, clamp_util(u3), resid3);
}

void CtlLog::event(double t_ms, int from, int to, const char* reason,
                    double u, double snr_db) {
  if (!f_) return;
  // u is u3 (not the s1 util) whenever reason is one of the s3_* reasons --
  // clamp unconditionally since that's the only case the raw value can run
  // away, and clamping the ordinary s1 util (already bounded) is a no-op.
  std::fprintf(f_, "E %.0f %d %d %s %.4f %.1f\n", t_ms, from, to, reason,
               clamp_util(u), snr_db);
}

void CtlLog::probe(double t_ms, int rung, const char* outcome, double snr_db,
                    double u_pred, int dur_ms) {
  if (!f_) return;
  std::fprintf(f_, "P %.0f %d %s %.1f %.4f %d\n", t_ms, rung, outcome,
               snr_db, clamp_util(u_pred), dur_ms);
}

void CtlLog::penalty(double t_ms, int rung, int k, double until_ms) {
  if (!f_) return;
  std::fprintf(f_, "N %.0f %d %d %.0f\n", t_ms, rung, k, until_ms);
}

}  // namespace maburgs
