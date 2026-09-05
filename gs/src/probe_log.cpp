#include "probe_log.h"

#include <cerrno>
#include <cstring>
#include <ctime>

namespace maburgs {

ProbeLog::ProbeLog(const std::string& dir, int index, int bpb) {
  // Date suffix is cosmetic only (RTC is wrong at boot) -- matches CtlLog's
  // convention. localtime_r failure (e.g. a corrupt/unset TZ) must not be
  // fatal for a log whose header promises never to crash over logging --
  // fall back to a fixed placeholder date; the caller-supplied index in the
  // filename is what actually matters.
  std::time_t now = std::time(nullptr);
  std::tm tmv{};
  char date[16] = {};
  if (::localtime_r(&now, &tmv)) {
    std::strftime(date, sizeof(date), "%Y%m%d", &tmv);
  } else {
    std::snprintf(date, sizeof(date), "00000000");
  }

  char fname[64];
  std::snprintf(fname, sizeof(fname), "probe-%04d_%s.log", index, date);
  path_ = dir + "/" + fname;

  f_ = std::fopen(path_.c_str(), "w");
  if (!f_) {
    std::fprintf(stderr, "probe-log: fopen '%s' failed: %s\n", path_.c_str(),
                 std::strerror(errno));
    return;
  }
  // Line-buffered: a power cut loses at most the current row.
  std::setvbuf(f_, nullptr, _IOLBF, 0);
  std::fprintf(f_, "probelog 2 bpb=%d\n", bpb);
}

ProbeLog::~ProbeLog() {
  if (f_) std::fclose(f_);
}

void ProbeLog::row(double t_ms, uint32_t seq, int mcs, uint16_t enh_fid,
                    int blocks_ok, uint32_t card_mask, double snr0,
                    double snr1, double evm0, double evm1, double first_ms) {
  if (!f_) return;
  std::fprintf(f_, "%.0f %u %d %u %d %u %.1f %.1f %.1f %.1f %.3f\n", t_ms,
               seq, mcs, enh_fid, blocks_ok, card_mask, snr0, snr1, evm0,
               evm1, first_ms);
}

}  // namespace maburgs
