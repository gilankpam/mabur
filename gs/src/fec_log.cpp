#include "fec_log.h"

#include <algorithm>
#include <cstdio>

namespace maburgs {

FecLog::FecLog(LogWriter& w, const std::string& dir)
    : w_(w), s_(w.open(dir, "fec.log", "feclog 1")) {}

void FecLog::row(double t_ms, int sid, int mcs, double ov,
                 const mabur::LossEpisode& e) {
  if (s_ == LogWriter::kBadStream) return;
  char b[160];
  const int n = std::snprintf(
      b, sizeof(b), "%.0f %d %d %.2f %u %u %u %u %u %u %u %u", t_ms, sid, mcs,
      ov, static_cast<uint32_t>(e.first_seq), e.span, e.missing, e.recovered,
      e.abandoned, e.stale, e.repairs, e.window);
  if (n > 0) w_.line(s_, b, std::min(static_cast<size_t>(n), sizeof(b) - 1));
}

}  // namespace maburgs
