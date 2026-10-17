#include "fec_log.h"

#include <algorithm>
#include <cstdio>

namespace maburgs {

FecLog::FecLog(LogWriter& w, const std::string& dir)
    : w_(w), s_(w.open(dir, "fec.log", "feclog 4")) {}

void FecLog::row(double t_ms, int sid, int mcs, int bw, double ov,
                 const mabur::LossEpisode& e) {
  if (s_ == LogWriter::kBadStream) return;
  char b[160];
  const int n = std::snprintf(
      b, sizeof(b), "%.0f %d %d %d %.2f %u %u %u %u %u %u %u %u %u", t_ms, sid,
      mcs, bw, ov, static_cast<uint32_t>(e.first_seq), e.span, e.missing, e.recovered,
      e.retx, e.abandoned, e.stale, e.repairs, e.window);
  if (n > 0) w_.line(s_, b, std::min(static_cast<size_t>(n), sizeof(b) - 1));
}

void FecLog::request(double t_ms, const char* reason, uint8_t epoch) {
  if (s_ == LogWriter::kBadStream) return;
  char b[64];
  const int n = std::snprintf(b, sizeof(b), "I %.0f %s %u", t_ms, reason,
                              static_cast<unsigned>(epoch));
  if (n > 0) w_.line(s_, b, std::min(static_cast<size_t>(n), sizeof(b) - 1));
}

void FecLog::heal(double t_ms, uint64_t heal_ms, uint64_t retries) {
  if (s_ == LogWriter::kBadStream) return;
  char b[64];
  const int n = std::snprintf(b, sizeof(b), "H %.0f %llu %llu", t_ms,
                              static_cast<unsigned long long>(heal_ms),
                              static_cast<unsigned long long>(retries));
  if (n > 0) w_.line(s_, b, std::min(static_cast<size_t>(n), sizeof(b) - 1));
}

}  // namespace maburgs
