#include "au_log.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace maburgs {

AuLog::AuLog(LogWriter& w, const std::string& dir)
    : w_(w), s_(w.open(dir, "au.log", "# aulog 4")) {}

void AuLog::payload(const uint8_t* d, size_t n) {
  if (head_n_ >= kHead || n == 0) return;
  const size_t take = std::min(n, kHead - head_n_);
  std::memcpy(head_ + head_n_, d, take);
  head_n_ += take;
}

int AuLog::nal0_of(const uint8_t* h, size_t n) {
  if (n >= 5 && h[0] == 0 && h[1] == 0) {
    if (h[2] == 1) return (h[3] >> 1) & 0x3F;
    if (n >= 6 && h[2] == 0 && h[3] == 1) return (h[4] >> 1) & 0x3F;
  }
  return -1;
}

void AuLog::row(uint64_t t_us, const AuRecordMeta& m) {
  if (s_ == LogWriter::kBadStream) return;
  char buf[160];
  const int n = std::snprintf(
      buf, sizeof(buf), "%llu %u %u %llu %u 0x%02x %d %llu %llu %u %u %u",
      static_cast<unsigned long long>(t_us), m.pts_us,
      static_cast<unsigned>(m.sid),
      static_cast<unsigned long long>(m.frame_id64), m.len,
      static_cast<unsigned>(m.flags), nal0_of(head_, head_n_),
      static_cast<unsigned long long>(m.t_first_us),
      static_cast<unsigned long long>(m.t_complete_us),
      static_cast<unsigned>(m.enc_us), static_cast<unsigned>(m.drone_q_ms),
      static_cast<unsigned>(m.drone_air_ms));
  if (n > 0) w_.line(s_, buf, static_cast<size_t>(n));
}

}  // namespace maburgs
