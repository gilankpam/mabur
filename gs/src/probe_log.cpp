#include "probe_log.h"

namespace maburgs {

ProbeLog::ProbeLog(LogWriter& w, const std::string& dir, int bpb)
    : w_(w), s_(w.open(dir, "probe.log",
                       "probelog 2 bpb=" + std::to_string(bpb))) {}

void ProbeLog::row(double t_ms, uint32_t seq, int mcs, uint16_t enh_fid,
                    int blocks_ok, uint32_t card_mask, double snr0,
                    double snr1, double evm0, double evm1, double first_ms) {
  if (s_ == LogWriter::kBadStream) return;
  char b[200];
  const int n = std::snprintf(
      b, sizeof(b), "%.0f %u %d %u %d %u %.1f %.1f %.1f %.1f %.3f", t_ms, seq,
      mcs, enh_fid, blocks_ok, card_mask, snr0, snr1, evm0, evm1, first_ms);
  if (n > 0) w_.line(s_, b, static_cast<size_t>(n));
}

}  // namespace maburgs
