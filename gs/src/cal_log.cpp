#include "cal_log.h"

#include <algorithm>

#include <unistd.h>

#include "cal_analysis.h"

namespace maburgs {

bool cal_log_header_due(const std::string& dir) {
  return ::access((dir + "/cal.log").c_str(), F_OK) != 0;
}

// LogWriter::open()'s own header parameter is unused here: header() and
// run() are distinct calls the caller makes explicitly (header() at most
// once per file, run() once per calibration run -- see cal_log.h), so both
// go through line() like every other record rather than through open()'s
// re-queue-on-reopen contract (which exists for the session-writer case,
// not this private one).
CalLog::CalLog(const std::string& dir)
    : s_(w_.open(dir, "cal.log", "")) {}

void CalLog::header() {
  if (s_ == LogWriter::kBadStream) return;
  static constexpr char kMarker[] = "callog 1";
  w_.line(s_, kMarker, sizeof(kMarker) - 1);
}

void CalLog::run(uint32_t nonce, int base_ref_idx, double margin_db) {
  if (s_ == LogWriter::kBadStream) return;
  char b[64];
  const int n = std::snprintf(b, sizeof(b), "R %u %d %.2f", nonce,
                              base_ref_idx, margin_db);
  if (n > 0)
    w_.line(s_, b, std::min(static_cast<size_t>(n), sizeof(b) - 1));
}

void CalLog::cell(uint8_t phase, uint8_t rate, uint8_t idx, const CalCell& c) {
  if (s_ == LogWriter::kBadStream) return;
  // -999 (kRssiNone) whenever a card had no reading, never the raw
  // rssi_dbm slot -- a caller that forgets to gate on have_rssi must not
  // leak a stale or zero-initialized value into the record.
  const int rssi0 = c.have_rssi[0] ? c.rssi_dbm[0] : kRssiNone;
  const int rssi1 = c.have_rssi[1] ? c.rssi_dbm[1] : kRssiNone;
  char b[160];
  const int n = std::snprintf(
      b, sizeof(b), "C %d %d %d %d %d %d %d %d %d", phase, rate, idx,
      c.expected, c.received[0], c.received[1], c.corrupt, rssi0, rssi1);
  if (n > 0)
    w_.line(s_, b, std::min(static_cast<size_t>(n), sizeof(b) - 1));
}

void CalLog::wall(uint8_t rate, const RateWall& w) {
  if (s_ == LogWriter::kBadStream) return;
  char b[128];
  const int n = std::snprintf(b, sizeof(b), "W %d %d %d %d %u", rate, w.wall,
                              w.floor_idx, w.best_card, w.flags);
  if (n > 0)
    w_.line(s_, b, std::min(static_cast<size_t>(n), sizeof(b) - 1));
}

void CalLog::verify(uint8_t rate, uint8_t idx, int pct) {
  if (s_ == LogWriter::kBadStream) return;
  char b[64];
  const int n = std::snprintf(b, sizeof(b), "V %d %d %d", rate, idx, pct);
  if (n > 0)
    w_.line(s_, b, std::min(static_cast<size_t>(n), sizeof(b) - 1));
}

void CalLog::flush() {
  if (s_ == LogWriter::kBadStream) return;
  w_.flush_now();
}

}  // namespace maburgs
