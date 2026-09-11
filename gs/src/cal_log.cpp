#include "cal_log.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <string>

#include <unistd.h>

#include "cal_analysis.h"

namespace maburgs {
namespace {
constexpr char kCalLogMarker[] = "callog 1";
}  // namespace

bool cal_log_header_due(const std::string& dir) {
  return ::access((dir + "/cal.log").c_str(), F_OK) != 0;
}

bool cal_log_prepare(const std::string& dir) {
  const std::string path = dir + "/cal.log";
  std::string first;
  {
    std::ifstream f(path);
    if (!f.good()) return true;  // no file at all: header is due
    std::getline(f, first);
  }
  if (first == kCalLogMarker) return false;
  // Anything else -- an older version, or the headerless file the
  // rejoined-session bug used to produce -- is retired under a name that
  // says what it holds, and the caller starts a fresh file.
  std::string tag = "unmarked";
  if (first.rfind("callog ", 0) == 0) {
    tag = "callog" + first.substr(7);
    tag.erase(std::remove_if(tag.begin(), tag.end(),
                             [](unsigned char c) { return !std::isalnum(c); }),
              tag.end());
  }
  std::rename(path.c_str(), (path + "." + tag).c_str());
  return true;
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
  w_.line(s_, kCalLogMarker, sizeof(kCalLogMarker) - 1);
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
