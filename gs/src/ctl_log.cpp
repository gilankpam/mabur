#include "ctl_log.h"

#include <algorithm>

namespace maburgs {

namespace {
// Mirrors StatsExporter's clamp_util() (Task 8): u3/u_pred are ratios that
// carry a 1e9 zero-guard sentinel when the divisor budget is 0 (see
// LadderController's u3_/u_pred computations in ladder_controller.cpp). A
// degenerate config must not put that raw magnitude in the log.
double clamp_util(double u) { return std::min(u, 1e3); }
}  // namespace

CtlLog::CtlLog(LogWriter& w, const std::string& dir,
               const std::string& header_info)
    : w_(w), s_(w.open(dir, "ctl.log", "ctllog 11 " + header_info)) {}

void CtlLog::sample(double t_ms, int rung, double u, double snr_db,
                     double resid, double u3, double resid3, double evm_db,
                     double resid_cur, double drssi, double dsnr,
                     double rssi_dbm, int probe_rung, double probe_u,
                     uint64_t probe_n) {
  if (s_ == LogWriter::kBadStream) return;
  char b[256];
  const int n = std::snprintf(
      b, sizeof(b),
      "S %.0f %d %.4f %.1f %.4f %.4f %.4f %.1f %.4f %.1f %.1f %.1f %d %.4f %llu",
      t_ms, rung, u, snr_db, resid, clamp_util(u3), resid3, evm_db, resid_cur,
      drssi, dsnr, rssi_dbm, probe_rung, clamp_util(probe_u),
      static_cast<unsigned long long>(probe_n));
  if (n > 0) w_.line(s_, b, static_cast<size_t>(n));
}

void CtlLog::event(double t_ms, int from, int to, const char* reason,
                    double u, double snr_db, double evm_db) {
  if (s_ == LogWriter::kBadStream) return;
  // u is u3 (not the ordinary BASE/sid0 util) whenever reason is one of the
  // s3_* reasons -- clamp unconditionally since that's the only case the raw
  // value can run away, and clamping the ordinary util (already bounded) is
  // a no-op.
  char b[256];
  const int n = std::snprintf(b, sizeof(b), "E %.0f %d %d %s %.4f %.1f %.1f",
                               t_ms, from, to, reason, clamp_util(u), snr_db,
                               evm_db);
  if (n > 0) w_.line(s_, b, static_cast<size_t>(n));
}

void CtlLog::probe(double t_ms, int rung, const char* state, double snr_db,
                    double u, int prev_dur_ms, double evm_db) {
  if (s_ == LogWriter::kBadStream) return;
  char b[256];
  const int n =
      std::snprintf(b, sizeof(b), "P %.0f %d %s %.1f %.4f %d %.1f", t_ms,
                    rung, state, snr_db, clamp_util(u), prev_dur_ms, evm_db);
  if (n > 0) w_.line(s_, b, static_cast<size_t>(n));
}

void CtlLog::penalty(double t_ms, int rung, int k, double until_ms) {
  if (s_ == LogWriter::kBadStream) return;
  char b[256];
  const int n =
      std::snprintf(b, sizeof(b), "N %.0f %d %d %.0f", t_ms, rung, k, until_ms);
  if (n > 0) w_.line(s_, b, static_cast<size_t>(n));
}

void CtlLog::rung(double t_ms, int rung, double u, double resid, double u3,
                   double resid3, double evm_db, double evm_sd_db, uint64_t n,
                   double age_s, double probe_u, uint64_t probe_n) {
  if (s_ == LogWriter::kBadStream) return;
  char b[256];
  const int len = std::snprintf(
      b, sizeof(b), "R %.0f %d %.4f %.4f %.4f %.4f %.1f %.2f %llu %.1f %.4f %llu",
      t_ms, rung, clamp_util(u), resid, clamp_util(u3), resid3, evm_db,
      evm_sd_db, static_cast<unsigned long long>(n), age_s,
      clamp_util(probe_u), static_cast<unsigned long long>(probe_n));
  if (len > 0) w_.line(s_, b, static_cast<size_t>(len));
}

}  // namespace maburgs
