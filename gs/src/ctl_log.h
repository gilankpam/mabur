#pragma once

#include <cstdio>
#include <string>

namespace maburgs {

// maburgs' own compact adaptive-link log (spec
// docs/superpowers/specs/2026-08-05-s3-probe-promote-design.md section 5).
// A per-boot indexed, line-buffered text file: `ctl-NNNN_<date>.log` in
// `dir` (NNNN = 1 + the highest existing ctl-* index, 0 on an empty dir;
// `<date>` is `%Y%m%d` from localtime, cosmetic only -- the GS RTC is wrong
// at boot). The learning dataset must not depend on any sideport consumer
// being alive, so this is maburgs writing its own record independent of
// StatsExporter/maburtop/flightreport's live feed.
//
// Record formats are LOCKED (a Python parser -- flightreport.py -- and
// tests/test_ctl_log.cpp depend on the exact byte layout):
//
//   ctllog 1 <header_info>                                  # once, first line
//   S <t_ms> <rung> <u> <snr_db> <resid> <u3> <resid3> <evm_db>  # 1 Hz dwell sample
//   E <t_ms> <from> <to> <reason> <u> <snr_db> <evm_db>      # rung transition
//   P <t_ms> <rung> <pass|fail|abort> <snr_db> <u_pred> <dur_ms> <evm_db>
//   N <t_ms> <rung> <k> <until_ms>                           # penalty booked
//
// evm_db is the s1 EVM label in dB, nan when unsampled -- label-only, like
// snr_db (see LinkHealth::s1_evm_db).
//
// Two encoding notes callers must know:
//  - S's u3 reads 0 while a probe is active (steady-state s3 utilization is
//    meaningless then -- s3 is deliberately running the probe candidate's
//    MCS, not the current rung's -- see LadderController::util3()).
//  - E's u is u3, not the s1 util, whenever reason is one of the s3_*
//    reasons (S3Residual/S3Util); the reason string is what disambiguates
//    which quantity `u` holds.
// u3 (S) and u_pred (P) both come from LadderController ratios that carry a
// 1e9 zero-guard sentinel when the divisor budget is 0 (see
// ladder_controller.cpp); CtlLog clamps both to <= 1e3 at the write site
// (mirrors the Task 8 sideport's clamp_util()) so a degenerate config never
// puts a nonsense magnitude in the log. The clamp also applies to E's u for
// s3_* reasons, since that field is u3 under the hood.
//
// Every failure mode (dir missing/unwritable, index scan failure, fopen
// failure, ...) is non-fatal: ok() reads false, the constructor prints the
// reason to stderr, and every record method becomes a silent no-op.
// maburgs must never exit or crash over this log.
class CtlLog {
 public:
  CtlLog(const std::string& dir, const std::string& header_info);
  ~CtlLog();

  CtlLog(const CtlLog&) = delete;
  CtlLog& operator=(const CtlLog&) = delete;

  bool ok() const { return f_ != nullptr; }
  const std::string& path() const { return path_; }

  void sample(double t_ms, int rung, double u, double snr_db, double resid,
              double u3, double resid3, double evm_db);
  void event(double t_ms, int from, int to, const char* reason, double u,
             double snr_db, double evm_db);
  void probe(double t_ms, int rung, const char* outcome, double snr_db,
             double u_pred, int dur_ms, double evm_db);
  void penalty(double t_ms, int rung, int k, double until_ms);

 private:
  std::FILE* f_ = nullptr;
  std::string path_;
};

}  // namespace maburgs
