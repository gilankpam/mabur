#pragma once

#include <cstdint>
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
//   ctllog 4 <header_info>                                  # once, first line
//   S <t_ms> <rung> <u> <snr_db> <resid> <u3> <resid3> <evm_db> <resid_cur>
//     <drssi> <dsnr>                                        # 1 Hz dwell sample
//   E <t_ms> <from> <to> <reason> <u> <snr_db> <evm_db>      # rung transition
//   P <t_ms> <rung> <pass|fail|abort> <snr_db> <u_pred> <dur_ms> <evm_db>
//   N <t_ms> <rung> <k> <until_ms>                           # penalty booked
//   R <t_ms> <rung> <u> <resid> <u3> <resid3> <evm> <evm_sd> <n> <age_s> <probe_u> <probe_n>
//                                                            # per-rung EWMA store snapshot
//
// evm_db is the RF EVM label in dB (s1+s3 pooled since ctllog 4, s1-class
// before it -- see the ctllog 4 note below), nan when unsampled --
// label-only, like snr_db (see LinkHealth::rf_evm_db).
//
// resid stays the TOTAL residual (abandoned/expected over the whole decode
// window); resid_cur is its attributed sibling -- the same ratio scored only
// against symbols the ladder could still have acted on (excludes stale
// abandons attributable to a transition already past), added 2026-08-14.
//
// drssi/dsnr are the fade-trigger deltas (baseline-minus-fast, raw dB;
// LadderController::fade_drssi()/fade_dsnr()), added 2026-08-14 (ctllog 3).
// Each reads nan until its underlying signal has ever been sampled -- nan is
// a normal steady-state value on a GS whose RF labels are stale, not a bug.
//
// ctllog 4 (2026-08-15): line formats are UNCHANGED from v3, but snr_db,
// evm_db, drssi and dsnr all change MEANING -- the label source is the
// s1+s3 pooled track rather than the s1 class, and the fade deltas are no
// longer periodically zeroed by the card-hop re-baseline. The bump exists
// so a recording identifies its own semantics; this repo has twice been
// bitten by recordings that could not (the 2026-08-04 SNR scale break, the
// 2026-08-14 EVM freshness gate).
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
//  - R lines (spec 2026-08-13) are the per-rung EWMA store: one line per
//    rung with any data, every link.rung_stats.rung_log_period_s AND a
//    full snapshot right after every E line. u/u3/probe_u get the same
//    <= 1e3 clamp; evm/evm_sd are nan until the rung has an EVM sample;
//    age_s is -1 when the rung was never parked-sampled.
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
              double u3, double resid3, double evm_db, double resid_cur,
              double drssi, double dsnr);
  void event(double t_ms, int from, int to, const char* reason, double u,
             double snr_db, double evm_db);
  void probe(double t_ms, int rung, const char* outcome, double snr_db,
             double u_pred, int dur_ms, double evm_db);
  void penalty(double t_ms, int rung, int k, double until_ms);
  void rung(double t_ms, int rung, double u, double resid, double u3,
            double resid3, double evm_db, double evm_sd_db, uint64_t n,
            double age_s, double probe_u, uint64_t probe_n);

 private:
  std::FILE* f_ = nullptr;
  std::string path_;
};

}  // namespace maburgs
