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
//   ctllog 11 <header_info>                                 # once, first line
//   S <t_ms> <rung> <u> <snr_db> <resid> <u3> <resid3> <evm_db> <resid_cur>
//     <drssi> <dsnr> <rssi_dbm> <probe_rung> <probe_u> <probe_n>
//                                              # dwell sample, link.ctl_log_period_ms
//   E <t_ms> <from> <to> <reason> <u> <snr_db> <evm_db>      # rung transition
//   P <t_ms> <rung> <clean|lossy|noinfo> <snr_db> <u> <dur_ms> <evm_db>
//                                                            # probe gate EDGE
//   N <t_ms> <rung> <k> <until_ms>                           # penalty booked
//   R <t_ms> <rung> <u> <resid> <u3> <resid3> <evm> <evm_sd> <n> <age_s> <probe_u> <probe_n>
//                                                            # per-rung EWMA store snapshot
//
// evm_db is the RF EVM label in dB (base+enh pooled since ctllog 4 -- s1-
// class before it, re-scoped from s1+s3 to sid0+sid1 by ctllog 7 -- see the
// ctllog 4/7 notes below), nan when unsampled -- label-only, like snr_db
// (see LinkHealth::rf_evm_db).
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
// ctllog 11 (2026-09-05, arrival tracker): no column change. The S line's
// <u> and <u3> (and the E line's u for util/probation reasons) are now
// ARRIVAL-booked pre-FEC loss (SwDecoder::arr_*, current-only side) --
// before this they were completion-booked (delivered+recovered+abandoned)
// and lagged the air by a repair window, reading a transition's re-key as
// 50-100 % loss on the first post-blank tick. Do not pool u across the
// version boundary (docs/data-provenance.md).
//
// ctllog 10 (2026-09-04, probe-stream): the S line gains three trailing
// fields -- probe_rung, probe_u, probe_n -- describing the CONTINUOUS probe
// gate's commanded state at the sample instant (LadderController::
// probe_gate()), not a discrete probe attempt: probe_rung is -1 and
// probe_u/probe_n are nan/0 whenever no probe is commanded this tick (the
// ordinary steady-state case). probe_u is the probe candidate's own utility
// sample (u_pred's continuous successor), probe_n the sample count backing
// it.
//
// The P line is REPURPOSED, not merely reformatted: through v9 it recorded
// one row per discrete probe ATTEMPT (pass/fail/abort, gs/src/probe_track.h
// era). From v10 it is a GATE EDGE -- one row every time
// LadderController::probe_gate()'s state changes (LadderController::
// last_probe_edge()) -- and the vocabulary in the third field is
// clean|lossy|noinfo, not pass|fail|abort. `u` is the util sample that
// caused the edge (the same continuous quantity as S's probe_u, clamped);
// `dur_ms` is the duration of the PREVIOUS state (how long the gate held
// before flipping), not a probe's own runtime; snr_db/evm_db are the pooled
// RF labels for the rung the gate is parked on at the edge. A parser
// counting v9 "pass" rows against v10 P lines is counting a different
// thing -- discrete attempts vs. continuous-state transitions -- and must
// not pool them.
//
// The E line's reason vocabulary gains `promote_probed` (a promote gated by
// a clean probe read, LadderController's probe-before-promote path) --
// existing reasons are unchanged.
//
// R's probe_u/probe_n are UNCHANGED in line position but change MEANING:
// through v9 they were the last completed discrete probe's stats; from v10
// they are the same continuous EWMA the S/P lines above describe (~20
// samples/s while a probe candidate is parked), so an R row's probe_u now
// updates far more often and never freezes between discrete attempts.
//
// ctllog 9 (2026-09-02, residual-phantom-demotes): line formats are
// UNCHANGED from v8, but every RESIDUAL quantity changes MEANING. Through
// v8, resid/resid_cur (S) and resid (R) were computed from the PACKET-level
// delivery window -- FRAG-seq continuity of completed packets,
// 1 - delivered/expected. That measure inferred loss from seq gaps and could
// not tell a lost unit from one that had not completed yet, so a late
// sliding-window FEC repair (completing an older unit after a newer one)
// fabricated loss: on the 2026-09-02 bench, 200 spurious `residual` demotes
// in 57 minutes while the FEC decoder's own abandonment counter sat frozen
// at 139 across 6.2M packets. From v9 all of them come from that abandonment
// counter instead (syms_abandoned / syms_abandoned_stale, order-independent
// seq arithmetic in SwDecoder::advance) via gs/src/ladder_residual.cpp.
//
// What this means when comparing recordings across the v8/v9 line:
//  - A v8 resid > 0 does NOT imply video was lost; a v9 resid > 0 does.
//    Do not pool per-rung resid across the boundary, and treat v8 rung
//    tables built from resid (flightreport's inversion detector) as
//    contaminated by reorder rate, which scales with packet rate and so
//    with rung.
//  - v9 resid is BOTH LAYERS pooled (base+enh) exactly as v8 was, but the
//    LADDER's demote input is now BASE ONLY -- v8 pooled enh into it, a
//    leftover from the 4-stream era. The S line still logs the pooled view,
//    so an E line with reason=residual can now fire on a rung whose logged
//    resid is diluted by clean enh traffic.
//  - v9 loss is booked ~80 ms later than v8's: a symbol counts abandoned
//    only once the sliding-window horizon passes it (seq_horizon 512 at
//    ~6.4k sym/s), where the packet measure fired on the next forward gap.
//  - The `suppressed` sideport counter (link.attrib.suppressed) is GONE --
//    it counted windows where the packet-level total and attributed views
//    disagreed, which is not a question v9 can ask.
// Attribution itself is UNCHANGED in spirit: resid_cur is still current-rung
// only, now via syms_abandoned_stale rather than the packet stale buckets.
//
// ctllog 8 (2026-08-30, same-rate-fixed-pairs): a HEADER-ONLY change -- the
// S/E/P/N/R line formats carry no overhead field at all (rung overhead only
// ever appeared in the header's `ladder=` token), so nothing in the per-tick
// records moved. The header's `ladder=<mcs>/<ov>,...` token becomes
// `ladder=<mcs>/<ovb>*100:<ove>*100,...` -- each rung's overhead splits into
// a base/enh pair (GS::config's per-rung overhead_base/overhead_enh, see
// gs/src/config.h) instead of the single scalar v7 and earlier wrote.
// v1-v7 logs keep their single value per rung; a parser reading an old log
// treats it as both base and enh (they were the same rung table entry).
//
// ctllog 7 (2026-08-30, airtime-balance-uep): line formats are UNCHANGED
// from v6, but the u3/resid3/evm_db/drssi/dsnr MEANING shifts again: the
// link collapsed from 4 UEP streams to 2 (BASE sid 0, ENH sid 1), and every
// quantity that used to read "stream 3" (the probe/enhancement layer) now
// reads sid 1, while the ordinary s1 quantities (u, resid) that used to read
// "stream 1" now read sid 0 (BASE, the mirror of the drone's mcs-1 rule).
// The pooled RF label source moved with it: base+enh (sid0+sid1), not
// s1+s3. budget()/util3() are also now the LITERAL FEC command overhead
// (overhead / (1 + overhead)) rather than a per-layer uep_layer_overhead
// scaling -- the two were numerically identical since the 2026-08-29 UEP
// flatten, so no logged value actually moved, only what it is computed from.
//
// ctllog 6 (2026-08-15): the S line's <rung> is now the rung the sample was
// MEASURED on (LadderController::measured_rung()), not the live rung. They
// differ on exactly the rows that matter: a demote steps the live rung down
// before the row is written, so v5 and earlier filed post-FEC loss against
// the rung the link demoted TO, hiding the rung that caused it. Measured on
// flights 2026-08-15: 15/16 and 13/13 loss samples landed within 200 ms of a
// demote, which made per-rung tables name the wrong culprit every time. Rows
// with no transition are unchanged. The R lines were always correct
// (RungStore observes before the decision blocks).
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
//  - S's u3 reads 0 while a probe is active (steady-state ENH/sid1
//    utilization is meaningless then -- sid 1 is deliberately running the
//    probe candidate's MCS, not the current rung's -- see
//    LadderController::util3()).
//  - E's u is u3, not the ordinary (BASE/sid0) util, whenever reason is one
//    of the s3_* reasons (S3Residual/S3Util); the reason string is what
//    disambiguates which quantity `u` holds.
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
  int index() const { return index_; }  // the NNNN this log opened with

  void sample(double t_ms, int rung, double u, double snr_db, double resid,
              double u3, double resid3, double evm_db, double resid_cur,
              double drssi, double dsnr, double rssi_dbm, int probe_rung,
              double probe_u, uint64_t probe_n);
  void event(double t_ms, int from, int to, const char* reason, double u,
             double snr_db, double evm_db);
  // Probe gate EDGE (ctllog 10): state is clean|lossy|noinfo, u is the
  // sample that caused the edge, prev_dur_ms the duration of the state the
  // gate just left.
  void probe(double t_ms, int rung, const char* state, double snr_db,
             double u, int prev_dur_ms, double evm_db);
  void penalty(double t_ms, int rung, int k, double until_ms);
  void rung(double t_ms, int rung, double u, double resid, double u3,
            double resid3, double evm_db, double evm_sd_db, uint64_t n,
            double age_s, double probe_u, uint64_t probe_n);

 private:
  std::FILE* f_ = nullptr;
  std::string path_;
  int index_ = 0;
};

}  // namespace maburgs
