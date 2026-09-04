#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

namespace maburgs {

// Per-body probe log (spec docs/superpowers/specs/2026-09-04-probe-stream
// section 6.2). One row per body ProbeTrack finalizes on the probe stream --
// far higher rate than CtlLog's dwell-period S lines, so it is its own file
// rather than another CtlLog record type. A per-boot indexed, line-buffered
// text file: `probe-NNNN_<date>.log` in `dir` (NNNN passed in by the caller
// -- unlike CtlLog, ProbeLog does not scan the directory itself; callers
// share a single index source, typically CtlLog::index(), so a probe-NNNN
// and ctl-NNNN pair from the same boot line up). `<date>` is `%Y%m%d` from
// localtime, cosmetic only -- the GS RTC is wrong at boot.
//
// Record format is LOCKED (tests/test_probe_log.cpp depends on the exact
// byte layout):
//
//   probelog 1 bpb=<bpb>                                     # once, first line
//   <t_ms> <seq> <mcs> <enh_fid> <blocks_ok> <card_mask> <snr0> <snr1>
//     <evm0> <evm1>                                          # one row per finalized body
//
// bpb (blocks-per-body) is the probe stream's block count, echoed in the
// header so a row's blocks_ok is self-describing without cross-referencing
// the live config. enh_fid is the join key back to the base-stream's own
// per-au log (`au-NNNN.log`, see gs/src/au_ring.cpp / the AU logger): the
// probe stream carries no frame id of its own, so a row correlates to
// video by the enhancement-layer frame id it rode alongside. card_mask is
// the bitmask of GS cards that contributed at least one delivered fragment
// to this body (bit i = card i), NOT which cards were merely up.
// snr0/snr1/evm0/evm1 are the per-card RF labels sampled for this body;
// nan (via %.1f -> "nan") when that card had no reading this row -- an
// unplugged/dead second card is a normal steady-state condition, not a bug.
//
// Every failure mode (dir missing/unwritable, fopen failure, ...) is
// non-fatal: ok() reads false, the constructor prints the reason to
// stderr, and row() becomes a silent no-op. maburgs must never exit or
// crash over this log.
class ProbeLog {
 public:
  ProbeLog(const std::string& dir, int index, int bpb);
  ~ProbeLog();

  ProbeLog(const ProbeLog&) = delete;
  ProbeLog& operator=(const ProbeLog&) = delete;

  bool ok() const { return f_ != nullptr; }
  const std::string& path() const { return path_; }

  void row(double t_ms, uint32_t seq, int mcs, uint16_t enh_fid,
           int blocks_ok, uint32_t card_mask, double snr0, double snr1,
           double evm0, double evm1);

 private:
  std::FILE* f_ = nullptr;
  std::string path_;
};

}  // namespace maburgs
