#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "log_writer.h"

namespace maburgs {

// Per-body probe log (spec docs/superpowers/specs/2026-09-04-probe-stream
// section 6.2). One row per body ProbeTrack finalizes on the probe stream --
// far higher rate than CtlLog's dwell-period S lines, so it is its own file
// rather than another CtlLog record type. Writes `probe.log` inside the
// session directory (DebugSession::dir()) via the shared LogWriter -- the
// 2026-09-06 consolidation collapsed the per-boot indexed
// `probe-NNNN_<date>.log` naming into one session directory per boot, so
// probe.log and ctl.log from the same boot always pair up by directory
// rather than by a shared NNNN.
//
// Record format is LOCKED (tests/test_probe_log.cpp depends on the exact
// byte layout):
//
//   probelog 2 bpb=<bpb>                                     # once, first line
//   <t_ms> <seq> <mcs> <enh_fid> <blocks_ok> <card_mask> <snr0> <snr1>
//     <evm0> <evm1> <first_ms>                               # one row per finalized body
//
// probelog 1 (2026-09-04, one day) had no first_ms column; everything
// else is identical, and flightreport.py reads both.
//
// bpb (blocks-per-body) is the probe stream's block count, echoed in the
// header so a row's blocks_ok is self-describing without cross-referencing
// the live config. enh_fid is the join key back to the base-stream's own
// per-au log (`au.log`, see gs/src/au_log.h): the
// probe stream carries no frame id of its own, so a row correlates to
// video by the enhancement-layer frame id it rode alongside. card_mask is
// the bitmask of GS cards that contributed at least one delivered fragment
// to this body (bit i = card i), NOT which cards were merely up.
// snr0/snr1/evm0/evm1 are the per-card RF labels sampled for this body;
// nan (via %.1f -> "nan") when that card had no reading this row -- an
// unplugged/dead second card is a normal steady-state condition, not a bug.
// first_ms is the radio's arrival stamp of the body's first sight on any
// card, mono ms printed to 3 decimals (µs resolution) -- t_ms is the
// finalize tick, ~10 ms coarse, and first_ms is what joins against
// au.log's t_complete (mono µs, same clock) to measure how far after
// an enh AU's completion the probe, i.e. the end of the burst, lands.
//
// Every failure mode (LogWriter::open() unable to prepare the file, ...) is
// non-fatal: ok() reads false and row() becomes a silent no-op. maburgs
// must never exit or crash over this log.
class ProbeLog {
 public:
  // dir is the session directory (DebugSession::dir()); the file is always
  // "probe.log" inside it.
  ProbeLog(LogWriter& w, const std::string& dir, int bpb);

  ProbeLog(const ProbeLog&) = delete;
  ProbeLog& operator=(const ProbeLog&) = delete;

  bool ok() const { return s_ != LogWriter::kBadStream; }
  const std::string& path() const { return w_.path(s_); }

  void row(double t_ms, uint32_t seq, int mcs, uint16_t enh_fid,
           int blocks_ok, uint32_t card_mask, double snr0, double snr1,
           double evm0, double evm1, double first_ms);

 private:
  LogWriter& w_;
  LogWriter::Stream s_;
};

}  // namespace maburgs
