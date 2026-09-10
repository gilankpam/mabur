#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "log_writer.h"

namespace maburgs {

struct CalCell;
struct RateWall;

// Raw per-cell record of a TX-power calibration run (spec
// docs/superpowers/specs/2026-09-10-tx-power-calibration-design.md). Writes
// `cal.log` inside the session directory (DebugSession::dir()) via a
// private LogWriter -- unlike ctl.log/probe.log/au.log this does not share
// main.cpp's one session writer, because a calibration run is rare and
// short-lived (a handful of sweeps, not a per-frame hot path), so there is
// nothing to gain from wiring it into the fixed-slot session writer and
// every caller that wants one just constructs it from a directory.
//
// This file exists because a later task deletes bench/txagcbench/ (the old
// sweep tooling) along with its Python analyzer. cal.log plus the
// `maburcal report` reader is what replaces that capability -- without it,
// deleting the old tool would cost the ability to examine a run's
// underlying data after the fact.
//
// Record format is LOCKED (gs/bundle/maburcal's reader and
// tests/test_cal_log.cpp depend on the exact byte layout):
//
//   callog 1                                               # once, first line
//   R <nonce> <base_ref> <margin_db>             # once per calibration run
//   C <phase> <rate> <idx> <expected> <recv0> <recv1> <corrupt> <rssi0> <rssi1>
//                                                # one row per measured cell
//   W <rate> <wall> <floor> <best_card> <flags>  # one row per rate, at Result
//   V <rate> <idx> <pct>                         # one row per verify cell
//
// The marker and the per-run parameters are deliberately split: `callog 1`
// versions the FILE (mirrors ctllog N elsewhere in this codebase) and is
// written at most once no matter how many runs share the session
// directory, while `R` carries one run's own (nonce, base_ref, margin_db)
// and is written once at the start of EVERY run. The normal retry path --
// `maburcal start`, see a narrow/saturated flag, move the drone, start
// again -- puts a second run's records in the same session directory
// (nothing about a run rotates the session), so a single shared header
// would leave the second run's C/W/V lines with no way to say which
// nonce/base_ref/margin they belong to. An R line is what a reader keys
// runs by, and it also reliably marks where one run's lines end and the
// next begins.
//
// margin_db prints to two decimals. A card with no RSSI reading in a cell
// (CalCell::have_rssi[i] false) writes -999 (kRssiNone), never a blank or a
// zero -- the sentinel is what lets the reader tell "no signal heard" apart
// from "heard it loud" without a second column. An undetermined rate (no
// wall could be derived) writes -1 for both wall and floor, mirroring
// RateWall's own in-memory sentinel.
//
// Per the project's compatibility policy the `callog 1` marker is what
// versions this file -- the schema itself is free to change, provided both
// in-repo consumers (gs/bundle/maburcal and this test) change in the same
// commit. No migration shim, no reserved bytes.
//
// Every failure mode (LogWriter::open() unable to prepare the file, ...) is
// non-fatal: ok() reads false and every record method becomes a silent
// no-op. maburgs must never exit or crash over this log.
class CalLog {
 public:
  // dir is the session directory (DebugSession::dir()); the file is always
  // "cal.log" inside it, opened for append. A wrapper respawn that rejoins
  // an existing session directory must not call header() again -- the
  // caller decides whether this construction is the FILE's first ever
  // writer (header()) or merely another run sharing the session (run()
  // only); this class never writes anything on its own.
  explicit CalLog(const std::string& dir);

  CalLog(const CalLog&) = delete;
  CalLog& operator=(const CalLog&) = delete;

  bool ok() const { return s_ != LogWriter::kBadStream; }

  // The format-marker line, `callog 1`. Call at most once per FILE (i.e.
  // never on a rejoin of an already-headed session directory) -- it
  // carries no per-run data, so unlike ctl.log's single combined header
  // there is no reason for a second calibration run in the same session to
  // repeat it.
  void header();
  // One run's own parameters: `R <nonce> <base_ref> <margin_db>`. Call once
  // at the start of EVERY calibration run, including the second and later
  // ones sharing a session directory -- this is what a reader keys a run's
  // C/W/V lines by and what delimits one run's lines from the next.
  void run(uint32_t nonce, int base_ref_idx, double margin_db);
  void cell(uint8_t phase, uint8_t rate, uint8_t idx, const CalCell& c);
  void wall(uint8_t rate, const RateWall& w);
  void verify(uint8_t rate, uint8_t idx, int pct);

 private:
  LogWriter w_;
  LogWriter::Stream s_;
};

}  // namespace maburgs
