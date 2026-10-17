#pragma once

#include <cstdint>
#include <string>

#include "log_writer.h"
#include "mabur/sw_decoder.h"

namespace maburgs {

// Per-episode FEC loss log: one row per LossEpisode a video
// layer's SwDecoder closes -- the raw inputs for "what FEC overhead would
// this link have needed", read by tools/flightreport.py's FEC section.
// Writes `fec.log` inside the session directory (DebugSession::dir()) via
// the shared LogWriter, rotating with the session like probe.log.
//
// Record format is LOCKED (tests/test_fec_log.cpp pins the byte layout):
//
//   feclog 3                                                   # first line
//   <t_ms> <sid> <mcs> <bw> <ov> <first_seq> <span> <m> <rec> <rtx> <aband> <stale> <r> <w>
//
// t_ms is the core-loop drain tick (mono ms, ~10 ms coarse; an episode
// closes at horizon eviction, ~a horizon after the loss itself). sid is the
// video layer (0 base, 1 enh). mcs, bw and ov are the op MCS, the op's
// width (20|40 MHz) and THAT sid's commanded overhead at drain time, so a
// row scores against the rung it flew on without a ctl.log join (a rung
// change inside the horizon is the case `stale` > 0 flags). first_seq is the
// wire (u32) seq of the first missing source; span, m (missing), rec, rtx,
// aband, stale, r (covering repairs) and w (repair window as flown) are
// LossEpisode verbatim.
//
// feclog 1 had no bw column: every row was 20 MHz, and flightreport.py
// reads it as such. feclog 2 added bw so 20/3 and 40/3 episodes do not
// pool, and had no rtx column (the NACK retransmit didn't exist yet).
// feclog 3 adds rtx: symbols a NACK retransmit filled -- video-whole but
// still residual loss to the ladder (ladder_residual.cpp counts them the
// same way).
//
// feclog 4 adds two row kinds that start with a letter, so an episode row
// still starts with its t_ms:
//   I <t_ms> <reason> <epoch>       an IDR request went out (reason: what
//                                   armed it -- hole, salvaged, lost, start,
//                                   resync, decoder, dvr; a retry repeats it)
//   H <t_ms> <heal_ms> <retries>    a complete base IRAP cleared the request
//
// Every failure mode is non-fatal: ok() reads false and row() is a no-op.
class FecLog {
 public:
  FecLog(LogWriter& w, const std::string& dir);

  FecLog(const FecLog&) = delete;
  FecLog& operator=(const FecLog&) = delete;

  bool ok() const { return s_ != LogWriter::kBadStream; }
  void rotate(const std::string& dir) { w_.reopen(s_, dir); }
  const std::string& path() const { return w_.path(s_); }

  void row(double t_ms, int sid, int mcs, int bw, double ov, const mabur::LossEpisode& e);

  void request(double t_ms, const char* reason, uint8_t epoch);
  void heal(double t_ms, uint64_t heal_ms, uint64_t retries);

 private:
  LogWriter& w_;
  LogWriter::Stream s_;
};

}  // namespace maburgs
