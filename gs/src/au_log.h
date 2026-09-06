#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "au_ring.h"
#include "log_writer.h"

namespace maburgs {

// Per-AU meta rows -- what tools/gs/flightrec.py's ring thread used to
// produce by reading the shm ring from outside. maburgs is the ring WRITER,
// so it has every field at hand: no mmap, no seqlock retry, no epoch resync,
// and no rows lost to a reader overrun.
//
// Format (`# aulog 4`), 12 columns, field order unchanged from aulog 3:
//   t_us pts sid fid len flags nal0 t_first t_complete enc dq air
// t_us is CLOCK_MONOTONIC µs. In aulog <= 3 it was WALL-clock µs and the log
// carried `# sync` anchors to bridge to the jsonl; one process writing both
// files on one clock makes the bridge unnecessary (docs/data-provenance.md).
//
// Call shape mirrors AuRingWriter so the two stay in step at the call site:
// begin() per AU, payload() for each fragment, row() after finish().
class AuLog {
 public:
  AuLog(LogWriter& w, const std::string& dir);

  AuLog(const AuLog&) = delete;
  AuLog& operator=(const AuLog&) = delete;

  bool ok() const { return s_ != LogWriter::kBadStream; }

  void begin() { head_n_ = 0; }
  // Latches the first kHead bytes of the AU; everything after is ignored.
  void payload(const uint8_t* d, size_t n);
  void row(uint64_t t_us, const AuRecordMeta& m);

  // First H.265 NAL unit type after a 3- or 4-byte start code, else -1.
  static int nal0_of(const uint8_t* h, size_t n);

 private:
  static constexpr size_t kHead = 6;

  LogWriter& w_;
  LogWriter::Stream s_;
  uint8_t head_[kHead] = {};
  size_t head_n_ = 0;
};

}  // namespace maburgs
