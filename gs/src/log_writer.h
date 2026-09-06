#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace maburgs {

// One writer thread for every debug-log file in a session directory.
//
// Producers format a line and hand it over with line(); the writer thread
// does every fopen/fwrite/fflush. This exists because au.log is ~120 rows/s
// and probe.log ~60 rows/s, both emitted from the RX core thread, onto
// /media/dvr -- a real disk. A blocking write there is a video-latency
// hazard (the burn-DVR write-combining spike is the precedent).
//
// SINGLE PRODUCER. Every maburgs debug log is fed from the core loop thread;
// the head/tail pair is only safe under that invariant. flush_now() may be
// called from the producer thread only.
//
// Failure is always non-fatal: open() returns kBadStream on an unusable
// directory, line() on kBadStream is a no-op, and a full ring drops the line
// and counts it. The writer thread emits "# dropped N" into the affected
// file when it next flushes, so a gap is always visible in the data.
class LogWriter {
 public:
  using Stream = int;
  static constexpr Stream kBadStream = -1;
  static constexpr size_t kRingBytes = 1u << 20;  // 1 MiB
  static constexpr size_t kMaxLine = 64u * 1024;  // a sideport datagram fits
  static constexpr size_t kRecHdr = 8;            // u32 len | u32 stream
  // maburgs opens exactly four (ctl, probe, au, flight.jsonl). Fixed capacity
  // is what lets open() publish a new stream to the writer thread without a
  // lock: the slots never move, so only the COUNT has to be synchronised.
  static constexpr size_t kMaxStreams = 8;

  LogWriter();
  ~LogWriter();
  LogWriter(const LogWriter&) = delete;
  LogWriter& operator=(const LogWriter&) = delete;

  // Opens <dir>/<name> for APPEND and queues `header` as its first line (an
  // empty header queues nothing and is not a drop). A rejoined session
  // therefore carries a second marker line partway through the file; every
  // parser must tolerate that (it re-states the format and the run's
  // header_info, which can change across a config edit).
  //
  // mark_drops=false suppresses the "# dropped N" line for a file whose
  // format cannot carry a comment. flight.jsonl is the case: it is
  // newline-delimited JSON, a "# ..." line would break every reader, and a
  // gap is already visible there from the datagram's own `seq` field.
  Stream open(const std::string& dir, const char* name,
              const std::string& header, bool mark_drops = true);

  void line(Stream s, const char* text, size_t len);
  const std::string& path(Stream s) const;
  uint64_t dropped(Stream s) const;

  // Blocks until the ring is drained and every file is flushed. Shutdown and
  // tests only -- never on a hot path.
  void flush_now();

 private:
  struct Out {
    std::FILE* f = nullptr;
    std::string path;
    std::atomic<uint64_t> dropped{0};
    uint64_t reported = 0;
    bool mark_drops = true;
  };

  void run_();
  void report_and_flush_();
  void put_(uint64_t at, const char* src, size_t n);
  void get_(uint64_t at, char* dst, size_t n) const;
  void drain_();

  // Fixed slots + an atomic count: open() (producer thread) fills slot n and
  // release-stores n+1; the writer thread acquire-loads the count. A vector
  // would reallocate under the reader, which is UB.
  std::array<std::unique_ptr<Out>, kMaxStreams> outs_{};
  std::atomic<size_t> n_outs_{0};
  std::vector<char> buf_;
  std::atomic<uint64_t> head_{0};  // producer-owned
  std::atomic<uint64_t> tail_{0};  // consumer-owned
  std::atomic<bool> stop_{false};
  // drain_() and report_and_flush_() are the only tail_ writers and the only
  // FILE* users. flush_now() runs them from the PRODUCER thread while the
  // writer thread may be inside the same code, so both take this. line() is
  // unaffected and stays lock-free.
  std::mutex drain_mu_;
  std::thread th_;
  std::string empty_;
};

}  // namespace maburgs
