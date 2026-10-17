#ifndef MABUR_PLAYER_LAT_LOG_H_
#define MABUR_PLAYER_LAT_LOG_H_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace maburplay {

// Written by maburgs' DebugSession; maburplay only ever reads it. Declared
// here rather than shared from gs/src because the player does not link
// mabur_gs_core -- the two constants must stay equal.
inline constexpr const char* kSessionMarker = "/tmp/mabur-session";

// Persists a player record into the CURRENT debug session, so it survives
// power-off (/tmp is tmpfs; the flight-0035 tail was lost exactly that
// way). Three instances exist: the 1 Hz `lat:` line (lat.log, "# latlog 2"),
// the per-frame rows behind it (frame.log, "# framelog 3", batched: each
// second's rows are written together at flush()) and the 1 Hz `regulator:`
// line (reg.log, "# reglog 1").
//
// maburplay holds no logging config: it follows the marker maburgs writes.
// No marker means no session means no file -- which is how one knob in
// maburgs.toml turns the whole GS's debug logging off.
//
// Format: the marker line, then "<mono_us> <payload>" per write, appended.
// There is no `# sync` clock bridge: maburgs and maburplay both stamp
// CLOCK_MONOTONIC on the same box, so every debug-log file shares one clock.
//
// Never blocks, never throws, never spams. Every file operation -- the
// marker read, fopen, the writes, fflush, fclose -- runs on the instance's
// own writer thread; write() and flush() only queue under a short lock. The
// caller is maburplay's main loop, which also drives decode and display: a
// write that waited on the SD card there froze the picture for as long as
// the card stalled. The queue is bounded (kQueueBytes); a row that does not
// fit is dropped and counted, and the file gets one "# dropped N" line where
// the gap is. A missing or unusable session is re-checked at most every 5 s
// of row stamps (kRecheckUs), and the lat: line always still goes to stderr
// via the caller.
class LatLog {
 public:
  explicit LatLog(const char* marker_path = kSessionMarker,
                  const char* file_name = "lat.log",
                  const char* header = "# latlog 2", bool batched = false);
  // Lands every queued row, then closes the file.
  ~LatLog();
  LatLog(const LatLog&) = delete;
  LatLog& operator=(const LatLog&) = delete;

  // Line-buffered by default: each row reaches the file as soon as the
  // writer thread takes it. A batched instance holds its rows in a
  // kBatchBytes stdio buffer until flush(), so a burst of rows costs one
  // syscall instead of one per row.
  void write(uint64_t mono_us, const char* payload);
  void flush();

  // Waits until the writer thread has handled everything queued so far.
  // Tests and shutdown only -- it waits on the disk.
  void sync();
  // The file currently written to, empty while there is no session.
  std::string path() const;
  // Rows dropped because the queue was full.
  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

  static constexpr size_t kQueueBytes = 256 * 1024;  // ~20 s of frame.log rows

 private:
  struct Rec {
    enum Kind : uint8_t { kLine, kFlush, kDrop } kind;
    uint64_t mono_us = 0;  // kLine: the row's stamp
    uint64_t n = 0;        // kDrop: rows dropped at this point
    std::string text;      // kLine: the payload
  };

  void push_(Rec r, size_t bytes);
  void run_();
  void handle_(const Rec& r);
  void reopen_(uint64_t mono_us);

  const char* marker_;
  const char* file_name_;
  const char* header_;
  static constexpr size_t kBatchBytes = 64 * 1024;  // > 1 s of frame.log rows

  // Shared between the caller and the writer thread, under mu_.
  mutable std::mutex mu_;
  std::condition_variable work_cv_, done_cv_;
  std::deque<Rec> q_;
  size_t q_bytes_ = 0;
  uint64_t pushed_ = 0, done_ = 0;
  bool stop_ = false;
  std::string path_;
  std::atomic<uint64_t> dropped_{0};

  // Writer thread only.
  std::unique_ptr<char[]> buf_;  // batched only: the stream's own buffer
  std::FILE* f_ = nullptr;
  std::string dir_;
  uint64_t last_check_us_ = 0;
  bool checked_ = false;

  std::thread th_;  // last: starts after every member above exists
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_LAT_LOG_H_
