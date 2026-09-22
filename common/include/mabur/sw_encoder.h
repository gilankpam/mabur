#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>
namespace mabur {

class FecWorker;
struct FecRepairJob;

// Config for the systematic sliding-window RLC FEC scheme (the sole FEC
// scheme; block RS is retired).
// window is the encoder ring / repair span in symbols, [2, 255] (wire u8).
struct SwConfig {
  int symbol_size = 64;
  int window = 128;
  double overhead = 0.25;  // repair symbols per source symbol
  int max_packet_size() const { return symbol_size - 2; }
};

// Systematic sliding-window encoder: packets concatenation-pack into
// fixed-size symbols (2-byte LE length prefix, zero pad, a packet NEVER
// spans two symbols), but each sealed symbol ships immediately as a source
// envelope — no block accumulation. Repairs are GF(256) linear combinations
// of the last <=window sealed symbols (coefficients from sw::repair_coeffs),
// emitted by a credit system: credit += overhead per seal, one repair per
// whole credit. Overlapping repair windows spread protection across
// subsequent air frames, buying time diversity without delaying sources.
class SwEncoder {
 public:
  // initial_seq seeds next_seq_ (default 0, which is what every existing
  // unit test and golden vector pins — do not change the default). A fresh
  // encoder instance MUST start >kResetSpan (sw_decoder.cpp) away from any
  // prior run's seqs with high probability, or a restarted drone's stream
  // is dropped as stale for its predecessor's lifetime: callers that
  // survive process restarts (UepEncoder, linkbench tx_main) should pass a
  // random draw (residual collision odds ~2^-11 against kResetSpan=2^20
  // over a ~2^32 seq space).
  // worker: optional shared FEC worker (spec 2026-07-17). nullptr (default)
  // = repairs are built synchronously inside add_packet/flush exactly as
  // before — same bytes, same order (this mode is what every pre-async test
  // pins, and the inline fallback when the worker queue is full). Non-null
  // = repairs are enqueued; their envelopes surface at a later
  // add_packet/flush return. Envelope BYTES are identical either way; only
  // emission order relaxes, which SwDecoder tolerates (no ordering
  // contract). Move the encoder only while no repairs are outstanding
  // (queued jobs hold a pointer to it) — in practice, at construction time
  // only, which is what UepEncoder does.
  explicit SwEncoder(const SwConfig& cfg, uint32_t initial_seq = 0,
                     FecWorker* worker = nullptr);
  ~SwEncoder();  // joins outstanding async repairs
  SwEncoder(SwEncoder&&) = default;
  SwEncoder& operator=(SwEncoder&&) = default;

  // Worker-thread entry point (called by FecWorker::loop): builds the
  // queued repair envelope from the ring and parks it for the next drain.
  // Not user API.
  void execute_repair_job(const FecRepairJob& job);

  // Feeds one packet. Returns any envelopes that became due: at most one
  // source (a symbol sealed to make room) plus credited repairs. A packet
  // larger than max_packet_size() returns empty and counts oversize_drops().
  std::vector<std::vector<uint8_t>> add_packet(const uint8_t* data, size_t len);

  // Seals a partially-filled symbol (if any) and emits one tail repair so a
  // burst tail is recoverable without waiting for the next source. The tail
  // repair fires at most once per sealed source (idle re-flushes are no-ops,
  // so UepEncoder's repeated poll cannot spam repairs). Async mode: does
  // NOT wait for the worker (fec-join-delete 2026-09-22) — returns whatever
  // repairs are already built; the rest surface at a later
  // add_packet/flush/collect drain, or at finish().
  std::vector<std::vector<uint8_t>> flush();

  // Async mode: joins every outstanding repair and returns them (what
  // flush() used to do at frame end). Shutdown (UepEncoder::flush_all) and
  // tests only — never on the per-frame path. Sync mode: returns empty.
  std::vector<std::vector<uint8_t>> finish();

  // Async mode: returns the repairs the worker has finished since the last
  // drain, never waits. The hot loop's between-frames harvest. Sync mode:
  // returns empty.
  std::vector<std::vector<uint8_t>> collect();

  // Async mode: true while a queued or in-flight repair has not been
  // parked for the next drain. Sync mode: false.
  bool repairs_outstanding() const;

  // Worker backlog bound: a credited repair is skipped (booked in
  // SwFecGauge::backlog_drops) when the shared worker queue already holds
  // this many jobs — ~1.3 frame periods of worker time at 60 fps and the
  // measured ~100-115 µs per repair. Sized above the per-frame depth peak
  // of the biggest legitimate frame (a 47 kB base frame at ov 1.0 queues
  // ~115 after its 2.7 ms feed; bench 2026-09-22 read qdepth_max 99 at
  // 18 Mb/s on the 0.5/0.25 pair) and below the 256-slot queue. A slow
  // core degrades to fewer repairs, never to a blocked producer or a
  // pinned venc ring.
  static constexpr uint32_t kMaxBacklogJobs = 192;

  // Takes effect immediately (no block boundary to wait for).
  void set_overhead(double overhead) { cfg_.overhead = overhead; }

  bool has_pending() const { return !current_symbol_.empty(); }
  size_t oversize_drops() const { return oversize_drops_; }
  uint64_t sources_out() const { return sources_out_; }
  uint64_t repairs_out() const { return repairs_out_; }

  // Async-path gauge (fec-compute handover 2026-09-01): splits the hot
  // thread's per-AU wall into wait-on-worker vs its own work, and exposes
  // the worker's per-repair build cost + queue depth. Sums/counts are
  // cumulative — the reader diffs across report windows; the *_max fields
  // are window maxima, reset by take_fec_gauge(). All fields are zero in
  // sync mode (no worker).
  struct SwFecGauge {
    uint64_t jobs = 0;             // repairs built via execute_repair_job
    uint64_t build_us = 0;         // wall µs inside those builds
    uint64_t inline_full = 0;      // queue-full inline fallbacks (hot thread)
    uint64_t join_waits = 0;       // join() calls that actually spun
    uint64_t join_wait_us = 0;     // hot-thread µs spent spinning in join()
    uint64_t join_wait_max_us = 0; // window max (reset on take)
    uint64_t enq_depth_max = 0;    // window max worker-queue depth at enqueue
    uint64_t backlog_drops = 0;    // repairs skipped at kMaxBacklogJobs
  };
  // Producer-thread-only, like add_packet/flush (it resets the window-max
  // fields in place).
  SwFecGauge take_fec_gauge();

 private:
  // Sealed symbols live in one contiguous 16 B-aligned fixed-stride ring of
  // window + kSlackRows rows (not a deque of vectors). The slack keeps a
  // row readable for kSlackRows further seals after it leaves the window —
  // the async repair path (spec 2026-07-17) queues jobs that reference ring
  // rows by slot instead of copying the window. 512 (was 64): with no
  // frame-end join the worker may legitimately trail the producer by up to
  // kMaxBacklogJobs repairs, i.e. several frames of seals at 332 B symbols,
  // and the row backstop join must stay a backstop, not a per-frame event.
  // ~170 kB per layer at 332 B.
  static constexpr size_t kSlackRows = 512;

  void append_to_current(const uint8_t* data, size_t len);
  void seal_current(std::vector<std::vector<uint8_t>>& out);
  // Envelope construction shared by the sync and (later) async paths; pure
  // reader of ring rows [start_slot, start_slot + window_len).
  std::vector<uint8_t> build_repair(uint32_t repair_key, uint32_t header_seq,
                                    int window_len, size_t start_slot) const;

  struct AsyncState {  // heap-held so SwEncoder stays movable
    std::atomic<int> outstanding{0};
    std::mutex done_m;
    std::vector<std::vector<uint8_t>> done;
    // Gauge sums the WORKER thread increments (the hot thread only reads);
    // heap-held with the rest so the encoder stays movable.
    std::atomic<uint64_t> gauge_jobs{0}, gauge_build_us{0};
  };

  // Sync mode: builds and returns the repair inline (today's exact
  // behavior). Async mode: enqueues (or builds inline on queue-full) and
  // returns nothing now. Both allocate key/counters identically.
  void emit_or_enqueue_repair(std::vector<std::vector<uint8_t>>& out);
  void drain_done(std::vector<std::vector<uint8_t>>& out);
  // Spins (relaxed + one acquire load) until no jobs are outstanding.
  // Reached only from finish() (shutdown/tests), the destructor, and the
  // ring-row backstop in seal_current — never per frame. Worst wait = the
  // whole backlog (≤ kMaxBacklogJobs builds); the hot-thread watchdog
  // (hot_beat) covers a wedged worker.
  void join();

  SwConfig cfg_;
  size_t stride_ = 0, cap_ = 0, count_ = 0, next_slot_ = 0;
  std::vector<uint8_t> ring_raw_;
  uint8_t* ring_ = nullptr;  // 16B-aligned base inside ring_raw_
  std::vector<uint8_t> current_symbol_;
  uint32_t next_seq_ = 0;   // seq the next sealed symbol gets
  uint32_t repair_key_ = 0;
  double credit_ = 0.0;
  bool tail_repair_pending_ = false;
  uint64_t sources_out_ = 0, repairs_out_ = 0;
  size_t oversize_drops_ = 0;

  FecWorker* worker_ = nullptr;
  std::unique_ptr<AsyncState> async_;  // null in sync mode
  long seals_since_join_ = 0;          // slack-bound backstop counter

  // Hot-thread-owned halves of SwFecGauge (see above for field semantics).
  uint64_t gauge_inline_full_ = 0;
  uint64_t gauge_join_waits_ = 0, gauge_join_wait_us_ = 0;
  uint64_t gauge_join_wait_max_us_ = 0;  // window max
  uint64_t gauge_enq_depth_max_ = 0;     // window max
  uint64_t gauge_backlog_drops_ = 0;
};

}  // namespace mabur
