#ifndef MABUR_PLAYER_RING_CLIENT_H_
#define MABUR_PLAYER_RING_CLIENT_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <sys/types.h>  // ssize_t
#include <vector>

#include "au_ring.h"

namespace maburplay {

// Ring v4 (spec 2026-10-10-h265-slices §6.2-6.3): a record can surface while
// maburgs is still filling it.
enum class AuEventKind : uint8_t {
  kOpen,   // first sight of the record being filled: meta + bytes so far
  kGrow,   // more bytes landed: au = all bytes so far (cumulative)
  kClose,  // the record is done: the pre-v4 event, unchanged fields
};

struct AuEvent {
  maburgs::AuRecordMeta meta;
  std::vector<uint8_t> au;
  // kClose: the decoder must flush before this AU (discont/resync).
  // kOpen/kGrow: a preview -- true iff this record's kClose will carry it
  // (a pending flush or its own discont flag), so a consumer can keep it
  // out of anything it would have to undo.
  bool flush_before;
  AuEventKind kind = AuEventKind::kClose;
  // kClose only: the record that surfaced open ends with nothing to decode
  // -- the writer aborted it (overflow), the ring resynced or the writer
  // restarted under it, or the enhance policy dropped it. au is empty; meta
  // is that record's. End whatever was started for it, nothing else: it is
  // not a delivery and is not counted as one.
  bool aborted = false;
};

// Bridges the maburgs AU ring (maburgs::AuRingReader, Task-1 hardened
// semantics) into a callback stream of decodable AUs, applying the native
// player's SVC-aware loss policy (see the POLICY comment below). The
// doorbell (a SOCK_SEQPACKET wakeup channel next to the ring, served by
// maburgs::AuDoorbell) is strictly a latency optimization: every behavior
// here is correct -- just capped by pump()'s timeout_ms instead of woken
// early -- with no doorbell socket present at all. All durable state
// (read cursor, discontinuity detection) lives in the ring/reader; the
// doorbell is never consulted for correctness.
class RingClient {
 public:
  struct Cfg {
    std::string ring_path, socket;
  };
  using Sink = std::function<void(AuEvent&&)>;

  RingClient(Cfg cfg, Sink sink);
  ~RingClient();
  RingClient(const RingClient&) = delete;
  RingClient& operator=(const RingClient&) = delete;

  // Maps the ring; doorbell connect is lazy/optional. wait_ms controls what
  // happens when the ring is not there yet: 0 tries once, >0 retries until
  // that many ms have passed, <0 retries forever.
  bool open(int wait_ms = 0);
  // Pumps everything currently readable through the sink; returns count.
  // Blocking wait strategy: poll(2) on the doorbell fd with timeout_ms when
  // connected, plain timeout sleep otherwise (doorbell is an optimization,
  // never a correctness dependency — state lives in the ring).
  size_t pump(int timeout_ms);
  // A second fd pump()'s wait also ends on (FeedLoop's eventfd: a park, a
  // drain or a stop must not wait for the next doorbell). Polled, never read
  // here -- its owner clears it. -1 = none.
  void set_wake_fd(int fd) { wake_fd_ = fd; }
  bool oneshot_drain();            // read retained records, then return (e2e)

  // Policy counters:
  uint64_t delivered() const { return delivered_; }
  uint64_t dropped_enhance_incomplete() const { return dropped_enhance_incomplete_; }
  uint64_t truncated_base() const { return truncated_base_; }
  // Slice salvage (spec 2026-10-10-h265-slices §5.5): counted separately
  // from truncated_base/dropped_enhance_incomplete -- a salvaged AU is
  // delivered (it is decodable), just not a plain complete record.
  uint64_t salvaged_base() const { return salvaged_base_; }
  uint64_t salvaged_enhance() const { return salvaged_enhance_; }
  // Ring v4: records surfaced while open, and those of them the ring ended
  // without a record (aborted closes from overflow/resync/restart; an
  // enhance dropped by policy is counted in dropped_enhance_incomplete).
  uint64_t opened() const { return opened_; }
  uint64_t open_aborted() const { return open_aborted_; }
  // Ring wake latency (spec §6.2's futex question): for each closed record
  // delivered since the last call, delivery time - the writer's
  // t_complete_us stamp, µs.
  struct Wake { uint32_t p50 = 0, p99 = 0; uint32_t n = 0; };
  Wake take_wake();
  uint64_t resyncs() const { return reader_.resyncs(); }
  bool dead() const { return reader_.dead(); }
  // Stall diagnostics: one line of reader/doorbell internals for fps-log.
  std::string debug_line() const;

 private:
  // POLICY (the point of the native player, spec §maburplay):
  //  - meta.sid == 1 && !maburgs::au_decodable(flags) -> drop whole, count.
  //    (Slice salvage, spec 2026-10-10-h265-slices §5.5: a salvaged AU is
  //    decodable even though it is not complete, so it is NOT dropped here
  //    -- counted via salvaged_enhance() instead.)
  //  - base AU (sid != 1, i.e. sid == 0) delivered always; truncated base
  //    counted (salvaged base counted via salvaged_base() instead).
  //    2-stream space (spec 2026-08-29-airtime-balance-uep):
  //    sid 0 = base, sid 1 = enhance — was sid 3 pre-fold, {0,1,2,3}.
  //  - flags & kFlagDiscont, or reader returned kResync -> next delivered
  //    AU carries flush_before = true.
  size_t drain_ring_();

  // Ring v4 plumbing. peek_open_: after next() said kNone; true = the ring
  // may hold more (an aborted record's slot can already hold the next AU).
  bool peek_open_();
  void emit_open_(AuEventKind kind);
  void end_open_();   // the record followed open is gone: aborted kClose
  void note_wake_(uint64_t t_complete_us);
  // Builds and sinks an aborted kClose for meta m. Callers do their own
  // bookkeeping (open_aborted_++ and resetting open_ where applicable) --
  // this only builds and sinks the event, so every aborted-close site (an
  // overflowed/resynced/restarted open record, and the enhance policy drop)
  // shares one event shape.
  void emit_aborted_(const maburgs::AuRecordMeta& m);

  // Doorbell client plumbing (wakeup optimization only, see class comment).
  void maybe_connect_door_();
  void service_door_(int timeout_ms);
  void handle_door_datagram_(const uint8_t* buf, ssize_t n);
  void door_mismatch_(const char* why);
  void drop_door_();

  Cfg cfg_;
  Sink sink_;
  maburgs::AuRingReader reader_;

  bool pending_flush_ = false;
  uint64_t delivered_ = 0;
  uint64_t dropped_enhance_incomplete_ = 0;
  uint64_t truncated_base_ = 0;
  uint64_t salvaged_base_ = 0;
  uint64_t salvaged_enhance_ = 0;

  maburgs::AuRingReader::OpenView open_;   // record followed open (lock 0 = none)
  uint64_t opened_ = 0;
  uint64_t open_aborted_ = 0;
  std::vector<uint32_t> wake_us_;

  int door_fd_ = -1;
  int wake_fd_ = -1;
  bool door_hello_ok_ = false;
  uint64_t door_last_attempt_ms_ = 0;   // 0 = never attempted
  bool door_mismatch_logged_ = false;   // suppress repeat log spam until a good hello lands
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_RING_CLIENT_H_
