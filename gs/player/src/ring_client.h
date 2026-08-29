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

struct AuEvent {
  maburgs::AuRecordMeta meta;
  std::vector<uint8_t> au;
  bool flush_before;   // decoder must flush before this AU (discont/resync)
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

  bool open();                     // maps ring; doorbell connect is lazy/optional
  // Pumps everything currently readable through the sink; returns count.
  // Blocking wait strategy: poll(2) on the doorbell fd with timeout_ms when
  // connected, plain timeout sleep otherwise (doorbell is an optimization,
  // never a correctness dependency — state lives in the ring).
  size_t pump(int timeout_ms);
  bool oneshot_drain();            // read retained records, then return (e2e)

  // Policy counters:
  uint64_t delivered() const { return delivered_; }
  uint64_t dropped_enhance_incomplete() const { return dropped_enhance_incomplete_; }
  uint64_t truncated_base() const { return truncated_base_; }
  uint64_t resyncs() const { return reader_.resyncs(); }
  bool dead() const { return reader_.dead(); }
  // Stall diagnostics: one line of reader/doorbell internals for fps-log.
  std::string debug_line() const;

 private:
  // POLICY (the point of the native player, spec §maburplay):
  //  - meta.sid == 1 && !(flags & kRecFlagComplete)  -> drop whole, count.
  //  - base AU (sid != 1, i.e. sid == 0) delivered always; truncated base
  //    counted. 2-stream space (spec 2026-08-29-airtime-balance-uep):
  //    sid 0 = base, sid 1 = enhance — was sid 3 pre-fold, {0,1,2,3}.
  //  - flags & kFlagDiscont, or reader returned kResync -> next delivered
  //    AU carries flush_before = true.
  size_t drain_ring_();

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

  int door_fd_ = -1;
  bool door_hello_ok_ = false;
  uint64_t door_last_attempt_ms_ = 0;   // 0 = never attempted
  bool door_mismatch_logged_ = false;   // suppress repeat log spam until a good hello lands
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_RING_CLIENT_H_
