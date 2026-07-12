#pragma once
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace mabur {

struct FragCompleted {
  uint16_t seq = 0;               // the FRAG seq, for delivery accounting
  std::vector<uint8_t> pkt;
};

// Receiver counterpart of Fragmenter: buffers 4-byte-FRAG-headed chunks
// (<HBB>: seq u16, idx u8, count u8) keyed by seq and emits the reassembled
// packet once all `count` chunks arrived. A packet whose fragments never all
// decode is simply never emitted — the per-packet UEP delivery semantics.
// Port of svc_uep_fec.py SvcUepDecoder._reassemble, hardened: bounded
// pending map (oldest evicted + counted), and malformed inputs (count == 0,
// non-contiguous indices from corruption) are dropped instead of crashing.
//
// With max_age_ms > 0 and callers passing now_ms, entries whose first
// fragment is older than max_age_ms are also evicted (throttled sweep) — an
// entry older than the FEC block horizon can never complete, and count-only
// eviction let a full map at sustained partial loss evict entries that WOULD
// have completed (bench 2026-07-13 `fe=` blowups). The count cap remains as
// a memory backstop.
class FragReassembler {
 public:
  explicit FragReassembler(size_t max_pending = 512, uint64_t max_age_ms = 0);

  // One recovered FEC packet in; zero or one completed packets out.
  // now_ms == 0 (or max_age_ms == 0) disables age eviction.
  std::vector<FragCompleted> add(const uint8_t* pkt, size_t len,
                                 uint64_t now_ms = 0);

  uint64_t completed() const { return completed_; }
  uint64_t evicted() const { return evicted_; }
  size_t pending() const { return pending_.size(); }

 private:
  struct Entry {
    std::map<int, std::vector<uint8_t>> chunks;  // idx -> payload
    int count = 0;
    uint64_t order = 0;                          // insertion order, for eviction
    uint64_t t_ms = 0;                           // first-fragment arrival
  };
  void sweep_expired(uint64_t now_ms);

  std::map<uint16_t, Entry> pending_;
  size_t max_pending_;
  uint64_t max_age_ms_;
  uint64_t last_sweep_ms_ = 0;
  uint64_t order_counter_ = 0, completed_ = 0, evicted_ = 0;
};

}  // namespace mabur
