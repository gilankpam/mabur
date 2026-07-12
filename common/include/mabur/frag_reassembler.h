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
class FragReassembler {
 public:
  explicit FragReassembler(size_t max_pending = 512);

  // One recovered FEC packet in; zero or one completed packets out.
  std::vector<FragCompleted> add(const uint8_t* pkt, size_t len);

  uint64_t completed() const { return completed_; }
  uint64_t evicted() const { return evicted_; }
  size_t pending() const { return pending_.size(); }

 private:
  struct Entry {
    std::map<int, std::vector<uint8_t>> chunks;  // idx -> payload
    int count = 0;
    uint64_t order = 0;                          // insertion order, for eviction
  };
  std::map<uint16_t, Entry> pending_;
  size_t max_pending_;
  uint64_t order_counter_ = 0, completed_ = 0, evicted_ = 0;
};

}  // namespace mabur
