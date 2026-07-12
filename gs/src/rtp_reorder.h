#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include <vector>

namespace maburgs {

// Restores RTP sequence order on the decoded stream before the UDP sink.
//
// The UEP decoder emits packets when FEC blocks COMPLETE, and the critical
// and base-layer streams decode through independent pipelines — so emission
// order is not arrival order (bench 2026-07-13: ~5% forward gaps + ~5% late
// emissions at 6.8 Mbps; a live H.265 depacketizer discards the disorder and
// renders ~0.3 Mbps of a 6.8 Mbps transport stream). This buffer holds
// packets keyed by unwrapped RTP seq and releases them in order; a packet
// missing for longer than hold_ms is declared lost and skipped so true FRAG
// losses cost one bounded stall, not a wedge.
class RtpReorder {
 public:
  using Emit = std::function<void(const std::vector<uint8_t>&)>;

  RtpReorder(Emit emit, uint64_t hold_ms = 100)
      : emit_(std::move(emit)), hold_ms_(hold_ms) {}

  void push(std::vector<uint8_t> pkt, uint64_t now_ms) {
    if (pkt.size() < 4) { emit_(pkt); return; }  // not RTP; pass through
    const uint16_t seq16 = static_cast<uint16_t>((pkt[2] << 8) | pkt[3]);
    const uint64_t seq = unwrap(seq16);
    if (has_next_ && seq < next_) { ++late_dropped_; return; }  // too late
    buf_.emplace(seq, Entry{std::move(pkt), now_ms});
    if (!has_next_) { next_ = seq; has_next_ = true; }
    flush(now_ms);
  }

  // Call ~every loop tick: releases the head after hold_ms even with a gap.
  void poll(uint64_t now_ms) { flush(now_ms); }

  uint64_t skipped() const { return skipped_; }        // seqs declared lost
  uint64_t late_dropped() const { return late_dropped_; }
  size_t depth() const { return buf_.size(); }

 private:
  struct Entry {
    std::vector<uint8_t> pkt;
    uint64_t t_ms;
  };

  // Maps seq16 onto a monotonic 64-bit line via signed 16-bit deltas (the
  // starting epoch is arbitrary — only order matters). Starts one epoch up
  // so an early "too late" comparison can't underflow.
  uint64_t unwrap(uint16_t seq16) {
    if (!has_unwrap_) {
      has_unwrap_ = true;
      last16_ = seq16;
      cur_ = 0x10000ULL + seq16;
      return cur_;
    }
    cur_ += static_cast<int16_t>(seq16 - last16_);
    last16_ = seq16;
    return cur_;
  }

  void flush(uint64_t now_ms) {
    while (!buf_.empty()) {
      auto it = buf_.begin();
      if (it->first == next_) {
        emit_(it->second.pkt);
        buf_.erase(it);
        ++next_;
      } else if (now_ms - it->second.t_ms >= hold_ms_) {
        skipped_ += it->first - next_;
        next_ = it->first;  // give up on the gap; emit from here
      } else {
        break;
      }
    }
  }

  Emit emit_;
  uint64_t hold_ms_;
  std::map<uint64_t, Entry> buf_;
  bool has_next_ = false, has_unwrap_ = false;
  uint64_t next_ = 0, skipped_ = 0, late_dropped_ = 0;
  uint16_t last16_ = 0;
  uint64_t cur_ = 0;
};

}  // namespace maburgs
