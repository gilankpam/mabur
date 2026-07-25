#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include <vector>
#include "mabur/frame_wire.h"

namespace maburgs {

struct FrameStreamCfg {
  uint64_t gap_timeout_ms = 50;  // unfilled gap older than this => truncate
  int lookahead = 8;             // frames ahead of head-of-line => force advance
};

// Reassembles whole frames from raw wide UEP fragments across layers, orders
// them strictly by ascending unwrapped frame_id, and streams each frame's
// contiguous chunk prefix out via callbacks as fragments arrive.
class FrameStream {
 public:
  struct Callbacks {
    std::function<void(const mabur::framewire::FrameHdr&)> begin_frame;
    std::function<void(const uint8_t*, size_t)> frame_data;  // Annex-B bytes, in order
    std::function<void(bool complete)> end_frame;
  };

  FrameStream(FrameStreamCfg cfg, Callbacks cb) : cfg_(cfg), cb_(std::move(cb)) {}

  void push_fragment(uint8_t stream_id, const uint8_t* pkt, size_t len, uint64_t now_ms);
  void poll(uint64_t now_ms);  // drives timeouts; call every loop tick
  void reset();                // session change

  uint64_t frames_clean() const { return clean_; }
  uint64_t frames_truncated() const { return truncated_; }
  uint64_t frames_dropped() const { return dropped_; }
  uint64_t bad_fragments() const { return bad_frags_; }

 private:
  struct Slot {
    uint8_t sid = 0;
    uint16_t fseq = 0;
    std::map<uint16_t, std::vector<uint8_t>> chunks;  // idx -> payload
    uint16_t count = 0;
    bool have_hdr = false;          // fragment 0 seen
    mabur::framewire::FrameHdr hdr;
    uint64_t id64 = 0;              // unwrapped frame_id (valid iff have_hdr)
    uint64_t first_ms = 0;          // first fragment arrival
    uint64_t last_progress_ms = 0;  // last time emitted_upto advanced
    uint16_t emitted_upto = 0;      // next chunk idx to emit
    bool began = false;
    bool discont = false;           // fragment 0's header had kFlagDiscont set
  };
  void try_emit(uint64_t now_ms);
  void finish(Slot& s, bool complete);
  uint64_t unwrap_id(uint16_t id, uint8_t flags);

  FrameStreamCfg cfg_;
  Callbacks cb_;
  std::map<uint32_t, Slot> slots_;   // key = (sid << 16) | fseq
  bool have_id_base_ = false;
  uint64_t last_id64_ = 0;           // unwrap reference
  uint64_t next_emit_id64_ = 0;      // head-of-line
  bool have_next_emit_ = false;
  uint64_t clean_ = 0, truncated_ = 0, dropped_ = 0, bad_frags_ = 0;
};

}  // namespace maburgs
