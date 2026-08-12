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
  uint64_t stall_reset_ms = 500; // frames arriving but none emitted for this
                                 // long => self-reset (0 disables). Backstop
                                 // for a wedged emit cursor, e.g. a producer
                                 // restart whose discont signal was lost.
};

// Reassembles whole frames from raw wide UEP fragments across layers, orders
// them strictly by ascending unwrapped frame_id, and streams each frame's
// contiguous chunk prefix out via callbacks as fragments arrive.
class FrameStream {
 public:
  struct Callbacks {
    std::function<void(const mabur::framewire::FrameHdr&, uint8_t sid)> begin_frame;
    std::function<void(const uint8_t*, size_t)> frame_data;  // Annex-B bytes, in order
    std::function<void(bool complete)> end_frame;
    // Optional (may be null): a frame was lost with a KNOWN sid — truncated
    // emit (truncated=true), late-arrival eviction, or headerless age-out
    // (truncated=false). Pure id64 gap skips and reset() teardown do NOT
    // fire (sid unknown / not a glitch). Spec 2026-08-11 idr-request.
    std::function<void(uint8_t sid, bool truncated)> frame_lost;
  };

  FrameStream(FrameStreamCfg cfg, Callbacks cb) : cfg_(cfg), cb_(std::move(cb)) {}

  void push_fragment(uint8_t stream_id, const uint8_t* pkt, size_t len, uint64_t now_ms);
  void poll(uint64_t now_ms);  // drives timeouts; call every loop tick
  void reset();                // session change

  uint64_t frames_clean() const { return clean_; }
  uint64_t frames_truncated() const { return truncated_; }
  uint64_t frames_dropped() const { return dropped_; }
  uint64_t bad_fragments() const { return bad_frags_; }
  uint64_t stall_resets() const { return stall_resets_; }

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
    bool discont = false;           // this frame re-based the id64 space
  };
  void try_emit(uint64_t now_ms);
  void finish(Slot& s, bool complete);
  uint64_t unwrap_id(uint16_t id, uint8_t flags, bool* rebased);

  FrameStreamCfg cfg_;
  Callbacks cb_;
  std::map<uint32_t, Slot> slots_;   // key = (sid << 16) | fseq
  bool have_id_base_ = false;
  uint64_t last_id64_ = 0;           // unwrap reference
  uint64_t next_emit_id64_ = 0;      // head-of-line
  bool have_next_emit_ = false;
  // Sticky "this stream carries an SVC-T enhance layer": gates the gap-skip
  // sid inference (see try_emit). A stream that never showed sid 3 has no
  // base/enhance alternation to key off, so every hole is base-class.
  bool saw_enhance_ = false;
  bool in_discont_run_ = false;      // last unwrapped frame carried kFlagDiscont
  bool stall_armed_ = false;         // a frame arrived with nothing emitted since
  uint64_t stall_arm_ms_ = 0;        // when that first post-emit frame arrived
  bool discont_seen_since_emit_ = false;
  uint64_t last_stall_log_ms_ = 0;
  uint64_t clean_ = 0, truncated_ = 0, dropped_ = 0, bad_frags_ = 0;
  uint64_t stall_resets_ = 0;
};

}  // namespace maburgs
