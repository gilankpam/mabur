#pragma once
#include <algorithm>
#include <cstdint>
#include <functional>
#include <map>
#include <vector>
#include "au_ring.h"
#include "mabur/frame_wire.h"

namespace maburgs {

// FRAG fragment arrival metadata (Task 7's DecodedFrag mirror at the
// FrameStream boundary). Fields default to 0/unknown so call sites that
// don't have real values (tests, the dry-run replay's synthetic feed) can
// rely on push_fragment's default argument.
struct FragArrival {
  uint64_t body_mono_us = 0;  // RX stamp of the completing body (0 = unknown)
  uint16_t q_ms = 0;          // SBI q_ms of that body (0 = unknown)
  uint16_t enc_us = 0;        // SBI enc_us of that body (0 = unknown)
};

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
    // lat.t_complete_us is always 0 here — the ring writer stamps finish
    // time (Task 6). See Slot::lat below for the other fields' latch rules.
    std::function<void(bool complete, const AuLatMeta& lat)> end_frame;
  };

  FrameStream(FrameStreamCfg cfg, Callbacks cb) : cfg_(cfg), cb_(std::move(cb)) {
    gap_ms_[0] = gap_ms_[1] = cfg_.gap_timeout_ms;
  }

  void push_fragment(uint8_t stream_id, const uint8_t* pkt, size_t len, uint64_t now_ms,
                     const FragArrival& arr = {});
  void poll(uint64_t now_ms);  // drives timeouts; call every loop tick
  void reset();                // session change

  // Rate-aware per-stream gap timeout (GapTimeoutPolicy): mid-frame gaps
  // and headerless slots wait their own sid's value; a WHOLE-frame gap
  // waits the max of both, because the missing frame's sid is unknowable.
  // Both default to cfg.gap_timeout_ms.
  void set_gap_timeout(int sid, uint64_t ms) {
    if (sid >= 0 && sid <= 1) gap_ms_[sid] = ms;
  }

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
    // Per-AU latency latch (Task 8), passed to end_frame at finish():
    //  - t_first_us: min over every stored fragment's nonzero body_mono_us
    //    (repair-completed heads carry the completing body's time —
    //    documented approximation, biased small on exactly the frames
    //    repaired at the wall).
    //  - drone_q_ms / enc_us: latched from the fragment that sets have_hdr
    //    (chunk idx 0, the AU's first data fragment); never overwritten
    //    after that; stays 0 if the frame was force-advanced without its
    //    idx-0 chunk.
    //  - t_complete_us: left 0 here — the ring writer stamps finish time.
    AuLatMeta lat;
  };
  void try_emit(uint64_t now_ms);
  void finish(Slot& s, bool complete);
  uint64_t unwrap_id(uint16_t id, uint8_t flags, bool* rebased);

  uint64_t gap_ms(uint8_t sid) const {
    return gap_ms_[sid <= 1 ? sid : 0];
  }
  uint64_t gap_ms_max() const { return std::max(gap_ms_[0], gap_ms_[1]); }

  FrameStreamCfg cfg_;
  uint64_t gap_ms_[2] = {0, 0};  // seeded from cfg in the ctor
  Callbacks cb_;
  std::map<uint32_t, Slot> slots_;   // key = (sid << 16) | fseq
  bool have_id_base_ = false;
  uint64_t last_id64_ = 0;           // unwrap reference
  uint64_t next_emit_id64_ = 0;      // head-of-line
  bool have_next_emit_ = false;
  bool in_discont_run_ = false;      // last unwrapped frame carried kFlagDiscont
  bool stall_armed_ = false;         // a frame arrived with nothing emitted since
  uint64_t stall_arm_ms_ = 0;        // when that first post-emit frame arrived
  bool discont_seen_since_emit_ = false;
  uint64_t last_stall_log_ms_ = 0;
  uint64_t clean_ = 0, truncated_ = 0, dropped_ = 0, bad_frags_ = 0;
  uint64_t stall_resets_ = 0;
};

}  // namespace maburgs
