#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>
#include "mabur/frame_wire.h"

namespace maburgs {

struct RtpPacketizerCfg {
  uint8_t payload_type = 97;         // MUST match waybeam rtp_session.c (HEVC)
  uint32_t ssrc = 0x4D414252;        // "MABR"; constant per maburgs run
  size_t max_payload = 1400;         // RTP payload bytes per packet
  uint32_t nominal_frame_us = 16667; // pts advance across a discontinuity
};

// Converts a streaming Annex-B H.265 byte flow (as delivered by FrameStream's
// begin_frame/data/end_frame callbacks) into RFC 7798 RTP packets: single-NAL
// packets when a NAL fits max_payload, FU-A-style fragmentation otherwise.
// Handles pts unwrap/rebase across kFlagDiscont and u32 wraparound, and
// truncated frames (end_frame(false)) by closing the in-flight NAL without
// the RTP marker or FU end bit.
class RtpPacketizer {
 public:
  using Emit = std::function<void(const std::vector<uint8_t>&)>;

  RtpPacketizer(RtpPacketizerCfg cfg, Emit emit);

  void begin_frame(const mabur::framewire::FrameHdr& h);
  void data(const uint8_t* p, size_t n);   // streaming Annex-B bytes
  void end_frame(bool complete);           // marker only when complete

  uint16_t next_seq() const { return seq_; }

 private:
  void emit_rtp(const uint8_t* payload, size_t n, bool marker);
  void feed_byte(uint8_t b);
  void close_nal(bool frame_end, bool complete);
  void drain_fu(bool force_all, bool allow_end);

  RtpPacketizerCfg cfg_;
  Emit emit_;

  uint16_t seq_ = 0;
  uint32_t ts_ = 0;

  bool have_pts_ = false;
  int64_t pts64_ = 0;
  uint32_t last_pts_ = 0;

  // Start-code scanner state.
  std::vector<uint8_t> pending_;   // bytes of the current NAL (start code stripped)
  int zero_run_ = 0;               // count of consecutive 0x00 bytes seen (capped)
  bool in_nal_ = false;            // true once the first start code has been seen

  // FU streaming state for the NAL currently in pending_.
  bool fu_started_ = false;   // an FU packet has been emitted for this NAL
  size_t fu_sent_ = 0;        // bytes of pending_ already sent via FU (from offset 2)
  bool marker_pending_ = false;  // set by close_nal() before the final drain_fu()
};

}  // namespace maburgs
