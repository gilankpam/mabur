#include "rtp_packetizer.h"
#include <cassert>
#include <cstring>

namespace maburgs {

RtpPacketizer::RtpPacketizer(RtpPacketizerCfg cfg, Emit emit)
    : cfg_(cfg), emit_(std::move(emit)) {
  // FU chunking needs headroom for the 2-byte PayloadHdr + 1-byte FU header;
  // a max_payload <= 3 would make chunk_cap 0 and silently drop bytes.
  assert(cfg_.max_payload > 3);
}

void RtpPacketizer::emit_rtp(const uint8_t* payload, size_t n, bool marker) {
  std::vector<uint8_t> p(12 + n);
  p[0] = 0x80;
  p[1] = static_cast<uint8_t>(cfg_.payload_type | (marker ? 0x80 : 0));
  p[2] = static_cast<uint8_t>(seq_ >> 8);
  p[3] = static_cast<uint8_t>(seq_ & 0xFF);
  p[4] = static_cast<uint8_t>(ts_ >> 24); p[5] = static_cast<uint8_t>(ts_ >> 16);
  p[6] = static_cast<uint8_t>(ts_ >> 8);  p[7] = static_cast<uint8_t>(ts_);
  p[8] = static_cast<uint8_t>(cfg_.ssrc >> 24); p[9] = static_cast<uint8_t>(cfg_.ssrc >> 16);
  p[10] = static_cast<uint8_t>(cfg_.ssrc >> 8); p[11] = static_cast<uint8_t>(cfg_.ssrc);
  std::memcpy(p.data() + 12, payload, n);
  ++seq_;
  emit_(p);
}

void RtpPacketizer::begin_frame(const mabur::framewire::FrameHdr& h) {
  if (!have_pts_) {
    have_pts_ = true;
    pts64_ = static_cast<int64_t>(h.pts_us);
  } else if (h.flags & mabur::framewire::kFlagDiscont) {
    pts64_ += static_cast<int64_t>(cfg_.nominal_frame_us);
  } else {
    int32_t delta = static_cast<int32_t>(h.pts_us - last_pts_);
    pts64_ += static_cast<int64_t>(delta);
  }
  last_pts_ = h.pts_us;
  ts_ = static_cast<uint32_t>((pts64_ * 9) / 100);

  // Reset NAL/scanner state for the new frame.
  pending_.clear();
  zero_run_ = 0;
  in_nal_ = false;
  fu_started_ = false;
  fu_sent_ = 0;
}

// Emits FU-start/middle packets out of pending_ (bytes at offset
// [2 + fu_sent_, pending_.size())), always retaining at least 1 unsent byte
// (unless force_all, used when closing the NAL) so the closing FU-end is
// never empty. FU payload = 2-byte PayloadHdr + 1-byte FU header + up to
// max_payload-3 NAL bytes. allow_end gates whether the final chunk of a
// force_all drain may set the E bit / marker (false for a truncated frame's
// in-flight NAL, which must close without a fake end).
void RtpPacketizer::drain_fu(bool force_all, bool allow_end) {
  if (pending_.size() < 2) return;  // need the 2-byte NAL header at least
  uint8_t orig0 = pending_[0];
  uint8_t orig1 = pending_[1];
  uint8_t orig_type = static_cast<uint8_t>((orig0 >> 1) & 0x3F);
  uint8_t payload_hdr0 = static_cast<uint8_t>((orig0 & 0x81) | (49 << 1));

  for (;;) {
    size_t avail = pending_.size() - (2 + fu_sent_);  // unsent NAL-body bytes
    size_t chunk_cap = cfg_.max_payload > 3 ? cfg_.max_payload - 3 : 0;
    if (chunk_cap == 0) return;

    size_t reserve = force_all ? 0 : 1;
    if (avail <= reserve) return;  // keep >=1 unsent byte unless forcing all

    size_t take = avail - reserve;
    if (take > chunk_cap) take = chunk_cap;
    if (take == 0) return;
    // MTU-greedy: mid-stream FUs go out only when full; a partial chunk
    // stays pending until more bytes arrive or the NAL closes.
    if (!force_all && take < chunk_cap) return;

    bool is_last_chunk = force_all && (fu_sent_ + take == pending_.size() - 2);
    bool s_bit = !fu_started_;
    bool e_bit = is_last_chunk && allow_end;

    std::vector<uint8_t> buf(3 + take);
    buf[0] = payload_hdr0;
    buf[1] = orig1;
    buf[2] = static_cast<uint8_t>((s_bit ? 0x80 : 0) | (e_bit ? 0x40 : 0) | orig_type);
    std::memcpy(buf.data() + 3, pending_.data() + 2 + fu_sent_, take);

    // Only the true final FU of a COMPLETE frame gets the marker.
    bool marker = e_bit && marker_pending_;

    emit_rtp(buf.data(), buf.size(), marker);
    fu_started_ = true;
    fu_sent_ += take;

    if (!force_all) continue;  // keep draining until reserve stops us, or...
    if (is_last_chunk) return;
  }
}

void RtpPacketizer::close_nal(bool frame_end, bool complete) {
  if (pending_.empty() && !in_nal_) return;
  if (pending_.empty()) { in_nal_ = false; return; }

  // A NAL closed mid-stream by the next start code is genuinely complete
  // (E bit allowed); only a truncated end_frame(false) closing the
  // still-in-flight NAL must suppress the E bit / marker.
  bool truncating = frame_end && !complete;
  bool allow_end = !truncating;
  bool marker = frame_end && complete;

  if (!fu_started_ && pending_.size() <= cfg_.max_payload) {
    emit_rtp(pending_.data(), pending_.size(), marker);
  } else {
    marker_pending_ = marker;
    drain_fu(/*force_all=*/true, allow_end);
  }

  pending_.clear();
  zero_run_ = 0;
  fu_started_ = false;
  fu_sent_ = 0;
  in_nal_ = false;
}

void RtpPacketizer::feed_byte(uint8_t b) {
  if (b == 0x00) {
    if (zero_run_ < 3) {
      ++zero_run_;
    } else {
      // More than 3 leading zeros: the oldest one can't be part of an
      // upcoming start code, so it belongs to the NAL body.
      pending_.push_back(0x00);
    }
    return;
  }
  if (b == 0x01 && zero_run_ >= 2) {
    // Start code found (00 00 01, or 00 00 00 01 when zero_run_ == 3).
    if (in_nal_) close_nal(/*frame_end=*/false, /*complete=*/false);
    zero_run_ = 0;
    pending_.clear();
    fu_started_ = false;
    fu_sent_ = 0;
    in_nal_ = true;
    return;
  }
  // Not a start code: flush any withheld zero bytes into the NAL body, then
  // append this byte.
  for (int i = 0; i < zero_run_; ++i) pending_.push_back(0x00);
  zero_run_ = 0;
  pending_.push_back(b);

  // Streaming FU: while more than max_payload+1 bytes are unsent, emit
  // FU-start/middle fragments, always retaining >=1 unsent byte.
  if (in_nal_ && pending_.size() >= 2) {
    size_t unsent = pending_.size() - (2 + fu_sent_);
    if (unsent > cfg_.max_payload + 1) drain_fu(/*force_all=*/false, /*allow_end=*/false);
  }
}

void RtpPacketizer::data(const uint8_t* p, size_t n) {
  for (size_t i = 0; i < n; ++i) feed_byte(p[i]);
}

void RtpPacketizer::end_frame(bool complete) {
  close_nal(/*frame_end=*/true, complete);
}

}  // namespace maburgs
