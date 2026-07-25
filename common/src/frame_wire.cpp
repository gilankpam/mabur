#include "mabur/frame_wire.h"

namespace mabur::framewire {

void pack_frame_hdr(const FrameHdr& h, uint8_t out[kFrameHdrLen]) {
  out[0] = static_cast<uint8_t>(h.frame_id & 0xFF);
  out[1] = static_cast<uint8_t>(h.frame_id >> 8);
  out[2] = h.flags;
  out[3] = h.codec;
  out[4] = static_cast<uint8_t>(h.pts_us & 0xFF);
  out[5] = static_cast<uint8_t>((h.pts_us >> 8) & 0xFF);
  out[6] = static_cast<uint8_t>((h.pts_us >> 16) & 0xFF);
  out[7] = static_cast<uint8_t>((h.pts_us >> 24) & 0xFF);
}

std::optional<FrameHdr> parse_frame_hdr(const uint8_t* buf, size_t len) {
  if (!buf || len < kFrameHdrLen) return std::nullopt;
  FrameHdr h;
  h.frame_id = static_cast<uint16_t>(buf[0] | (buf[1] << 8));
  h.flags = buf[2];
  h.codec = buf[3];
  h.pts_us = static_cast<uint32_t>(buf[4]) | (static_cast<uint32_t>(buf[5]) << 8) |
             (static_cast<uint32_t>(buf[6]) << 16) |
             (static_cast<uint32_t>(buf[7]) << 24);
  return h;
}

}  // namespace mabur::framewire
