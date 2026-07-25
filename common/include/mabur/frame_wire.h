#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>

namespace mabur::framewire {

// Per-frame wire header prepended (by maburd) to the raw Annex-B frame
// before fragmentation, replacing the 8-byte VencFrameMeta from waybeam's
// frame ring byte-for-byte in the hot-thread buffer. All fields LE.
// frame_id is a GLOBAL counter across all UEP layers — the GS orders
// access units across layers by it (the job waybeam's RTP seq used to do).
inline constexpr size_t kFrameHdrLen = 8;
inline constexpr uint8_t kFlagIdr = 0x01;
inline constexpr uint8_t kFlagDiscont = 0x02;  // pts/frame_id re-base point
inline constexpr uint8_t kCodecH265 = 0x01;    // mirrors VENC_FRAME_CODEC_H265

struct FrameHdr {
  uint16_t frame_id = 0;
  uint8_t flags = 0;
  uint8_t codec = kCodecH265;
  uint32_t pts_us = 0;  // capture time, µs, u32-truncated (from VencFrameMeta)
};

void pack_frame_hdr(const FrameHdr& h, uint8_t out[kFrameHdrLen]);
std::optional<FrameHdr> parse_frame_hdr(const uint8_t* buf, size_t len);

}  // namespace mabur::framewire
