#pragma once
// The one place a control-path frame is assembled: radiotap, then the 24-byte
// 802.11 management header, then the body. DISC_ACK, Telem, MSP, the MAVLink
// uplink and calibration replies all go through build_control_frame, so their
// framing cannot drift apart; the GS parses the same header.
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace mabur {

inline constexpr uint8_t kCanonicalSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
inline constexpr size_t kDot11HeaderLen = 24;

inline std::vector<uint8_t> build_dot11_header(uint16_t seq) {
  std::vector<uint8_t> h(kDot11HeaderLen, 0);
  h[0] = 0x40;
  h[1] = 0x00;
  h[2] = 0x00;
  h[3] = 0x00;
  std::memset(h.data() + 4, 0xff, 6);
  std::memcpy(h.data() + 10, kCanonicalSa, 6);
  std::memcpy(h.data() + 16, kCanonicalSa, 6);
  uint16_t seq_ctl = static_cast<uint16_t>(seq << 4);
  h[22] = static_cast<uint8_t>(seq_ctl & 0xff);
  h[23] = static_cast<uint8_t>((seq_ctl >> 8) & 0xff);
  return h;
}

inline std::vector<uint8_t> build_control_frame(const std::vector<uint8_t>& radiotap,
                                                uint16_t dot11_seq,
                                                const uint8_t* body, size_t n) {
  std::vector<uint8_t> frame;
  frame.reserve(radiotap.size() + kDot11HeaderLen + n);
  frame.insert(frame.end(), radiotap.begin(), radiotap.end());
  const auto hdr = build_dot11_header(dot11_seq);
  frame.insert(frame.end(), hdr.begin(), hdr.end());
  frame.insert(frame.end(), body, body + n);
  return frame;
}

}  // namespace mabur
