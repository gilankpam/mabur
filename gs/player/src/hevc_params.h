#ifndef MABUR_PLAYER_HEVC_PARAMS_H_
#define MABUR_PLAYER_HEVC_PARAMS_H_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace maburplay {

// A NAL unit found inside an Annex-B access unit. `p`/`n` bound the bytes
// starting at the 2-byte NAL header (the start code itself is excluded);
// `type` is the HEVC NAL unit type, byte0 bits [6:1]: (p[0] >> 1) & 0x3F.
struct NalView {
  const uint8_t* p;
  size_t n;
  uint8_t type;
};

// Splits an Annex-B access unit into NAL views at 00 00 01 / 00 00 00 01
// start codes. Mirrors the zero-run scanning style of RtpPacketizer
// (gs/src/rtp_packetizer.cpp) but operates over a whole buffer rather than
// a byte stream. A start code with nothing after it before the next start
// code (or the buffer end) yields no NalView for that gap — an empty NAL
// is skipped, not emitted as a zero-length view.
std::vector<NalView> split_nals(const uint8_t* au, size_t n);

// True if any NAL in the AU has a type in the IRAP range [16, 23]
// (BLA_W_LP .. RSV_IRAP_VCL23; IDR_W_RADL == 19 is the common case).
bool au_is_irap(const uint8_t* au, size_t n);

// Harvests VPS(32)/SPS(33)/PPS(34) NAL units from Annex-B access units.
// Some encoders split the three parameter sets across separate AUs (e.g.
// VPS+SPS in one, PPS in the next), so this accumulates across feed()
// calls rather than requiring them all in one AU.
class HevcParams {
 public:
  // Feeds one AU; returns complete().
  bool feed(const uint8_t* au, size_t n);
  bool complete() const { return !vps_.empty() && !sps_.empty() && !pps_.empty(); }

  // Assembles an ISO/IEC 14496-15 HEVCDecoderConfigurationRecord (hvcC).
  // complete() must be true; asserts otherwise.
  std::vector<uint8_t> hvcc() const;

  const std::vector<uint8_t>& vps() const { return vps_; }
  const std::vector<uint8_t>& sps() const { return sps_; }
  const std::vector<uint8_t>& pps() const { return pps_; }

 private:
  std::vector<uint8_t> vps_, sps_, pps_;
};

// Annex-B (start codes) -> 4-byte big-endian length-prefixed (AVCC-style)
// conversion, for fMP4 samples.
std::vector<uint8_t> annexb_to_length_prefixed(const uint8_t* au, size_t n);

}  // namespace maburplay

#endif  // MABUR_PLAYER_HEVC_PARAMS_H_
