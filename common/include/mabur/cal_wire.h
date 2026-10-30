#pragma once
// Calibration frame stamp: the raw (FEC-free) body the drone marks with
// what kind of calibration frame it is and the cell (rate, TXAGC index,
// width) it was sent in, so the GS can attribute a received frame with no
// clock sync and no sequence negotiation. Frame loss IS the measurement
// for Power/Verify, so this format must survive operating points where
// nothing else on the link does; Capacity frames instead carry a body
// sized to the production A-MPDU payload, to measure throughput rather
// than per-cell delivery.
// idx is SIGNED and RELATIVE to the chip's own efuse TXAGC anchor, not an
// absolute TXAGC index -- it lives in the chip's 7-bit signed diff-field
// range, [-64, 63].
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace mabur::cal {

enum class Kind : uint8_t { Power = 1, Verify = 2, Capacity = 3 };

constexpr uint8_t kFlagLdpc = 1;
constexpr uint8_t kFlagStbc = 2;

constexpr size_t kCalPayloadLen = 64;  // power/verify frames
constexpr size_t kCalHeaderLen = 12;   // magic(4) kind rate idx bw flags seq_hi seq(2)
constexpr char kCalMagic[4] = {'M', 'C', 'A', 'L'};

struct CalFrameInfo {
  Kind kind = Kind::Power;
  uint8_t rate = 0;   // HT MCS 0..7
  int8_t idx = 0;     // index RELATIVE to the chip's anchor, [-64, 63]
  uint8_t bw = 20;    // 20 or 40
  uint8_t flags = 0;  // kFlagLdpc | kFlagStbc
  uint32_t seq = 0;   // 24-bit per-cell frame counter, wraps
};

// Payload: magic | kind | rate | idx | bw | flags | seq_hi | seq lo16 LE |
// fill, sized to `len` (64 for power/verify, a production body size for
// capacity). Fill is (0x5A ^ idx) so a payload corruption that survives
// the FCS is caught by parse rather than mis-attributed to a cell. `len`
// must be at least kCalHeaderLen.
inline std::vector<uint8_t> build_cal_payload(Kind kind, uint8_t rate,
                                              int8_t idx, uint8_t bw,
                                              uint8_t flags, uint32_t seq,
                                              size_t len = kCalPayloadLen) {
  std::vector<uint8_t> p(len);
  std::memcpy(p.data(), kCalMagic, 4);
  p[4] = static_cast<uint8_t>(kind);
  p[5] = rate;
  p[6] = static_cast<uint8_t>(idx);
  p[7] = bw;
  p[8] = flags;
  p[9] = static_cast<uint8_t>((seq >> 16) & 0xFF);
  p[10] = static_cast<uint8_t>(seq & 0xFF);
  p[11] = static_cast<uint8_t>((seq >> 8) & 0xFF);
  const uint8_t fill = static_cast<uint8_t>(0x5A ^ static_cast<uint8_t>(idx));
  if (len > kCalHeaderLen)
    std::memset(p.data() + kCalHeaderLen, fill, len - kCalHeaderLen);
  return p;
}

// False on anything that is not an intact calibration payload.
//
// maburgs sets rx.keep_corrupted unconditionally, so CRC-bad frames reach
// this parser. A corrupt frame's idx byte may be garbage, and attributing it
// to a cell would corrupt that cell's delivery ratio -- which is the entire
// measurement. The fill check is what makes that impossible, so it is
// load-bearing here rather than merely defensive.
//
// devourer's RX contract hands consumers the full 802.11 frame INCLUDING the
// trailing 4-byte FCS, and gs/src/radio_frontend.cpp strips only the 24-byte
// dot11 header, so the body may carry a trailing 4-byte FCS or not. `len` is
// not required to match a fixed size (Capacity frames are body-sized, not
// kCalPayloadLen), so both the no-FCS and the FCS-tolerant reading of `len`
// are tried against the fill, in that order.
inline bool parse_cal_payload(const uint8_t* p, size_t len, CalFrameInfo* out) {
  if (len < kCalHeaderLen) return false;
  if (std::memcmp(p, kCalMagic, 4) != 0) return false;
  const uint8_t kind_byte = p[4];
  const uint8_t rate = p[5];
  const int8_t idx = static_cast<int8_t>(p[6]);
  const uint8_t bw = p[7];
  const uint8_t flags = p[8];
  if (kind_byte < 1 || kind_byte > 3) return false;
  if (rate > 7) return false;
  if (idx < -64 || idx > 63) return false;
  if (bw != 20 && bw != 40) return false;
  if (flags & ~static_cast<uint8_t>(kFlagLdpc | kFlagStbc)) return false;

  const uint8_t fill = static_cast<uint8_t>(0x5A ^ p[6]);
  auto fill_ok = [&](size_t total) {
    for (size_t i = kCalHeaderLen; i < total; ++i)
      if (p[i] != fill) return false;
    return true;
  };
  size_t body_len;
  if (fill_ok(len)) {
    body_len = len;
  } else if (len >= kCalHeaderLen + 4 && fill_ok(len - 4)) {
    body_len = len - 4;
  } else {
    return false;
  }
  (void)body_len;

  out->kind = static_cast<Kind>(kind_byte);
  out->rate = rate;
  out->idx = idx;
  out->bw = bw;
  out->flags = flags;
  out->seq = (static_cast<uint32_t>(p[9]) << 16) | static_cast<uint32_t>(p[10]) |
            (static_cast<uint32_t>(p[11]) << 8);
  return true;
}

}  // namespace mabur::cal
