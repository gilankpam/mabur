#pragma once
// Calibration frame stamp: the raw (FEC-free) body a capacity sweep's
// frames carry, marked with the kind and the point (rate, width) they were
// sent at, so the GS can attribute a received frame with no clock sync and
// no sequence negotiation. The body is sized to the production A-MPDU
// payload, so a point measures throughput at the size video flies at.
// idx is SIGNED and RELATIVE to the chip's own efuse TXAGC anchor, in the
// chip's 7-bit signed diff-field range [-64, 63]; capacity frames carry 0.
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace mabur::cal {

// Wall is measured on the drone itself and never airs a stamped frame;
// only Capacity frames carry this stamp.
enum class Kind : uint8_t { Wall = 1, Capacity = 3 };

constexpr uint8_t kFlagLdpc = 1;
constexpr uint8_t kFlagStbc = 2;

constexpr size_t kCalHeaderLen = 12;   // magic(4) kind rate idx bw flags seq_hi seq(2)
constexpr char kCalMagic[4] = {'M', 'C', 'A', 'L'};

struct CalFrameInfo {
  Kind kind = Kind::Capacity;
  uint8_t rate = 0;   // HT MCS 0..7
  int8_t idx = 0;     // index RELATIVE to the chip's anchor, [-64, 63]
  uint8_t bw = 20;    // 20 or 40
  uint8_t flags = 0;  // kFlagLdpc | kFlagStbc
  uint32_t seq = 0;   // 24-bit per-cell frame counter, wraps
  size_t len = 0;     // payload bytes the fill matched, any trailing FCS excluded
};

// Payload: magic | kind | rate | idx | bw | flags | seq_hi | seq lo16 LE |
// fill, sized to `len` (a production body size). Fill is (0x5A ^ idx) so a payload corruption that survives
// the FCS is caught by parse rather than mis-attributed to a cell. `len`
// must be at least kCalHeaderLen.
inline std::vector<uint8_t> build_cal_payload(Kind kind, uint8_t rate,
                                              int8_t idx, uint8_t bw,
                                              uint8_t flags, uint32_t seq,
                                              size_t len) {
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
// to a point would corrupt that point's delivery count -- which is the
// measurement. The fill check is what makes that impossible, so it is
// load-bearing here rather than merely defensive.
//
// devourer's RX contract hands consumers the full 802.11 frame INCLUDING the
// trailing 4-byte FCS, and gs/src/radio_frontend.cpp strips only the 24-byte
// dot11 header, so the body may carry a trailing 4-byte FCS or not. `len` is
// not required to match a fixed size (frames are body-sized), so both the no-FCS and the FCS-tolerant reading of `len`
// are tried against the fill, in that order; `out->len` is the one that
// matched, so a caller sizing the frame on the air counts the FCS once.
inline bool parse_cal_payload(const uint8_t* p, size_t len, CalFrameInfo* out) {
  if (len < kCalHeaderLen) return false;
  if (std::memcmp(p, kCalMagic, 4) != 0) return false;
  const uint8_t kind_byte = p[4];
  const uint8_t rate = p[5];
  const int8_t idx = static_cast<int8_t>(p[6]);
  const uint8_t bw = p[7];
  const uint8_t flags = p[8];
  if (kind_byte != static_cast<uint8_t>(Kind::Capacity)) return false;
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

  out->kind = static_cast<Kind>(kind_byte);
  out->rate = rate;
  out->idx = idx;
  out->bw = bw;
  out->flags = flags;
  out->seq = (static_cast<uint32_t>(p[9]) << 16) | static_cast<uint32_t>(p[10]) |
            (static_cast<uint32_t>(p[11]) << 8);
  out->len = body_len;
  return true;
}

}  // namespace mabur::cal
