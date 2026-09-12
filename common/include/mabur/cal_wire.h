#pragma once
// Calibration sweep-frame payload: the raw (FEC-free) body the drone stamps
// with the rate and TXAGC index it was sent at, so the GS can attribute a
// received frame to a (rate, idx) cell with no clock sync and no sequence
// negotiation. Frame loss IS the measurement, so this format must survive
// operating points where nothing else on the link does.
// Spec: docs/superpowers/specs/2026-09-10-tx-power-calibration-design.md.
#include <cstdint>
#include <cstring>
#include <vector>

namespace mabur::cal {

constexpr size_t kCalPayloadLen = 64;
constexpr size_t kCalHeaderLen = 9;  // magic(4) | rate | idx | phase | seq(2)
constexpr char kCalMagic[4] = {'M', 'C', 'A', 'L'};

constexpr uint8_t kPhaseCoarse = 1;
constexpr uint8_t kPhaseFine = 2;
constexpr uint8_t kPhaseVerify = 3;

struct CalFrameInfo {
  uint8_t rate = 0;   // HT MCS 0..7
  uint8_t idx = 0;    // TXAGC index 0..127 (Jaguar3 is 7-bit)
  uint8_t phase = 0;  // kPhaseCoarse / kPhaseFine / kPhaseVerify
  uint16_t seq = 0;   // per-cell frame counter, wraps
};

// Payload: magic | rate | idx | phase | seq LE | fill, always 64 B. Fill is
// (0x5A ^ idx) so a payload corruption that survives the FCS is caught by
// parse rather than mis-attributed to a cell.
inline std::vector<uint8_t> build_cal_payload(uint8_t rate, uint8_t idx,
                                              uint8_t phase, uint16_t seq) {
  std::vector<uint8_t> p(kCalPayloadLen);
  std::memcpy(p.data(), kCalMagic, 4);
  p[4] = rate;
  p[5] = idx;
  p[6] = phase;
  p[7] = static_cast<uint8_t>(seq & 0xFF);
  p[8] = static_cast<uint8_t>((seq >> 8) & 0xFF);
  const uint8_t fill = static_cast<uint8_t>(0x5A ^ idx);
  std::memset(p.data() + kCalHeaderLen, fill, kCalPayloadLen - kCalHeaderLen);
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
// dot11 header, so the body may be kCalPayloadLen or kCalPayloadLen + 4.
// Both lengths are accepted; only the first kCalPayloadLen bytes are read.
inline bool parse_cal_payload(const uint8_t* p, size_t len, CalFrameInfo* out) {
  if (len != kCalPayloadLen && len != kCalPayloadLen + 4) return false;
  if (std::memcmp(p, kCalMagic, 4) != 0) return false;
  const uint8_t rate = p[4];
  const uint8_t idx = p[5];
  const uint8_t phase = p[6];
  if (rate > 7) return false;
  if (idx > 127) return false;
  if (phase != kPhaseCoarse && phase != kPhaseFine && phase != kPhaseVerify)
    return false;
  const uint8_t fill = static_cast<uint8_t>(0x5A ^ idx);
  for (size_t i = kCalHeaderLen; i < kCalPayloadLen; ++i)
    if (p[i] != fill) return false;
  out->rate = rate;
  out->idx = idx;
  out->phase = phase;
  out->seq = static_cast<uint16_t>(p[7] | (p[8] << 8));
  return true;
}

}  // namespace mabur::cal
