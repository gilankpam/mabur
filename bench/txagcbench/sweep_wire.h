#pragma once
// txagcbench wire format: the raw (FEC-free) sweep frame payload TX stamps
// with the TXAGC index it was sent at, so RX can attribute per-frame RSSI
// to an index with no clock sync. Spec:
// docs/superpowers/specs/2026-07-16-txagcbench-design.md.
#include <cstdint>
#include <cstring>
#include <vector>

namespace txagcbench {

constexpr size_t kDot11HeaderLen = 24;
constexpr size_t kSweepPayloadLen = 64;
constexpr size_t kSweepHeaderLen = 9;  // "TAGC" + idx + pass + seq(2) + mcs
constexpr char kMagic[4] = {'T', 'A', 'G', 'C'};

struct SweepInfo {
  uint8_t idx = 0;
  uint8_t pass = 0;   // 1 = ascending sweep, 2 = descending
  uint16_t seq = 0;   // global frame counter (debug aid, wraps)
  uint8_t mcs = 0;
};

// Payload: magic | idx | pass | seq LE | mcs | fill, always 64 B. Fill is
// (0x5A ^ idx) so payload corruption that survives the FCS is caught by
// parse rather than mis-attributed to an index.
inline std::vector<uint8_t> build_sweep_payload(uint8_t idx, uint8_t pass,
                                                uint16_t seq, uint8_t mcs) {
  std::vector<uint8_t> p(kSweepPayloadLen);
  std::memcpy(p.data(), kMagic, 4);
  p[4] = idx;
  p[5] = pass;
  p[6] = static_cast<uint8_t>(seq & 0xFF);
  p[7] = static_cast<uint8_t>((seq >> 8) & 0xFF);
  p[8] = mcs;
  const uint8_t fill = static_cast<uint8_t>(0x5A ^ idx);
  std::memset(p.data() + kSweepHeaderLen, fill,
              kSweepPayloadLen - kSweepHeaderLen);
  return p;
}

// False on anything that is not an intact sweep payload (short/long body,
// wrong magic, idx out of TXAGC range, fill mismatch).
//
// Devourer's RX contract hands consumers the full 802.11 frame INCLUDING
// the trailing 4-byte FCS; protocol boundaries strip it themselves.
// maburgs::RadioFrontend::on_packet (gs/src/radio_frontend.cpp)
// strips only the 24-byte dot11 header, so the body handed to us here may be
// either the bare kSweepPayloadLen-byte payload or that plus the 4-byte FCS.
// Accept both lengths; only the first kSweepPayloadLen bytes are validated.
inline bool parse_sweep_payload(const uint8_t* p, size_t len, SweepInfo* out) {
  if (len != kSweepPayloadLen && len != kSweepPayloadLen + 4) return false;
  if (std::memcmp(p, kMagic, 4) != 0) return false;
  out->idx = p[4];
  out->pass = p[5];
  out->seq = static_cast<uint16_t>(p[6] | (p[7] << 8));
  out->mcs = p[8];
  if (out->idx > 127) return false;  // Jaguar3 TXAGC is 7-bit (0..127)
  const uint8_t fill = static_cast<uint8_t>(0x5A ^ out->idx);
  for (size_t i = kSweepHeaderLen; i < kSweepPayloadLen; ++i)
    if (p[i] != fill) return false;
  return true;
}

// Canonical probe-req dot11 header (fourth intentional copy of these fixed
// bytes — see bench/linkbench/bench_wire.h for why copies beat a shared
// header coupling the daemons to the benches).
inline std::vector<uint8_t> build_dot11_header(uint16_t seq) {
  static constexpr uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  std::vector<uint8_t> h(kDot11HeaderLen, 0);
  h[0] = 0x40;
  std::memset(h.data() + 4, 0xff, 6);
  std::memcpy(h.data() + 10, kSa, 6);
  std::memcpy(h.data() + 16, kSa, 6);
  const uint16_t seq_ctl = static_cast<uint16_t>(seq << 4);
  h[22] = static_cast<uint8_t>(seq_ctl & 0xff);
  h[23] = static_cast<uint8_t>((seq_ctl >> 8) & 0xff);
  return h;
}

}  // namespace txagcbench
