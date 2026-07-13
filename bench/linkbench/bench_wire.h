#pragma once
// Bench wire format: the app-level test packet linkbench-tx feeds into
// mabur's RS encoder and linkbench-rx verifies after decode, plus the same
// canonical dot11 header maburd/maburgs use (mirrored locals there; kept
// local here too — three copies of 15 fixed bytes beat a shared header that
// couples the daemons to the bench).
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace linkbench {

// SBI stream id for the bench stream — distinct from UEP layers 0..3 so a
// stray maburd/maburgs on the same channel ignores bench traffic and
// vice versa.
constexpr uint8_t kBenchStreamId = 0xB0;
constexpr size_t kDot11HeaderLen = 24;

// Bench app packet: u32 seq LE | u16 len LE | fill, where every fill byte
// is (0xA5 ^ low byte of seq). len is the TOTAL packet length; RX checks it
// against the decoded packet size and verifies the fill, so any FEC/decode
// corruption is caught, not just sequence gaps.
constexpr size_t kBenchPktHeader = 6;

inline std::vector<uint8_t> build_bench_packet(uint32_t seq, size_t len) {
  if (len < kBenchPktHeader) len = kBenchPktHeader;
  std::vector<uint8_t> p(len);
  p[0] = static_cast<uint8_t>(seq & 0xFF);
  p[1] = static_cast<uint8_t>((seq >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((seq >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((seq >> 24) & 0xFF);
  p[4] = static_cast<uint8_t>(len & 0xFF);
  p[5] = static_cast<uint8_t>((len >> 8) & 0xFF);
  const uint8_t fill = static_cast<uint8_t>(0xA5 ^ (seq & 0xFF));
  std::memset(p.data() + kBenchPktHeader, fill, len - kBenchPktHeader);
  return p;
}

// Returns false when the buffer cannot be a bench packet (short, or the
// embedded len doesn't match). On true, *seq is set and *pattern_ok reports
// whether every fill byte matched.
inline bool parse_bench_packet(const uint8_t* p, size_t len, uint32_t* seq,
                               bool* pattern_ok) {
  if (len < kBenchPktHeader) return false;
  const uint16_t embedded = static_cast<uint16_t>(p[4] | (p[5] << 8));
  if (embedded != len) return false;
  *seq = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  const uint8_t fill = static_cast<uint8_t>(0xA5 ^ (*seq & 0xFF));
  bool ok = true;
  for (size_t i = kBenchPktHeader; i < len; ++i)
    if (p[i] != fill) { ok = false; break; }
  *pattern_ok = ok;
  return true;
}

// Canonical probe-req dot11 header (mirrors drone/src/main.cpp
// build_dot11_header and gs/src/radio_frontend.cpp build_control_frame).
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

// "8M" / "1.5M" / "800k" / "12345" → bits per second; 0 on any parse error.
inline uint64_t parse_rate_bps(const std::string& s) {
  if (s.empty()) return 0;
  double mult = 1.0;
  std::string num = s;
  const char last = s.back();
  if (last == 'M' || last == 'm') { mult = 1e6; num = s.substr(0, s.size() - 1); }
  else if (last == 'K' || last == 'k') { mult = 1e3; num = s.substr(0, s.size() - 1); }
  if (num.empty()) return 0;
  char* end = nullptr;
  const double v = std::strtod(num.c_str(), &end);
  if (end == nullptr || *end != '\0' || v <= 0) return 0;
  return static_cast<uint64_t>(v * mult);
}

}  // namespace linkbench
