#include "relay_wire.h"

namespace maburgs::relay {
namespace {
uint16_t get16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t get32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
std::vector<uint8_t> hdr(Type t, size_t extra) {
  std::vector<uint8_t> v;
  v.reserve(kHdrLen + extra);
  v.push_back(kMagic & 0xFF);
  v.push_back(kMagic >> 8);
  v.push_back(kVer);
  v.push_back(t);
  return v;
}
}  // namespace

std::vector<uint8_t> pack_hello() { return hdr(kHello, 0); }

std::vector<uint8_t> pack_tune(uint16_t tune_id, uint8_t channel, uint8_t sec) {
  auto v = hdr(kTune, 4);
  v.push_back(tune_id & 0xFF);
  v.push_back(tune_id >> 8);
  v.push_back(channel);
  v.push_back(sec);
  return v;
}

std::vector<uint8_t> pack_tx(uint8_t mcs, uint8_t flags, const uint8_t* dot11, size_t len) {
  auto v = hdr(kTx, 2 + len);
  v.push_back(mcs);
  v.push_back(flags);
  v.insert(v.end(), dot11, dot11 + len);
  return v;
}

int msg_type(const uint8_t* b, size_t n) {
  if (n < kHdrLen || get16(b) != kMagic || b[2] != kVer || b[3] < kFrame || b[3] > kTx) return -1;
  return b[3];
}

bool parse_frame(const uint8_t* b, size_t n, FrameMeta& m, const uint8_t*& dot11,
                 size_t& dot11_len) {
  if (msg_type(b, n) != kFrame || n < kFrameHdrLen) return false;
  m.seq = get32(b + 4);
  m.rx_channel = b[8];
  m.sec = b[9];
  m.flags = b[10];
  m.mcs = b[11];
  m.rssi[0] = static_cast<int8_t>(b[12]);
  m.rssi[1] = static_cast<int8_t>(b[13]);
  m.noise[0] = static_cast<int8_t>(b[14]);
  m.noise[1] = static_cast<int8_t>(b[15]);
  m.tsf_lo = get32(b + 16);
  dot11 = b + kFrameHdrLen;
  dot11_len = n - kFrameHdrLen;
  return true;
}

bool parse_status(const uint8_t* b, size_t n, Status& s) {
  if (msg_type(b, n) != kStatus || n < kStatusLen) return false;
  s.tune_id = get16(b + 4);
  s.state = b[6];
  s.channel = b[7];
  s.sec = b[8];
  s.owner = b[9];
  s.you_own = b[10];
  s.rx = get32(b + 11);
  s.fwd = get32(b + 15);
  s.foreign = get32(b + 19);
  s.bad_fcs = get32(b + 23);
  s.your_drops = get32(b + 27);
  s.uptime_s = get32(b + 31);
  s.tx = get32(b + 35);
  s.tx_fail = get32(b + 39);
  s.tx_refused = get32(b + 43);
  return true;
}
}  // namespace maburgs::relay
