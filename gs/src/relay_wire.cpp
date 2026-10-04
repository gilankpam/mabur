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

std::vector<uint8_t> pack_scan(uint16_t scan_id, uint8_t passes, uint8_t observe_ms,
                               const std::vector<uint8_t>& ch) {
  auto v = hdr(kScan, 5 + ch.size());
  v.push_back(scan_id & 0xFF);
  v.push_back(scan_id >> 8);
  v.push_back(passes);
  v.push_back(observe_ms);
  v.push_back(static_cast<uint8_t>(ch.size()));
  v.insert(v.end(), ch.begin(), ch.end());
  return v;
}

int msg_type(const uint8_t* b, size_t n) {
  if (n < kHdrLen || get16(b) != kMagic || b[2] != kVer || b[3] < kFrame || b[3] > kScanResult) return -1;
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
  s.tx_scan_drop = get32(b + 47);
  return true;
}

bool parse_survey(const uint8_t* b, size_t n, Survey& s) {
  if (msg_type(b, n) != kSurvey || n < kSurveyLen) return false;
  s.channel = b[4]; s.sec = b[5]; s.gen = get16(b + 6);
  s.active_ms = get32(b + 8); s.busy_ms = get32(b + 12); s.rx_ms = get32(b + 16);
  s.tx_ms = get32(b + 20); s.ofdm_err = get32(b + 24); s.foreign = get32(b + 28);
  return true;
}

bool parse_scan_result(const uint8_t* b, size_t n, SweepResult& r) {
  if (msg_type(b, n) != kScanResult || n < kScanResultHdrLen) return false;
  const size_t cnt = b[9];
  if (n < kScanResultHdrLen + cnt * kScanEntryLen) return false;
  r.scan_id = get16(b + 4); r.status = b[6]; r.back_channel = b[7]; r.back_sec = b[8];
  r.entries.clear();
  r.entries.reserve(cnt);
  for (size_t i = 0; i < cnt; ++i) {
    const uint8_t* p = b + kScanResultHdrLen + i * kScanEntryLen;
    SweepEntry e;
    e.ch = p[0]; e.pass = p[1]; e.valid = p[2] != 0;
    e.active_ms = get16(p + 3); e.busy_ms = get16(p + 5); e.rx_ms = get16(p + 7);
    e.foreign = get16(p + 9); e.ofdm_err = get16(p + 11);
    r.entries.push_back(e);
  }
  return true;
}
}  // namespace maburgs::relay
