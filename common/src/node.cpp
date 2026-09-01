#include "mabur/node.h"

#include "mabur/crc16.h"

namespace mabur::node {
namespace {
constexpr size_t kRxBodyHdr = 21;
constexpr size_t kCardStatusLen = 24 + 2;

void put_u16(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>(x >> 8));
}
void put_u32(std::vector<uint8_t>& v, uint32_t x) {
  for (int i = 0; i < 4; ++i) v.push_back(static_cast<uint8_t>(x >> (8 * i)));
}
void put_u64(std::vector<uint8_t>& v, uint64_t x) {
  for (int i = 0; i < 8; ++i) v.push_back(static_cast<uint8_t>(x >> (8 * i)));
}
uint16_t rd_u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
uint32_t rd_u32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
uint64_t rd_u64(const uint8_t* p) {
  uint64_t x = 0;
  for (int i = 7; i >= 0; --i) x = (x << 8) | p[i];
  return x;
}
}  // namespace

std::vector<uint8_t> pack_rx_body(const RxBody& m) {
  std::vector<uint8_t> v;
  v.reserve(kRxBodyHdr + m.body.size() + 2);
  put_u16(v, RXBODY_MAGIC);
  v.push_back(NODE_VERSION);
  v.push_back(m.card_id);
  put_u64(v, m.mono_us);
  v.push_back(m.rssi[0]);
  v.push_back(m.rssi[1]);
  v.push_back(static_cast<uint8_t>(m.snr[0]));
  v.push_back(static_cast<uint8_t>(m.snr[1]));
  v.push_back(static_cast<uint8_t>((m.crc_ok ? 1 : 0) | (m.phy_valid ? 2 : 0)));
  put_u16(v, m.mac_seq);
  put_u16(v, static_cast<uint16_t>(m.body.size()));
  v.insert(v.end(), m.body.begin(), m.body.end());
  put_u16(v, crc16_ccitt(v.data(), v.size()));
  return v;
}

std::optional<RxBody> parse_rx_body(const uint8_t* buf, size_t len) {
  if (len < kRxBodyHdr + 2 || rd_u16(buf) != RXBODY_MAGIC || buf[2] != NODE_VERSION)
    return std::nullopt;
  const size_t body_len = rd_u16(buf + 19);
  if (len != kRxBodyHdr + body_len + 2) return std::nullopt;
  if (crc16_ccitt(buf, len - 2) != rd_u16(buf + len - 2)) return std::nullopt;
  RxBody m;
  m.card_id = buf[3];
  m.mono_us = rd_u64(buf + 4);
  m.rssi[0] = buf[12];
  m.rssi[1] = buf[13];
  m.snr[0] = static_cast<int8_t>(buf[14]);
  m.snr[1] = static_cast<int8_t>(buf[15]);
  m.crc_ok = (buf[16] & 1) != 0;
  m.phy_valid = (buf[16] & 2) != 0;
  m.mac_seq = rd_u16(buf + 17);
  m.body.assign(buf + kRxBodyHdr, buf + kRxBodyHdr + body_len);
  return m;
}

std::vector<uint8_t> pack_card_status(const CardStatus& m) {
  std::vector<uint8_t> v;
  v.reserve(kCardStatusLen);
  put_u16(v, CARDSTATUS_MAGIC);
  v.push_back(NODE_VERSION);
  v.push_back(m.card_id);
  put_u64(v, m.mono_us);
  put_u32(v, m.frames_seen);
  put_u32(v, m.crc_fail);
  put_u32(v, m.uptime_s);
  put_u16(v, crc16_ccitt(v.data(), v.size()));
  return v;
}

std::optional<CardStatus> parse_card_status(const uint8_t* buf, size_t len) {
  if (len != kCardStatusLen || rd_u16(buf) != CARDSTATUS_MAGIC ||
      buf[2] != NODE_VERSION)
    return std::nullopt;
  if (crc16_ccitt(buf, len - 2) != rd_u16(buf + len - 2)) return std::nullopt;
  CardStatus m;
  m.card_id = buf[3];
  m.mono_us = rd_u64(buf + 4);
  m.frames_seen = rd_u32(buf + 12);
  m.crc_fail = rd_u32(buf + 16);
  m.uptime_s = rd_u32(buf + 20);
  return m;
}

}  // namespace mabur::node
