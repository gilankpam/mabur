#include "mabur/rc_proto.h"
#include <algorithm>
#include <cmath>
#include "mabur/crc16.h"

namespace mabur::rc {
namespace {

void put16(std::vector<uint8_t>& out, uint16_t v) {
  out.push_back(static_cast<uint8_t>(v & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

void put32(std::vector<uint8_t>& out, uint32_t v) {
  out.push_back(static_cast<uint8_t>(v & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}

uint16_t get16(const uint8_t* buf, size_t off) {
  return static_cast<uint16_t>(buf[off] | (static_cast<uint16_t>(buf[off + 1]) << 8));
}

uint32_t get32(const uint8_t* buf, size_t off) {
  return static_cast<uint32_t>(buf[off]) | (static_cast<uint32_t>(buf[off + 1]) << 8) |
         (static_cast<uint32_t>(buf[off + 2]) << 16) | (static_cast<uint32_t>(buf[off + 3]) << 24);
}

void put_crc(std::vector<uint8_t>& body) {
  uint16_t crc = crc16_ccitt(body.data(), body.size());
  put16(body, crc);
}

constexpr size_t RCF_HEAD_LEN = 19;
constexpr size_t DISC_LEN = 21;
constexpr size_t DISC_ACK_LEN = 19;

}  // namespace

std::vector<uint8_t> pack_rcf(const Rcf& r) {
  std::vector<uint8_t> layers;
  layers.reserve(r.layer_delivery.size());
  for (uint8_t x : r.layer_delivery)
    layers.push_back(static_cast<uint8_t>(std::min<int>(100, std::max<int>(0, x))));

  std::vector<uint8_t> body;
  body.reserve(RCF_HEAD_LEN + layers.size() + 2);
  put16(body, RC_MAGIC);
  body.push_back(RC_VERSION);
  body.push_back(T_RCF);
  body.push_back(r.flags);
  put32(body, r.vtx_id);
  put16(body, r.seq);
  put16(body, r.ack_seq);
  body.push_back(r.profile);
  put16(body, r.score);
  body.push_back(r.pwr_idx);
  body.push_back(r.fec_overhead_16ths);
  body.push_back(static_cast<uint8_t>(layers.size()));
  body.insert(body.end(), layers.begin(), layers.end());

  put_crc(body);
  return body;
}

std::optional<Rcf> parse_rcf(const uint8_t* buf, size_t len) {
  if (len < RCF_HEAD_LEN + 2) return std::nullopt;
  uint16_t magic = get16(buf, 0);
  uint8_t ver = buf[2];
  uint8_t type = buf[3];
  if (magic != RC_MAGIC || ver != RC_VERSION || type != T_RCF) return std::nullopt;

  uint8_t n_layers = buf[18];
  size_t body_len = RCF_HEAD_LEN + n_layers;
  if (len < body_len + 2) return std::nullopt;

  uint16_t crc = get16(buf, body_len);
  if (crc != crc16_ccitt(buf, body_len)) return std::nullopt;

  Rcf r;
  r.flags = buf[4];
  r.vtx_id = get32(buf, 5);
  r.seq = get16(buf, 9);
  r.ack_seq = get16(buf, 11);
  r.profile = buf[13];
  r.score = get16(buf, 14);
  r.pwr_idx = buf[16];
  r.fec_overhead_16ths = buf[17];
  r.layer_delivery.assign(buf + RCF_HEAD_LEN, buf + RCF_HEAD_LEN + n_layers);
  return r;
}

std::vector<uint8_t> pack_disc(const Disc& d) {
  std::vector<uint8_t> body;
  body.reserve(DISC_LEN + 2);
  put16(body, RC_MAGIC);
  body.push_back(RC_VERSION);
  body.push_back(T_DISC);
  body.push_back(F_DISCOVERY);
  put32(body, d.vtx_id);
  put32(body, d.vrx_nonce);
  body.push_back(d.op_channel);
  body.push_back(d.op_width);
  body.push_back(d.table_ver);
  body.push_back(d.init_profile);
  put16(body, d.cap_bits);
  put16(body, d.seq);

  put_crc(body);
  return body;
}

std::optional<Disc> parse_disc(const uint8_t* buf, size_t len) {
  if (len < DISC_LEN + 2) return std::nullopt;
  uint16_t magic = get16(buf, 0);
  uint8_t ver = buf[2];
  uint8_t type = buf[3];
  if (magic != RC_MAGIC || ver != RC_VERSION || type != T_DISC) return std::nullopt;

  uint16_t crc = get16(buf, DISC_LEN);
  if (crc != crc16_ccitt(buf, DISC_LEN)) return std::nullopt;

  Disc d;
  d.vtx_id = get32(buf, 5);
  d.vrx_nonce = get32(buf, 9);
  d.op_channel = buf[13];
  d.op_width = buf[14];
  d.table_ver = buf[15];
  d.init_profile = buf[16];
  d.cap_bits = get16(buf, 17);
  d.seq = get16(buf, 19);
  return d;
}

std::vector<uint8_t> pack_disc_ack(const DiscAck& a) {
  std::vector<uint8_t> body;
  body.reserve(DISC_ACK_LEN + 2);
  put16(body, RC_MAGIC);
  body.push_back(RC_VERSION);
  body.push_back(T_DISC_ACK);
  body.push_back(F_DISCOVERY);
  put32(body, a.vtx_id);
  put32(body, a.vrx_nonce);
  put16(body, a.chip_caps);
  body.push_back(a.agreed_channel);
  body.push_back(a.agreed_width);
  put16(body, a.seq);

  put_crc(body);
  return body;
}

std::optional<DiscAck> parse_disc_ack(const uint8_t* buf, size_t len) {
  if (len < DISC_ACK_LEN + 2) return std::nullopt;
  uint16_t magic = get16(buf, 0);
  uint8_t ver = buf[2];
  uint8_t type = buf[3];
  if (magic != RC_MAGIC || ver != RC_VERSION || type != T_DISC_ACK) return std::nullopt;

  uint16_t crc = get16(buf, DISC_ACK_LEN);
  if (crc != crc16_ccitt(buf, DISC_ACK_LEN)) return std::nullopt;

  DiscAck a;
  a.vtx_id = get32(buf, 5);
  a.vrx_nonce = get32(buf, 9);
  a.chip_caps = get16(buf, 13);
  a.agreed_channel = buf[15];
  a.agreed_width = buf[16];
  a.seq = get16(buf, 17);
  return a;
}

int frame_type(const uint8_t* buf, size_t len) {
  if (len < 4) return -1;
  uint16_t magic = get16(buf, 0);
  uint8_t ver = buf[2];
  if (magic != RC_MAGIC || ver != RC_VERSION) return -1;
  return buf[3];
}

uint8_t overhead_to_16ths(double ov) {
  double n = std::round(ov * 16.0);
  if (n < 1.0) n = 1.0;
  if (n > 16.0) n = 16.0;
  return static_cast<uint8_t>(n);
}

}  // namespace mabur::rc
