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
constexpr size_t TELEM_LEN = 63;

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
  body.push_back(r.pwr_offset_biased);
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
  r.pwr_offset_biased = buf[16];
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

std::vector<uint8_t> pack_telem(const Telem& t) {
  std::vector<uint8_t> body;
  body.reserve(TELEM_LEN + 2);
  put16(body, RC_MAGIC);
  body.push_back(RC_VERSION);
  body.push_back(T_TELEM);
  body.push_back(t.flags);
  put16(body, t.tlm_seq);
  body.push_back(t.state);
  put32(body, t.generation);
  body.push_back(t.applied_profile);
  body.push_back(t.applied_ov_x100);
  body.push_back(t.applied_off_qdb);
  body.push_back(t.derate_qdb);
  put16(body, t.rcf_age_ms);
  put32(body, t.rcf_rx);
  put32(body, t.enc_frames);
  put32(body, t.enc_kbytes);
  put16(body, t.cmd_kbps);
  body.push_back(t.qp);
  put16(body, t.ring_drops);
  body.push_back(t.txq_depth);
  body.push_back(t.txq_cap);
  put32(body, t.txq_drops);
  put32(body, t.radio_sent);
  put32(body, t.radio_drops);
  put16(body, t.usb_fail);
  body.push_back(t.up_rssi[0]);
  body.push_back(t.up_rssi[1]);
  body.push_back(static_cast<uint8_t>(t.up_snr[0]));
  body.push_back(static_cast<uint8_t>(t.up_snr[1]));
  body.push_back(static_cast<uint8_t>(t.soc_temp_c));
  body.push_back(static_cast<uint8_t>(t.thermal_delta));
  put16(body, t.load_x100);
  put16(body, t.idr_disagree);
  put16(body, t.enhance_disagree);

  put_crc(body);
  return body;
}

std::optional<Telem> parse_telem(const uint8_t* buf, size_t len) {
  if (len < TELEM_LEN + 2) return std::nullopt;
  uint16_t magic = get16(buf, 0);
  uint8_t ver = buf[2];
  uint8_t type = buf[3];
  if (magic != RC_MAGIC || ver != RC_VERSION || type != T_TELEM) return std::nullopt;

  uint16_t crc = get16(buf, TELEM_LEN);
  if (crc != crc16_ccitt(buf, TELEM_LEN)) return std::nullopt;

  Telem t;
  t.flags = buf[4];
  t.tlm_seq = get16(buf, 5);
  t.state = buf[7];
  t.generation = get32(buf, 8);
  t.applied_profile = buf[12];
  t.applied_ov_x100 = buf[13];
  t.applied_off_qdb = buf[14];
  t.derate_qdb = buf[15];
  t.rcf_age_ms = get16(buf, 16);
  t.rcf_rx = get32(buf, 18);
  t.enc_frames = get32(buf, 22);
  t.enc_kbytes = get32(buf, 26);
  t.cmd_kbps = get16(buf, 30);
  t.qp = buf[32];
  t.ring_drops = get16(buf, 33);
  t.txq_depth = buf[35];
  t.txq_cap = buf[36];
  t.txq_drops = get32(buf, 37);
  t.radio_sent = get32(buf, 41);
  t.radio_drops = get32(buf, 45);
  t.usb_fail = get16(buf, 49);
  t.up_rssi[0] = buf[51];
  t.up_rssi[1] = buf[52];
  t.up_snr[0] = static_cast<int8_t>(buf[53]);
  t.up_snr[1] = static_cast<int8_t>(buf[54]);
  t.soc_temp_c = static_cast<int8_t>(buf[55]);
  t.thermal_delta = static_cast<int8_t>(buf[56]);
  t.load_x100 = get16(buf, 57);
  t.idr_disagree = get16(buf, 59);
  t.enhance_disagree = get16(buf, 61);
  return t;
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
