#include "mabur/rc_proto.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include "mabur/crc16.h"

namespace mabur::rc {
namespace {

template <typename T, typename V>
T saturate(V v) {
  constexpr V lo = static_cast<V>(std::numeric_limits<T>::min());
  constexpr V hi = static_cast<V>(std::numeric_limits<T>::max());
  if (v < lo) return std::numeric_limits<T>::min();
  if (v > hi) return std::numeric_limits<T>::max();
  return static_cast<T>(v);
}

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

void put64(std::vector<uint8_t>& out, uint64_t v) {
  put32(out, static_cast<uint32_t>(v & 0xFFFFFFFFu));
  put32(out, static_cast<uint32_t>(v >> 32));
}

uint64_t get64(const uint8_t* buf, size_t off) {
  return static_cast<uint64_t>(get32(buf, off)) |
         (static_cast<uint64_t>(get32(buf, off + 4)) << 32);
}

void put_crc(std::vector<uint8_t>& body) {
  uint16_t crc = crc16_ccitt(body.data(), body.size());
  put16(body, crc);
}

constexpr size_t RCF_HEAD_LEN = 15;
constexpr size_t DISC_LEN = 21;
constexpr size_t DISC_ACK_LEN = 19;
constexpr size_t TELEM_LEN = 88;  // 2026-09-10: +cal_base_ref_idx (u8)

// magic(2) | ver | type | flags | vtx(4) | nonce(4) | phase | fpc(2) |
// settle(2) | gap(2) | n_windows(1) | n * 4 bytes
//
// Offsets: hdr 0..4, vtx 5..8, nonce 9..12, phase 13, fpc 14..15,
// settle 16..17, gap 18..19, n_windows 20. The constant INCLUDES the
// n_windows byte, so buf[kCalCmdFixedLen - 1] IS n_windows and the windows
// array starts at kCalCmdFixedLen. tests/test_rc.cpp hard-codes 20 for the
// same byte -- the two must agree.
constexpr size_t kCalCmdFixedLen = 5 + 4 + 4 + 1 + 2 + 2 + 2 + 1;  // 21
// magic(2) | ver | type | flags | vtx(4) | nonce(4) | walls(8*2) |
// legacy(2) | flags32(4)
constexpr size_t kCalResultLen = 5 + 4 + 4 + 16 + 2 + 4;

}  // namespace

std::vector<uint8_t> pack_rcf(const Rcf& r) {
  std::vector<uint8_t> body;
  body.reserve(RCF_HEAD_LEN + 2);
  put16(body, RC_MAGIC);
  body.push_back(RC_VERSION);
  body.push_back(T_RCF);
  body.push_back(0);  // flags: nothing
  put32(body, r.vtx_id);
  put16(body, r.seq);
  body.push_back(r.profile);
  body.push_back(overhead_to_x100(r.fec_overhead_base));
  body.push_back(overhead_to_x100(r.fec_overhead_enh));
  body.push_back(r.probe_profile);
  put_crc(body);
  return body;
}

std::optional<Rcf> parse_rcf(const uint8_t* buf, size_t len) {
  if (len < RCF_HEAD_LEN + 2) return std::nullopt;
  if (get16(buf, 0) != RC_MAGIC || buf[2] != RC_VERSION || buf[3] != T_RCF)
    return std::nullopt;
  if (get16(buf, RCF_HEAD_LEN) != crc16_ccitt(buf, RCF_HEAD_LEN)) return std::nullopt;
  Rcf r;
  r.vtx_id = get32(buf, 5);
  r.seq = get16(buf, 9);
  r.profile = buf[11];
  r.fec_overhead_base = buf[12] / 100.0;
  r.fec_overhead_enh = buf[13] / 100.0;
  r.probe_profile = buf[14];
  return r;
}

std::vector<uint8_t> pack_cal_cmd(const CalCmd& c) {
  std::vector<uint8_t> body;
  const size_t n = c.windows.size() > kMaxCalWindows ? kMaxCalWindows
                                                     : c.windows.size();
  body.reserve(kCalCmdFixedLen + n * 4 + 2);
  put16(body, RC_MAGIC);
  body.push_back(RC_VERSION);
  body.push_back(T_CAL_CMD);
  body.push_back(0);  // flags: nothing
  put32(body, c.vtx_id);
  put32(body, c.nonce);
  body.push_back(c.phase);
  put16(body, c.frames_per_cell);
  put16(body, c.settle_ms);
  put16(body, c.gap_us);
  body.push_back(static_cast<uint8_t>(n));
  for (size_t i = 0; i < n; ++i) {
    body.push_back(c.windows[i].rate);
    body.push_back(c.windows[i].idx_lo);
    body.push_back(c.windows[i].idx_hi);
    body.push_back(c.windows[i].idx_step);
  }
  put_crc(body);
  return body;
}

std::optional<CalCmd> parse_cal_cmd(const uint8_t* buf, size_t len) {
  if (len < kCalCmdFixedLen + 2) return std::nullopt;
  if (get16(buf, 0) != RC_MAGIC || buf[2] != RC_VERSION || buf[3] != T_CAL_CMD)
    return std::nullopt;
  const uint8_t n = buf[kCalCmdFixedLen - 1];
  if (n == 0 || n > kMaxCalWindows) return std::nullopt;
  const size_t plen = kCalCmdFixedLen + static_cast<size_t>(n) * 4;
  if (len < plen + 2) return std::nullopt;
  if (get16(buf, plen) != crc16_ccitt(buf, plen)) return std::nullopt;
  CalCmd c;
  c.vtx_id = get32(buf, 5);
  c.nonce = get32(buf, 9);
  c.phase = buf[13];
  c.frames_per_cell = get16(buf, 14);
  c.settle_ms = get16(buf, 16);
  c.gap_us = get16(buf, 18);
  for (uint8_t i = 0; i < n; ++i) {
    const size_t o = kCalCmdFixedLen + static_cast<size_t>(i) * 4;
    CalWindow w;
    w.rate = buf[o];
    w.idx_lo = buf[o + 1];
    w.idx_hi = buf[o + 2];
    w.idx_step = buf[o + 3];
    if (w.rate > 7 || w.idx_step == 0 || w.idx_hi < w.idx_lo)
      return std::nullopt;
    c.windows.push_back(w);
  }
  return c;
}

std::vector<uint8_t> pack_cal_result(const CalResult& r) {
  std::vector<uint8_t> body;
  body.reserve(kCalResultLen + 2);
  put16(body, RC_MAGIC);
  body.push_back(RC_VERSION);
  body.push_back(T_CAL_RESULT);
  body.push_back(0);
  put32(body, r.vtx_id);
  put32(body, r.nonce);
  for (int i = 0; i < 8; ++i)
    put16(body, static_cast<uint16_t>(r.walls[static_cast<size_t>(i)]));
  put16(body, static_cast<uint16_t>(r.legacy_wall));
  put32(body, r.flags);
  put_crc(body);
  return body;
}

std::optional<CalResult> parse_cal_result(const uint8_t* buf, size_t len) {
  if (len < kCalResultLen + 2) return std::nullopt;
  if (get16(buf, 0) != RC_MAGIC || buf[2] != RC_VERSION ||
      buf[3] != T_CAL_RESULT)
    return std::nullopt;
  if (get16(buf, kCalResultLen) != crc16_ccitt(buf, kCalResultLen))
    return std::nullopt;
  CalResult r;
  r.vtx_id = get32(buf, 5);
  r.nonce = get32(buf, 9);
  for (int i = 0; i < 8; ++i)
    r.walls[static_cast<size_t>(i)] =
        static_cast<int16_t>(get16(buf, 13 + static_cast<size_t>(i) * 2));
  r.legacy_wall = static_cast<int16_t>(get16(buf, 29));
  r.flags = get32(buf, 31);
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
  body.push_back(saturate<uint8_t>(std::lround(t.applied_ov_base * 100.0)));
  body.push_back(saturate<uint8_t>(std::lround(t.applied_ov_enh * 100.0)));
  put16(body, t.rcf_age_ms);
  put16(body, t.rcf_seq_echo);
  put64(body, t.pts_at_build);
  put32(body, t.rcf_rx);
  put32(body, t.enc_frames);
  put32(body, t.enc_kbytes);
  put16(body, t.cmd_kbps);
  body.push_back(static_cast<uint8_t>(t.roi_qp));
  put16(body, t.ring_drops);
  body.push_back(t.txq_depth);
  body.push_back(t.txq_cap);
  put32(body, t.txq_drops);
  put16(body, t.txq_wait_max_ms);
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
  put16(body, t.vanished_base);
  put16(body, t.vanished_enh);
  put16(body, t.self_idr_refused);
  put16(body, t.venc_full_drops);
  body.push_back(t.venc_ring_fill_pct);
  put16(body, t.air_backlog_max_ms);
  put16(body, t.air_shed_drops);
  body.push_back(t.cal_base_ref_idx);

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
  t.applied_ov_base = buf[13] / 100.0;
  t.applied_ov_enh = buf[14] / 100.0;
  t.rcf_age_ms = get16(buf, 15);
  t.rcf_seq_echo = get16(buf, 17);
  t.pts_at_build = get64(buf, 19);
  t.rcf_rx = get32(buf, 27);
  t.enc_frames = get32(buf, 31);
  t.enc_kbytes = get32(buf, 35);
  t.cmd_kbps = get16(buf, 39);
  t.roi_qp = static_cast<int8_t>(buf[41]);
  t.ring_drops = get16(buf, 42);
  t.txq_depth = buf[44];
  t.txq_cap = buf[45];
  t.txq_drops = get32(buf, 46);
  t.txq_wait_max_ms = get16(buf, 50);
  t.radio_sent = get32(buf, 52);
  t.radio_drops = get32(buf, 56);
  t.usb_fail = get16(buf, 60);
  t.up_rssi[0] = buf[62];
  t.up_rssi[1] = buf[63];
  t.up_snr[0] = static_cast<int8_t>(buf[64]);
  t.up_snr[1] = static_cast<int8_t>(buf[65]);
  t.soc_temp_c = static_cast<int8_t>(buf[66]);
  t.thermal_delta = static_cast<int8_t>(buf[67]);
  t.load_x100 = get16(buf, 68);
  t.idr_disagree = get16(buf, 70);
  t.enhance_disagree = get16(buf, 72);
  t.vanished_base = get16(buf, 74);
  t.vanished_enh = get16(buf, 76);
  t.self_idr_refused = get16(buf, 78);
  t.venc_full_drops = get16(buf, 80);
  t.venc_ring_fill_pct = buf[82];
  t.air_backlog_max_ms = get16(buf, 83);
  t.air_shed_drops = get16(buf, 85);
  t.cal_base_ref_idx = buf[87];
  return t;
}

int frame_type(const uint8_t* buf, size_t len) {
  if (len < 4) return -1;
  uint16_t magic = get16(buf, 0);
  uint8_t ver = buf[2];
  if (magic != RC_MAGIC || ver != RC_VERSION) return -1;
  return buf[3];
}

bool is_foreign_rc_version(const uint8_t* buf, size_t len) {
  // Same 4-byte minimum as frame_type(): below it there is no frame to talk
  // about, so a truncated body is never reported as a version mismatch.
  if (len < 4) return false;
  return get16(buf, 0) == RC_MAGIC && buf[2] != RC_VERSION;
}

uint8_t overhead_to_x100(double ov) {
  double n = std::round(std::clamp(ov, 0.05, 2.0) * 100.0);
  return static_cast<uint8_t>(n);
}

}  // namespace mabur::rc
