#include "mabur/rc_proto.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include "mabur/crc16.h"
#include "mabur/siphash.h"

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

// Writes the 8-byte SipHash tag for a tagged frame (DISC/RCF/CAL_CMD/
// CAL_RESULT): keyed MAC over `body` so far (the frame's bytes up to but
// not including the tag) concatenated with ctx's three u32s, which are
// hashed in but never sent (TagCtx doc comment).
void put_tag(std::vector<uint8_t>& body, const LinkKey& key, const TagCtx& ctx) {
  std::vector<uint8_t> m(body);
  put32(m, ctx.vrx_nonce);
  put32(m, ctx.vtx_nonce);
  put32(m, ctx.seq32);
  put64(body, siphash24(key, m.data(), m.size()));
}

constexpr size_t RCF_HEAD_LEN = 15;  // 2026-10-01: vtx_id deleted (19)
constexpr size_t DISC_LEN = 17;
constexpr size_t DISC_ACK_LEN = 20;  // 2026-10-01: vtx_nonce(4) + flags(1) added (15)
constexpr size_t TELEM_LEN = 48;  // 2026-09-30: maburtop-only fields dropped (98)

// magic(2) | ver | type | flags | nonce(4) | phase | fpc(2) |
// settle(2) | gap(2) | n_windows(1) | n * 4 bytes
//
// Offsets: hdr 0..4, nonce 5..8, phase 9, fpc 10..11,
// settle 12..13, gap 14..15, n_windows 16. The constant INCLUDES the
// n_windows byte, so buf[kCalCmdFixedLen - 1] IS n_windows and the windows
// array starts at kCalCmdFixedLen. tests/test_rc.cpp hard-codes 16 for the
// same byte -- the two must agree.
constexpr size_t kCalCmdFixedLen = 5 + 4 + 1 + 2 + 2 + 2 + 1;  // 17
// magic(2) | ver | type | flags | nonce(4) | walls(8*2) |
// legacy(2)
constexpr size_t kCalResultLen = 5 + 4 + 16 + 2;

}  // namespace

std::vector<uint8_t> pack_rcf(const Rcf& r, const LinkKey& key, const TagCtx& ctx) {
  std::vector<uint8_t> body;
  body.reserve(RCF_HEAD_LEN + kTagLen + 2);
  put16(body, RC_MAGIC);
  body.push_back(RC_VERSION);
  body.push_back(T_RCF);
  body.push_back(0);  // flags: nothing
  put16(body, r.seq);
  body.push_back(r.profile);
  body.push_back(overhead_to_x100(r.fec_overhead_base));
  body.push_back(overhead_to_x100(r.fec_overhead_enh));
  body.push_back(r.probe_profile);
  body.push_back(r.hop_ch);
  body.push_back(r.hop_epoch);
  body.push_back(r.rec);
  body.push_back(r.idr_epoch);
  put_tag(body, key, ctx);
  put_crc(body);
  return body;
}

std::optional<Rcf> parse_rcf(const uint8_t* buf, size_t len) {
  if (len < RCF_HEAD_LEN + kTagLen + 2) return std::nullopt;
  if (get16(buf, 0) != RC_MAGIC || buf[2] != RC_VERSION || buf[3] != T_RCF)
    return std::nullopt;
  if (get16(buf, RCF_HEAD_LEN + kTagLen) != crc16_ccitt(buf, RCF_HEAD_LEN + kTagLen))
    return std::nullopt;
  Rcf r;
  r.seq = get16(buf, 5);
  r.profile = buf[7];
  r.fec_overhead_base = buf[8] / 100.0;
  r.fec_overhead_enh = buf[9] / 100.0;
  r.probe_profile = buf[10];
  r.hop_ch = buf[11];
  r.hop_epoch = buf[12];
  r.rec = buf[13];
  r.idr_epoch = buf[14];
  return r;
}

std::vector<uint8_t> pack_cal_cmd(const CalCmd& c, const LinkKey& key, const TagCtx& ctx) {
  std::vector<uint8_t> body;
  const size_t n = c.windows.size() > kMaxCalWindows ? kMaxCalWindows
                                                     : c.windows.size();
  body.reserve(kCalCmdFixedLen + n * 4 + kTagLen + 2);
  put16(body, RC_MAGIC);
  body.push_back(RC_VERSION);
  body.push_back(T_CAL_CMD);
  body.push_back(0);  // flags: nothing
  put32(body, c.nonce);
  body.push_back(c.phase);
  put16(body, c.frames_per_cell);
  put16(body, c.settle_ms);
  put16(body, c.gap_us);
  body.push_back(static_cast<uint8_t>(n));
  for (size_t i = 0; i < n; ++i) {
    body.push_back(c.windows[i].rate);
    body.push_back(static_cast<uint8_t>(c.windows[i].idx_lo));
    body.push_back(static_cast<uint8_t>(c.windows[i].idx_hi));
    body.push_back(c.windows[i].idx_step);
  }
  put_tag(body, key, ctx);
  put_crc(body);
  return body;
}

std::optional<CalCmd> parse_cal_cmd(const uint8_t* buf, size_t len) {
  if (len < kCalCmdFixedLen + 2) return std::nullopt;
  if (get16(buf, 0) != RC_MAGIC || buf[2] != RC_VERSION || buf[3] != T_CAL_CMD)
    return std::nullopt;
  const uint8_t n = buf[kCalCmdFixedLen - 1];
  if (n == 0 || n > kMaxCalWindows) return std::nullopt;
  const size_t plen = kCalCmdFixedLen + static_cast<size_t>(n) * 4 + kTagLen;
  if (len < plen + 2) return std::nullopt;
  if (get16(buf, plen) != crc16_ccitt(buf, plen)) return std::nullopt;
  CalCmd c;
  c.nonce = get32(buf, 5);
  c.phase = buf[9];
  c.frames_per_cell = get16(buf, 10);
  c.settle_ms = get16(buf, 12);
  c.gap_us = get16(buf, 14);
  for (uint8_t i = 0; i < n; ++i) {
    const size_t o = kCalCmdFixedLen + static_cast<size_t>(i) * 4;
    CalWindow w;
    w.rate = buf[o];
    w.idx_lo = static_cast<int8_t>(buf[o + 1]);
    w.idx_hi = static_cast<int8_t>(buf[o + 2]);
    w.idx_step = buf[o + 3];
    if (w.rate > 7 || w.idx_step == 0 || w.idx_hi < w.idx_lo ||
        w.idx_lo < kRelMin || w.idx_hi > kRelMax)
      return std::nullopt;
    c.windows.push_back(w);
  }
  return c;
}

std::vector<uint8_t> pack_cal_result(const CalResult& r, const LinkKey& key, const TagCtx& ctx) {
  std::vector<uint8_t> body;
  body.reserve(kCalResultLen + kTagLen + 2);
  put16(body, RC_MAGIC);
  body.push_back(RC_VERSION);
  body.push_back(T_CAL_RESULT);
  body.push_back(0);
  put32(body, r.nonce);
  for (int i = 0; i < 8; ++i)
    put16(body, static_cast<uint16_t>(r.walls[static_cast<size_t>(i)]));
  put16(body, static_cast<uint16_t>(r.legacy_wall));
  put_tag(body, key, ctx);
  put_crc(body);
  return body;
}

std::optional<CalResult> parse_cal_result(const uint8_t* buf, size_t len) {
  if (len < kCalResultLen + kTagLen + 2) return std::nullopt;
  if (get16(buf, 0) != RC_MAGIC || buf[2] != RC_VERSION ||
      buf[3] != T_CAL_RESULT)
    return std::nullopt;
  if (get16(buf, kCalResultLen + kTagLen) != crc16_ccitt(buf, kCalResultLen + kTagLen))
    return std::nullopt;
  CalResult r;
  r.nonce = get32(buf, 5);
  for (int i = 0; i < 8; ++i)
    r.walls[static_cast<size_t>(i)] =
        static_cast<int16_t>(get16(buf, 9 + static_cast<size_t>(i) * 2));
  r.legacy_wall = static_cast<int16_t>(get16(buf, 25));
  return r;
}

std::vector<uint8_t> pack_disc(const Disc& d, const LinkKey& key) {
  std::vector<uint8_t> body;
  body.reserve(DISC_LEN + kTagLen + 2);
  put16(body, RC_MAGIC);
  body.push_back(RC_VERSION);
  body.push_back(T_DISC);
  body.push_back(F_DISCOVERY);
  put32(body, d.vrx_nonce);
  body.push_back(d.op_channel);
  body.push_back(d.op_width);
  body.push_back(d.table_ver);
  body.push_back(d.init_profile);
  put16(body, d.cap_bits);
  put16(body, d.seq);

  put_tag(body, key, TagCtx{});  // ctx all-zero: pre-rendezvous, no nonces/seq yet
  put_crc(body);
  return body;
}

std::optional<Disc> parse_disc(const uint8_t* buf, size_t len) {
  if (len < DISC_LEN + kTagLen + 2) return std::nullopt;
  uint16_t magic = get16(buf, 0);
  uint8_t ver = buf[2];
  uint8_t type = buf[3];
  if (magic != RC_MAGIC || ver != RC_VERSION || type != T_DISC) return std::nullopt;

  uint16_t crc = get16(buf, DISC_LEN + kTagLen);
  if (crc != crc16_ccitt(buf, DISC_LEN + kTagLen)) return std::nullopt;

  Disc d;
  d.vrx_nonce = get32(buf, 5);
  d.op_channel = buf[9];
  d.op_width = buf[10];
  d.table_ver = buf[11];
  d.init_profile = buf[12];
  d.cap_bits = get16(buf, 13);
  d.seq = get16(buf, 15);
  return d;
}

std::vector<uint8_t> pack_disc_ack(const DiscAck& a) {
  std::vector<uint8_t> body;
  body.reserve(DISC_ACK_LEN + 2);
  put16(body, RC_MAGIC);
  body.push_back(RC_VERSION);
  body.push_back(T_DISC_ACK);
  body.push_back(F_DISCOVERY);
  put32(body, a.vrx_nonce);
  put32(body, a.vtx_nonce);
  put16(body, a.chip_caps);
  body.push_back(a.agreed_channel);
  body.push_back(a.agreed_width);
  body.push_back(a.flags);
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
  a.vrx_nonce = get32(buf, 5);
  a.vtx_nonce = get32(buf, 9);
  a.chip_caps = get16(buf, 13);
  a.agreed_channel = buf[15];
  a.agreed_width = buf[16];
  a.flags = buf[17];
  a.seq = get16(buf, 18);
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
  put16(body, t.rcf_age_ms);
  put16(body, t.rcf_seq_echo);
  put64(body, t.pts_at_build);
  put32(body, t.rcf_rx);
  put16(body, t.cmd_kbps);
  put32(body, t.txq_drops);
  put16(body, t.txq_wait_max_ms);
  put16(body, t.usb_fail);
  body.push_back(t.up_rssi[0]);
  body.push_back(t.up_rssi[1]);
  body.push_back(static_cast<uint8_t>(t.up_snr[0]));
  body.push_back(static_cast<uint8_t>(t.up_snr[1]));
  body.push_back(static_cast<uint8_t>(t.soc_temp_c));
  put16(body, t.cpu_busy_x100);
  put16(body, t.rx_own);
  put16(body, t.rx_foreign);
  put16(body, t.rx_crcfail);
  body.push_back(t.rec_status);

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
  t.rcf_age_ms = get16(buf, 8);
  t.rcf_seq_echo = get16(buf, 10);
  t.pts_at_build = get64(buf, 12);
  t.rcf_rx = get32(buf, 20);
  t.cmd_kbps = get16(buf, 24);
  t.txq_drops = get32(buf, 26);
  t.txq_wait_max_ms = get16(buf, 30);
  t.usb_fail = get16(buf, 32);
  t.up_rssi[0] = buf[34];
  t.up_rssi[1] = buf[35];
  t.up_snr[0] = static_cast<int8_t>(buf[36]);
  t.up_snr[1] = static_cast<int8_t>(buf[37]);
  t.soc_temp_c = static_cast<int8_t>(buf[38]);
  t.cpu_busy_x100 = get16(buf, 39);
  t.rx_own = get16(buf, 41);
  t.rx_foreign = get16(buf, 43);
  t.rx_crcfail = get16(buf, 45);
  t.rec_status = buf[47];
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

bool verify_control(const uint8_t* buf, size_t len, const LinkKey& key, const TagCtx& ctx) {
  if (len < 5 + kTagLen + 2) return false;
  const size_t tag_at = len - 2 - kTagLen;
  std::vector<uint8_t> m(buf, buf + tag_at);
  put32(m, ctx.vrx_nonce);
  put32(m, ctx.vtx_nonce);
  put32(m, ctx.seq32);
  const uint64_t want = siphash24(key, m.data(), m.size());
  const uint64_t got = get64(buf, tag_at);
  return ((want ^ got) == 0);   // single 64-bit compare: no early-out on partial match
}

uint8_t overhead_to_x100(double ov) {
  double n = std::round(std::clamp(ov, 0.05, 2.0) * 100.0);
  return static_cast<uint8_t>(n);
}

}  // namespace mabur::rc
