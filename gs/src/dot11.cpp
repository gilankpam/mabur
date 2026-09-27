#include "dot11.h"

#include <cstring>

#include "RadiotapBuilder.h"
#include "TxMode.h"

namespace maburgs {
namespace {
constexpr size_t kDot11 = 24;
constexpr uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};

const std::vector<uint8_t>& max_range_radiotap() {
  static const std::vector<uint8_t> rt = [] {
    devourer::TxMode m;
    m.mode = devourer::TxMode::Mode::HT;
    m.ht_mcs = 0;
    m.bw_mhz = 20;
    m.ldpc = true;
    m.stbc = true;
    return devourer::build_stream_radiotap(m);
  }();
  return rt;
}
}  // namespace

std::vector<uint8_t> build_control_frame(uint16_t seq, const uint8_t* body,
                                         size_t len) {
  const auto& rt = max_range_radiotap();
  std::vector<uint8_t> f(rt.size() + kDot11 + len);
  std::memcpy(f.data(), rt.data(), rt.size());
  uint8_t* d = f.data() + rt.size();
  d[0] = 0x40;
  d[1] = 0x00;
  d[2] = 0x00;
  d[3] = 0x00;
  std::memset(d + 4, 0xff, 6);
  std::memcpy(d + 10, kSa, 6);
  std::memcpy(d + 16, kSa, 6);
  const uint16_t seq_ctl = static_cast<uint16_t>(seq << 4);
  d[22] = static_cast<uint8_t>(seq_ctl & 0xff);
  d[23] = static_cast<uint8_t>(seq_ctl >> 8);
  if (len) std::memcpy(d + kDot11, body, len);
  return f;
}

bool sa_canonical(const uint8_t* dot11, size_t len) {
  return len >= 16 && std::memcmp(dot11 + 10, kSa, 6) == 0;
}

size_t dot11_body_offset(const uint8_t* dot11, size_t len) {
  const size_t off = (len >= 1 && dot11[0] == 0x88) ? 26 : 24;
  return len >= off + 1 ? off : 0;
}

RxVerdict fill_rx_body(const uint8_t* d, size_t len, const RxMeta& meta,
                       mabur::node::RxBody& m) {
  const size_t body_off = dot11_body_offset(d, len);
  if (body_off == 0) return RxVerdict::Short;
  if (!meta.crc_err && !sa_canonical(d, len)) return RxVerdict::Foreign;
  m.rssi[0] = meta.rssi[0];
  m.rssi[1] = meta.rssi[1];
  m.snr[0] = meta.snr[0];
  m.snr[1] = meta.snr[1];
  m.evm[0] = meta.evm[0];
  m.evm[1] = meta.evm[1];
  m.phy_valid = meta.physt;
  m.crc_ok = !meta.crc_err;
  // RX rate code -> HT MCS index. devourer's RxAtrib.data_rate carries TWO
  // encodings depending on chip family, and both HT-1SS ranges are mapped
  // here (they cannot collide):
  //  - jaguar1/2/3 (8812/8822B/8822E...): the raw Realtek DESC_RATE index --
  //    HT MCS0..7 = 0x0C..0x13 (DESC_RATEMCS0, ieee80211_radiotap.h). This
  //    is what the GS's 8822E cards produce; verified 2026-08-14 on the
  //    bench when the original 0x80-only mapping left every attribution
  //    boundary unclosed (link.attrib.close_ms null through 5 promotes).
  //  - kestrel (8852B/C): the AX 9-bit code, HT = 0x80 + mcs (the encoding
  //    RxPacket.h's comment describes; it does NOT apply to jaguar chips).
  // Everything else (legacy CCK/OFDM, VHT, HE, 2SS) is "unknown" for
  // attribution purposes -- the drone injects HT-1SS only.
  const uint16_t dr = meta.data_rate;
  m.mcs = (dr >= 0x0C && dr <= 0x13) ? static_cast<uint8_t>(dr - 0x0C)
          : (dr >= 0x80 && dr <= 0x87) ? static_cast<uint8_t>(dr - 0x80)
                                       : 255;
  m.mac_seq = static_cast<uint16_t>(static_cast<uint16_t>(d[22] | (d[23] << 8)) >> 4);
  m.body.assign(d + body_off, d + len);
  m.tsfl = meta.tsfl;
  return RxVerdict::Body;
}

}  // namespace maburgs
