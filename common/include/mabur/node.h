#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mabur::node {

// Card <-> aggregator messages, wire format v0. In-process in v1; the same
// serialization is the future multi-node UDP payload (OpenWrt forwarder ->
// aggregator host), so it is golden-vector-pinned (tools/genvectors) like
// every other mabur wire format. Little-endian, CRC16-CCITT tail over all
// preceding bytes. RxBody header is 21 bytes:
//   <HBBQBBbbBHH> magic, ver, card_id, mono_us, rssi_a, rssi_b, snr_a,
//   snr_b, flags(bit0=crc_ok, bit1=phy_valid), mac_seq, body_len — then
//   body, then crc16.
constexpr uint16_t RXBODY_MAGIC = 0xF5A0;
constexpr uint16_t CARDSTATUS_MAGIC = 0xF5A5;
constexpr uint8_t NODE_VERSION = 0;

struct RxBody {
  uint8_t card_id = 0;
  uint64_t mono_us = 0;      // sender's monotonic clock, microseconds
  // per-chain raw, dBm = value - 110 (both chains valid on 8822E) — only
  // meaningful when phy_valid is true; non-first A-MPDU subframes carry no
  // PHY status and this reads 0/garbage (see phy_valid below).
  uint8_t rssi[2] = {0, 0};
  int8_t snr[2] = {0, 0};
  // Per-chain RX EVM, raw half-dB from devourer (negative = clean, dB =
  // raw / 2). 0 = not sampled (no phy status on this frame, e.g. non-first
  // A-MPDU subframes) — consumers must skip zeros, never average them.
  int8_t evm[2] = {0, 0};
  // PHY status present on this frame's RX descriptor (devourer
  // rx_pkt_attrib.physt). False for non-first A-MPDU subframes, whose
  // rssi/snr/evm read 0/garbage — consumers must not fold RF stats from
  // frames with phy_valid == false. Default true keeps frame-file replay
  // and old captures meaningful.
  bool phy_valid = true;
  bool crc_ok = true;        // 802.11 FCS; corrupt frames still carry bodies
  uint16_t mac_seq = 0;      // 12-bit hw seq from the dot11 header
  // Chip TSF (low 32 bits, µs) stamped on the RX descriptor. Diagnostic:
  // air-time continuity across per-card seq gaps (MABUR_GAPLOG).
  uint32_t tsfl = 0;
  std::vector<uint8_t> body; // frame body, dot11 header stripped
  // RX HT MCS from the radio's RX descriptor (devourer rx_pkt_attrib
  // .data_rate: HT codes are 0x80 + mcs). 255 = unknown — legacy/VHT rate,
  // or a source that carries no rate (frame-file replay). Descriptor
  // metadata, valid independent of the body CRC. Consumed by the
  // transition-attribution boundary (UepDecoder::add_body).
  uint8_t mcs = 255;
};

struct CardStatus {
  uint8_t card_id = 0;
  uint64_t mono_us = 0;
  uint32_t frames_seen = 0;
  uint32_t crc_fail = 0;
  uint32_t uptime_s = 0;
};

std::vector<uint8_t> pack_rx_body(const RxBody& m);
std::optional<RxBody> parse_rx_body(const uint8_t* buf, size_t len);
std::vector<uint8_t> pack_card_status(const CardStatus& m);
std::optional<CardStatus> parse_card_status(const uint8_t* buf, size_t len);

}  // namespace mabur::node
