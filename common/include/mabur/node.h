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
//   snr_b, flags(bit0=crc_ok), mac_seq, body_len — then body, then crc16.
constexpr uint16_t RXBODY_MAGIC = 0xF5A0;
constexpr uint16_t CARDSTATUS_MAGIC = 0xF5A5;
constexpr uint8_t NODE_VERSION = 0;

struct RxBody {
  uint8_t card_id = 0;
  uint64_t mono_us = 0;      // sender's monotonic clock, microseconds
  uint8_t rssi[2] = {0, 0};  // per-chain raw (chain A is off-scale on 8822E)
  int8_t snr[2] = {0, 0};
  bool crc_ok = true;        // 802.11 FCS; corrupt frames still carry bodies
  uint16_t mac_seq = 0;      // 12-bit hw seq from the dot11 header
  std::vector<uint8_t> body; // frame body, dot11 header stripped
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
