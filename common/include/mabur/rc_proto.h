#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
namespace mabur::rc {

// RC control-plane framing (adaptive-link feedback + rendezvous). Byte-exact
// port of devourer's tools/precoder/rc_proto.py: RCF (VRX->VTX feedback),
// DISC (VRX->VTX discovery beacon), DISC_ACK (VTX->VRX rendezvous reply).
// All multi-byte fields are little-endian; every frame ends with a u16
// CRC16-CCITT (mabur::crc16_ccitt) over every byte before it.

constexpr uint16_t RC_MAGIC = 0x5243;  // "RC"
constexpr uint8_t RC_VERSION = 1;

constexpr uint8_t T_RCF = 1;
constexpr uint8_t T_DISC = 2;
constexpr uint8_t T_DISC_ACK = 3;

constexpr uint8_t F_AUTH_ADVISORY = 0x01;
constexpr uint8_t F_FAILSAFE = 0x02;
constexpr uint8_t F_DISCOVERY = 0x04;

constexpr uint8_t PWR_NO_CHANGE = 0xFF;

// VRX -> VTX feedback: GS-authoritative profile + alink-style score +
// explicit power/FEC + per-layer delivery stats.
struct Rcf {
  uint32_t vtx_id = 0;
  uint16_t seq = 0;
  uint16_t ack_seq = 0;
  uint8_t profile = 0;
  uint16_t score = 1000;
  uint8_t pwr_idx = PWR_NO_CHANGE;
  uint8_t fec_overhead_16ths = 4;
  uint8_t flags = 0;
  std::vector<uint8_t> layer_delivery;

  double fec_overhead() const { return fec_overhead_16ths / 16.0; }
};

// VRX -> VTX discovery beacon (rendezvous), addressed to a VTX_ID.
struct Disc {
  uint32_t vtx_id = 0;
  uint32_t vrx_nonce = 0;
  uint8_t op_channel = 0;
  uint8_t op_width = 20;
  uint8_t table_ver = 1;
  uint8_t init_profile = 0;
  uint16_t cap_bits = 0;
  uint16_t seq = 0;
};

// VTX -> VRX reply completing rendezvous + agreeing the op channel.
struct DiscAck {
  uint32_t vtx_id = 0;
  uint32_t vrx_nonce = 0;
  uint16_t chip_caps = 0;
  uint8_t agreed_channel = 0;
  uint8_t agreed_width = 20;
  uint16_t seq = 0;
};

std::vector<uint8_t> pack_rcf(const Rcf& r);
std::optional<Rcf> parse_rcf(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_disc(const Disc& d);
std::optional<Disc> parse_disc(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_disc_ack(const DiscAck& a);
std::optional<DiscAck> parse_disc_ack(const uint8_t* buf, size_t len);

// Peeks the RC frame type without a full parse (no CRC check). Returns -1 if
// the buffer is too short or doesn't carry the RC magic/version.
int frame_type(const uint8_t* buf, size_t len);

// Converts a fractional FEC overhead (e.g. 0.25) to the wire's 1/16ths unit,
// rounded to nearest and clamped to [1, 16].
uint8_t overhead_to_16ths(double ov);

}  // namespace mabur::rc
