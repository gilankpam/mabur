#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>
namespace mabur {

// Config for the Reed-Solomon block erasure FEC scheme — mirrors devourer's
// stream_fec.FecConfig (repair_count/max_packet_size) combined with the
// RS-specific k/symbol_size/overhead knobs from stream_fec_rs.RsEncoder.
struct RsConfig {
  int k = 8;
  int symbol_size = 64;
  double overhead = 0.25;

  int repair_count() const { return static_cast<int>(std::ceil(k * overhead)); }
  int n() const { return k + repair_count(); }
  int max_packet_size() const { return symbol_size - 2; }
};

// Concatenation-packs IP packets, then Reed-Solomon-encodes K source symbols
// into N = K + repair_count systematic + parity symbols. Byte-exact port of
// devourer/tools/precoder/stream_fec_rs.py's RsEncoder.
//
// Wire envelope: 11-byte header + symbol_size payload bytes. Header is
// packed little-endian as <HBBBHHBB>:
//   MAGIC=0xF540(u16), flags=0(u8), k(u8), kreal(u8), symbol_size(u16),
//   block_id(u16), esi(u8), n(u8)
class RsEncoder {
 public:
  explicit RsEncoder(const RsConfig& cfg);

  // Feeds one IP packet in. Returns the encoded envelopes of a freshly
  // completed block, or empty if the block isn't full yet. A packet larger
  // than max_packet_size() returns empty and increments oversize_drops()
  // instead of throwing (hot path can't take exceptions) — callers
  // pre-fragment so this should never trigger in practice.
  std::vector<std::vector<uint8_t>> add_packet(const uint8_t* data, size_t len);

  // Seals any partially-filled symbol (zero-padded), zero-pads the pending
  // symbol list up to k, and encodes the final partial block with
  // kreal = the number of real (non-padding) symbols. Returns empty if
  // there is nothing pending.
  std::vector<std::vector<uint8_t>> flush();

  // Changes the repair overhead. Takes effect at the next block boundary
  // (i.e. once no symbols are pending).
  void set_overhead(double overhead);

  bool has_pending() const;

  size_t oversize_drops() const { return oversize_drops_; }

 private:
  void append_to_current(const uint8_t* data, size_t len);
  void seal_current_symbol();
  std::vector<std::vector<uint8_t>> maybe_encode_full_block();
  std::vector<std::vector<uint8_t>> encode_block(int kreal);
  void maybe_apply_pending_overhead();

  RsConfig cfg_;
  int n_;
  std::vector<std::vector<uint8_t>> pending_symbols_;
  std::vector<uint8_t> current_symbol_;
  uint16_t block_id_ = 0;
  size_t oversize_drops_ = 0;

  bool has_pending_overhead_ = false;
  double pending_overhead_ = 0.0;
};

}  // namespace mabur
