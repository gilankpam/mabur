#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>
namespace mabur {

// Config for the systematic sliding-window RLC FEC scheme (replaces RsConfig).
// window is the encoder ring / repair span in symbols, [2, 255] (wire u8).
struct SwConfig {
  int symbol_size = 64;
  int window = 128;
  double overhead = 0.25;  // repair symbols per source symbol
  int max_packet_size() const { return symbol_size - 2; }
};

// Systematic sliding-window encoder: packets concatenation-pack into
// fixed-size symbols exactly like RsEncoder did (2-byte LE length prefix,
// zero pad, a packet NEVER spans two symbols), but each sealed symbol ships
// immediately as a source envelope — no k-block accumulation. Repairs are
// GF(256) linear combinations of the last <=window sealed symbols
// (coefficients from sw::repair_coeffs), emitted by a credit system:
// credit += overhead per seal, one repair per whole credit. Overlapping
// repair windows spread protection across subsequent air frames — the time
// diversity the SymbolInterleaver used to buy, without delaying sources.
class SwEncoder {
 public:
  explicit SwEncoder(const SwConfig& cfg);

  // Feeds one packet. Returns any envelopes that became due: at most one
  // source (a symbol sealed to make room) plus credited repairs. A packet
  // larger than max_packet_size() returns empty and counts oversize_drops().
  std::vector<std::vector<uint8_t>> add_packet(const uint8_t* data, size_t len);

  // Seals a partially-filled symbol (if any) and emits one tail repair so a
  // burst tail is recoverable without waiting for the next source. The tail
  // repair fires at most once per sealed source (idle re-flushes are no-ops,
  // so UepEncoder's repeated poll cannot spam repairs).
  std::vector<std::vector<uint8_t>> flush();

  // Takes effect immediately (no block boundary to wait for).
  void set_overhead(double overhead) { cfg_.overhead = overhead; }

  bool has_pending() const { return !current_symbol_.empty(); }
  size_t oversize_drops() const { return oversize_drops_; }
  uint64_t sources_out() const { return sources_out_; }
  uint64_t repairs_out() const { return repairs_out_; }

 private:
  void append_to_current(const uint8_t* data, size_t len);
  void seal_current(std::vector<std::vector<uint8_t>>& out);
  std::vector<uint8_t> make_repair();

  SwConfig cfg_;
  std::deque<std::vector<uint8_t>> window_;  // last <=window sealed payloads
  std::vector<uint8_t> current_symbol_;
  uint32_t next_seq_ = 0;   // seq the next sealed symbol gets
  uint32_t repair_key_ = 0;
  double credit_ = 0.0;
  bool tail_repair_pending_ = false;
  uint64_t sources_out_ = 0, repairs_out_ = 0;
  size_t oversize_drops_ = 0;
};

}  // namespace mabur
