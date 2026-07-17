#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
namespace mabur {

// Config for the systematic sliding-window RLC FEC scheme (the sole FEC
// scheme; block RS is retired).
// window is the encoder ring / repair span in symbols, [2, 255] (wire u8).
struct SwConfig {
  int symbol_size = 64;
  int window = 128;
  double overhead = 0.25;  // repair symbols per source symbol
  int max_packet_size() const { return symbol_size - 2; }
};

// Systematic sliding-window encoder: packets concatenation-pack into
// fixed-size symbols (2-byte LE length prefix, zero pad, a packet NEVER
// spans two symbols), but each sealed symbol ships immediately as a source
// envelope — no block accumulation. Repairs are GF(256) linear combinations
// of the last <=window sealed symbols (coefficients from sw::repair_coeffs),
// emitted by a credit system: credit += overhead per seal, one repair per
// whole credit. Overlapping repair windows spread protection across
// subsequent air frames, buying time diversity without delaying sources.
class SwEncoder {
 public:
  // initial_seq seeds next_seq_ (default 0, which is what every existing
  // unit test and golden vector pins — do not change the default). A fresh
  // encoder instance MUST start >kResetSpan (sw_decoder.cpp) away from any
  // prior run's seqs with high probability, or a restarted drone's stream
  // is dropped as stale for its predecessor's lifetime: callers that
  // survive process restarts (UepEncoder, linkbench tx_main) should pass a
  // random draw (residual collision odds ~2^-11 against kResetSpan=2^20
  // over a ~2^32 seq space).
  explicit SwEncoder(const SwConfig& cfg, uint32_t initial_seq = 0);

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
  // Sealed symbols live in one contiguous 16 B-aligned fixed-stride ring of
  // window + kSlackRows rows (not a deque of vectors). The slack keeps a
  // row readable for kSlackRows further seals after it leaves the window —
  // the async repair path (spec 2026-07-17) queues jobs that reference ring
  // rows by slot instead of copying the window.
  static constexpr size_t kSlackRows = 64;

  void append_to_current(const uint8_t* data, size_t len);
  void seal_current(std::vector<std::vector<uint8_t>>& out);
  std::vector<uint8_t> make_repair();
  // Envelope construction shared by the sync and (later) async paths; pure
  // reader of ring rows [start_slot, start_slot + window_len).
  std::vector<uint8_t> build_repair(uint32_t repair_key, uint32_t header_seq,
                                    int window_len, size_t start_slot) const;
  const uint8_t* row(size_t oldest_i) const;  // 0 = oldest held symbol

  SwConfig cfg_;
  size_t stride_ = 0, cap_ = 0, count_ = 0, next_slot_ = 0;
  std::vector<uint8_t> ring_raw_;
  uint8_t* ring_ = nullptr;  // 16B-aligned base inside ring_raw_
  std::vector<uint8_t> current_symbol_;
  uint32_t next_seq_ = 0;   // seq the next sealed symbol gets
  uint32_t repair_key_ = 0;
  double credit_ = 0.0;
  bool tail_repair_pending_ = false;
  uint64_t sources_out_ = 0, repairs_out_ = 0;
  size_t oversize_drops_ = 0;
};

}  // namespace mabur
