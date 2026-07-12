#pragma once
#include <algorithm>
#include <cstdint>
#include <deque>
#include <iterator>
#include <vector>

namespace mabur {

// Reorders RS envelopes so each SBI body carries one symbol from `depth`
// DIFFERENT blocks instead of `depth` symbols of the same block. Without it
// a block rides ~n/blocks_per_body air frames and one lost frame erases
// blocks_per_body of its n symbols — the FEC then dies at single-digit frame
// loss (bench 2026-07-13, docs/handover-video-delivery.md). Interleaved, a
// block spans n frames and tolerates its full repair budget in lost frames.
//
// Not in the Python reference (svc_uep_fec.py) — a deliberate parity break,
// so it sits behind UepEncoder's `interleave` flag, default off. The decoder
// needs no counterpart: envelopes are self-describing (block_id/esi) and
// UepDecoder is symbol-order-agnostic.
class SymbolInterleaver {
 public:
  explicit SymbolInterleaver(int depth) : depth_(depth < 1 ? 1 : depth) {}

  // Adds one completed block's envelopes to the window. Once `depth` blocks
  // are pending, emits rounds of one-envelope-per-block (feeding a packer of
  // blocks_per_body == depth, each round becomes exactly one body). Blocks
  // of equal n drain in lockstep, so steady state alternates: buffer depth
  // blocks, emit n bodies.
  std::vector<std::vector<uint8_t>> add_block(
      std::vector<std::vector<uint8_t>> envs) {
    if (!envs.empty())
      window_.emplace_back(std::make_move_iterator(envs.begin()),
                           std::make_move_iterator(envs.end()));
    std::vector<std::vector<uint8_t>> out;
    while (window_.size() >= static_cast<size_t>(depth_)) emit_round(out);
    return out;
  }

  // Emits ONE round (an envelope from each pending block, possibly fewer
  // than depth) for the flush path, or empty when the window is drained.
  // The caller closes each round as its own short body — feeding sub-depth
  // rounds into a depth-sized packer would misalign it and produce bodies
  // carrying two symbols of the same block.
  std::vector<std::vector<uint8_t>> drain_round() {
    std::vector<std::vector<uint8_t>> out;
    if (!window_.empty()) emit_round(out);
    return out;
  }

  size_t pending_blocks() const { return window_.size(); }

 private:
  void emit_round(std::vector<std::vector<uint8_t>>& out) {
    const size_t take = std::min(window_.size(), static_cast<size_t>(depth_));
    for (size_t i = 0; i < take; ++i) {
      out.push_back(std::move(window_[i].front()));
      window_[i].pop_front();
    }
    window_.erase(
        std::remove_if(window_.begin(), window_.end(),
                       [](const auto& q) { return q.empty(); }),
        window_.end());
  }

  int depth_;
  std::deque<std::deque<std::vector<uint8_t>>> window_;
};

}  // namespace mabur
