#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mabur/node.h"

namespace maburgs {

// Dry-run stand-in for the radio front-ends: replays a maburd
// `--dry-run --out` frame file (u32-LE length + [radiotap | dot11 | body])
// as N virtual cards with independent, deterministic per-card loss (LCG per
// card) — so multi-card union/dedup is testable without hardware. Frames
// whose radiotap/dot11 headers don't fit are skipped and counted malformed.
class FrameFileSource {
 public:
  struct Options {
    int cards = 1;
    int drop_pct = 0;
    uint32_t seed = 1;
  };

  FrameFileSource(const std::string& path, Options opt);

  bool ok() const { return ok_; }
  std::optional<mabur::node::RxBody> next();
  uint64_t frames_read() const { return frames_read_; }
  uint64_t dropped() const { return dropped_; }
  uint64_t malformed() const { return malformed_; }

 private:
  struct Frame {
    uint16_t mac_seq = 0;
    std::vector<uint8_t> body;
  };
  bool load(const std::string& path);
  bool card_drops(int card);

  Options opt_;
  bool ok_ = false;
  std::vector<Frame> frames_;
  std::vector<uint32_t> rng_;   // one LCG state per card
  size_t frame_i_ = 0;
  int card_i_ = 0;
  uint64_t frames_read_ = 0, dropped_ = 0, malformed_ = 0;
};

}  // namespace maburgs
