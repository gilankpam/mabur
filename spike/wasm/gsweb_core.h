// THROWAWAY SPIKE (branch wasm-spike): maburgs's receive core without the
// control link -- dot11 frame -> RxBody -> UepDecoder -> FrameStream -> AU.
// Single-threaded; shared by gsweb's replay and live modes (native + WASM).
#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "frame_stream.h"
#include "mabur/node.h"
#include "mabur/uep_decoder.h"

namespace gsweb {

struct Au {
  std::vector<uint8_t> data;  // Annex-B, FrameHdr stripped
  uint32_t pts_us = 0;        // drone capture stamp (u32 wraps)
  uint8_t sid = 0;            // 0 base, 1 enh
  uint8_t flags = 0;          // framewire: 0x01 IDR, 0x02 DISCONT
  bool complete = false;
  uint64_t t_first_us = 0, t_complete_us = 0;
};

// dot11 frame (radiotap already stripped) -> out.body/crc_ok/mac_seq. False
// = drop: too short, or CRC-clean with a foreign SA. CRC-failed frames pass
// (their intact SBI sub-blocks are salvageable), like RadioFrontend.
bool frame_to_body(const uint8_t* dot11, size_t len, bool crc_ok,
                   mabur::node::RxBody& out);

struct CoreStats {
  uint64_t bodies = 0, rc = 0, side = 0, aus_complete = 0, aus_truncated = 0,
           bad_cfg = 0;
};

class RxCore {
 public:
  RxCore(int symbol_size, std::function<void(Au&&)> on_au);
  void on_body(const mabur::node::RxBody& m, uint64_t now_ms);
  void poll(uint64_t now_ms);
  CoreStats stats() const;

 private:
  int block_payload_;  // 14-byte SW envelope header + symbol_size
  uint64_t hdr_bad_cfg_ = 0;
  mabur::UepDecoder dec_;
  maburgs::FrameStream fs_;
  Au cur_;
  CoreStats st_;
  std::function<void(Au&&)> on_au_;
};

}  // namespace gsweb
