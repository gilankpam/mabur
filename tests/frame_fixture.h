#pragma once
// Helpers for tests/fixtures/frame_stream.bin — the whole-frame records
// waybeam's frame-shm ring publishes:
//
//   u32-LE record length | VencFrameMeta | Annex-B frame
//   VencFrameMeta = pts u32-LE (µs) | codec u8 | flags u8 | enc_us u16-LE
//   (µs, 0 = unknown; replaced the write-only gdr_pos/gdr_len bytes
//   2026-08-30, see drone/vendor/venc_frame_ring.h)
//
// Plus the two steps every consumer test repeats: building the wire unit
// maburd feeds UepEncoder::add_frame (FrameHdr stamped over the producer meta,
// then the Annex-B bytes), and reassembling the wide FRAG fragments UepDecoder
// emits back into whole units.
#include <cstdint>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mabur/frag.h"
#include "mabur/frame_wire.h"
#include "mabur/nal.h"
#include "mabur/uep_decoder.h"

namespace mtest {

inline constexpr uint8_t kVencFrameFlagIdr = 0x01;

struct FrameRecord {
  uint32_t pts_us = 0;
  uint8_t codec = 0;
  uint8_t flags = 0;
  std::vector<uint8_t> annexb;

  bool idr() const { return (flags & kVencFrameFlagIdr) != 0; }
  // Layer maburd would pick: the Annex-B scan, protected up by the producer's
  // IDR flag (drone/src/frame_pipeline.h does exactly this).
  int stream_id() const {
    if (idr()) return 0;
    return mabur::classify_frame(annexb.data(), annexb.size());
  }
};

inline std::vector<FrameRecord> load_frame_fixture(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::vector<FrameRecord> out;
  while (true) {
    uint8_t lenb[4];
    f.read(reinterpret_cast<char*>(lenb), 4);
    if (f.gcount() != 4) break;
    const uint32_t len = static_cast<uint32_t>(lenb[0]) |
                         (static_cast<uint32_t>(lenb[1]) << 8) |
                         (static_cast<uint32_t>(lenb[2]) << 16) |
                         (static_cast<uint32_t>(lenb[3]) << 24);
    if (len < 8) throw std::runtime_error(path + ": record shorter than meta");
    std::vector<uint8_t> rec(len);
    f.read(reinterpret_cast<char*>(rec.data()), len);
    if (f.gcount() != static_cast<std::streamsize>(len)) break;
    FrameRecord r;
    r.pts_us = static_cast<uint32_t>(rec[0]) | (static_cast<uint32_t>(rec[1]) << 8) |
               (static_cast<uint32_t>(rec[2]) << 16) | (static_cast<uint32_t>(rec[3]) << 24);
    r.codec = rec[4];
    r.flags = rec[5];
    r.annexb.assign(rec.begin() + 8, rec.end());
    out.push_back(std::move(r));
  }
  return out;
}

inline std::vector<uint8_t> frame_unit(const FrameRecord& r, uint16_t frame_id,
                                       bool discont = false) {
  mabur::framewire::FrameHdr h;
  h.frame_id = frame_id;
  h.flags = static_cast<uint8_t>((r.idr() ? mabur::framewire::kFlagIdr : 0) |
                                 (discont ? mabur::framewire::kFlagDiscont : 0));
  h.codec = r.codec;
  h.pts_us = r.pts_us;
  std::vector<uint8_t> unit(mabur::framewire::kFrameHdrLen + r.annexb.size());
  mabur::framewire::pack_frame_hdr(h, unit.data());
  std::copy(r.annexb.begin(), r.annexb.end(),
            unit.begin() + static_cast<long>(mabur::framewire::kFrameHdrLen));
  return unit;
}

// Reassembles complete units from the raw wide FRAG fragments UepDecoder
// emits, keyed by (stream_id, FRAG seq). Completion order is arrival order,
// which for a lossless channel is the order the encoder produced them.
class FragCollector {
 public:
  void add(const mabur::DecodedFrag& f) {
    if (f.frag.size() < mabur::Fragmenter::kHdrLen) return;
    const uint16_t seq = static_cast<uint16_t>(f.frag[0] | (f.frag[1] << 8));
    const uint16_t idx = static_cast<uint16_t>(f.frag[2] | (f.frag[3] << 8));
    const uint16_t count = static_cast<uint16_t>(f.frag[4] | (f.frag[5] << 8));
    if (count == 0) return;
    auto& e = pending_[(static_cast<uint32_t>(f.stream_id) << 16) | seq];
    e.count = count;
    e.chunks[idx].assign(f.frag.begin() + static_cast<long>(mabur::Fragmenter::kHdrLen),
                         f.frag.end());
    if (e.chunks.size() != count) return;
    std::vector<uint8_t> unit;
    for (uint16_t i = 0; i < count; ++i) {
      auto c = e.chunks.find(i);
      if (c == e.chunks.end()) return;  // non-contiguous: cannot complete
      unit.insert(unit.end(), c->second.begin(), c->second.end());
    }
    completed_.emplace_back(f.stream_id, std::move(unit));
    pending_.erase((static_cast<uint32_t>(f.stream_id) << 16) | seq);
  }

  // (stream_id, unit) pairs in completion order.
  const std::vector<std::pair<int, std::vector<uint8_t>>>& completed() const {
    return completed_;
  }
  size_t pending() const { return pending_.size(); }

 private:
  struct Entry {
    std::map<uint16_t, std::vector<uint8_t>> chunks;
    uint16_t count = 0;
  };
  std::map<uint32_t, Entry> pending_;
  std::vector<std::pair<int, std::vector<uint8_t>>> completed_;
};

}  // namespace mtest
