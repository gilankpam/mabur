// THROWAWAY SPIKE (native only): synthetic maburd --dry-run --out file from
// the real UepEncoder at production FEC geometry (symbol 332, window 32,
// overhead base 0.5 / enh 0.25, 4 blocks/body). GOP: VPS+SPS+PPS+IDR every
// 120 frames, otherwise TRAIL_R (base) / TRAIL_N (enh) alternating.
// usage: mkbodies <out> <frames> [kbytes_per_frame=40] [seed=1]
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

#include "mabur/frame_wire.h"
#include "mabur/nal.h"
#include "mabur/uep_encoder.h"

namespace {
std::vector<uint8_t> nal(std::mt19937& rng, uint8_t type, size_t len) {
  std::vector<uint8_t> v = {0, 0, 0, 1, static_cast<uint8_t>(type << 1), 1};
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = static_cast<uint8_t>(rng());
    v.push_back(b == 0 ? 1 : b);  // no accidental start codes in the payload
  }
  return v;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: mkbodies <out> <frames> [kbytes_per_frame=40] [seed=1]\n");
    return 2;
  }
  const int frames = std::atoi(argv[2]);
  const size_t kb = argc > 3 ? static_cast<size_t>(std::atoi(argv[3])) : 40;
  std::mt19937 rng(argc > 4 ? static_cast<uint32_t>(std::atol(argv[4])) : 1u);
  std::array<mabur::UepLayerCfg, 2> layers{};
  const double ov[2] = {0.5, 0.25};
  for (int i = 0; i < 2; ++i) {
    layers[i].fec = mabur::SwConfig{332, 32, ov[i]};
    layers[i].blocks_per_body = 4;
  }
  mabur::UepEncoder enc(layers, 15);
  FILE* f = std::fopen(argv[1], "wb");
  if (!f) return 2;
  // 8-byte radiotap (it_len 8, no fields) + 26-byte QoS-Data header.
  const uint8_t rt[8] = {0, 0, 8, 0, 0, 0, 0, 0};
  uint8_t dot11[26] = {0x88, 0};
  std::memset(dot11 + 4, 0xff, 6);
  const uint8_t sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  std::memcpy(dot11 + 10, sa, 6);
  std::memcpy(dot11 + 16, sa, 6);
  uint16_t seq = 0;
  size_t bodies = 0;
  for (int fi = 0; fi < frames; ++fi) {
    std::vector<uint8_t> ab;
    auto app = [&](std::vector<uint8_t> v) { ab.insert(ab.end(), v.begin(), v.end()); };
    const bool idr = fi % 120 == 0;
    if (idr) { app(nal(rng, 32, 20)); app(nal(rng, 33, 40)); app(nal(rng, 34, 8));
               app(nal(rng, 19, kb * 1024 * 3)); }
    else app(nal(rng, fi % 2 ? 0 : 1, kb * 1024));
    const int sid = mabur::classify_frame(ab.data(), ab.size());
    std::vector<uint8_t> unit(mabur::framewire::kFrameHdrLen + ab.size());
    mabur::framewire::FrameHdr h;
    h.frame_id = static_cast<uint16_t>(fi);
    h.flags = idr ? mabur::framewire::kFlagIdr : 0;
    h.pts_us = 16667u * static_cast<uint32_t>(fi);
    mabur::framewire::pack_frame_hdr(h, unit.data());
    std::memcpy(unit.data() + mabur::framewire::kFrameHdrLen, ab.data(), ab.size());
    for (auto& b : enc.add_frame(sid, unit.data(), unit.size(), static_cast<uint64_t>(fi))) {
      const uint16_t sc = static_cast<uint16_t>(seq++ << 4);
      dot11[22] = static_cast<uint8_t>(sc & 0xff);
      dot11[23] = static_cast<uint8_t>(sc >> 8);
      const uint32_t len = static_cast<uint32_t>(sizeof rt + sizeof dot11 + b.body.size());
      std::fwrite(&len, 4, 1, f);
      std::fwrite(rt, 1, sizeof rt, f);
      std::fwrite(dot11, 1, sizeof dot11, f);
      std::fwrite(b.body.data(), 1, b.body.size(), f);
      ++bodies;
    }
  }
  std::fclose(f);
  std::fprintf(stderr, "mkbodies: %d frames, %zu bodies\n", frames, bodies);
  return 0;
}
