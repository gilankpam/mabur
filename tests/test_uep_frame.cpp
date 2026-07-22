#include <algorithm>
#include <cstring>
#include <map>
#include <random>
#include "mtest.h"
#include "mabur/frame_wire.h"
#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"
using namespace mabur;

namespace {
std::array<UepLayerCfg, 4> wide_layers() {
  std::array<UepLayerCfg, 4> l{};
  for (auto& c : l) {
    c.fec.symbol_size = 164;
    c.fec.window = 128;
    c.fec.overhead = 0.5;
    c.blocks_per_body = 4;
    c.wide_frag = true;
  }
  return l;
}

std::vector<uint8_t> mk_frame_unit(uint16_t frame_id, uint32_t pts,
                                   size_t payload_len, uint8_t fill) {
  std::vector<uint8_t> v(framewire::kFrameHdrLen + payload_len);
  framewire::FrameHdr h;
  h.frame_id = frame_id;
  h.pts_us = pts;
  framewire::pack_frame_hdr(h, v.data());
  for (size_t i = 0; i < payload_len; ++i)
    v[framewire::kFrameHdrLen + i] = static_cast<uint8_t>(fill + i);
  return v;
}
}  // namespace

TEST(add_frame_roundtrips_through_wide_decoder) {
  UepEncoder enc(wide_layers(), 15);
  UepDecoder dec(wide_layers(), 200);
  dec.set_wide_frag(true);

  // 60 KB frame on layer 1 — >255 fragments, impossible in narrow format.
  auto unit = mk_frame_unit(7, 1000000, 60 * 1024, 0x10);
  auto bodies = enc.add_frame(1, unit.data(), unit.size(), 1);
  REQUIRE(!bodies.empty());

  // Reassemble from the RAW fragments wide mode emits (6-byte FRAG header).
  std::map<uint16_t, std::vector<uint8_t>> chunks;
  uint16_t count = 0;
  for (auto& b : bodies) {
    auto pkts = dec.add_body(b.body.data(), b.body.size(), 1);
    for (auto& p : pkts) {
      REQUIRE(p.stream_id == 1);
      REQUIRE(p.pkt.size() >= 6);
      uint16_t idx = static_cast<uint16_t>(p.pkt[2] | (p.pkt[3] << 8));
      count = static_cast<uint16_t>(p.pkt[4] | (p.pkt[5] << 8));
      chunks[idx].assign(p.pkt.begin() + 6, p.pkt.end());
    }
  }
  REQUIRE(count > 255);
  REQUIRE(chunks.size() == count);
  std::vector<uint8_t> got;
  for (uint16_t i = 0; i < count; ++i)
    got.insert(got.end(), chunks[i].begin(), chunks[i].end());
  CHECK(got == unit);
}

TEST(add_frame_frame_end_flush_seals_tail) {
  // Two frames whose sizes don't align to the symbol size: without the
  // frame-end flush, frame 1's tail symbol would only seal when frame 2
  // arrives. With it, each add_frame's bodies alone reconstruct the frame.
  UepEncoder enc(wide_layers(), 15);
  for (int fi = 0; fi < 2; ++fi) {
    UepDecoder dec(wide_layers(), 200);
    dec.set_wide_frag(true);
    auto unit = mk_frame_unit(static_cast<uint16_t>(fi), 1000 * fi, 5000 + 37 * fi, 0x20);
    auto bodies = enc.add_frame(2, unit.data(), unit.size(), 1);
    size_t bytes = 0;
    std::map<uint16_t, std::vector<uint8_t>> chunks;
    for (auto& b : bodies)
      for (auto& p : dec.add_body(b.body.data(), b.body.size(), 1))
        chunks[static_cast<uint16_t>(p.pkt[2] | (p.pkt[3] << 8))]
            .assign(p.pkt.begin() + 6, p.pkt.end());
    for (auto& [i, c] : chunks) bytes += c.size();
    CHECK(bytes == unit.size());  // tail sealed — nothing waiting on frame 2
  }
}

TEST(add_frame_shed_layer_drops) {
  UepEncoder enc(wide_layers(), 15);
  enc.set_shed(3, true);
  auto unit = mk_frame_unit(1, 0, 1000, 0);
  CHECK(enc.add_frame(3, unit.data(), unit.size(), 1).empty());
  CHECK(enc.dropped(3) == 1);
}

TEST(set_wide_frag_false_keeps_narrow_path) {
  // Narrow mode unchanged: completed (reassembled) packets come out.
  UepDecoder dec(wide_layers(), 200);  // wide cfg but narrow reasm default
  auto layers = wide_layers();
  for (auto& c : layers) c.wide_frag = false;
  UepEncoder enc(layers, 15);
  std::vector<uint8_t> rtp(200, 0x33);
  rtp[0] = 0x80;  // minimal RTP-ish; classify_rtp routes unparseable to 0
  auto bodies = enc.add_rtp(rtp.data(), rtp.size(), 1);
  auto flushed = enc.flush_all();
  bodies.insert(bodies.end(), flushed.begin(), flushed.end());
  std::vector<DecodedRtp> out;
  for (auto& b : bodies)
    for (auto& p : dec.add_body(b.body.data(), b.body.size(), 1)) out.push_back(p);
  REQUIRE(out.size() == 1);
  CHECK(out[0].pkt == rtp);
}

MTEST_MAIN
