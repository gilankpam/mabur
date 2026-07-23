// tests/test_frame_e2e.cpp — spec's host loopback: synthetic frames ->
// wire bodies -> decode -> RTP; assert 0 truncations, monotonic ts,
// markers per frame, exact Annex-B reconstruction (incl. under body loss
// within FEC overhead).
#include <cstring>
#include <map>
#include <random>
#include <vector>
#include "mtest.h"
#include "frame_stream.h"
#include "rtp_packetizer.h"
#include "mabur/frame_wire.h"
#include "mabur/nal.h"
#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"
using namespace mabur;

namespace {
std::array<UepLayerCfg, 4> layers() {
  std::array<UepLayerCfg, 4> l{};
  const double ov[4] = {1.0, 0.75, 0.5, 0.5};
  for (int i = 0; i < 4; ++i) {
    l[i].fec.symbol_size = 164;
    l[i].fec.window = 128;
    l[i].fec.overhead = ov[i];
    l[i].blocks_per_body = 4;
    l[i].wide_frag = true;
  }
  return l;
}

std::vector<uint8_t> mk_annexb_frame(std::mt19937& rng, bool idr, uint8_t tid,
                                     size_t slice_len) {
  auto nal = [&](uint8_t type, size_t len) {
    std::vector<uint8_t> v = {0, 0, 0, 1, static_cast<uint8_t>(type << 1),
                              static_cast<uint8_t>(tid + 1)};
    for (size_t i = 0; i < len; ++i)
      v.push_back(static_cast<uint8_t>(rng()));
    return v;
  };
  std::vector<uint8_t> f;
  auto app = [&](std::vector<uint8_t> v) { f.insert(f.end(), v.begin(), v.end()); };
  if (idr) { app(nal(32, 30)); app(nal(33, 60)); app(nal(34, 12)); app(nal(19, slice_len)); }
  else app(nal(1, slice_len));
  return f;
}
}  // namespace

TEST(frame_e2e_clean_and_lossy) {
  std::mt19937 rng(1234);
  UepEncoder enc(layers(), 15);
  UepDecoder dec(layers(), 200);
  dec.set_wide_frag(true);

  std::vector<std::vector<uint8_t>> emitted_frames;
  std::vector<uint8_t> cur;
  uint64_t clean = 0, truncated = 0;
  std::vector<uint32_t> ts_per_frame;
  int markers = 0;
  uint32_t last_ts = 0;

  maburgs::RtpPacketizer pktz(
      {97, 1, 1400, 16667}, [&](const std::vector<uint8_t>& p) {
        if (p[1] & 0x80) ++markers;
        last_ts = (static_cast<uint32_t>(p[4]) << 24) | (p[5] << 16) |
                  (p[6] << 8) | p[7];
      });
  maburgs::FrameStream fs(
      {50, 8},
      {[&](const framewire::FrameHdr& h) { pktz.begin_frame(h); cur.clear(); },
       [&](const uint8_t* d, size_t n) { cur.insert(cur.end(), d, d + n);
         pktz.data(d, n); },
       [&](bool c) { pktz.end_frame(c);
         if (c) { ++clean; emitted_frames.push_back(cur);
                  // last_ts now holds THIS frame's ts (its marker packet
                  // just emitted) — record it here, not at begin_frame.
                  ts_per_frame.push_back(last_ts); }
         else ++truncated; }});

  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::vector<std::vector<uint8_t>> inputs;
  uint64_t now = 1;
  for (int fi = 0; fi < 120; ++fi) {
    bool idr = fi % 30 == 0;
    uint8_t tid = static_cast<uint8_t>(fi % 3);
    // IDR bursts >40 KB to exercise wide fragmentation end-to-end.
    auto ab = mk_annexb_frame(rng, idr, idr ? 0 : tid, idr ? 60000 : 12000);
    inputs.push_back(ab);
    int sid = classify_frame(ab.data(), ab.size());
    std::vector<uint8_t> unit(framewire::kFrameHdrLen + ab.size());
    framewire::FrameHdr h;
    h.frame_id = static_cast<uint16_t>(fi);
    h.flags = idr ? framewire::kFlagIdr : 0;
    h.pts_us = 16667u * static_cast<uint32_t>(fi);
    framewire::pack_frame_hdr(h, unit.data());
    std::memcpy(unit.data() + 8, ab.data(), ab.size());
    auto bodies = enc.add_frame(sid, unit.data(), unit.size(), now);
    for (auto& b : bodies) {
      if (u(rng) < 0.05) continue;  // 5% body loss — well inside overhead
      for (auto& p : dec.add_body(b.body.data(), b.body.size(), now))
        fs.push_fragment(p.stream_id, p.pkt.data(), p.pkt.size(), now);
    }
    fs.poll(now);
    ++now;
  }
  fs.poll(now + 100);

  // FEC must absorb 5% body loss: every frame reconstructs exactly.
  CHECK(truncated == 0);
  REQUIRE(clean == 120);
  for (size_t i = 0; i < inputs.size(); ++i) CHECK(emitted_frames[i] == inputs[i]);
  CHECK(markers == 120);  // one marker per complete access unit
  // 90 kHz media clock: ts == floor(pts_us * 9 / 100) exactly. The frame
  // period is 16667 us -> 1500.03 ticks, so consecutive deltas are 1500 or
  // 1501 (mean 1500.03); pinning the absolute value is the correct, stronger
  // invariant (a constant 1500-tick delta is mathematically impossible here).
  for (size_t i = 0; i < ts_per_frame.size(); ++i) {
    uint32_t expect =
        static_cast<uint32_t>((static_cast<uint64_t>(16667u) * i * 9) / 100);
    CHECK(ts_per_frame[i] == expect);
  }
}

MTEST_MAIN
