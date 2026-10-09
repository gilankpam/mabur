// Fixture AUs through the real encoder -> body loss -> decoder ->
// FrameStream: salvaged AUs appear, and each is a gap-free 4-slice picture
// whose kept slices are byte-identical to the originals.
#include <cstring>
#include <vector>
#include "frame_stream.h"
#include "mabur/frame_wire.h"
#include "mabur/hevc_ps.h"
#include "mabur/hevc_slice.h"
#include "mabur/nal.h"
#include "mabur/uep_decoder.h"
#include "mabur/uep_encoder.h"
#include "mtest.h"
#include "slice_fixture.h"
using namespace mabur;

namespace {
std::array<UepLayerCfg, 2> layers() {
  std::array<UepLayerCfg, 2> l{};
  for (auto& c : l) {
    c.fec.symbol_size = 332;
    c.fec.window = 32;
    c.fec.overhead = 0.1;   // little repair: a burst stays a hole
    c.blocks_per_body = 4;
  }
  return l;
}
}  // namespace

TEST(salvaged_aus_are_gapfree_pictures_with_exact_kept_slices) {
  const auto aus = mtest::load_slice_fixture();
  hevc::ParamTracker pt;
  pt.feed(aus[3].data(), aus[3].size());
  UepEncoder enc(layers(), 15);
  UepDecoder dec(layers());
  struct Ev { std::vector<uint8_t> b; bool complete; maburgs::SliceSalvage s; };
  std::vector<Ev> evs;
  std::vector<uint8_t> cur;
  maburgs::FrameStream fs({50, 8},
      {[&](const framewire::FrameHdr&, uint8_t) { cur.clear(); },
       [&](const uint8_t* d, size_t n) { cur.insert(cur.end(), d, d + n); },
       [&](bool c, const maburgs::AuLatMeta& lat) { evs.push_back({cur, c, lat.slice}); }});
  std::vector<size_t> sent;   // fixture index per frame
  uint64_t now = 1;
  for (int fi = 0; fi < 60; ++fi, now += 17) {
    const size_t src = fi % 15 == 0 ? 3 : 4 + (fi % 3);   // refresh start every 15
    const auto& ab = aus[src];
    std::vector<uint8_t> unit(framewire::kFrameHdrLen + ab.size());
    framewire::FrameHdr h;
    h.frame_id = static_cast<uint16_t>(fi);
    h.slice_rows = src == 3 ? 0 : 5;
    h.pts_us = 16667u * static_cast<uint32_t>(fi);
    framewire::pack_frame_hdr(h, unit.data());
    std::memcpy(unit.data() + 8, ab.data(), ab.size());
    auto bodies = enc.add_frame(classify_frame(ab.data(), ab.size()), unit.data(), unit.size(), now);
    const bool burst = fi % 15 == 7;   // one damaged frame per refresh period
    // Drop the middle third of this frame's bodies: wide enough that FEC's
    // modest overhead (0.1) cannot repair it, so the AU reaches FrameStream
    // with a genuine mid-frame hole rather than a fully recovered picture.
    const size_t b0 = bodies.size() / 3, b1 = bodies.size() * 2 / 3;
    for (size_t bi = 0; bi < bodies.size(); ++bi) {
      if (burst && bi >= b0 && bi < b1) continue;
      for (auto& p : dec.add_body(bodies[bi].body.data(), bodies[bi].body.size(), now))
        fs.push_fragment(p.stream_id, p.frag.data(), p.frag.size(), now);
    }
    sent.push_back(src);
    fs.poll(now);
  }
  fs.poll(now + 200);
  REQUIRE(evs.size() == sent.size());
  size_t salvaged = 0;
  for (size_t i = 0; i < evs.size(); ++i) {
    const auto& e = evs[i];
    const auto& orig = aus[sent[i]];
    if (e.complete) { CHECK(e.b == orig); continue; }
    if (!e.s.salvaged) continue;
    ++salvaged;
    const auto got = mtest::slice_nals(e.b);
    const auto want = mtest::slice_nals(orig);
    REQUIRE(got.size() == want.size());
    for (size_t k = 0; k < got.size(); ++k) {
      if (got[k] == want[k]) continue;
      hevc::SliceHeader sh;
      REQUIRE(hevc::parse_slice_header(got[k].data() + 4, got[k].size() - 4, pt.sps(), pt.pps(), &sh) ==
              hevc::SliceParse::kOk);
      CHECK(sh.address == 150u * k);
    }
  }
  CHECK(salvaged >= 2);
  CHECK(fs.slice_salvaged() == salvaged);
}

MTEST_MAIN
