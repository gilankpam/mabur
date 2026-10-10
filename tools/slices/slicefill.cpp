// Dev oracle feed for SkipSliceWriter (spec 2026-10-10-h265-slices §5.7):
// rewrites ONE picture of an Annex-B H.265 stream, replacing some of its
// slices with make_skip_slice() fills, so tools/slices/slicefill_check.py
// can decode both streams with ffmpeg (and the GS hardware) and compare.
//   slicefill IN OUT PICTURE MODE      MODE = first | middle | last | tail2
// Prints "rewrote <picture> <row0> <row1>" (luma rows of the filled band).
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "mabur/hevc_params.h"
#include "mabur/hevc_ps.h"
#include "mabur/hevc_skip_slice.h"
#include "mabur/hevc_slice.h"

using namespace mabur::hevc;

namespace {
size_t sc_pos(const std::vector<uint8_t>& b, const uint8_t* body) {
  const size_t p = static_cast<size_t>(body - b.data());
  return (p >= 4 && b[p - 4] == 0 && b[p - 3] == 0 && b[p - 2] == 0 && b[p - 1] == 1) ? p - 4 : p - 3;
}
struct Nal { size_t s, e; uint8_t type; bool first; };
}  // namespace

int main(int argc, char** argv) {
  if (argc != 5) { std::fprintf(stderr, "usage: slicefill IN OUT PICTURE MODE\n"); return 2; }
  std::ifstream f(argv[1], std::ios::binary);
  const std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  const long want = std::stol(argv[3]);
  const std::string mode = argv[4];
  if (mode != "first" && mode != "middle" && mode != "last" && mode != "tail2") {
    std::fprintf(stderr, "slicefill: unknown MODE '%s' (first|middle|last|tail2)\n"
                         "usage: slicefill IN OUT PICTURE MODE\n", mode.c_str());
    return 2;
  }

  std::vector<Nal> nals;
  for (const mabur::NalView& v : mabur::split_nals(b.data(), b.size()))
    nals.push_back({sc_pos(b, v.p), static_cast<size_t>(v.p - b.data()) + v.n, v.type,
                    v.type < 32 && v.n > 2 && (v.p[2] & 0x80)});

  std::vector<uint8_t> out;
  ParamTracker pt;
  long pic = -1;
  size_t i = 0;
  while (i < nals.size()) {
    // One picture: leading non-VCL NALs, then VCL NALs until the next first slice.
    size_t j = i;
    while (j < nals.size() && nals[j].type >= 32) ++j;
    size_t k = j + (j < nals.size() ? 1 : 0);
    while (k < nals.size() && !(nals[k].type < 32 && nals[k].first) && !(nals[k].type >= 32)) ++k;
    const std::vector<uint8_t> au(b.begin() + static_cast<long>(nals[i].s),
                                  b.begin() + static_cast<long>(k < nals.size() ? nals[k].s : b.size()));
    pt.feed(au.data(), au.size());
    ++pic;
    const size_t n_slices = k - j;
    bool done = false;
    if (pic == want && n_slices >= 2 && pt.usable()) {
      std::vector<SliceHeader> hs(n_slices);
      bool ok = true;
      for (size_t s = 0; s < n_slices && ok; ++s) {
        const Nal& nn = nals[j + s];
        const size_t body = nn.s + (b[nn.s + 2] == 1 ? 3 : 4);
        ok = parse_slice_header(b.data() + body, nn.e - body, pt.sps(), pt.pps(), &hs[s]) == SliceParse::kOk;
      }
      std::vector<bool> repl(n_slices, false);
      if (mode == "first") repl[0] = true;
      else if (mode == "middle") repl[1] = true;
      else if (mode == "last") repl[n_slices - 1] = true;
      else if (mode == "tail2") repl[n_slices - 2] = repl[n_slices - 1] = true;
      size_t t = 0;
      while (t < n_slices && repl[t]) ++t;
      if (ok && t < n_slices && hs[t].slice_type != 2) {
        const uint32_t total = pt.sps().pic_w_ctbs() * pt.sps().pic_h_ctbs();
        for (size_t s = i; s < j; ++s) out.insert(out.end(), b.begin() + static_cast<long>(nals[s].s), b.begin() + static_cast<long>(nals[s].e));
        uint32_t row0 = 0, row1 = 0;
        bool first_band = true;
        for (size_t s = 0; s < n_slices; ++s) {
          const uint32_t a = hs[s].first ? 0 : hs[s].address;
          const uint32_t e = s + 1 < n_slices ? hs[s + 1].address : total;
          if (repl[s]) {
            auto fill = make_skip_slice(pt.sps(), pt.pps(), hs[t], a, e);
            if (!fill) { std::fprintf(stderr, "fill failed\n"); return 1; }
            out.insert(out.end(), fill->begin(), fill->end());
            const uint32_t r0 = (a / pt.sps().pic_w_ctbs()) << pt.sps().log2_ctb;
            const uint32_t r1 = std::min(pt.sps().height, (e / pt.sps().pic_w_ctbs()) << pt.sps().log2_ctb);
            if (first_band) { row0 = r0; first_band = false; }
            row1 = r1;
          } else {
            const Nal& nn = nals[j + s];
            out.insert(out.end(), b.begin() + static_cast<long>(nn.s), b.begin() + static_cast<long>(nn.e));
          }
        }
        std::printf("rewrote %ld %u %u\n", pic, row0, row1);
        done = true;
      }
    }
    if (!done) out.insert(out.end(), au.begin(), au.end());
    i = k;
  }
  std::ofstream o(argv[2], std::ios::binary);
  o.write(reinterpret_cast<const char*>(out.data()), static_cast<long>(out.size()));
  return 0;
}
