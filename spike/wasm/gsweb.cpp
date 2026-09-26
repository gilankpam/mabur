// THROWAWAY SPIKE (branch wasm-spike): gsweb -- mabur's receive path for the
// browser. replay: maburd --dry-run --out file -> AU records (parity with
// maburgs --dry-run --out-aus). live: added in a later task.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "au_file.h"
#include "gsweb_core.h"

namespace {

struct Opts {
  int symbol_size = 332;
  int drop_pct = 0;
  uint32_t seed = 1;
  size_t skip = 0;
};

void print_stats(const gsweb::CoreStats& s) {
  std::fprintf(stderr, "STAT bodies=%llu rc=%llu side=%llu aus=%llu trunc=%llu bad_cfg=%llu \n",
               (unsigned long long)s.bodies, (unsigned long long)s.rc,
               (unsigned long long)s.side, (unsigned long long)s.aus_complete,
               (unsigned long long)s.aus_truncated, (unsigned long long)s.bad_cfg);
}

// Mirrors maburgs FrameFileSource with cards=1: u32 LE length | radiotap |
// dot11 | body; per-frame LCG drop (seed+0); mono_us = (index+1)*900; the
// FrameStream clock is mono_us/1000 and poll() runs after every delivered
// body, then once at last+gap+1.
int run_replay(const char* in, const char* out, const Opts& o) {
  FILE* f = std::fopen(in, "rb");
  if (!f) { std::fprintf(stderr, "error: cannot read %s\n", in); return 2; }
  gsweb::AuFileWriter w;
  if (!w.open(out)) { std::fprintf(stderr, "error: cannot write %s\n", out); return 2; }
  gsweb::RxCore core(o.symbol_size, [&](gsweb::Au&& a) { w.write(a); });
  uint32_t rng = o.seed;
  uint64_t last_ms = 0;
  size_t index = 0;
  uint8_t lenb[4];
  while (std::fread(lenb, 1, 4, f) == 4) {
    const uint32_t len = lenb[0] | (lenb[1] << 8) | (uint32_t(lenb[2]) << 16) |
                         (uint32_t(lenb[3]) << 24);
    std::vector<uint8_t> frame(len);
    if (len == 0 || std::fread(frame.data(), 1, len, f) != len) break;
    const size_t i = index++;
    if (len < 4) continue;
    const size_t rl = frame[2] | (frame[3] << 8);
    if (rl + 1 > len) continue;
    // FrameFileSource discards malformed frames at load time, before any
    // LCG step: same dot11-length rule here, ahead of the rng.
    if (rl + ((frame[rl] == 0x88) ? 26 : 24) > len) continue;
    rng = rng * 1664525u + 1013904223u;
    const bool drop = static_cast<int>((rng >> 16) % 100) < o.drop_pct;
    if (drop || i < o.skip) continue;
    mabur::node::RxBody m;
    if (!gsweb::frame_to_body(frame.data() + rl, len - rl, true, m)) continue;
    m.mono_us = (i + 1) * 900;
    const uint64_t now_ms = m.mono_us / 1000;
    core.on_body(m, now_ms);
    core.poll(now_ms);
    last_ms = now_ms;
  }
  std::fclose(f);
  core.poll(last_ms + 50 + 1);
  print_stats(core.stats());
  return 0;
}

int usage() {
  std::fprintf(stderr,
               "usage: gsweb replay <in-bodies> <out-aus> [--symbol-size N] "
               "[--drop-pct P] [--seed S] [--skip N]\n");
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) return usage();
  const std::string mode = argv[1];
  if (mode == "replay") {
    if (argc < 4) return usage();
    Opts o;
    for (int i = 4; i + 1 < argc; i += 2) {
      const std::string k = argv[i];
      if (k == "--symbol-size") o.symbol_size = std::atoi(argv[i + 1]);
      else if (k == "--drop-pct") o.drop_pct = std::atoi(argv[i + 1]);
      else if (k == "--seed") o.seed = static_cast<uint32_t>(std::atol(argv[i + 1]));
      else if (k == "--skip") o.skip = static_cast<size_t>(std::atol(argv[i + 1]));
      else return usage();
    }
    return run_replay(argv[2], argv[3], o);
  }
  return usage();
}
