// rcf_dump: reads a raw RCF wire frame (as dumped by maburgs's
// MABUR_HOP_INJECT test seam, gs/src/main.cpp's run_hop_inject_test()) and
// prints its fields, decoded with the REAL mabur::rc::parse_rcf() -- so the
// gs_e2e hop scenario (tests/integration/run_gs_e2e.sh) checks what the
// wire actually says instead of a shell script inferring byte offsets by
// hand (task-15-brief.md: "parse the bytes with parse_rcf, do not infer").
#include <cstdio>
#include <vector>

#include "mabur/rc_proto.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: rcf_dump <rcf.bin>\n");
    return 2;
  }
  FILE* f = std::fopen(argv[1], "rb");
  if (!f) {
    std::fprintf(stderr, "error: cannot open %s\n", argv[1]);
    return 2;
  }
  std::vector<uint8_t> buf(1024);
  const size_t n = std::fread(buf.data(), 1, buf.size(), f);
  std::fclose(f);
  buf.resize(n);

  if (mabur::rc::frame_type(buf.data(), buf.size()) != mabur::rc::T_RCF) {
    std::fprintf(stderr, "error: %s is not a T_RCF frame (%zu bytes)\n", argv[1], n);
    return 1;
  }
  const auto r = mabur::rc::parse_rcf(buf.data(), buf.size());
  if (!r) {
    std::fprintf(stderr, "error: parse_rcf failed on %zu bytes from %s\n", n, argv[1]);
    return 1;
  }
  // key=value, one per line: trivial for the shell test to grep/cut.
  std::printf("vtx_id=%u\n", r->vtx_id);
  std::printf("seq=%u\n", r->seq);
  std::printf("profile=%u\n", r->profile);
  std::printf("hop_ch=%u\n", r->hop_ch);
  std::printf("hop_epoch=%u\n", r->hop_epoch);
  std::printf("probe_profile=%u\n", r->probe_profile);
  return 0;
}
