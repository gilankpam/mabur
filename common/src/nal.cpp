#include "mabur/nal.h"

namespace mabur {

NalInfo parse_hevc_nal(const uint8_t* nal, size_t len) {
  NalInfo n;
  if (len < 2) return n;  // malformed -> treat as base layer
  n.type = (nal[0] >> 1) & 0x3F;
  int tid = static_cast<int>(nal[1] & 0x07) - 1;
  n.tid = tid < 0 ? 0 : static_cast<uint8_t>(tid);
  n.critical = (n.type >= 16 && n.type <= 23) || (n.type >= 32 && n.type <= 34);
  return n;
}

int classify_frame(const uint8_t* annexb, size_t len) {
  if (!annexb || len < 5) return 0;
  int sid = -1;
  for (size_t i = 0; i + 4 < len; ++i) {
    if (annexb[i] != 0x00 || annexb[i + 1] != 0x00 || annexb[i + 2] != 0x01)
      continue;
    const uint8_t* nal = annexb + i + 3;
    size_t nal_max = len - (i + 3);
    NalInfo n = parse_hevc_nal(nal, nal_max);
    if (n.critical) return 0;
    // nal_max >= 2: a truncated header parses as type 0, which now means
    // TRAIL_N (sid 3, droppable) — never classify from a default-constructed
    // NalInfo; skipping keeps the protect-up fallback.
    if (sid < 0 && nal_max >= 2 && n.type < 16)
      sid = n.type == 0 ? 3 : 1 + (n.tid < 2 ? n.tid : 2);
    i += 2;  // skip past the start code; loop ++i lands on the NAL header
  }
  return sid < 0 ? 0 : sid;
}

}  // namespace mabur
