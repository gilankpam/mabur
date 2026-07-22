#include "mabur/nal.h"
#include <algorithm>

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

namespace {

int stream_from(uint8_t tid, bool critical) {
  return critical ? 0 : 1 + std::min(static_cast<int>(tid), 2);
}

}  // namespace

int classify_rtp(const uint8_t* pkt, size_t len) {
  if (pkt == nullptr || len < 14) return 0;  // protect-up: unparseable -> critical
  if ((pkt[0] >> 6) != 2) return 0;          // not RTP v2

  size_t csrc_count = pkt[0] & 0x0F;
  size_t offset = 12 + 4 * csrc_count;
  if (offset > len) return 0;

  if (pkt[0] & 0x10) {  // extension bit
    if (offset + 4 > len) return 0;
    uint16_t ext_len = (static_cast<uint16_t>(pkt[offset + 2]) << 8) |
                        static_cast<uint16_t>(pkt[offset + 3]);
    size_t ext_total = 4 + 4 * static_cast<size_t>(ext_len);
    if (offset + ext_total > len) return 0;
    offset += ext_total;
  }

  if (offset >= len) return 0;  // no payload left

  const uint8_t* p = pkt + offset;
  size_t plen = len - offset;

  uint8_t nal_type = (p[0] >> 1) & 0x3F;

  if (nal_type == 49) {  // FU
    if (plen < 3) return 0;
    uint8_t real_type = p[2] & 0x3F;
    int tid = static_cast<int>(p[1] & 0x07) - 1;
    uint8_t tidv = tid < 0 ? 0 : static_cast<uint8_t>(tid);
    bool critical = (real_type >= 16 && real_type <= 23) || (real_type >= 32 && real_type <= 34);
    return stream_from(tidv, critical);
  }

  if (nal_type == 48) {  // AP -> always critical stream
    return 0;
  }

  NalInfo info = parse_hevc_nal(p, plen);
  return stream_from(info.tid, info.critical);
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
    if (sid < 0 && n.type < 16)
      sid = 1 + (n.tid < 2 ? n.tid : 2);
    i += 2;  // skip past the start code; loop ++i lands on the NAL header
  }
  return sid < 0 ? 0 : sid;
}

}  // namespace mabur
