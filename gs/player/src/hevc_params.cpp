#include "hevc_params.h"

#include <cassert>

namespace maburplay {
namespace {

// Locates the earliest start code (00 00 01 or 00 00 00 01) at or after
// `from`. Returns false if none remains before `n`. Mirrors the zero-run
// idea in RtpPacketizer::feed_byte (gs/src/rtp_packetizer.cpp), but scans
// a whole buffer at once rather than a byte stream.
bool find_start_code(const uint8_t* au, size_t n, size_t from, size_t* pos,
                      size_t* code_len) {
  for (size_t j = from; j + 3 <= n; ++j) {
    if (au[j] != 0x00 || au[j + 1] != 0x00) continue;
    if (au[j + 2] == 0x01) {
      *pos = j;
      *code_len = 3;
      return true;
    }
    if (au[j + 2] == 0x00 && j + 4 <= n && au[j + 3] == 0x01) {
      *pos = j;
      *code_len = 4;
      return true;
    }
  }
  return false;
}

void push_array(std::vector<uint8_t>& out, uint8_t nal_type,
                 const std::vector<uint8_t>& nal) {
  out.push_back(static_cast<uint8_t>(0x80 | nal_type));  // array_completeness=1
  out.push_back(0x00);
  out.push_back(0x01);  // numNalus=1
  uint16_t len = static_cast<uint16_t>(nal.size());
  out.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>(len & 0xFF));
  out.insert(out.end(), nal.begin(), nal.end());
}

}  // namespace

std::vector<NalView> split_nals(const uint8_t* au, size_t n) {
  std::vector<NalView> out;
  size_t pos = 0, code_len = 0;
  if (!find_start_code(au, n, 0, &pos, &code_len)) return out;
  size_t nal_begin = pos + code_len;
  for (;;) {
    size_t next_pos = 0, next_len = 0;
    bool found_next = find_start_code(au, n, nal_begin, &next_pos, &next_len);
    size_t nal_end = found_next ? next_pos : n;
    // nal_end == nal_begin happens for a start code at the buffer end, or
    // two consecutive start codes — an empty NAL, skipped rather than
    // emitted as a zero-length view.
    if (nal_end > nal_begin) {
      uint8_t type = static_cast<uint8_t>((au[nal_begin] >> 1) & 0x3F);
      out.push_back(NalView{au + nal_begin, nal_end - nal_begin, type});
    }
    if (!found_next) break;
    nal_begin = next_pos + next_len;
  }
  return out;
}

bool au_is_irap(const uint8_t* au, size_t n) {
  for (const NalView& nal : split_nals(au, n)) {
    if (nal.type >= 16 && nal.type <= 23) return true;
  }
  return false;
}

bool HevcParams::feed(const uint8_t* au, size_t n) {
  for (const NalView& nal : split_nals(au, n)) {
    switch (nal.type) {
      case 32:
        vps_.assign(nal.p, nal.p + nal.n);
        break;
      case 33:
        sps_.assign(nal.p, nal.p + nal.n);
        break;
      case 34:
        pps_.assign(nal.p, nal.p + nal.n);
        break;
      default:
        break;
    }
  }
  return complete();
}

std::vector<uint8_t> HevcParams::hvcc() const {
  assert(complete());
  std::vector<uint8_t> out;
  out.push_back(0x01);  // configurationVersion

  // sps_[0..1] is the 2-byte NAL header; sps_[2] packs
  // sps_video_parameter_set_id(4)/sps_max_sub_layers_minus1(3)/
  // sps_temporal_id_nesting_flag(1). profile_tier_level immediately
  // follows at sps_[3], byte-aligned throughout (1 byte
  // profile_space/tier/profile_idc, 4 bytes compat flags, 6 bytes
  // constraint flags, 1 byte level_idc = 12 bytes, i.e. sps_[3..14]
  // inclusive), so it copies verbatim. NOTE: this assumes no
  // emulation-prevention (0x00 0x00 0x03) bytes fall inside sps_[3..14],
  // which holds for the real encoder output this player targets.
  assert(sps_.size() >= 15);
  out.insert(out.end(), sps_.begin() + 3, sps_.begin() + 15);

  out.push_back(0xF0);
  out.push_back(0x00);  // min_spatial_segmentation_idc=0 (reserved 1111 | 0)
  out.push_back(0xFC);  // parallelismType=0 (reserved 111111 | 00)
  out.push_back(0xFC | 0x01);  // chromaFormat=1 (reserved 111111 | 01)
  out.push_back(0xF8);         // bitDepthLumaMinus8=0 (reserved 11111 | 000)
  out.push_back(0xF8);         // bitDepthChromaMinus8=0
  out.push_back(0x00);
  out.push_back(0x00);  // avgFrameRate=0

  // constantFrameRate(2)=0 | numTemporalLayers(3)=2 (SVC-T) |
  // temporalIdNested(1)=0 | lengthSizeMinusOne(2)=3 -> 00 010 0 11 = 0x13
  out.push_back(0x13);

  out.push_back(0x03);  // numOfArrays: VPS, SPS, PPS

  push_array(out, 32, vps_);
  push_array(out, 33, sps_);
  push_array(out, 34, pps_);

  return out;
}

std::vector<uint8_t> annexb_to_length_prefixed(const uint8_t* au, size_t n) {
  std::vector<uint8_t> out;
  for (const NalView& nal : split_nals(au, n)) {
    uint32_t len = static_cast<uint32_t>(nal.n);
    out.push_back(static_cast<uint8_t>((len >> 24) & 0xFF));
    out.push_back(static_cast<uint8_t>((len >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(len & 0xFF));
    out.insert(out.end(), nal.p, nal.p + nal.n);
  }
  return out;
}

}  // namespace maburplay
