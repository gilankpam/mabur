#include "frame_file_source.h"

#include <cstdio>

namespace maburgs {
namespace {
uint16_t rd_u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
// Mirrors gs/src/radio_frontend.cpp's dot11_body_offset(): QoS-Data (FC
// 0x88, the post-A-MPDU drone wire) carries a 26-byte header; everything
// else (the legacy probe-req 0x40 wire) parses at 24. Duplicated rather
// than shared to avoid a mabur_gs_core <-> mabur_gs_radio link cycle
// (frame_file_source.cpp lives in gs_core, which gs_radio depends on).
size_t dot11_len(const uint8_t* dot11) { return dot11[0] == 0x88 ? 26 : 24; }
}  // namespace

FrameFileSource::FrameFileSource(const std::string& path, Options opt)
    : opt_(opt) {
  if (opt_.cards < 1) opt_.cards = 1;
  for (int c = 0; c < opt_.cards; ++c)
    rng_.push_back(opt_.seed + static_cast<uint32_t>(c));
  ok_ = load(path);
}

bool FrameFileSource::load(const std::string& path) {
  FILE* f = fopen(path.c_str(), "rb");
  if (!f) return false;
  uint8_t lenb[4];
  while (fread(lenb, 1, 4, f) == 4) {
    const uint32_t len = static_cast<uint32_t>(lenb[0]) | (lenb[1] << 8) |
                         (static_cast<uint32_t>(lenb[2]) << 16) |
                         (static_cast<uint32_t>(lenb[3]) << 24);
    std::vector<uint8_t> frame(len);
    if (len == 0 || fread(frame.data(), 1, len, f) != len) break;
    ++frames_read_;
    if (len < 4) { ++malformed_; continue; }
    const size_t rl = rd_u16(frame.data() + 2);   // radiotap it_len
    if (rl + 1 > len) { ++malformed_; continue; }
    const size_t dot11_hdr = dot11_len(frame.data() + rl);
    if (rl + dot11_hdr > len) { ++malformed_; continue; }
    Frame out;
    out.mac_seq = static_cast<uint16_t>(rd_u16(frame.data() + rl + 22) >> 4);
    out.body.assign(frame.begin() + static_cast<long>(rl + dot11_hdr), frame.end());
    frames_.push_back(std::move(out));
  }
  fclose(f);
  return true;
}

bool FrameFileSource::card_drops(int card) {
  uint32_t& s = rng_[static_cast<size_t>(card)];
  s = s * 1664525u + 1013904223u;
  return static_cast<int>((s >> 16) % 100) < opt_.drop_pct;
}

std::optional<mabur::node::RxBody> FrameFileSource::next() {
  while (frame_i_ < frames_.size()) {
    const size_t frame_index = frame_i_;  // capture before ++card_i_ block may bump frame_i_
    const Frame& f = frames_[frame_index];
    const int card = card_i_;
    if (++card_i_ >= opt_.cards) {
      card_i_ = 0;
      ++frame_i_;
    }
    if (card_drops(card)) {
      ++dropped_;
      continue;
    }
    mabur::node::RxBody m;
    m.card_id = static_cast<uint8_t>(card);
    m.mono_us = (frame_index + 1) * 900;  // same air frame -> same timestamp across cards, monotone across frames
    m.rssi[0] = 40; m.rssi[1] = 42;
    m.snr[0] = 25; m.snr[1] = 24;
    m.crc_ok = true;
    m.mac_seq = f.mac_seq;
    m.body = f.body;
    return m;
  }
  return std::nullopt;
}

}  // namespace maburgs
