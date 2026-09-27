#include "gsweb_core.h"

#include <array>
#include <cstring>
#include <utility>

#include "mabur/rc_proto.h"
#include "mabur/sbi.h"

namespace gsweb {
namespace {
constexpr uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};

// Same as maburgs Config::uep_layers(): only symbol_size is decode-relevant.
std::array<mabur::UepLayerCfg, 2> layers(int symbol_size) {
  std::array<mabur::UepLayerCfg, 2> l{};
  for (auto& x : l) {
    x.fec = mabur::SwConfig{symbol_size, 128, 0.5};
    x.blocks_per_body = 4;
  }
  return l;
}
}  // namespace

bool frame_to_body(const uint8_t* d, size_t len, bool crc_ok,
                   mabur::node::RxBody& out) {
  const size_t off = (len >= 1 && d[0] == 0x88) ? 26 : 24;
  if (len < off + 1) return false;
  if (crc_ok && std::memcmp(d + 10, kSa, 6) != 0) return false;
  out.crc_ok = crc_ok;
  out.mac_seq = static_cast<uint16_t>((d[22] | (d[23] << 8)) >> 4);
  out.body.assign(d + off, d + len);
  return true;
}

RxCore::RxCore(int symbol_size, std::function<void(Au&&)> on_au)
    : block_payload_(14 + symbol_size),
      dec_(layers(symbol_size), 512, 192),
      fs_({50, 8},
          {[this](const mabur::framewire::FrameHdr& h, uint8_t sid) {
             cur_ = Au{};
             cur_.pts_us = h.pts_us;
             cur_.sid = sid;
             cur_.flags = h.flags;
             if (on_first) on_first();
           },
           [this](const uint8_t* p, size_t n) {
             cur_.data.insert(cur_.data.end(), p, p + n);
           },
           [this](bool complete, const maburgs::AuLatMeta& lat) {
             cur_.complete = complete;
             cur_.t_first_us = lat.t_first_us;
             cur_.t_complete_us = lat.t_complete_us;
             ++(complete ? st_.aus_complete : st_.aus_truncated);
             on_au_(std::move(cur_));
             cur_ = Au{};
           }}),
      on_au_(std::move(on_au)) {}

void RxCore::on_body(const mabur::node::RxBody& m, uint64_t now_ms) {
  ++st_.bodies;
  const uint8_t* b = m.body.data();
  const size_t n = m.body.size();
  if (mabur::rc::frame_type(b, n) >= 0) { ++st_.rc; return; }
  const int sid = mabur::sbi_peek_stream_id(b, n);
  if (sid == mabur::kMspStreamId || sid == mabur::kProbeStreamId) {
    ++st_.side;
    return;
  }
  // A symbol_size mismatch is rejected by sbi_unpack's header check before
  // SwDecoder's bad_cfg counter ever sees it, and UepDecoder does not count
  // it: peek the SBI header's block_payload (u16 LE at offset 4) ourselves
  // on CRC-good bodies, so "0 AUs" is explained.
  if (m.crc_ok && n >= 6 && (b[4] | (b[5] << 8)) != block_payload_) ++hdr_bad_cfg_;
  for (auto& f : dec_.add_body(b, n, now_ms, m.mcs, m.mono_us, m.crc_ok))
    fs_.push_fragment(f.stream_id, f.frag.data(), f.frag.size(), now_ms,
                      maburgs::FragArrival{f.body_mono_us, f.q_ms, f.enc_us, f.air_ms});
}

void RxCore::poll(uint64_t now_ms) { fs_.poll(now_ms); }

CoreStats RxCore::stats() const {
  CoreStats s = st_;
  s.bad_cfg = hdr_bad_cfg_ + dec_.stats(0).symbols_bad_cfg + dec_.stats(1).symbols_bad_cfg;
  return s;
}

}  // namespace gsweb
