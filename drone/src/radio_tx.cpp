#include "radio_tx.h"

#include <cstring>

#include "RadiotapBuilder.h"
#include "TxMode.h"

namespace mabur {

namespace {

// Canonical SA devourer's build_dot11_probe_req() uses
// (../devourer/examples/streamtx/main.cpp).
constexpr uint8_t kCanonicalSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
constexpr size_t kDot11HeaderLen = 24;

devourer::TxMode to_tx_mode(const rc::LayerTxSpec& s, uint8_t bw) {
  devourer::TxMode m;
  if (s.mode == rc::PhyMode::VHT) {
    m.mode = devourer::TxMode::Mode::VHT;
    m.vht_mcs = s.mcs;
    m.vht_nss = 1;
  } else {
    m.mode = devourer::TxMode::Mode::HT;
    m.ht_mcs = s.mcs;
  }
  m.bw_mhz = bw;
  m.sgi = s.sgi;
  m.ldpc = s.ldpc;
  m.stbc = s.stbc;
  return m;
}

// Writes the 24-byte 802.11 header (frame control | duration | DA | SA | BSSID
// | seq_ctl) at `out`, matching devourer's build_dot11_probe_req() layout with
// seq_ctl carrying `seq` in the sequence-number subfield (fragment 0).
void write_dot11_header(uint8_t* out, uint16_t seq) {
  out[0] = 0x40;
  out[1] = 0x00;
  out[2] = 0x00;
  out[3] = 0x00;
  std::memset(out + 4, 0xff, 6);
  std::memcpy(out + 10, kCanonicalSa, 6);
  std::memcpy(out + 16, kCanonicalSa, 6);
  uint16_t seq_ctl = static_cast<uint16_t>(seq << 4);
  out[22] = static_cast<uint8_t>(seq_ctl & 0xff);
  out[23] = static_cast<uint8_t>((seq_ctl >> 8) & 0xff);
}

}  // namespace

RadioTx::RadioTx(FrameSink& sink) : sink_(sink) {
  cache_.store(std::make_shared<Cache>());
}

void RadioTx::set_ladder(const std::array<rc::LayerTxSpec, 4>& ladder) {
  auto next = std::make_shared<Cache>();
  for (size_t i = 0; i < ladder.size(); ++i) {
    const rc::LayerTxSpec& layer = ladder[i];
    next->layers[i].radiotap =
        devourer::build_stream_radiotap(to_tx_mode(layer, layer.bw));
  }
  // Single atomic swap: the whole radiotap table changes together, so
  // send_body() (hot thread) can never observe a torn mix of old and new
  // layer entries.
  cache_.store(next);
}

bool RadioTx::build_frame(const Cache& cache, uint8_t stream_id,
                          const uint8_t* body, size_t len,
                          std::vector<uint8_t>& out) {
  size_t idx = stream_id < cache.layers.size() ? stream_id : cache.layers.size() - 1;
  const LayerCache& lc = cache.layers[idx];

  // Missing radiotap cache entry (e.g. called before set_ladder). Sequence
  // is consumed regardless to let a ground-station gap detector see the drop.
  if (lc.radiotap.empty()) {
    seq_ = static_cast<uint16_t>((seq_ + 1) & 0xFFF);
    ++drops_;
    return false;
  }

  const std::vector<uint8_t>& radiotap = lc.radiotap;

  size_t rl = radiotap.size();
  size_t frame_len = rl + kDot11HeaderLen + len;
  if (out.size() < frame_len) out.resize(frame_len);

  std::memcpy(out.data(), radiotap.data(), rl);
  write_dot11_header(out.data() + rl, seq_);
  if (len > 0) std::memcpy(out.data() + rl + kDot11HeaderLen, body, len);
  out.resize(frame_len);

  // Sequence is consumed regardless of the eventual send() outcome, so a
  // ground-station gap detector observes any drop as a skipped seq number.
  seq_ = static_cast<uint16_t>((seq_ + 1) & 0xFFF);
  return true;
}

bool RadioTx::send_body(uint8_t stream_id, const uint8_t* body, size_t len) {
  auto cache = cache_.load();
  if (!build_frame(*cache, stream_id, body, len, scratch_)) return false;

  bool ok = sink_.send(scratch_.data(), scratch_.size());
  if (ok) {
    ++sent_;
  } else {
    ++drops_;
  }
  return ok;
}

size_t RadioTx::send_bodies(const std::vector<UepBody>& bodies) {
  if (bodies.empty()) return 0;
  auto cache = cache_.load();
  if (pool_.size() < bodies.size()) pool_.resize(bodies.size());

  std::vector<FrameSink::View> views;
  views.reserve(bodies.size());
  size_t built = 0;
  for (const auto& b : bodies) {
    if (!build_frame(*cache, b.stream_id, b.body.data(), b.body.size(),
                     pool_[built]))
      continue;  // drop already counted; seq consumed
    views.push_back(FrameSink::View{pool_[built].data(), pool_[built].size()});
    ++built;
  }
  if (views.empty()) return 0;

  size_t ok = sink_.send_many(views.data(), views.size());
  sent_ += ok;
  drops_ += views.size() - ok;
  return ok;
}

}  // namespace mabur
