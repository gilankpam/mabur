#include "aggregator.h"

#include "mabur/rc_proto.h"

namespace maburgs {
namespace {
// A monster gap is a link outage, not per-frame information — cap its
// contribution so one outage doesn't dominate the delivery ratio (same
// stance as the Python RungWindow's bounded walk).
constexpr uint16_t kMaxSeqGap = 512;
constexpr double kEmaAlpha = 0.1;
}  // namespace

Aggregator::Aggregator(const std::array<mabur::UepLayerCfg, 4>& layers,
                       uint64_t block_max_age_ms, int n_cards)
    : dec_(layers, block_max_age_ms),
      cards_(static_cast<size_t>(n_cards)) {}

void Aggregator::on_rx_body(const mabur::node::RxBody& m) {
  if (m.card_id >= cards_.size()) {
    ++bad_card_msgs_;
    return;
  }
  CardTrack& c = cards_[m.card_id];
  ++c.frames;
  c.last_frame_us = m.mono_us;

  if (!m.crc_ok) {
    ++c.crc_fail;
  } else {
    // Per-card delivery from 12-bit hw-seq gaps. The drone's counter numbers
    // every injected frame (video and the rare DISC_ACK alike).
    if (c.has_seq) {
      uint16_t gap = static_cast<uint16_t>((m.mac_seq - c.last_seq) & 0x0FFF);
      if (gap == 0) gap = 1;  // duplicate/retry: count as one delivered frame
      c.seq_expected += gap > kMaxSeqGap ? 1 : gap;
    } else {
      c.seq_expected += 1;
      c.has_seq = true;
    }
    c.seq_received += 1;
    c.last_seq = m.mac_seq;
    last_video_seq_ = m.mac_seq;

    const double snr = static_cast<double>(m.snr[0] > m.snr[1] ? m.snr[0] : m.snr[1]);
    if (!c.has_ema) {
      c.rssi_b_ema = m.rssi[1];
      c.snr_ema = snr;
      c.has_ema = true;
    } else {
      c.rssi_b_ema = (1 - kEmaAlpha) * c.rssi_b_ema + kEmaAlpha * m.rssi[1];
      c.snr_ema = (1 - kEmaAlpha) * c.snr_ema + kEmaAlpha * snr;
    }
  }

  if (mabur::rc::frame_type(m.body.data(), m.body.size()) >= 0) {
    ++c.rc_frames;
    if (rc_sink_) rc_sink_(m.card_id, m.body, m.mono_us);
    return;
  }
  ++c.video_bodies;
  last_video_us_ = m.mono_us;
  for (const auto& r : dec_.add_body(m.body.data(), m.body.size(), m.mono_us / 1000))
    if (rtp_sink_) rtp_sink_(r);
}

}  // namespace maburgs
