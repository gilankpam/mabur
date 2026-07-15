#include "aggregator.h"

#include "mabur/rc_proto.h"
#include "mabur/sbi.h"

namespace maburgs {
namespace {
// A monster gap is a link outage, not per-frame information — cap its
// contribution so one outage doesn't dominate the delivery ratio (same
// stance as the Python RungWindow's bounded walk).
constexpr uint16_t kMaxSeqGap = 512;
constexpr double kEmaAlpha = 0.1;
}  // namespace

Aggregator::Aggregator(const std::array<mabur::UepLayerCfg, 4>& layers,
                       uint64_t decode_deadline_ms, uint32_t seq_horizon,
                       int n_cards)
    : dec_(layers, decode_deadline_ms, seq_horizon),
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
    // every injected frame (video and the rare DISC_ACK alike). last_seq is
    // a MAX-seq high-water mark, not the previous frame: the drone's
    // parallel USB feed can swap ≤3-frame URB batches on air, and a
    // late-but-delivered frame must credit delivery (received++ with no
    // expected bump — its slot was already counted when the leading frame
    // advanced the mark). Walking the previous frame instead booked every
    // swap as an outage AND re-counted the gap on the way back up.
    if (c.has_seq) {
      const uint16_t adv =
          static_cast<uint16_t>((m.mac_seq - c.last_seq) & 0x0FFF);
      if (adv == 0) {
        c.seq_expected += 1;  // duplicate/retry: count as one delivered frame
      } else if (adv < 2048) {
        c.seq_expected += adv > kMaxSeqGap ? 1 : adv;
        c.last_seq = m.mac_seq;
      }
      // adv >= 2048: behind the mark (reorder) — expected already counted.
    } else {
      c.seq_expected += 1;
      c.has_seq = true;
      c.last_seq = m.mac_seq;
    }
    c.seq_received += 1;
    last_video_seq_ = m.mac_seq;

    const double snr = static_cast<double>(m.snr[0] > m.snr[1] ? m.snr[0] : m.snr[1]);
    if (!c.has_ema) {
      c.rssi_b_ema = m.rssi[1];
      c.snr_ema = snr;
      c.snr_a_ema = m.snr[0];
      c.snr_b_ema = m.snr[1];
      c.has_ema = true;
    } else {
      c.rssi_b_ema = (1 - kEmaAlpha) * c.rssi_b_ema + kEmaAlpha * m.rssi[1];
      c.snr_ema = (1 - kEmaAlpha) * c.snr_ema + kEmaAlpha * snr;
      c.snr_a_ema = (1 - kEmaAlpha) * c.snr_a_ema + kEmaAlpha * m.snr[0];
      c.snr_b_ema = (1 - kEmaAlpha) * c.snr_b_ema + kEmaAlpha * m.snr[1];
    }
  }

  if (mabur::rc::frame_type(m.body.data(), m.body.size()) >= 0) {
    ++c.rc_frames;
    if (rc_sink_) rc_sink_(m.card_id, m.body, m.mono_us);
    return;
  }
  if (mabur::sbi_peek_stream_id(m.body.data(), m.body.size()) == mabur::kMspStreamId) {
    if (msp_sink_) msp_sink_(m.body.data(), m.body.size(), m.mono_us);
    return;
  }
  ++c.video_bodies;
  last_video_us_ = m.mono_us;
  for (const auto& r : dec_.add_body(m.body.data(), m.body.size(), m.mono_us / 1000))
    if (rtp_sink_) rtp_sink_(r);
}

}  // namespace maburgs
