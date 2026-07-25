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

  // Classify once: GS-self-originated RC frames (RCF/DISC) heard back on
  // the GS's own monitor-mode capture are diverted before any accounting —
  // they never touch frames/rx_bytes/seq/EMA, only self_frames + the rc
  // routing. Gated on crc_ok: real self frames are point-blank captures and
  // always CRC-clean, so a corrupt frame whose bytes happen to parse as
  // T_RCF/T_DISC must NOT be diverted here — it still owes frames/crc_fail/
  // rx_bytes accounting like any other received frame.
  const int rc_t = mabur::rc::frame_type(m.body.data(), m.body.size());
  const bool is_self =
      m.crc_ok && (rc_t == mabur::rc::T_RCF || rc_t == mabur::rc::T_DISC);
  if (is_self) {
    ++c.self_frames;
    ++c.rc_frames;
    if (rc_sink_) rc_sink_(m.card_id, m.body, m.mono_us);
    return;
  }

  const int stream_id = mabur::sbi_peek_stream_id(m.body.data(), m.body.size());

  ++c.frames;
  c.rx_bytes += m.body.size();
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

    const double rssi = static_cast<double>(m.rssi[0] > m.rssi[1] ? m.rssi[0] : m.rssi[1]);
    const double snr = static_cast<double>(m.snr[0] > m.snr[1] ? m.snr[0] : m.snr[1]);
    if (!c.has_ema) {
      c.rssi_ema = rssi;
      c.rssi_a_ema = m.rssi[0];
      c.rssi_b_ema = m.rssi[1];
      c.snr_ema = snr;
      c.snr_a_ema = m.snr[0];
      c.snr_b_ema = m.snr[1];
      c.has_ema = true;
    } else {
      c.rssi_ema = (1 - kEmaAlpha) * c.rssi_ema + kEmaAlpha * rssi;
      c.rssi_a_ema = (1 - kEmaAlpha) * c.rssi_a_ema + kEmaAlpha * m.rssi[0];
      c.rssi_b_ema = (1 - kEmaAlpha) * c.rssi_b_ema + kEmaAlpha * m.rssi[1];
      c.snr_ema = (1 - kEmaAlpha) * c.snr_ema + kEmaAlpha * snr;
      c.snr_a_ema = (1 - kEmaAlpha) * c.snr_a_ema + kEmaAlpha * m.snr[0];
      c.snr_b_ema = (1 - kEmaAlpha) * c.snr_b_ema + kEmaAlpha * m.snr[1];
    }

    // Per-RF-class EMA, mirroring the pooled block above exactly. Ctrl
    // (non-self rc frames, e.g. DISC_ACK) takes priority over stream id;
    // an unparseable/misrouted body gets no class (card totals only).
    int class_idx = -1;
    if (rc_t >= 0) {
      class_idx = static_cast<int>(RfClass::Ctrl);
    } else if (stream_id == mabur::kMspStreamId) {
      class_idx = static_cast<int>(RfClass::Msp);
    } else if (stream_id >= 0 && stream_id < 4) {
      class_idx = stream_id;
    }
    if (class_idx >= 0) {
      ClassTrack& ct = c.cls[static_cast<size_t>(class_idx)];
      ++ct.frames;
      if (!ct.has_ema) {
        ct.rssi_ema = rssi;
        ct.rssi_a_ema = m.rssi[0];
        ct.rssi_b_ema = m.rssi[1];
        ct.snr_ema = snr;
        ct.snr_a_ema = m.snr[0];
        ct.snr_b_ema = m.snr[1];
        ct.has_ema = true;
      } else {
        ct.rssi_ema = (1 - kEmaAlpha) * ct.rssi_ema + kEmaAlpha * rssi;
        ct.rssi_a_ema = (1 - kEmaAlpha) * ct.rssi_a_ema + kEmaAlpha * m.rssi[0];
        ct.rssi_b_ema = (1 - kEmaAlpha) * ct.rssi_b_ema + kEmaAlpha * m.rssi[1];
        ct.snr_ema = (1 - kEmaAlpha) * ct.snr_ema + kEmaAlpha * snr;
        ct.snr_a_ema = (1 - kEmaAlpha) * ct.snr_a_ema + kEmaAlpha * m.snr[0];
        ct.snr_b_ema = (1 - kEmaAlpha) * ct.snr_b_ema + kEmaAlpha * m.snr[1];
      }
    }
  }

  if (rc_t >= 0) {
    ++c.rc_frames;
    if (rc_sink_) rc_sink_(m.card_id, m.body, m.mono_us);
    return;
  }
  if (stream_id == mabur::kMspStreamId) {
    if (msp_sink_) msp_sink_(m.body.data(), m.body.size(), m.mono_us);
    return;
  }
  ++c.video_bodies;
  last_video_us_ = m.mono_us;
  for (const auto& r : dec_.add_body(m.body.data(), m.body.size(), m.mono_us / 1000))
    if (frag_sink_) frag_sink_(r);
}

}  // namespace maburgs
