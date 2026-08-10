#include "aggregator.h"

#include "mabur/rc_proto.h"
#include "mabur/sbi.h"

namespace maburgs {
namespace {
// A monster gap is a link outage, not per-frame information — cap its
// contribution so one outage doesn't dominate the delivery ratio (same
// bounded-walk stance as common/src/uep_decoder.cpp's note_delivery()).
constexpr uint16_t kMaxSeqGap = 512;
constexpr double kEmaAlpha = 0.1;

// EVM fold shared by CardTrack and ClassTrack (identical field names).
// Unlike rssi/snr, zero samples are skipped per chain — devourer's own
// RxQualityAccumulator does the same so mixed streams don't bias toward 0.
template <typename Track>
static void fold_evm(Track& t, int8_t a, int8_t b) {
  // 0 = no phy status on this frame; -128 (int8 min) = the chip's
  // "not measured" sentinel for an absent spatial stream (every 1SS
  // non-STBC frame carries evm[1] = -128 — bench-measured 2026-08-10,
  // txagcbench sweep). Neither is a sample; folding -128 would peg the
  // EMA (and the best-of min below) at an impossible -64 dB.
  auto sampled = [](int8_t v) { return v != 0 && v != INT8_MIN; };
  auto fold = [](double& ema, bool& has, double v) {
    if (!has) { ema = v; has = true; }
    else ema = (1 - kEmaAlpha) * ema + kEmaAlpha * v;
  };
  if (sampled(a)) fold(t.evm_a_ema, t.evm_a_has, a);
  if (sampled(b)) fold(t.evm_b_ema, t.evm_b_has, b);
  const int8_t best = (sampled(a) && sampled(b)) ? (a < b ? a : b)
                                                 : (sampled(a) ? a : b);
  if (sampled(best)) fold(t.evm_ema, t.evm_has, best);
}
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
    // Per-card delivery from 12-bit hw-seq gaps. The drone's VIDEO counter
    // numbers every injected decoder-bound body. last_seq is a MAX-seq
    // high-water mark, not the previous frame: the drone's parallel USB feed
    // can swap ≤3-frame URB batches on air, and a late-but-delivered frame
    // must credit delivery (received++ with no expected bump — its slot was
    // already counted when the leading frame advanced the mark). Walking the
    // previous frame instead booked every swap as an outage AND re-counted
    // the gap on the way back up.
    //
    // Restricted to decoder-bound bodies (neither rc nor MSP): drone-
    // originated RC frames (DISC_ACK, the 1 Hz T_TELEM) and MSP frames carry
    // their OWN independent 802.11 seq counters on the drone, so walking them
    // against the video high-water mark can book up to kMaxSeqGap phantom
    // expected seqs per such frame — a periodic phantom loss_pct spike.
    // Mirrors main.cpp's score-window filter, which excludes RC+MSP from
    // ScoreWindow::seq_gap_loss() for exactly this reason.
    const bool decoder_bound_seq =
        rc_t < 0 && stream_id != mabur::kMspStreamId;
    if (decoder_bound_seq) {
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
    }

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
    fold_evm(c, m.evm[0], m.evm[1]);

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
      ct.bytes += m.body.size();
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
      fold_evm(ct, m.evm[0], m.evm[1]);
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
