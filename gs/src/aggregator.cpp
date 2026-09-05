#include "aggregator.h"

#include "mabur/rc_proto.h"
#include "mabur/sbi.h"

#include <cstdio>
#include <cstdlib>

namespace maburgs {
namespace {
// A monster gap is a link outage, not per-frame information — cap its
// contribution so one outage doesn't dominate the delivery ratio (same
// bounded-walk stance common/src/uep_decoder.cpp's note_delivery() used
// before it was deleted 2026-09-02).
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

// RSSI/SNR fold shared by CardTrack, ClassTrack and the base+enh pool
// (identical field names). Mirrors fold_evm's shape, but the validity gate
// is frame-level, not per-chain: under A-MPDU, non-first aggregated
// subframes carry no PHY status at all (rx_pkt_attrib.physt false) and
// their rssi/snr read 0/garbage on both chains, not just one. Callers must
// check RxBody.phy_valid before calling fold_rf — this function itself
// trusts its caller and always folds.
template <typename Track>
static void fold_rf(Track& t, double rssi, double snr, const uint8_t* rssi_ab,
                    const int8_t* snr_ab) {
  if (!t.has_ema) {
    t.rssi_ema = rssi;
    t.rssi_a_ema = rssi_ab[0];
    t.rssi_b_ema = rssi_ab[1];
    t.snr_ema = snr;
    t.snr_a_ema = snr_ab[0];
    t.snr_b_ema = snr_ab[1];
    t.has_ema = true;
    return;
  }
  t.rssi_ema = (1 - kEmaAlpha) * t.rssi_ema + kEmaAlpha * rssi;
  t.rssi_a_ema = (1 - kEmaAlpha) * t.rssi_a_ema + kEmaAlpha * rssi_ab[0];
  t.rssi_b_ema = (1 - kEmaAlpha) * t.rssi_b_ema + kEmaAlpha * rssi_ab[1];
  t.snr_ema = (1 - kEmaAlpha) * t.snr_ema + kEmaAlpha * snr;
  t.snr_a_ema = (1 - kEmaAlpha) * t.snr_a_ema + kEmaAlpha * snr_ab[0];
  t.snr_b_ema = (1 - kEmaAlpha) * t.snr_b_ema + kEmaAlpha * snr_ab[1];
}
}  // namespace

Aggregator::Aggregator(const std::array<mabur::UepLayerCfg, 2>& layers,
                       uint32_t seq_horizon, int n_cards)
    : dec_(layers, seq_horizon), cards_(static_cast<size_t>(n_cards)) {}

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
    static const bool gaplog_self = std::getenv("MABUR_GAPLOG") != nullptr;
    if (gaplog_self)
      std::fprintf(stderr,
                   "self card=%u seq=%u mono=%llu tsfl=%u len=%zu mcs=%u "
                   "physt=%d rssi=%d/%d snr=%d/%d rc=%d dprev_tsf=%u "
                   "dprev_mono=%llu prev_agg=%u\n",
                   static_cast<unsigned>(m.card_id),
                   static_cast<unsigned>(m.mac_seq),
                   static_cast<unsigned long long>(m.mono_us), m.tsfl,
                   m.body.size(), static_cast<unsigned>(m.mcs),
                   m.phy_valid ? 1 : 0, m.rssi[0], m.rssi[1], m.snr[0],
                   m.snr[1], rc_t, m.tsfl - c.gap_prev_tsfl,
                   static_cast<unsigned long long>(m.mono_us - c.gap_prev_mono_us),
                   static_cast<unsigned>(c.gap_prev_agg_pos));
    ++c.rc_frames;
    if (rc_sink_) rc_sink_(m.card_id, m.body, m.mono_us);
    return;
  }

  const int stream_id = mabur::sbi_peek_stream_id(m.body.data(), m.body.size());

#ifdef MABUR_LOSS_SIM
  // BENCH RIG (MABUR_LOSS_SIM): injected loss, --loss-sim.
  // Placed BEFORE any accounting so a dropped body is indistinguishable from
  // one the air ate — no frames/rx_bytes/seq credit, no EMA, no class track,
  // never reaches the decoder. The seq gap appears on its own: last_seq is a
  // high-water mark, so the next surviving body advances it by more than one
  // and books the loss exactly as real loss would.
  // crc_ok gate: a corrupt body's peeked stream_id is untrustworthy, and
  // injecting on it would drop a randomly-misidentified stream.
  if (m.crc_ok && loss_sim_.should_drop(m.card_id, stream_id)) return;
#endif

  ++c.frames;
  c.rx_bytes += m.body.size();
  c.last_frame_us = m.mono_us;

  if (!m.crc_ok) {
    ++c.crc_fail;
    static const bool gaplog_cf = std::getenv("MABUR_GAPLOG") != nullptr;
    if (gaplog_cf)
      std::fprintf(stderr,
                   "crcfail card=%u seq=%u mono=%llu tsfl=%u len=%zu mcs=%u "
                   "physt=%d rssi=%d/%d snr=%d/%d sid=%d\n",
                   static_cast<unsigned>(m.card_id),
                   static_cast<unsigned>(m.mac_seq),
                   static_cast<unsigned long long>(m.mono_us), m.tsfl,
                   m.body.size(), static_cast<unsigned>(m.mcs),
                   m.phy_valid ? 1 : 0, m.rssi[0], m.rssi[1], m.snr[0],
                   m.snr[1], stream_id);
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
    // Walk EVERY drone frame. The GS reads the 12-bit seq the 8812EU
    // stamps in hardware: devourer sets EN_HWSEQ on the TX descriptor
    // (FrameParserJaguar3.h), so the chip's single counter numbers every
    // injected frame — video bodies, MSP bodies AND the drone's RC frames
    // (DISC_ACK, the 1 Hz T_TELEM) — and the software seq_ctl maburd writes
    // (RadioTx::seq_, control_seq) never reaches the air. The old walk
    // excluded MSP and RC on the belief that they carried their own
    // counters and booked one phantom lost frame per such frame (~0.3 % of
    // loss_pct on the bench; measured 2026-09-03, both classes matched the
    // missing seqs exactly — docs/gs-uplink-self-blanking-findings-2026-09-02.md).
    // Self-originated frames were diverted above and never reach here.
    const bool decoder_bound_seq = true;
    static const bool gaplog = std::getenv("MABUR_GAPLOG") != nullptr;
    if (gaplog && !decoder_bound_seq) {
      std::fprintf(stderr, "nonvid card=%u seq=%u mono=%llu len=%zu sid=%d rc=%d\n",
                   static_cast<unsigned>(m.card_id),
                   static_cast<unsigned>(m.mac_seq),
                   static_cast<unsigned long long>(m.mono_us), m.body.size(),
                   stream_id, rc_t);
    }
    if (decoder_bound_seq) {
      c.agg_pos = m.phy_valid ? 0 : c.agg_pos + 1;
      if (c.has_seq) {
        const uint16_t adv =
            static_cast<uint16_t>((m.mac_seq - c.last_seq) & 0x0FFF);
        if (gaplog && adv >= 2048) {
          std::fprintf(stderr, "late card=%u seq=%u behind=%u mono=%llu len=%zu\n",
                       static_cast<unsigned>(m.card_id),
                       static_cast<unsigned>(m.mac_seq),
                       static_cast<unsigned>(4096 - adv),
                       static_cast<unsigned long long>(m.mono_us), m.body.size());
        }
        if (adv == 0) {
          c.seq_expected += 1;  // duplicate/retry: count as one delivered frame
        } else if (adv < 2048) {
          c.seq_expected += adv > kMaxSeqGap ? 1 : adv;
          // MABUR_GAPLOG=1: one stderr line per per-card seq gap. dtsf =
          // chip-TSF advance across the gap (air-time actually consumed),
          // dhost = host mono advance; n = missing seqs. Diagnostic only.
          if (gaplog && adv > 1 && adv <= kMaxSeqGap) {
            std::fprintf(stderr,
                         "gap card=%u seq=%u..%u n=%u dtsf=%u dhost=%llu "
                         "mono=%llu prev_len=%zu len=%zu sid=%d prev_agg=%u "
                         "after_physt=%d prev_mcs=%u mcs=%u prev_sid=%d\n",
                         static_cast<unsigned>(m.card_id),
                         static_cast<unsigned>(c.gap_prev_seq),
                         static_cast<unsigned>(m.mac_seq),
                         static_cast<unsigned>(adv - 1),
                         static_cast<unsigned>(m.tsfl - c.gap_prev_tsfl),
                         static_cast<unsigned long long>(m.mono_us - c.gap_prev_mono_us),
                         static_cast<unsigned long long>(m.mono_us),
                         c.gap_prev_len, m.body.size(), stream_id,
                         static_cast<unsigned>(c.gap_prev_agg_pos),
                         m.phy_valid ? 1 : 0,
                         static_cast<unsigned>(c.gap_prev_mcs),
                         static_cast<unsigned>(m.mcs), c.gap_prev_sid);
          }
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
      if (c.last_seq == m.mac_seq) {  // in-order/high-water frame: new gap anchor
        c.gap_prev_mono_us = m.mono_us;
        c.gap_prev_tsfl = m.tsfl;
        c.gap_prev_seq = m.mac_seq;
        c.gap_prev_len = m.body.size();
        c.gap_prev_agg_pos = c.agg_pos;
        c.gap_prev_mcs = m.mcs;
        c.gap_prev_sid = stream_id;
      }
    }

    const double rssi = static_cast<double>(m.rssi[0] > m.rssi[1] ? m.rssi[0] : m.rssi[1]);
    const double snr = static_cast<double>(m.snr[0] > m.snr[1] ? m.snr[0] : m.snr[1]);
    if (m.phy_valid) fold_rf(c, rssi, snr, m.rssi, m.snr);
    fold_evm(c, m.evm[0], m.evm[1]);

    // Per-RF-class EMA, mirroring the pooled block above exactly. Ctrl
    // (non-self rc frames, e.g. DISC_ACK) takes priority over stream id;
    // an unparseable/misrouted body gets no class (card totals only).
    int class_idx = -1;
    if (rc_t >= 0) {
      class_idx = static_cast<int>(RfClass::Ctrl);
    } else if (stream_id == mabur::kMspStreamId) {
      class_idx = static_cast<int>(RfClass::Msp);
    } else if (stream_id == mabur::kProbeStreamId) {
      class_idx = static_cast<int>(RfClass::Probe);
    } else if (stream_id >= 0 && stream_id < 2) {
      class_idx = stream_id;
    }
    if (class_idx >= 0) {
      ClassTrack& ct = c.cls[static_cast<size_t>(class_idx)];
      ++ct.frames;
      ct.bytes += m.body.size();
      if (m.phy_valid) fold_rf(ct, rssi, snr, m.rssi, m.snr);
      fold_evm(ct, m.evm[0], m.evm[1]);

      // base+enh pooled track (spec 2026-08-15, re-scoped for the 2-stream
      // split-rate ladder): the RF label source and the fade trigger read
      // this. Deliberately excludes msp and ctrl.
      if (class_idx == static_cast<int>(RfClass::S0) ||
          class_idx == static_cast<int>(RfClass::S1)) {
        ClassTrack& pt = c.rf_pool;
        ++pt.frames;
        pt.bytes += m.body.size();
        if (m.phy_valid) fold_rf(pt, rssi, snr, m.rssi, m.snr);
        fold_evm(pt, m.evm[0], m.evm[1]);
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
  if (stream_id == mabur::kProbeStreamId) {
    // Probe stream (spec 2026-09-04): scored by ProbeTrack, never decoded.
    // FCS-failed bodies still reach the sink — parse_probe_body salvages
    // CRC-clean sub-blocks exactly as the video decoder does, so probe loss
    // stays comparable to video loss.
    if (probe_sink_) probe_sink_(m.card_id, m);
    return;
  }
  ++c.video_bodies;
  last_video_us_ = m.mono_us;
  for (const auto& r :
       dec_.add_body(m.body.data(), m.body.size(), m.mono_us / 1000, m.mcs,
                     m.mono_us, m.crc_ok))
    if (frag_sink_) frag_sink_(r);
}

}  // namespace maburgs
