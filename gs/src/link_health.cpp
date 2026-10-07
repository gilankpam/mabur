#include "link_health.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "ladder_residual.h"
#include "mabur/probe_wire.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"
#include "snr_units.h"

namespace maburgs {

LinkHealthAssembler::LinkHealthAssembler(const LinkHealthCfg& cfg)
    : cfg_(cfg),
      probe_track_(ProbeTrackCfg{cfg.probe_bpb, 100, cfg.n_cards}),
      probe_card_loss_(static_cast<size_t>(cfg.n_cards)),
      prev_pool_frames_(static_cast<size_t>(cfg.n_cards), 0),
      label_card_inputs_(static_cast<size_t>(cfg.n_cards)) {}

void LinkHealthAssembler::on_au_begin(uint8_t sid, uint16_t frame_id,
                                      double now_ms) {
  // One probe expectation per video access unit, base and enh alike
  // (probe per AU, 2026-09-16): the probe body rides the AU's send
  // opportunity, so the AU count is what "expected" means
  // (probe_track.h explains why seq gaps cannot be).
  probe_track_.on_au(sid, frame_id, now_ms);
}

// Probe stream bodies (SBI stream 5): every body is scored per-card;
// ProbeTrack merges the cards into the union the ladder reads. The RF labels
// are this body's own, not the card's EMA -- a probe sample must carry the
// radio conditions it actually flew through. The slotter's "burst off air"
// release (rcf_slot.on_probe_tail) stays with the caller, which owns the
// slotter.
void LinkHealthAssembler::on_probe_body(uint8_t card,
                                        const mabur::node::RxBody& m) {
  mabur::probe::ProbeRx rx;
  if (!mabur::probe::parse_probe_body(m.body.data(), m.body.size(),
                                      cfg_.probe_block_payload, &rx))
    return;
  const double snr = (m.phy_valid && snr_ok(card))
                          ? std::max(m.snr[0], m.snr[1]) * maburgs::kSnrRawToDb
                          : std::nan("");
  // EVM: raw half-dB, negative = clean, 0 = not sampled (node.h) -- pick
  // the better sampled chain, NaN when neither chain reported.
  const int8_t evm_raw =
      (m.evm[0] != 0 && (m.evm[1] == 0 || m.evm[0] < m.evm[1])) ? m.evm[0]
                                                                : m.evm[1];
  const double evm = evm_raw != 0 ? evm_raw * maburgs::kEvmRawToDb : std::nan("");
  probe_track_.on_body(card, rx, snr, evm,
                       static_cast<double>(m.mono_us) / 1000.0);
}

LinkHealthAssembler::Tick LinkHealthAssembler::tick(double now_ms,
                                                    Aggregator& agg,
                                                    const LinkHealthInputs& in) {
  Tick out;
  // Transition boundaries for loss attribution + settle-blank of the two
  // residual decision windows + clear of the two util decision windows:
  // gs/src/transition_edge.h (extracted 2026-09-05 so
  // tests/test_transition_edge.cpp can pin what an edge blanks; the util
  // windows rejoined it 2026-10-06, flight 0026).
  edge_.on_tick(in.op, agg.decoder(), s1_resid_cur_, s3_resid_cur_,
                s1_loss_cur_, s3_loss_cur_, now_ms);

  // Control step: post-FEC residual from the FEC decoder's own abandonment
  // counters — ONE formula for every consumer since 2026-09-02, see
  // gs/src/ladder_residual.cpp. Per-layer delivery is operator-facing only
  // — it rides the stats sideport below, never the RCF (RC_VERSION 3
  // dropped it; maburd never read it).
  std::array<uint8_t, 2> ld{};
  for (int s = 0; s < 2; ++s) {
    const auto lc =
        maburgs::residual_counts(agg.decoder(), s, /*cur=*/false);
    auto& w = layer_resid_[static_cast<size_t>(s)];
    w.add(lc.expected, lc.arrived, now_ms);
    const auto ls = w.sample(now_ms);
    // Idle layer reads 100, matching the deleted window_delivery_pct (the
    // 2026-07 controller-wedge finding in docs/bench-validation.md turns
    // on that convention). Truncating like the old integer ratio did.
    const int pct =
        ls.valid ? static_cast<int>(100.0 * (1.0 - ls.loss)) : 100;
    ld[static_cast<size_t>(s)] =
        static_cast<uint8_t>(pct < 0 ? 0 : (pct > 100 ? 100 : pct));
  }
  const auto rc_tot =
      maburgs::residual_counts_pooled(agg.decoder(), /*cur=*/false);
  const auto rc_cur =
      maburgs::residual_counts_pooled(agg.decoder(), /*cur=*/true);
  pool_resid_.add(rc_tot.expected, rc_tot.arrived, now_ms);
  pool_resid_cur_.add(rc_cur.expected, rc_cur.arrived, now_ms);
  const auto pr = pool_resid_.sample(now_ms);
  const auto prc = pool_resid_cur_.sample(now_ms);
  std::optional<double> residual;
  if (pr.valid) residual = pr.loss;
  std::optional<double> residual_cur;
  if (prc.valid) residual_cur = prc.loss;
  // Zero completed packets while video frames still arrive = decode
  // collapse; the ladder's video_starved path forces the failsafe rung
  // then rather than trusting this window's (survivor-biased) loss sample.
  // Gate on having ever seen video so a pre-link idle window doesn't count
  // as starvation.
  //
  // packets_out is CUMULATIVE, so this diffs against a snapshot taken at
  // the same boundary the deleted reset_window() used (the controller's
  // last non-disc step) — the residual counters are cumulative too now, so
  // "expected == 0 this window" is no longer a thing the decoder can say.
  const uint64_t pkts_now = agg.decoder().stats(0).packets_out +
                            agg.decoder().stats(1).packets_out;
  const bool starved = (pkts_now == prev_pkts_out_) && agg.last_video_us() != 0;

  // s1_* names are historical (predate the 2-stream flatten): this is the
  // BASE sid's (0) window, the critical always-decode layer that drives
  // the ladder's ordinary demote/promote decisions.
  const auto s1 = agg.decoder().stats(0);
  // Pre-FEC (util) accounting from the ArrivalTracker (spec 2026-09-05):
  // expected = seq advance past the settle line, arrived = heard, booked
  // at arrival -- no repair/completion lag, and a denominator that does
  // not depend on decoder progress after a re-key. The total feeds the
  // sideport/ctl-log gauge, the current-only side feeds block 5 (util).
  s1_loss_.add(s1.arr_expected, s1.arr_arrived, now_ms);
  const auto s1_sample = s1_loss_.sample(now_ms);
  s1_loss_cur_.add(s1.arr_expected - s1.arr_expected_stale,
                  s1.arr_arrived - s1.arr_arrived_stale, now_ms);
  const auto s1_cur_sample = s1_loss_cur_.sample(now_ms);
  // Both layers pooled for the exported gauge (sin.pre_fec_loss below):
  // the enh layer's erasures are as real on air as the base layer's, and
  // the OSD's LOSS row is the pilot's view of the downlink, not of the
  // ladder's input. Summed monotonic totals keep S1LossWindow's reset
  // detection intact (both layers re-key together).
  {
    const auto s2 = agg.decoder().stats(1);
    pre_loss_all_.add((s1.arr_expected - s1.arr_expected_stale) +
                         (s2.arr_expected - s2.arr_expected_stale),
                     (s1.arr_arrived - s1.arr_arrived_stale) +
                         (s2.arr_arrived - s2.arr_arrived_stale),
                     now_ms);
  }
  const auto pre_all_sample = pre_loss_all_.sample(now_ms);

  // Block 4's instant-demote input: BASE post-FEC loss from the FEC
  // decoder's own abandonment count, mirroring s3_resid_cur below. See
  // gs/src/ladder_residual.cpp for why this replaced the packet-level
  // delivery window on 2026-09-02.
  const auto s1_rc = maburgs::ladder_residual_counts(agg.decoder());
  s1_resid_cur_.add(s1_rc.expected, s1_rc.arrived, now_ms);
  const auto s1_rcur_sample = s1_resid_cur_.sample(now_ms);

  // s3 feedback for the probe-before-promote / s3-demote logic: pre-FEC
  // loss (same shape as s1's window), scored against the CURRENT rung's
  // budget. s3_* names are historical too: this is the ENH sid's (1)
  // window — the shed-able layer the probe candidate rides.
  const auto s3 = agg.decoder().stats(1);
  s3_loss_.add(s3.arr_expected, s3.arr_arrived, now_ms);
  const auto s3_sample = s3_loss_.sample(now_ms);

  // + syms_retx mirrors ladder_residual.cpp's abandoned term (enh never has
  // retx today; keeps the two paths symmetric).
  const uint64_t s3_ab_cur =
      s3.syms_abandoned - s3.syms_abandoned_stale + s3.syms_retx;
  const uint64_t s3_exp_cur = s3.syms_delivered + s3.syms_recovered + s3_ab_cur;
  s3_loss_cur_.add(s3.arr_expected - s3.arr_expected_stale,
                  s3.arr_arrived - s3.arr_arrived_stale, now_ms);
  s3_resid_cur_.add(s3_exp_cur, s3_exp_cur - s3_ab_cur, now_ms);
  const auto s3_cur_sample = s3_loss_cur_.sample(now_ms);
  const auto s3_rcur_sample = s3_resid_cur_.sample(now_ms);

  // Probe window (spec 2026-09-04 section 3.3): union block counters for
  // the profile currently commanded, windowed by the same machinery as the
  // s1/s3 windows above. Blanked on a commanded-profile change so it
  // refills only from bodies carrying the new profile (RCF lag + the
  // finalize window). Per-card siblings are diagnostics only -- the ladder
  // reads the union, because that is the path production video takes.
  probe_track_.tick(now_ms);
  if (const uint8_t pc = in.probe_profile; pc != probe_cmd_last_) {
    probe_cmd_last_ = pc;
    probe_track_.set_commanded(pc, now_ms);
    probe_loss_.blank_until(now_ms + kProbeSwitchBlankMs);
    for (auto& w : probe_card_loss_) w.blank_until(now_ms + kProbeSwitchBlankMs);
    // The probe body's own airtime is the floor of the slotter's learned
    // completion->probe offset (rcf_slot.h "Probe tail"): the burst
    // cannot end sooner than one probe body after the AU completes.
    int tail = 0;
    if (pc != mabur::rc::kNoProbeProfile) {
      mabur::rc::PhyMode pm; uint8_t pmcs, pbw;
      mabur::rc::decode_profile(pc, pm, pmcs, pbw);
      const auto spec = mabur::rc::ladder_from(pm, pmcs, pbw)[1];
      const double rate = mabur::rc::phy_rate_mbps(spec);
      const double us = rate > 0 ? mabur::probe::probe_body_len(cfg_.probe_bpb, cfg_.probe_block_payload) * 8.0 / rate : 0.0;
      tail = static_cast<int>(std::ceil(us / 1000.0));
      if (tail < 1) tail = 1;
    }
    out.probe_tail_ms = tail;
  }
  const auto& pu = probe_track_.union_counts();
  probe_loss_.add(pu.expected_blocks, pu.arrived_blocks, now_ms);
  for (int i = 0; i < cfg_.n_cards; ++i) {
    const auto& pcnt = probe_track_.card_counts(i);
    probe_card_loss_[static_cast<size_t>(i)].add(pcnt.expected_blocks,
                                                pcnt.arrived_blocks, now_ms);
  }
  const auto probe_sample = probe_loss_.sample(now_ms);
  // Drain every iteration even with no log open: ProbeTrack's bounded
  // structure is its pending ring, NOT the finalized list -- that is a
  // plain std::vector it appends to and only take_finalized() clears, so
  // skipping the drain would grow it without limit for the life of the
  // process.
  probe_rows_ = probe_track_.take_finalized();

  // The strongest card that ACTUALLY RECEIVED s1-or-s3 this feedback
  // window supplies all three RF labels. Freshness is part of the argmax,
  // not a filter after it (select_label_card, rf_labels.h): a card whose
  // front-end wedged keeps a frozen-high EMA and would otherwise outrank
  // a live sibling forever. -1 = nothing measured this window, so all
  // three labels stay NaN -- inert for the fade trigger, null on the wire.
  for (int i = 0; i < cfg_.n_cards; ++i) {
    const auto& ct = agg.card(i).rf_pool;
    label_card_inputs_[static_cast<size_t>(i)] = maburgs::CardLabelInput{
        ct.has_ema, ct.frames, prev_pool_frames_[static_cast<size_t>(i)],
        ct.snr_ema, snr_ok(i)};
  }
  const int best_card = maburgs::select_label_card(label_card_inputs_);
  // SNR (label + fade input) and EVM (label only) come from that ONE card,
  // never independently-best across cards, so the three are a coherent
  // single-card snapshot of the same radio at the same instant.
  double rf_snr_db = std::numeric_limits<double>::quiet_NaN();
  double rf_evm_db = std::numeric_limits<double>::quiet_NaN();
  double rf_rssi_dbm = std::numeric_limits<double>::quiet_NaN();
  if (best_card >= 0) {
    const auto& ct = agg.card(best_card).rf_pool;
    // Raw units are devourer's half-dB (snr_units.h); raw - 110 is the
    // exporter's own dBm conversion (stats_exporter.cpp rssi keys).
    rf_snr_db = ct.snr_ema * maburgs::kSnrRawToDb;
    if (ct.evm_has) rf_evm_db = ct.evm_ema * maburgs::kEvmRawToDb;
    rf_rssi_dbm = ct.rssi_ema - 110.0;
  }

  // Every demote input reads the CURRENT-rung (attributed) value; stale
  // transition debris can no longer fire any demote. Unconditional since
  // 2026-08-15. sample_valid / s3_valid / s3_expected_syms stay
  // total-based on purpose: they gate "was there traffic at all", and an
  // all-stale window must still count as feedback (a cur-based valid
  // would un-stamp last_feedback_ms_ and could walk into the blind-side
  // timeout during a long boundary).
  maburgs::LinkHealth health{
      s1_sample.valid,
      s1_cur_sample.valid ? s1_cur_sample.loss : 0.0,
      s1_rcur_sample.valid ? s1_rcur_sample.loss : 0.0,
      starved};
  health.s3_valid = s3_sample.valid;
  health.s3_pre_fec_loss = s3_cur_sample.valid ? s3_cur_sample.loss : 0.0;
  health.s3_residual_loss =
      s3_rcur_sample.valid ? s3_rcur_sample.loss : 0.0;
  health.s3_expected_syms = s3_loss_.expected_in_window(now_ms);
  // Probe gate inputs (spec 2026-09-04 section 3.3). probe_rung is the rung
  // the sample was commanded at. The h.probe_rung == pr equality the
  // controller checks against is always true in practice -- both sides
  // read probe_rung() on the same tick -- and is kept only as a cheap
  // invariant check, not the staleness guard: the real guard against a
  // sample surviving a profile switch is the 150 ms
  // probe_loss.blank_until() set at the switch edge plus mark_transition's
  // streak reset.
  health.probe_valid = probe_sample.valid;
  health.probe_loss = probe_sample.valid ? probe_sample.loss : 0.0;
  health.probe_expected_syms = probe_loss_.expected_in_window(now_ms);
  // Cumulative expected bodies for the body-count clean streak. Expected
  // books bpb blocks per AU (ProbeTrack), so this is exact.
  health.probe_bodies_total = pu.expected_blocks / static_cast<uint64_t>(cfg_.probe_bpb);
  health.probe_rung = in.probe_rung;
  health.rf_snr_db = rf_snr_db;
  health.rf_evm_db = rf_evm_db;
  health.rf_rssi_dbm = rf_rssi_dbm;

  // Last-tick views for the caller's sideport / ctl log / OSD readers.
  residual_ = residual;
  residual_cur_ = residual_cur;
  ld_ = ld;
  pre_all_ = pre_all_sample;
  probe_sample_ = probe_sample;
  pkts_now_ = pkts_now;
  last_health_ = health;
  out.health = health;
  return out;
}

void LinkHealthAssembler::on_step_sent(Aggregator& agg) {
  prev_pkts_out_ = pkts_now_;  // window == RCF period
  // The RF staleness window and the loss window MUST share this
  // boundary: both are "since the last health the controller acted on".
  for (int i = 0; i < cfg_.n_cards; ++i)
    prev_pool_frames_[static_cast<size_t>(i)] = agg.card(i).rf_pool.frames;
}

}  // namespace maburgs
