#include "vrx_controller.h"

#include "mabur/rc_proto.h"
#include "mabur/profile.h"

namespace maburgs {

VrxController::VrxController(const LinkTable& lt, VrxCfg cfg)
    : cfg_(cfg),
      ctrl_(lt, cfg.ctrl),
      win_(cfg.score),
      rz_(VrxRzConfig{cfg.vtx_id, 1000, 20, cfg.op_channel}),
      cur_op_(max_range()) {
  if (cfg_.bw_set.size() > 1) rungs_.emplace(cfg_.bw_set);
}

void VrxController::on_video(double rssi, double snr, bool crc_err,
                             uint16_t seq, double now_ms) {
  win_.add_frame(rssi, snr, crc_err, seq, now_ms / 1000.0);
  if (rungs_) rungs_->add_seq(seq);
  rz_.feed_video(now_ms);
}

void VrxController::on_rc_frame(const uint8_t* buf, size_t len, double now_ms) {
  if (mabur::rc::frame_type(buf, len) != mabur::rc::T_DISC_ACK) return;
  auto ack = mabur::rc::parse_disc_ack(buf, len);
  if (ack) rz_.feed_disc_ack(*ack, now_ms);
}

std::optional<VrxController::Out> VrxController::step(
    double now_ms, const std::array<uint8_t, 4>& layer_delivery,
    std::optional<double> residual_loss, bool video_starved) {
  // Blind-side failsafe: with no feedback (no video -> no update() calls) the
  // controller pins MAX_RANGE, so the first RCF after recovery commands the
  // conservative floor, not the last aggressive point.
  if (cfg_.pin_mcs >= 0) {
    // Static-link mode: fixed op, estimator fully out of the loop.
    cur_op_ = OpPoint{false, cfg_.pin_mcs, 20, false, cfg_.pin_txagc,
                      cfg_.pin_overhead, 0.0, 0.0, 1.0};
    cur_txagc_ = cfg_.pin_txagc;
  } else if (auto op = ctrl_.on_tick(now_ms)) {
    cur_op_ = *op;
    cur_txagc_ = op->txagc;
  }
  const VrxAction act = rz_.tick(now_ms);
  if (act == VrxAction::Beacon)
    return Out{mabur::rc::pack_disc(rz_.beacon()), true};
  if (act != VrxAction::TxFeedback) return std::nullopt;

  // Fix (a): SESSION keep-alive DISC, replacing this tick's RCF slot.
  if (now_ms - last_keepalive_ms_ >= cfg_.beacon_keepalive_ms) {
    last_keepalive_ms_ = now_ms;
    return Out{mabur::rc::pack_disc(rz_.beacon()), true};
  }
  if (now_ms - last_fb_ms_ < cfg_.feedback_ms) return std::nullopt;
  last_fb_ms_ = now_ms;

  if (rungs_) ctrl_.report_rung_delivery(rungs_->stats(), now_ms);
  // Decode collapse: the frames still decoding are the lucky strong ones, so
  // the SNR window lies high while the stream is dead. Withhold the update
  // and let on_tick's blind-side timeout walk the op back to MAX_RANGE.
  if (auto snr = win_.snr_estimate(); snr && !video_starved && cfg_.pin_mcs < 0) {
    if (auto op = ctrl_.update(*snr, cur_txagc_, now_ms)) {
      cur_op_ = *op;
      cur_txagc_ = op->txagc;
    }
  }
  seq_ = static_cast<uint16_t>(seq_ + 1);

  mabur::rc::Rcf r;
  r.vtx_id = cfg_.vtx_id;
  r.seq = seq_;
  r.ack_seq = win_.ack_seq();
  r.profile = mabur::rc::encode_profile(
      cur_op_.vht ? mabur::rc::PhyMode::VHT : mabur::rc::PhyMode::HT,
      static_cast<uint8_t>(cur_op_.mcs), static_cast<uint8_t>(cur_op_.bw));
  r.score = static_cast<uint16_t>(win_.score(residual_loss));
  r.pwr_idx = static_cast<uint8_t>(cur_op_.txagc);
  r.fec_overhead_16ths = mabur::rc::overhead_to_16ths(cur_op_.overhead);
  r.layer_delivery.assign(layer_delivery.begin(), layer_delivery.end());
  return Out{mabur::rc::pack_rcf(r), false};
}

const OpPoint& VrxController::cur_op() const { return cur_op_; }
VrxState VrxController::link_state() const { return rz_.state(); }
uint16_t VrxController::rcf_seq() const { return seq_; }

}  // namespace maburgs
