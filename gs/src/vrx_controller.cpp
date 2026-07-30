#include "vrx_controller.h"

#include <algorithm>
#include "mabur/rc_proto.h"
#include "mabur/profile.h"

namespace maburgs {

namespace {
// Rebuild an OpPoint from the ladder's current rung. PHY offset is always 0
// in ladder mode (see step()); bw/sgi/snr_req are dead fields the
// model-driven resolver used to populate — stats_exporter still reads
// mcs/bw/overhead/offset/vht, so they keep emitting benign 0s here.
OpPoint op_from_rung(const Rung& r) {
  return OpPoint{false, r.mcs, 20, false, 0, r.overhead, 0.0};
}
}  // namespace

VrxController::VrxController(VrxCfg cfg)
    : cfg_(cfg),
      ctrl_(cfg.ladder),
      win_(cfg.score),
      // link_lost_ms 1000 / beacon_period_ms 20 are deliberately fixed, not
      // config: every hw validation ran with these, and a slower fallback to
      // BEACONING after video loss only delays re-rendezvous. The removed
      // link.video_silence_ms key claimed to tune the 1000 but never did.
      rz_(VrxRzConfig{cfg.vtx_id, 1000, 20, cfg.op_channel}),
      cur_op_(op_from_rung(ctrl_.op())) {}

void VrxController::on_video(double rssi, double snr, bool crc_err,
                             uint16_t seq, double now_ms) {
  if (stress_anchor_ms_ < 0.0) stress_anchor_ms_ = now_ms;
  win_.add_frame(rssi, snr, crc_err, seq, now_ms / 1000.0);
  rz_.feed_video(now_ms);
}

void VrxController::on_rc_frame(const uint8_t* buf, size_t len, double now_ms) {
  if (mabur::rc::frame_type(buf, len) != mabur::rc::T_DISC_ACK) return;
  auto ack = mabur::rc::parse_disc_ack(buf, len);
  if (ack && rz_.feed_disc_ack(*ack, now_ms)) {
    peer_caps_ = ack->chip_caps;
    peer_acked_ = true;
  }
}

std::optional<VrxController::Out> VrxController::step(
    double now_ms, const std::array<uint8_t, 4>& layer_delivery,
    const LinkHealth& health) {
  // Blind-side failsafe: with no feedback the ladder's own on_tick() forces
  // rung 0 after feedback_timeout_ms, so the first RCF after recovery
  // commands the conservative floor, not the last aggressive point.
  if (cfg_.pin_mcs >= 0) {
    // Static-link mode: fixed op, ladder fully out of the loop (never
    // ticked/updated — health is ignored entirely).
    cur_op_ = OpPoint{false, cfg_.pin_mcs, 20, false, cfg_.pin_offset_qdb,
                      cfg_.pin_overhead, 0.0};
  } else if (ctrl_.on_tick(now_ms)) {
    cur_op_ = op_from_rung(ctrl_.op());
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

  if (cfg_.pin_mcs < 0 && ctrl_.update(health, now_ms)) {
    cur_op_ = op_from_rung(ctrl_.op());
  }
  // Bench stress instrument: every ladder-mode RCF carries the (possibly
  // ramping) stress offset — 0 when the knob is off. Stamped after any
  // rung-change rebuild so op_from_rung's 0 never leaks onto the wire.
  if (cfg_.pin_mcs < 0) cur_op_.pwr_offset_qdb = stress_offset_qdb(now_ms);
  seq_ = static_cast<uint16_t>(seq_ + 1);

  mabur::rc::Rcf r;
  r.vtx_id = cfg_.vtx_id;
  r.seq = seq_;
  r.ack_seq = win_.ack_seq();
  r.profile = mabur::rc::encode_profile(
      cur_op_.vht ? mabur::rc::PhyMode::VHT : mabur::rc::PhyMode::HT,
      static_cast<uint8_t>(cur_op_.mcs), static_cast<uint8_t>(cur_op_.bw));
  r.score = static_cast<uint16_t>(win_.score(health.residual_loss));
  r.pwr_offset_biased = mabur::rc::encode_pwr_offset_qdb(cur_op_.pwr_offset_qdb);
  r.fec_overhead_16ths = mabur::rc::overhead_to_16ths(cur_op_.overhead);
  r.layer_delivery.assign(layer_delivery.begin(), layer_delivery.end());
  return Out{mabur::rc::pack_rcf(r), false};
}

int VrxController::stress_offset_qdb(double now_ms) const {
  if (cfg_.stress_qdb == 0 && cfg_.stress_step_qdb == 0) return 0;
  if (stress_anchor_ms_ < 0.0) return cfg_.stress_qdb;
  int steps = 0;
  if (cfg_.stress_step_qdb != 0) {
    steps = static_cast<int>((now_ms - stress_anchor_ms_) /
                             (std::max(1, cfg_.stress_period_s) * 1000.0));
  }
  return std::max(cfg_.stress_floor_qdb,
                  cfg_.stress_qdb + cfg_.stress_step_qdb * steps);
}

const OpPoint& VrxController::cur_op() const { return cur_op_; }
VrxState VrxController::link_state() const { return rz_.state(); }
uint16_t VrxController::rcf_seq() const { return seq_; }

}  // namespace maburgs
