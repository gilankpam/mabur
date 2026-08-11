#include "vrx_controller.h"

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
    const LinkHealth& health, bool idr_request) {
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

  if (cfg_.pin_mcs < 0) {
    LinkHealth h = health;
    h.probe_allowed = (peer_caps_ & mabur::rc::CAP_S3_PROBE) != 0;
    if (ctrl_.update(h, now_ms)) cur_op_ = op_from_rung(ctrl_.op());
  }
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
  if (cfg_.pin_mcs < 0 && ctrl_.probing()) {
    r.probe3 = true;
    r.probe_profile = mabur::rc::encode_profile(
        cur_op_.vht ? mabur::rc::PhyMode::VHT : mabur::rc::PhyMode::HT,
        static_cast<uint8_t>(ctrl_.probe_mcs()), static_cast<uint8_t>(cur_op_.bw));
  }
  // GS-latched IDR request (spec 2026-08-11 idr-request): a level the
  // caller re-asserts every RCF; only toward a peer that advertised the
  // cap, so an old drone never sees the bit. Orthogonal to the ladder —
  // static-pin mode carries it identically.
  if (idr_request && (peer_caps_ & mabur::rc::CAP_IDR_REQ) != 0)
    r.flags |= mabur::rc::RCF_F_IDR_REQ;
  return Out{mabur::rc::pack_rcf(r), false};
}

const OpPoint& VrxController::cur_op() const { return cur_op_; }
VrxState VrxController::link_state() const { return rz_.state(); }
uint16_t VrxController::rcf_seq() const { return seq_; }

}  // namespace maburgs
