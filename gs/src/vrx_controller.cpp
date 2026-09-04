#include "vrx_controller.h"

#include "mabur/rc_proto.h"
#include "mabur/profile.h"

namespace maburgs {

namespace {
// Rebuild an OpPoint from the ladder's current rung. Power is constant and
// is not part of the operating point (spec 2026-08-12-constant-txpower).
OpPoint op_from_rung(const Rung& r) {
  return OpPoint{false, r.mcs, 20, false, r.overhead_base, r.overhead_enh, 0.0};
}
}  // namespace

VrxController::VrxController(VrxCfg cfg)
    : cfg_(cfg),
      ctrl_(cfg.ladder),
      // link_lost_ms 1000 / beacon_period_ms 20 are deliberately fixed, not
      // config: every hw validation ran with these, and a slower fallback to
      // BEACONING after video loss only delays re-rendezvous. The removed
      // link.video_silence_ms key claimed to tune the 1000 but never did.
      rz_(VrxRzConfig{cfg.vtx_id, 1000, 20, cfg.op_channel}),
      cur_op_(op_from_rung(ctrl_.op())) {}

void VrxController::on_video(double now_ms) { rz_.feed_video(now_ms); }

void VrxController::on_rc_frame(const uint8_t* buf, size_t len, double now_ms) {
  if (mabur::rc::frame_type(buf, len) != mabur::rc::T_DISC_ACK) return;
  auto ack = mabur::rc::parse_disc_ack(buf, len);
  if (ack && rz_.feed_disc_ack(*ack, now_ms)) {
    peer_caps_ = ack->chip_caps;
    peer_acked_ = true;
  }
}

std::optional<VrxController::Out> VrxController::step(double now_ms,
                                                      const LinkHealth& health) {
  // Blind-side failsafe: with no feedback the ladder's own on_tick() forces
  // rung 0 after feedback_timeout_ms, so the first RCF after recovery
  // commands the conservative floor, not the last aggressive point.
  if (cfg_.pin_mcs >= 0) {
    // Static-link mode: fixed op, ladder fully out of the loop (never
    // ticked/updated — health is ignored entirely).
    cur_op_ = OpPoint{false, cfg_.pin_mcs, 20, false,
                     cfg_.pin_overhead_base, cfg_.pin_overhead_enh, 0.0};
  } else if (ctrl_.on_tick(now_ms)) {
    cur_op_ = op_from_rung(ctrl_.op());
  }
  const VrxAction act = rz_.tick(now_ms);
  if (act == VrxAction::Beacon)
    return Out{mabur::rc::pack_disc(rz_.beacon()), true};
  if (act != VrxAction::TxFeedback) return std::nullopt;

  // Fix (a): SESSION keep-alive DISC, replacing this tick's RCF slot.
  // Fast cadence until the peer's caps are known (stale-caps fix).
  const int keepalive_ms =
      peer_acked_ ? cfg_.beacon_keepalive_ms : cfg_.unacked_keepalive_ms;
  if (now_ms - last_keepalive_ms_ >= keepalive_ms) {
    last_keepalive_ms_ = now_ms;
    return Out{mabur::rc::pack_disc(rz_.beacon()), true};
  }
  if (now_ms - last_fb_ms_ < cfg_.feedback_ms) return std::nullopt;
  last_fb_ms_ = now_ms;

  if (cfg_.pin_mcs < 0) {
    LinkHealth h = health;
    h.probe_allowed = true;
    if (ctrl_.update(h, now_ms)) cur_op_ = op_from_rung(ctrl_.op());
  }
  mabur::rc::Rcf r = build_rcf();

  // RCF repeat burst (rcf-uplink-loss findings 2026-08-14): the drone's
  // half-duplex TX kills 30-50% of uplink control frames, and a lost
  // op-CHANGING RCF costs a full feedback_ms. Arm `copies` repeats at
  // rcf_repeat_ms spacing whenever the commanded content differs from the
  // previously sent frame; steady-state re-sends of an unchanged command
  // are already their own retries and arm nothing.
  const bool changed = !have_last_cmd_ || r.profile != last_cmd_profile_ ||
                       mabur::rc::overhead_to_x100(r.fec_overhead_base) != last_cmd_ovx100_b_ ||
                       mabur::rc::overhead_to_x100(r.fec_overhead_enh) != last_cmd_ovx100_e_ ||
                       r.probe_profile != last_cmd_probe_profile_;
  note_cmd(r);
  if (changed && cfg_.rcf_repeat_copies > 0) {
    repeats_left_ = cfg_.rcf_repeat_copies;
    next_repeat_ms_ = now_ms + cfg_.rcf_repeat_ms;
  }
  return Out{mabur::rc::pack_rcf(r), false};
}

mabur::rc::Rcf VrxController::build_rcf() {
  seq_ = static_cast<uint16_t>(seq_ + 1);
  mabur::rc::Rcf r;
  r.vtx_id = cfg_.vtx_id;
  r.seq = seq_;
  r.profile = mabur::rc::encode_profile(
      cur_op_.vht ? mabur::rc::PhyMode::VHT : mabur::rc::PhyMode::HT,
      static_cast<uint8_t>(cur_op_.mcs), static_cast<uint8_t>(cur_op_.bw));
  r.fec_overhead_base = cur_op_.overhead_base;
  r.fec_overhead_enh = cur_op_.overhead_enh;
  r.probe_profile = mabur::rc::kNoProbeProfile;
  if (cfg_.pin_mcs < 0 && ctrl_.probing()) {
    r.probe_profile = mabur::rc::encode_profile(
        cur_op_.vht ? mabur::rc::PhyMode::VHT : mabur::rc::PhyMode::HT,
        static_cast<uint8_t>(ctrl_.probe_mcs()), static_cast<uint8_t>(cur_op_.bw));
  }
  return r;
}

void VrxController::note_cmd(const mabur::rc::Rcf& r) {
  have_last_cmd_ = true;
  last_cmd_profile_ = r.profile;
  last_cmd_ovx100_b_ = mabur::rc::overhead_to_x100(r.fec_overhead_base);
  last_cmd_ovx100_e_ = mabur::rc::overhead_to_x100(r.fec_overhead_enh);
  last_cmd_probe_profile_ = r.probe_profile;
}

std::optional<std::vector<uint8_t>> VrxController::poll_repeat(double now_ms) {
  if (repeats_left_ <= 0 || now_ms < next_repeat_ms_) return std::nullopt;
  --repeats_left_;
  next_repeat_ms_ = now_ms + cfg_.rcf_repeat_ms;
  // Rebuild from CURRENT state, not a stashed frame: if the op moved again
  // between arm and drain, the repeat carries the newest command (and the
  // next step() emission compares against what actually went out).
  mabur::rc::Rcf r = build_rcf();
  note_cmd(r);
  return mabur::rc::pack_rcf(r);
}

const OpPoint& VrxController::cur_op() const { return cur_op_; }
VrxState VrxController::link_state() const { return rz_.state(); }
uint16_t VrxController::rcf_seq() const { return seq_; }

}  // namespace maburgs
