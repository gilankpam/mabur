#pragma once

#include <array>
#include <optional>
#include <vector>

#include "mabur/link_key.h"
#include "mabur/rc_proto.h"

#include "ladder_controller.h"
#include "op_point.h"
#include "rendezvous.h"

namespace maburgs {

struct VrxCfg {
  // Link key: tags every DISC/RCF this controller builds (link pairing).
  // config.link.key on the GS.
  mabur::LinkKey key = mabur::kDefaultLinkKey;
  uint8_t op_channel = 149;
  int feedback_ms = 100;
  int beacon_keepalive_ms = 1000;
  // Measured-loss ladder controller config (see LinkCfg::ladder_cfg,
  // config.h). Consulted every tick unless pinned.
  LadderCfg ladder;
  // Static-link pin: mcs >= 0 bypasses the adaptive controller (see
  // LinkCfg::static_mcs). overhead pair used only when pinned (from
  // LinkCfg::static_overhead_base/enh).
  int pin_mcs = -1;
  int pin_bw = 20;  // link.static_bw
  double pin_overhead_base = 0.25;
  double pin_overhead_enh = 0.25;
  // link.probe.pin_mcs: static-pin mode only -- probe a fixed MCS while
  // pinned (bench validation).
  int probe_pin_mcs = -1;
  // Rendezvous vrx_nonce: 0 = random per process (a restarted GS is a new
  // session, rendezvous.h). Only a replay pins one, so its sent frames (DISC
  // nonce, RCF tags) are reproducible -- the native/WASM parity trace.
  uint32_t rz_nonce = 0;
};

// A DISC proposes the channel it is sent on: the GS builds one DISC per tick proposing op and fans it to
// every target card, but a search-burst copy leaves on the scout card's
// member X -- proposing op there made the drone ack "agreed op" and retune
// itself to op on promotion while the GS linked on X. This rewrites
// Disc.op_channel to `ch` (the sending card's channel) and re-packs/re-tags
// the frame under `key`. Returned unchanged when it does not parse as a
// DISC, ch is 0 (unknown) or it already proposes ch. DISC is not
// RTT-matchable: the slotter needs nothing else.
std::vector<uint8_t> disc_for_channel(const std::vector<uint8_t>& disc, uint8_t ch,
                                      const mabur::LinkKey& key);

class VrxController {
 public:
  explicit VrxController(VrxCfg cfg);
  // A video body arrived: feeds the rendezvous video-silence timer only. The
  // RSSI/SNR/seq the old ScoreWindow consumed here went nowhere but the RCF
  // score/ack_seq fields, both deleted from the wire in RC_VERSION 3.
  void on_video(double now_ms);
  void on_rc_frame(const uint8_t* buf, size_t len, double now_ms);
  struct Out {
    std::vector<uint8_t> frame;
    bool is_disc;
  };
  // health: this window's measured loss (LadderController::LinkHealth, see
  // ladder_controller.h). Ignored entirely in pin mode. The ladder's own
  // internal checks (video_starved forces the failsafe rung; sample_valid
  // gates everything else) replace the old SNR-survivor-bias special case
  // here — see ladder_controller.cpp update().
  std::optional<Out> step(double now_ms, const LinkHealth& health);
  const OpPoint& cur_op() const;
  // The ladder controller itself, for the sideport link.ctl block and
  // the "ctl: rung a->b" transition line in main.cpp. Exists even in pin
  // mode (constructed unconditionally) but is never ticked/updated there.
  const LadderController& ctl() const { return ctrl_; }
  VrxState link_state() const;
  // Low 16 bits of rcf_seq32(): the wire seq of the last built RCF.
  uint16_t rcf_seq() const;
  // The RCF tag counter: tag ctx = {vrx_nonce, vtx_nonce, seq32}. Restarts at
  // 0 on each NEW vtx_nonce adoption so the drone's fresh per-session seq
  // tracker and our ctx agree from the first RCF.
  uint32_t rcf_seq32() const { return seq32_; }
  // The probe byte the last built RCF carried (kNoProbeProfile when none).
  uint8_t probe_profile() const { return last_cmd_probe_profile_; }
  // chip_caps from the most recently accepted DiscAck; 0 before any accept.
  // Gates GS main's video tail on mabur::rc::CAP_FRAME_WIRE.
  uint16_t peer_caps() const { return peer_caps_; }
  // Whether any DiscAck has ever been accepted. peer_caps() == 0 is ambiguous
  // on its own — a peer may genuinely advertise no caps — so callers that
  // want to complain about a peer's missing capability must wait for this to
  // be true or they complain about a peer they have not heard from yet. A
  // key_mismatch-flagged ack does not count.
  bool peer_acked() const { return peer_acked_; }
  // In-flight channel hop. set_hop() is carried in
  // every RCF built from now on (build_rcf() stamps r.hop_ch/r.hop_epoch);
  // restore_rung()/blank_store() forward straight to the ladder controller
  // -- restore_rung() additionally refreshes cur_op_ immediately (the same
  // way step() does after a rung change) so an RCF built in the SAME tick
  // already carries the restored profile.
  void set_hop(uint8_t hop_ch, uint8_t hop_epoch) {
    hop_ch_ = hop_ch;
    hop_epoch_ = hop_epoch;
  }
  void restore_rung(int rung, double now_ms);
  void blank_store(double until_ms) { ctrl_.blank_store(until_ms); }
  // Static-link pin, live (config.link.static_*): mcs < 0 is adaptive
  // (the ladder drives cur_op()), mcs >= 0 bypasses it with a fixed op.
  // Takes effect on the next step(), exactly as the boot-time cfg does; on
  // a change back to adaptive, sync_op_() so cur_op() reads the ladder's
  // op before that next tick rather than the stale pinned value.
  void set_pin(int mcs, int bw, double overhead_base, double overhead_enh) {
    cfg_.pin_mcs = mcs;
    cfg_.pin_bw = bw;
    cfg_.pin_overhead_base = overhead_base;
    cfg_.pin_overhead_enh = overhead_enh;
    if (mcs < 0) sync_op_();
  }
  int pin_mcs() const { return cfg_.pin_mcs; }
  // Rebuild the ladder controller inside the session (config.link.ladder,
  // live): the old op rung's {mcs, bw} is looked up in the new ladder and
  // restored to that index when present, else rung 0. restore() always
  // runs (even landing on rung 0) so the re-key settle window opens for
  // every op-point change this causes, not only the ones that move rung().
  void replace_ladder(const LadderCfg& ladder, double now_ms);
  const LadderCfg& ladder() const { return cfg_.ladder; }
  uint8_t hop_ch() const { return hop_ch_; }
  uint8_t hop_epoch() const { return hop_epoch_; }

  // Rendezvous nonce for test construction of acceptable DiscAcks.
  uint32_t rz_nonce() const { return rz_.nonce(); }
  void set_proposal(uint8_t ch) { rz_.set_proposal(ch); }
  // Hold the SESSION keep-alive DISC (a hop order is in flight): its proposal
  // is the old op by construction, and a drone that has already followed the
  // order's RCF would otherwise process the DISC it received on the old
  // channel and retune straight back (seen on the bench). RCFs are unaffected;
  // the keep-alive is due at once when the hold lifts. Ignored until the
  // peer has acked (the stale-caps fast cadence always runs).
  void set_keepalive_hold(bool hold) { keepalive_hold_ = hold; }
  // VTX recorder wish for every RCF (RecControl::wire()).
  void set_rec_wish(uint8_t w) { rec_wish_ = w; }
  // GS-requested IDR epoch for every RCF: maburgs sets it from its
  // RefreshRequester each tick, the web GS from its own request counter.
  void set_idr_epoch(uint8_t e) { idr_epoch_ = e; }
  // Phase lock: maburplay's sensor-rate offset (via RecControl), carried in
  // every RCF. 0 = no steering.
  void set_snr_off(int16_t mfps) { snr_off_ = mfps; }
  uint8_t proposal() const { return rz_.proposal(); }
  // Last accepted ack's agreed_channel (0 before any accept). Set BEFORE
  // peer_caps_ so a caller reading both on one tick sees a consistent pair.
  uint8_t agreed_channel() const { return agreed_channel_; }
  // Channel-move edge (link pairing): the drone
  // retunes only after our first RCF verifies, so the GS follows it to the
  // agreed channel when a Telem says LINKED (the drone emits one on the old
  // channel as it promotes the session) or, if that frame is lost, after
  // kMoveAfterRcfs RCFs sent under this session. Armed on each NEW
  // vtx_nonce adoption; true once per arming, cleared by read.
  static constexpr int kMoveAfterRcfs = 5;
  bool take_move_edge() { const bool e = move_edge_; move_edge_ = false; return e; }
  // Test seam: arm the move edge as a LINKED Telem under a fresh vtx_nonce would.
  void test_set_move_edge() { move_edge_ = true; }
  // Telem.state from the drone (2 == RcAgent::State::LINKED).
  void note_drone_state(uint8_t telem_state);
  // The drone answers our DISCs with key_mismatch: our key differs from its.
  // No RCF/cal goes out; beacons continue.
  bool key_mismatch() const { return rz_.state() == VrxState::KEY_MISMATCH; }
  // Tag ctx for cal frames: {vrx, vtx, 0}; vtx 0 when no session is held.
  mabur::rc::TagCtx session_ctx() const {
    return mabur::rc::TagCtx{rz_.nonce(), rz_.vtx_nonce().value_or(0), 0};
  }

 private:
  mabur::rc::Rcf build_rcf();
  void note_cmd(const mabur::rc::Rcf& r);
  // cur_op_ = OpPoint derived from ctrl_.op() -- the one place step() and
  // restore_rung() both refresh the cached operating point from the ladder.
  void sync_op_();

  VrxCfg cfg_;
  LadderController ctrl_;
  VrxRendezvous rz_;
  double last_fb_ms_ = -1e18;
  double last_keepalive_ms_ = -1e18;
  uint32_t seq32_ = 0;
  OpPoint cur_op_;
  uint16_t peer_caps_ = 0;
  bool peer_acked_ = false;
  bool keepalive_hold_ = false;
  uint8_t last_cmd_probe_profile_ = mabur::rc::kNoProbeProfile;
  uint8_t agreed_channel_ = 0;
  bool move_armed_ = false;
  bool move_edge_ = false;
  int rcfs_since_adopt_ = 0;
  uint8_t hop_ch_ = 0;
  uint8_t hop_epoch_ = 0;
  uint8_t rec_wish_ = 0;
  uint8_t idr_epoch_ = 0;
  int16_t snr_off_ = 0;
};

}  // namespace maburgs
