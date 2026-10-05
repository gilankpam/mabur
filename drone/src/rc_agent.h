#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "config.h"
#include "mabur/channel_set.h"
#include "mabur/link_key.h"
#include "mabur/profile.h"
#include "mabur/rc_proto.h"

namespace mabur {

// The resolved operating point: the 2-slot ladder ([0]=BASE at profile_mcs
// -1, [1]=ENH at profile_mcs — spec 2026-08-29-airtime-balance-uep §2), the
// per-stream literal FEC air overhead PAIR (Task 6, RC_VERSION 5: the
// single fec_overhead scalar split into fec_ov_base/fec_ov_enh — fixed
// per-rung pairs, spec 2026-08-30-same-rate-fixed-pairs; applied directly
// to the UEP layers via apply_op_to_uep, no ladder translation), per-layer
// shed flags (failsafe-forced OR local congestion-directed; shed[1] is the
// enh shed, the old shed[3] — the old reserved layer and its shed[2] slot
// are gone), and
// a generation counter bumped only when a *new* operating point
// (ladder/FEC) is applied (BOOT/RCF/failsafe entry) — NOT on every
// publish. Congestion shed re-applies the *current* op (same ladder/FEC)
// with updated shed flags and publishes a fresh AppliedOp WITHOUT bumping
// generation, so consumers MUST NOT use generation to detect "did a new
// AppliedOp get published" — every apply_op() call (via Actuator::apply_op)
// constructs and stores a brand new object, so callers that need "did the
// published object change" should identity-compare the shared_ptr they last
// observed against the newly loaded one instead.
struct AppliedOp {
  std::array<rc::LayerTxSpec, 2> ladder;
  double fec_ov_base = 2.0;
  double fec_ov_enh = 2.0;
  std::array<bool, 2> shed = {false, false};
  uint64_t generation = 0;

  // Probe stream (spec 2026-09-04): the RCF's probe_profile byte as received
  // (kNoProbeProfile = no stream) and the resolved TX spec for it — same
  // mode as the op, but the probe rung's OWN bw (a 20/4 op probes 40/3),
  // LDPC+STBC like the video slots. RadioTx slot 2.
  uint8_t probe_profile = rc::kNoProbeProfile;
  rc::LayerTxSpec probe;
};

// Volatile per-layer overhead override (bench sweeps, set via the debug
// HTTP :8301 POST /venc/set?ov_base_pct=N / ov_enh_pct=N; -1 = off, both
// must be >= 0 to take effect). main.cpp's hot loop applies the forced pair
// to the UEP layers directly (the op pair loses), and run_bitrate_policy
// reads the same pair so its target describes what is ACTUALLY flying
// rather than the commanded op. Not persisted; a daemon restart clears it.
//
// Plain atomics, no lock: the HTTP thread writes, the hot loop and the
// agent thread read. Torn reads are impossible (each field loaded
// independently) and a stale value by one tick is harmless for a rate
// target. May be null (tests), in which case no override is ever armed.
//
// This struct is all that survives of AirFeedOut. AirFeed itself — the
// measurement half of the deleted AirBalancer — was removed on 2026-09-01
// along with the bitrate blend that was its only consumer; see
// run_bitrate_policy.
struct OvOverride {
  std::atomic<int> ovr_base_pct{-1};
  std::atomic<int> ovr_enh_pct{-1};
};

// Everything RcAgent does to the outside world funnels through this
// interface, so tests can substitute a recording double instead of driving
// real radio/encoder hardware.
class Actuator {
 public:
  virtual ~Actuator() = default;
  virtual void apply_op(const AppliedOp&) = 0;                     // ladder+FEC+shed
  virtual void send_control(const std::vector<uint8_t>& body) = 0;  // DISC_ACK
  // The two encoder-parameter verbs report whether the encoder actually
  // took the value. RcAgent latches its "last commanded" state ONLY on
  // true, so a transient failure is re-sent on the next policy tick
  // instead of leaving the encoder silently diverged from the ladder for
  // the rest of the flight (the waybeam-wedge failure mode, in-process
  // edition — memory: waybeam-bitrate-wedge).
  virtual bool set_bitrate_kbps(int) = 0;
  virtual bool set_roi_qp(int) = 0;
  // Low-power mode (spec 2026-09-20): the encoder's frame rate. Same
  // contract as set_bitrate_kbps -- true iff the encoder took it; RcAgent
  // latches commanded_fps_ only on true and retries otherwise.
  virtual bool set_fps(int fps) = 0;
  // No return: an IDR is a one-shot request with no latched state to keep
  // consistent, and the pacer has already spent its budget. A failed one is
  // re-raised by whatever produced it (the next chain break, or the next
  // RCF that re-enters LINKED).
  virtual void request_idr() = 0;
  // Requests a retune to the given channel (spec 2026-09-13
  // auto-channel-select §6: FastRetune). Called from the agent thread only,
  // same as every other verb on this interface. No return: a retune that
  // fails to take leaves the radio on its previous channel, which the
  // move-confirm/fallback-home machinery in RcAgent::tick already treats as
  // "nothing heard on the new channel" and recovers from on its own.
  //
  // `reason` is a stable, short literal naming WHY the move is happening --
  // "disc" (a DISC proposed a channel we agreed to), "hop" (an RCF carried
  // a new hop order), "move_unconfirmed" (the post-move confirm window
  // expired, so the drone returned to the channel it came from). It is
  // spec §7 observability only: the real
  // actuator prints it on the retune line so a stderr/serial capture says
  // which of the three fired. Never null, never freed -- always a string
  // literal.
  virtual void retune(uint8_t ch, const char* reason) = 0;
  // VTX onboard recorder (spec 2026-09-26): hand the operator's wish to the
  // recorder. true = handed over (the recorder reports its own outcome in
  // Telem::rec_status); RcAgent latches the wish only on true.
  virtual bool set_record(bool on) = 0;
  // A move the GS has confirmed (the first accepted RCF on the new channel):
  // persist it so the next boot parks there (spec 2026-10-03 §2 state
  // files). Default no-op: tests and dry-run have nowhere to write.
  virtual void remember_channel(uint8_t ch) { (void)ch; }
};

// Radio-side health signals sampled once per tick. thermal_delta is
// telemetry-only (nothing acts on it); tx_drops and txq_depth/txq_cap feed
// the congestion-shed policy (run_congestion_guard).
struct RadioHealth {
  int thermal_delta = 0;
  uint64_t tx_drops = 0;  // TxStats::failed: USB bulk-OUT failures
  // TxQueue occupancy at the tick (main.cpp). 0/0 when unknown (tests,
  // pre-radio ticks) -- never reads as pressure.
  size_t txq_depth = 0;
  size_t txq_cap = 0;
};

// Control-plane state machine: BOOT -> RENDEZVOUS -> LINKED <-> FAILSAFE.
// Consumes inbound RC frames (RCF feedback, DISC discovery beacons) and a
// periodic tick (which also carries radio health for the congestion
// guard), and drives an Actuator with the resolved operating point. All
// timing is driven by the `now_ms` parameters callers pass in — RcAgent
// makes no real-time calls of its own, so tests can simulate any clock.
class RcAgent {
 public:
  enum class State { BOOT, RENDEZVOUS, LINKED, FAILSAFE };

  // ovr is the debug-HTTP per-layer overhead override (bench sweeps); may
  // be null (tests), in which case no override is ever armed and the
  // bitrate policy always builds its target from the commanded pair.
  // start_ch is the channel to park on (spec 2026-10-03-auto-channel-set
  // §2: the remembered member from the state file, read by main.cpp); 0 (or
  // a non-member) means cfg.radio.channels.front().
  RcAgent(const Config& cfg, Actuator& act, OvOverride* ovr = nullptr, uint8_t start_ch = 0);

  // Parses `body` as an RC frame (RCF or DISC; anything else, or a frame
  // failing CRC, is silently ignored) and applies its effect.
  void on_rc_frame(const uint8_t* body, size_t len, uint64_t now_ms);

  // The encoder lost a reference frame (a ring-full drop ate it), so the
  // decode chain downstream is broken until an IDR re-seeds it. Called from
  // the venc encoder thread (venc_core.h's on_chain_break) — the ONLY
  // cross-thread entry point on RcAgent, which is why it is a bare atomic
  // set and nothing else. The request is consumed on the next tick(), on the
  // agent thread, where it goes through the same IDR pacer as every other
  // producer.
  void note_chain_break();

  // The FC's arm state as decoded from an MSP_STATUS reply (spec
  // 2026-09-20 §1), stamped with the MSP thread's clock. The second
  // cross-thread entry point after note_chain_break(), and like it a bare
  // atomic store; consumed at the top of tick() on the agent thread.
  void note_arm_state(bool armed, uint64_t now_ms);

  // Advances the failsafe/rendezvous timers and re-evaluates the
  // congestion guard against the given health sample. On BOOT, the first
  // call applies the MAX_RANGE op and transitions to RENDEZVOUS.
  void tick(uint64_t now_ms, const RadioHealth& health);

  State state() const { return state_; }
  const AppliedOp& current() const { return applied_; }

  // The channel the agent last commanded (requested, not radio-confirmed)
  // (spec 2026-10-03-auto-channel-set §2/§3).
  uint8_t channel() const { return channel_; }

  // The epoch of the last in-flight hop order applied from an RCF (spec
  // 2026-09-14 §1). 0 before any hop order has ever been heard.
  uint8_t hop_epoch() const { return hop_epoch_; }

  // GS-requested IDRs actually issued (spec 2026-09-28), lifetime. Left the
  // wire with Telem.idr_gs (2026-09-30); kept as the observable the
  // request/pacer tests pin.
  uint64_t idr_gs_total() const { return idr_gs_total_; }

  // Telemetry accessors (spec 2026-07-26 drone-telemetry): read-only
  // snapshots of RcAgent-internal state the T_TELEM collector needs but
  // that isn't otherwise exposed. All same-thread reads (the agent thread
  // owns both RcAgent and the telemetry collector call site in main.cpp).
  bool failsafe_shed() const { return failsafe_shed_; }
  // True while run_congestion_guard holds any shed level (TxQueue at/past
  // half cap, or USB TX failures) — Telem flags bit4. Reported separately
  // from failsafe_shed so a shed caused by the encoder overshooting its
  // command (flight-0011) is countable on the bench and attributable in
  // flightreport, instead of vanishing into an unexplained enh gap.
  bool congestion_shed() const { return shed_level_ > 0; }
  bool have_feedback() const { return have_last_fb_; }
  uint64_t last_feedback_ms() const { return last_fb_ms_; }
  uint64_t rcf_accepted() const { return rcf_accepted_; }
  // Session bookkeeping (link pairing, spec 2026-10-01 §6/§7). All RAM,
  // nothing persisted. A DISC from vrx_nonce gets a vtx_nonce (a PENDING
  // pair); the first RCF that verifies under it makes it CURRENT.
  // last_seq32 is the extended RCF seq the tag is computed over; a frame is
  // fresh only if its seq32 is strictly greater.
  struct Session {
    uint32_t vrx_nonce = 0, vtx_nonce = 0;
    bool valid = false;
    uint32_t last_seq32 = 0;
    bool have_seq = false;
    uint8_t agreed_ch = 0;     // the DISC proposal this session was acked with
  };
  const Session& current_session() const { return current_; }

  // link-rtt: seq of the RCF that last_feedback_ms/rcf_age_ms age against.
  // Empty outside LINKED: failsafe entry rebases last_fb_ms_ to now (and an
  // unconfirmed move drops to RENDEZVOUS), so the age is fresh but no RCF
  // backs it -- echoing the session's (kept) seq would let the GS fabricate
  // an RTT sample from the wrong send time. The next accepted RCF re-enters
  // LINKED and makes it valid again.
  std::optional<uint16_t> last_feedback_seq() const {
    if (state_ != State::LINKED || !current_.valid || !current_.have_seq) return std::nullopt;
    return static_cast<uint16_t>(current_.last_seq32);
  }

  // True once after a pending pair became current (main.cpp sends one
  // "promote" Telem on the current channel before tick() runs any deferred
  // channel move). Agent thread only.
  bool take_session_promoted() {
    const bool v = session_promoted_;
    session_promoted_ = false;
    return v;
  }
  // True once if any control frame failed verification since the last read
  // (Telem flags bit1). Atomic: set from the agent thread (DISC/RCF) and the
  // TX writer thread (verify_cal_frame), read once per Telem period.
  bool take_auth_reject() { return auth_reject_.exchange(false, std::memory_order_relaxed); }
  // Cal frames are verified on the TX writer thread against the CURRENT
  // session, read atomically (published_session_). No pending fallback: a
  // sweep can only open under a linked session. Sets the auth_reject flag
  // on failure.
  //
  // `sweep_running` (CalSweep::active()) extends that for the life of one
  // run: the newest pair a cal frame verified under stays accepted until
  // the sweep closes, even after failsafe clears current_ -- which happens
  // in every run, because the GS is radio-silent while each phase airs.
  // Without it the CAL_RESULT, sent into the silent verify window, never
  // verifies and nothing is ever applied (bench repro 2026-10-03). Passing
  // false drops the latch, so it cannot outlive the sweep.
  //
  // Cal frames are tagged with seq32 = 0, so freshness is the cal nonce: a
  // T_CAL_CMD whose nonce was already accepted in this link session -- other
  // than the most recent one (a retransmission or the next phase of the
  // running sweep) -- is refused like a failed tag. The seen-nonce ring
  // (kCalNonceRing deep) belongs to the pair its nonces verified under and
  // is forgotten when a cal frame first verifies under a different one.
  // Owned by verify_cal_frame's caller thread (the TX writer); not
  // thread-safe against concurrent verify_cal_frame calls.
  bool verify_cal_frame(const uint8_t* body, size_t len, bool sweep_running = false);
  // SPIKE 2026-10-05 (fec-nack): verify a T_NACK's tag against the
  // published session with the frame's own seq32 as ctx. Any thread.
  bool verify_session_tagged(const uint8_t* body, size_t len, uint32_t seq32) const;
  static constexpr size_t kCalNonceRing = 8;
  // Replay harness only (maburd --dry-run): install a known pair so a file
  // of RCFs tagged under (vrx, vtx) verifies without a DISC exchange.
  void install_session_for_replay(uint32_t vrx_nonce, uint32_t vtx_nonce);

  // True while the drone is emitting the probe stream: LINKED and the last
  // accepted RCF commanded a probe MCS. Telem flags bit2.
  bool probe_on() const {
    return state_ == State::LINKED && applied_.probe_profile != rc::kNoProbeProfile;
  }

  // True while the low-power operating point is in force (spec 2026-09-20
  // §2): mode enabled, the last arm report says DISARMED, and that report is
  // fresher than low_power.stale_ms. Telem flags bit7. Follows the FC's
  // current state both ways -- a DISARMED report after an arm re-enters, so
  // a downed-but-powered aircraft falls back to the thin stream.
  bool low_power() const { return low_power_active_; }
  // Latched by the first ARMED report; never clears. Observability only
  // (the drone stats line's armed=): "has this process ever seen the FC
  // armed", which separates a pre-flight drone from a downed one. NOT a
  // policy input -- low_power() deliberately does not read it.
  bool armed_latched() const { return armed_latched_; }

  // Latched on a BOOT/RENDEZVOUS -> LINKED transition — the process-(re)start
  // link-up, when frames encoded so far never reached the air and the GS may
  // hold a stale frame-id cursor. The caller consumes it to re-mark the frame
  // discontinuity window (FramePipeline::mark_discontinuity). Deliberately
  // NOT latched on FAILSAFE -> LINKED: a routine RF flap re-basing the GS's
  // id space would evict its in-flight frames for nothing.
  bool take_link_established() {
    bool v = link_established_;
    link_established_ = false;
    return v;
  }

 private:
  const Config& cfg_;
  Actuator& act_;
  OvOverride* ovr_;  // may be null — see the constructor comment
  State state_ = State::BOOT;
  bool link_established_ = false;  // see take_link_established()

  // channel_ is the start channel (the remembered member, else
  // channels.front()) at construction and tracks the channel the agent
  // last commanded. The drone is always on a member and moves only on a GS
  // proposal, a GS order, or an unconfirmed move back to move_from_ch_.
  // move_pending_/move_at_ms_ track an unconfirmed move: set when tick()
  // runs a promoted session's deferred move (or an RCF hop order) and
  // act_.retune() is called, cleared by the first subsequent accepted RCF
  // that is not itself a new move (confirms the move) or by the
  // move-confirm fallback in tick() (which returns to move_from_ch_).
  uint8_t channel_;
  bool move_pending_ = false;
  uint64_t move_at_ms_ = 0;
  // The channel a move (a DISC move OR a hop order) moved us FROM, while
  // that move is unconfirmed (0 = none: the revert already spent). The
  // move-confirm fallback returns here and stays -- there is no home to
  // fall through to. This is also where a withdrawing GS goes (spec
  // 2026-09-14 §1 step 4). Bench 2026-09-26: with the hop target == home,
  // going "home" was a no-op and the pair split 60 s.
  uint8_t move_from_ch_ = 0;

  // In-flight hop order (spec 2026-09-14 §1). have_hop_ is false until the
  // first RCF carrying a nonzero hop_ch is accepted; hop_epoch_/hop_ch_ then
  // track the (epoch, ch) PAIR of the last applied order (spec §1: "an RCF
  // whose (hop_epoch, hop_ch) differs from the last pair applied") so a
  // repeat of the same pair is idempotent but a same-epoch new channel is
  // still applied. Reset at every session boundary (session promotion,
  // unconfirmed-move fallback, FAILSAFE entry) so a restarted GS's hop epoch
  // numbering can't leave a stale latch here silently swallowing its first
  // hop order.
  uint8_t hop_epoch_ = 0;
  uint8_t hop_ch_ = 0;
  bool have_hop_ = false;

  AppliedOp applied_;

  uint64_t last_fb_ms_ = 0;
  bool have_last_fb_ = false;

  Session current_, pending_;
  // (vrx << 32 | vtx) of current_, 0 = none. Written on the agent thread,
  // read by verify_cal_frame on the TX writer thread.
  std::atomic<uint64_t> published_session_{0};
  // verify_cal_frame's seen-cal-nonce ring (TX writer thread only).
  uint64_t cal_ring_session_ = 0;      // the pair the ring's nonces verified under
  uint64_t cal_latch_session_ = 0;     // newest pair a cal frame verified under, 0 = none
  std::array<uint32_t, kCalNonceRing> cal_seen_{};
  size_t cal_seen_n_ = 0, cal_seen_next_ = 0;
  bool have_cal_current_ = false;
  uint32_t cal_current_ = 0;           // newest accepted cal nonce
  std::atomic<bool> auth_reject_{false};
  bool session_promoted_ = false;
  uint8_t deferred_move_ch_ = 0;   // executed in tick() after main sent the promote Telem
  uint32_t fresh_vtx_nonce_();
  // true + fills *seq32 if the RCF bytes verify under s with a fresh seq.
  bool verify_rcf_(const uint8_t* body, size_t len, const rc::Rcf& r, const Session& s,
                   uint32_t* seq32) const;
  void publish_session_();
  // Drops current_ and pending_ and republishes: every exit from LINKED.
  void clear_sessions_();

  // Cumulative count of RCFs accepted (fresh) — feeds
  // Telem.rcf_rx. Never reset (a session-boundary reset would make the GS's
  // rate computation, which is over a measured interval, ambiguous).
  uint64_t rcf_accepted_ = 0;

  // IDR policy state (spec 2026-08-28 venc-foldin §4). Every IDR producer
  // — the GS-driven RCF-after-failsafe path, the encoder's chain-break
  // signal, and RCF idr_epoch requests — funnels through idr_due();
  // nothing else may call act_.request_idr(). chain_break_pending_ is the
  // venc thread's handoff (see note_chain_break); the two timestamps are
  // agent-thread-only.
  std::atomic<bool> chain_break_pending_{false};
  // have_* companions rather than a 0 sentinel (the file's own idiom, cf.
  // have_last_fb_/have_last_bitrate_eval_): now_ms is a caller-supplied
  // clock that legitimately starts at 0, and a first IDR at t=0 must still
  // arm the 100 ms floor.
  uint64_t last_idr_ms_ = 0;
  bool have_last_idr_ = false;
  uint64_t last_chain_idr_ms_ = 0;
  bool have_last_chain_idr_ = false;
  bool idr_due(uint64_t now_ms, bool chain);

  // Low-power (pre-arm) state, spec 2026-09-20. Everything below
  // arm_report_ is agent-thread-only, unpacked from it once per tick by
  // intake_arm_state_().
  //
  // Arm report handoff (the MSP thread's, the second cross-thread entry
  // point after chain_break_pending_ above -- see note_arm_state):
  // ((ms + 1) << 1) | armed, 0 = never reported. A newer report simply
  // overwrites; the tick unpacks whatever is current.
  std::atomic<uint64_t> arm_report_{0};
  bool armed_latched_ = false;
  bool arm_reported_ = false;
  bool last_armed_ = false;
  uint64_t last_arm_ms_ = 0;
  bool low_power_active_ = false;
  // Last fps the encoder ACCEPTED (0 = never), the fps twin of
  // last_bitrate_kbps_.
  int commanded_fps_ = 0;
  // Last recorder wish the actuator TOOK (-1 = none yet). Only a KNOWN wish
  // (rc::kRecKnown) is applied; link loss never touches it.
  int rec_applied_ = -1;
  void intake_arm_state_(uint64_t now_ms);

  // GS-requested IDR (spec 2026-09-28 web-idr-request). The last RCF
  // idr_epoch seen this session (0 at every session edge) and whether a
  // request is waiting for the pacer. Agent-thread-only.
  uint8_t idr_epoch_seen_ = 0;
  bool idr_gs_pending_ = false;
  uint64_t idr_gs_total_ = 0;

  // Bitrate policy state.
  int last_bitrate_kbps_ = 0;
  bool have_last_bitrate_ = false;
  uint64_t last_bitrate_eval_ms_ = 0;
  bool have_last_bitrate_eval_ = false;
  bool roi_low_ = false;
  // Set by run_bitrate_policy() whenever one of its three encoder verbs --
  // set_fps() (low-power mode only), set_bitrate_kbps() or set_roi_qp() --
  // returns false, cleared when all of them are in the state the policy
  // wants. Drives the per-tick retry half of the periodic re-assert (see
  // kReassertMs / tick()).
  bool verb_apply_failed_ = false;
  // "run_bitrate_policy() already ran at this now_ms" — the tick-level
  // re-assert below consults it so a tick that has ALREADY run the policy
  // (a max-range/failsafe entry on this very tick, or an RCF the agent loop
  // drained at the same millisecond) does not immediately run it a second
  // time. At most one policy run per distinct now_ms.
  uint64_t last_policy_ms_ = 0;
  bool have_last_policy_ = false;

  // Congestion-shed state.
  int shed_level_ = 0;
  uint64_t last_drop_rise_ms_ = 0;
  bool have_last_drop_rise_ = false;
  uint64_t last_tx_drops_ = 0;
  bool have_last_tx_drops_ = false;

  // True for as long as MAX_RANGE is the operating point — i.e. RENDEZVOUS
  // (including BOOT's initial apply) or FAILSAFE — set in apply_max_range()
  // and cleared the moment a resolved RCF op takes the agent back to
  // LINKED (apply_ladder_op()). OR'd with the congestion-level shed
  // whenever (re)building AppliedOp.shed[1] (the enh layer — the old
  // shed[2]/[3] pair collapsed to this single slot when the reserved layer
  // was deleted), so a later congestion-guard reapply (which recomputes
  // shed from shed_level_ alone) can never silently drop the MAX_RANGE-
  // forced shed — most importantly, FAILSAFE's.
  bool failsafe_shed_ = false;

  // Periodic re-assert interval, ms. run_bitrate_policy() only pushes on
  // CHANGE, so without this nothing ever restates the commanded rate: a
  // verb that failed during a FAILSAFE entry stayed unrepaired until the
  // next RCF or rendezvous (up to rendezvous_ms, 30 s), and anything that
  // moved the encoder behind RcAgent's back — the debug endpoint's
  // POST /venc/set, an in-process re-init — won until the ladder happened
  // to change rung. 5 s is chosen against those two: short enough that a
  // stuck bitrate costs at most one GOP-ish window of wrong rate rather
  // than a flight, long enough that the re-apply is ~1/50 of the tick rate
  // (tick_ms 100) and cannot itself become a source of MI-call churn.
  static constexpr uint64_t kReassertMs = 5000;

  void apply_max_range(uint64_t now_ms);
  void apply_ladder_op(const std::array<rc::LayerTxSpec, 2>& ladder,
                        double ov_base, double ov_enh, uint8_t probe_profile);
  void reapply_with_shed();
  void run_bitrate_policy(uint64_t now_ms, bool force);
  void run_congestion_guard(uint64_t now_ms, const RadioHealth& health);
  rc::DiscAck make_disc_ack(uint32_t vrx_nonce, uint32_t vtx_nonce, uint8_t flags, uint16_t seq,
                            uint8_t agreed) const;

  bool member_(uint8_t ch) const { return mabur::channel_set_member(cfg_.radio.channels, ch); }
};

}  // namespace mabur
