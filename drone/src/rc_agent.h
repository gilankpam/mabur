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
// -1, [1]=ENH at profile_mcs), the per-stream literal FEC air overhead PAIR
// (RC_VERSION 5: the single fec_overhead scalar split into
// fec_ov_base/fec_ov_enh — fixed per-rung pairs; applied directly to the
// UEP layers via apply_op_to_uep, no ladder translation), per-layer
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

  // Probe stream: the RCF's probe_profile byte as received
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
// measurement half of the deleted AirBalancer — was removed along with the
// bitrate blend that was its only consumer; see run_bitrate_policy.
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
  // Low-power mode: the encoder's frame rate. Same
  // contract as set_bitrate_kbps -- true iff the encoder took it; RcAgent
  // latches commanded_fps_ only on true and retries otherwise.
  virtual bool set_fps(int fps) = 0;
  // No return: an IDR is a one-shot request with no latched state to keep
  // consistent, and the pacer has already spent its budget. A failed one is
  // re-raised by whatever produced it (the next chain break, or the next
  // RCF that re-enters LINKED).
  virtual void request_idr() = 0;
  // Requests a retune to the given channel (FastRetune). Called from the agent
  // thread only, same as every other verb on this interface. No return: a
  // retune that fails to take leaves the radio on its previous channel, which
  // the move-confirm/fallback-home machinery in RcAgent::tick already treats
  // as "nothing heard on the new channel" and recovers from on its own.
  //
  // `reason` is a stable, short literal naming WHY the move is happening --
  // "disc" (a DISC proposed a channel we agreed to), "hop" (an RCF carried
  // a new hop order), "move_unconfirmed" (the post-move confirm window
  // expired, so the drone returned to the channel it came from). It is
  // observability only: the real actuator prints it on the retune line so a
  // stderr/serial capture says which of the three fired. Never null, never
  // freed -- always a string literal.
  virtual void retune(uint8_t ch, const char* reason) = 0;
  // VTX onboard recorder: hand the operator's wish to the
  // recorder. true = handed over (the recorder reports its own outcome in
  // Telem::rec_status); RcAgent latches the wish only on true.
  virtual bool set_record(bool on) = 0;
  // Phase lock: the GS's sensor-rate offset (milli-fps on top of the boot
  // trim; 0 = the trim's own rate). No return: venc stores it and applies it
  // from the encoder thread once the trim has locked, clamped to its range.
  // Default no-op: dry-run and builds without venc have no sensor.
  virtual void set_sensor_offset(int mfps) { (void)mfps; }
  // A move the GS has confirmed (the first accepted RCF on the new channel):
  // persist it to the state file so the next boot parks there. Default
  // no-op: tests and dry-run have nowhere to write.
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
  // start_ch is the channel to park on (the remembered member from the
  // state file, read by main.cpp); 0 (or a non-member) means
  // cfg.radio.channels.front().
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

  // The FC's arm state as decoded from an MSP_STATUS reply, stamped with
  // the MSP thread's clock. The second
  // cross-thread entry point after note_chain_break(), and like it a bare
  // atomic store; consumed at the top of tick() on the agent thread.
  void note_arm_state(bool armed, uint64_t now_ms);

  // Advances the failsafe/rendezvous timers and re-evaluates the
  // congestion guard against the given health sample. On BOOT, the first
  // call applies the MAX_RANGE op and transitions to RENDEZVOUS.
  void tick(uint64_t now_ms, const RadioHealth& health);

  State state() const { return state_; }
  const AppliedOp& current() const { return applied_; }

  // The channel the agent last commanded (requested, not radio-confirmed).
  uint8_t channel() const { return channel_; }

  // The epoch of the last in-flight hop order applied from an RCF. 0 before
  // any hop order has ever been heard.
  uint8_t hop_epoch() const { return hop_epoch_; }

  // GS-requested IDRs actually issued, lifetime. Left the wire with
  // Telem.idr_gs; kept as the observable the request/pacer tests pin.
  uint64_t idr_gs_total() const { return idr_gs_total_; }

  // Chain-break IDRs actually issued, lifetime (served, not merely
  // requested -- a request deferred by the pacer is not counted until it
  // fires).
  uint64_t idr_chain_total() const { return idr_chain_total_; }

  // Telemetry accessors: read-only
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
  // Session bookkeeping (link pairing). All RAM,
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
  // (Telem flags bit1). Atomic: set from the agent thread (DISC/RCF), the RX
  // thread (check_nack) and the MAV thread (check_mav), read once per Telem
  // period.
  bool take_auth_reject() { return auth_reject_.exchange(false, std::memory_order_relaxed); }
  // fec-nack: verify a T_NACK's tag against the
  // published session with the frame's own counter as seq32 ctx. Any
  // thread. On success, *session (if given) is the published pair the tag
  // verified under -- hand it to accept_nack_counter so a session change
  // between the two calls cannot charge this request to the new pair.
  bool verify_session_tagged(const uint8_t* body, size_t len, uint32_t seq32,
                             uint64_t* session = nullptr) const;
  // T_NACK replay guard: true iff `counter` is greater than the last one
  // accepted in this link session, and stores it. The counter space is the
  // session's (keyed by its vtx nonce, which every new pair draws fresh), so
  // the first counter after the drone adopts a new pair is accepted again.
  // `session` is what verify_session_tagged reported; a request verified
  // under a pair that is no longer published is refused. The one-argument
  // form uses the currently published pair. Any thread (lock-free CAS).
  bool accept_nack_counter(uint32_t counter);
  bool accept_nack_counter(uint32_t counter, uint64_t session);
  // Raise the auth_reject flag (Telem flags bit1) for a control frame
  // refused outside the agent: the RX thread's T_NACK handler.
  void note_auth_reject() { auth_reject_.store(true, std::memory_order_relaxed); }
  // The RX thread's whole T_NACK gate: parse_nack, sid == 0,
  // verify_session_tagged, accept_nack_counter. kMalformed = parse (CRC,
  // length, version) failed or sid != 0 -- radio corruption, NOT an auth
  // failure, so auth_reject stays down (the RCF path drops a parse failure
  // silently too). kRejected = parsed but the tag or the counter failed:
  // raises auth_reject. kOk fills *out. Any thread.
  enum class NackCheck { kOk, kMalformed, kRejected };
  NackCheck check_nack(const uint8_t* body, size_t len, rc::Nack* out);
  // True while a link session is published (current_ is valid). Any thread.
  bool has_published_session() const {
    return published_session_.load(std::memory_order_acquire) != 0;
  }
  // T_MAV replay guard: accept_nack_counter's rule and CAS shape on its own
  // state (mav_last_), so the GS's MAV and NACK counters never refuse each
  // other. Any thread.
  bool accept_mav_counter(uint32_t counter, uint64_t session);
  // The MAV thread's whole T_MAV gate: parse_mav (tagged layout), the tag
  // with the counter as seq32, then the counter. kMalformed = the parse
  // failed (radio corruption): auth_reject stays down. kRejected = parsed
  // but the tag or the counter failed: raises auth_reject. kOk / kOkLatched
  // fill *out.
  //
  // Two counter domains per pair, chosen by content: a frame whose payload
  // is all calibration packets (cal::all_packets_are_cal) checks and
  // advances cal_last_, anything else mav_last_ -- the GS's calibration lane
  // restarts its counter at 1 whenever the run's pair (vtx nonce) changes,
  // and a second run on the same pair continues counting, while its live
  // lane keeps the session's own counter, so one shared counter would
  // refuse the cal lane as replays.
  //
  // kOk: the tag verified under the published session and the counter is
  // fresh in the frame's domain. A cal-only frame's counter is checked
  // against the pair its tag verified under, not a second read of the
  // published session.
  //
  // kOkLatched: the published session's tag check failed, but a run is open
  // (`cal_run_open`, CalSweep::active()) and the frame verified under the
  // run pair -- the pair of the last accepted cal-only frame -- is
  // cal-only, and its counter is above that pair's cal counter. Every sweep
  // outlasts the link failsafe (the GS is radio-silent while each phase
  // airs), so the drone clears its session inside every run while the
  // run's later cal frames stay tagged under the pair it went under. A
  // frame carrying anything but cal packets (a parameter set) never takes
  // this path: it is kRejected whole.
  //
  // The run pair is recorded on every accepted cal-only frame (kOk or
  // kOkLatched), so the SWEEP that starts a run -- accepted before the run
  // is open -- sets it, and a cal frame accepted under a newer published
  // pair moves it (a mid-run re-pair). It is honoured only while
  // `cal_run_open`; a closed run does not erase it, since it is only "the
  // pair of the last accepted cal frame", and the next run can only open
  // from a SWEEP accepted under the published session, which re-records it.
  //
  // kLateCal: the published session's tag check failed and no run is open,
  // but the frame is cal-only and verifies under the run pair -- the GS's
  // own run trailing off after the drone closed it (the repeats of its
  // closing Abort, the first of which closed the run). Dropped, counted in
  // cal_late(), and NOT an
  // auth reject: it proves the GS holds the key, and counting it would
  // read as a key mismatch at the end of every run. Its counter is neither
  // checked nor consumed, since the frame is dropped either way. A frame
  // carrying anything but cal packets, or verifying under no known pair,
  // is still kRejected.
  //
  // MAV thread only: the run pair is plain state owned by check_mav's one
  // caller; not safe against concurrent check_mav calls.
  enum class MavCheck { kOk, kOkLatched, kLateCal, kMalformed, kRejected };
  MavCheck check_mav(const uint8_t* body, size_t len, rc::Mav* out, bool cal_run_open);
  // kLateCal frames since start. Any thread.
  uint64_t cal_late() const { return cal_late_.load(std::memory_order_relaxed); }
  // Replay harness only (maburd --dry-run): install a known pair so a file
  // of RCFs tagged under (vrx, vtx) verifies without a DISC exchange.
  void install_session_for_replay(uint32_t vrx_nonce, uint32_t vtx_nonce);

  // True while the drone is emitting the probe stream: LINKED and the last
  // accepted RCF commanded a probe MCS. Telem flags bit2.
  bool probe_on() const {
    return state_ == State::LINKED && applied_.probe_profile != rc::kNoProbeProfile;
  }

  // True while the low-power operating point is in force: mode enabled, the
  // last arm report says DISARMED, and that report is fresher than
  // low_power.stale_ms. Telem flags bit7. Follows the FC's current state both
  // ways -- a DISARMED report after an arm re-enters, so a downed-but-powered
  // aircraft falls back to the thin stream.
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

  // Live config knobs: a MAVLink parameter set applies these without a
  // daemon restart. Any thread (plain atomics); seeded from `cfg` in the
  // constructor, so an RcAgent that never gets a set_live_knobs call behaves
  // exactly as if it only ever read the matching Config field.
  void set_live_knobs(int bitrate_max_kbps, double airtime_budget, bool low_power_enable,
                       int venc_fps);

  // The live low-power-enable knob (the MSP thread reads it to decide
  // whether polling arm state still matters). Any thread.
  bool low_power_enabled() const;

  // The in-process encoder pipeline was just rebuilt (a fresh MI_VENC
  // session under the same PID, e.g. after a venc.fps/size change): on the
  // next tick, a known record wish carried by the next RCF re-issues
  // set_record (the new pipeline has no memory of the old one's recorder
  // state), and the fps/bitrate verbs are reasserted immediately instead of
  // waiting for the periodic kReassertMs cadence. Any thread; a bare atomic
  // flag, consumed on the agent thread.
  void note_pipeline_rebuilt();

  // The LDPC flag changed (radio.ldpc hot-reload): on the next tick the
  // agent rebuilds the current operating point's ladder -- and its probe
  // spec, if one is active -- for the SAME rung under the new flag and
  // republishes it; every ladder built from then on uses it too. Any
  // thread; a one-slot pending value, consumed on the agent thread.
  void note_ldpc_changed(bool ldpc);

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
  // fall through to. This is also where a withdrawing GS goes. Bench: with
  // the hop target == home, going "home" was a no-op and the pair split
  // 60 s.
  uint8_t move_from_ch_ = 0;

  // In-flight hop order. have_hop_ is false until the first RCF carrying a
  // nonzero hop_ch is accepted; hop_epoch_/hop_ch_ then track the (epoch,
  // ch) PAIR of the last applied order (an RCF whose (hop_epoch, hop_ch)
  // differs from the last pair applied is a new order) so a
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
  // read by check_nack (RX thread) and check_mav (MAV thread).
  std::atomic<uint64_t> published_session_{0};
  // check_mav's run pair: (vrx << 32 | vtx) of the last accepted cal-only
  // T_MAV, 0 = none. MAV thread only.
  uint64_t cal_run_session_ = 0;
  std::atomic<uint64_t> cal_late_{0};  // check_mav's kLateCal count
  std::atomic<bool> auth_reject_{false};
  // accept_nack_counter's state: (vtx_nonce << 32) | last accepted counter.
  // A vtx other than the current pair's means "nothing accepted yet".
  std::atomic<uint64_t> nack_last_{0};
  // accept_mav_counter's state, same encoding as nack_last_.
  std::atomic<uint64_t> mav_last_{0};
  // check_mav's cal-only counter domain, same encoding. Keyed like the
  // others, so a pair other than the one it last advanced under reads as
  // "nothing accepted yet".
  std::atomic<uint64_t> cal_last_{0};
  // accept_nack_counter / accept_mav_counter / check_mav: the published
  // pair check, then advance_counter_.
  bool accept_session_counter_(std::atomic<uint64_t>& last_word, uint32_t counter,
                               uint64_t session);
  // The CAS on one state word for `session`'s key, with no published-pair
  // check (check_mav's latched path verifies against the run pair).
  static bool advance_counter_(std::atomic<uint64_t>& last_word, uint32_t counter,
                               uint64_t session);
  // True iff the tagged frame verifies under `session` (nonzero) with seq32.
  bool verifies_under_(const uint8_t* body, size_t len, uint64_t session, uint32_t seq32) const;
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

  // IDR policy state. Every IDR producer
  // — the GS-driven RCF-after-failsafe path, the encoder's chain-break
  // signal, and RCF idr_epoch requests — funnels through idr_due();
  // nothing else may call act_.request_idr(). chain_break_pending_ is the
  // venc thread's handoff (see note_chain_break); the two timestamps are
  // agent-thread-only.
  std::atomic<bool> chain_break_pending_{false};
  // A chain break the pacer refused (floor or chain holdoff) waits here
  // for the next due tick instead of being dropped: without a periodic
  // refresh there is no GOP to self-clear an unhealed break. Agent-thread
  // only; cleared at every session edge. A failsafe or rendezvous edge
  // ends in the link-up IDR, which heals whatever broke while unlinked. A
  // session promotion while LINKED (a fast GS restart) has no link-up IDR:
  // there only the new GS session's own request covers it -- its first RCF
  // reads as one whenever its epoch is non-zero (the drone's seen epoch
  // restarts at 0), and otherwise it asks once one of its own arming
  // sources fires (a damaged or lost base AU, a player report). A break
  // dropped before the frame ring leaves no frame-id gap for the GS to see.
  bool chain_idr_pending_ = false;
  uint64_t idr_chain_total_ = 0;
  // have_* companions rather than a 0 sentinel (the file's own idiom, cf.
  // have_last_fb_/have_last_bitrate_eval_): now_ms is a caller-supplied
  // clock that legitimately starts at 0, and a first IDR at t=0 must still
  // arm the 100 ms floor.
  uint64_t last_idr_ms_ = 0;
  bool have_last_idr_ = false;
  uint64_t last_chain_idr_ms_ = 0;
  bool have_last_chain_idr_ = false;
  bool idr_due(uint64_t now_ms, bool chain);

  // Low-power (pre-arm) state. Everything below
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
  int16_t snr_off_sent_ = 0;  // last offset handed to the actuator
  void intake_arm_state_(uint64_t now_ms);

  // Live config knobs (set_live_knobs): any thread may write, and the agent
  // thread reads these, never the boot Config, for encoder.bitrate_max_kbps,
  // encoder.airtime_budget, low_power.enable and venc.fps. int, not atomic<double>, for airtime_budget: std::atomic<double>
  // is not guaranteed lock-free on armv7 (is_always_lock_free), and the
  // policy's own 100 kbps rounding grid already swallows milli-scale error.
  std::atomic<int> knob_bitrate_max_kbps_{0};
  std::atomic<int> knob_airtime_budget_milli_{0};  // airtime_budget * 1000
  std::atomic<bool> knob_low_power_enable_{false};
  std::atomic<int> knob_venc_fps_{0};

  // note_pipeline_rebuilt's handoff: consumed at the top of tick(), the
  // same bare-flag shape as chain_break_pending_/arm_report_ above.
  std::atomic<bool> rebuilt_pending_{false};
  // note_ldpc_changed's handoff: -1 = none pending, else the new flag as
  // 0/1 (an atomic<bool> can't also carry "nothing pending").
  std::atomic<int> ldpc_pending_{-1};
  // Agent-thread copy of the live LDPC flag every ladder build reads
  // instead of cfg_.radio.ldpc; seeded from it in the constructor and
  // updated only by tick()'s consumption of ldpc_pending_.
  bool ldpc_live_ = true;

  // Builds the 2-slot ladder for (mode, mcs, bw) under the live LDPC flag
  // -- the one construction both the RCF path (on_rc_frame/apply_ladder_op)
  // and note_ldpc_changed's rebuild share, so neither can forget it.
  std::array<rc::LayerTxSpec, 2> ladder_live_(rc::PhyMode mode, uint8_t mcs, uint8_t bw) const;
  // note_ldpc_changed's agent-thread half: rebuilds the current op's ladder
  // (and probe spec, if one is active) for its own rung under the new flag
  // and republishes. Not a new operating point -- the rung is unchanged --
  // so generation is not bumped, the same rule reapply_with_shed follows
  // for an unchanged rung.
  void rebuild_current_op_ldpc_();

  // GS-requested IDR. The last RCF
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
