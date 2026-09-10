#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#include "mabur/cal_wire.h"
namespace mabur::rc {

// RC control-plane framing (adaptive-link feedback + rendezvous): RCF
// (VRX->VTX feedback), DISC (VRX->VTX discovery beacon), DISC_ACK (VTX->VRX
// rendezvous reply), T_TELEM (VTX->VRX drone telemetry). Originally a
// byte-exact port of devourer's tools/precoder/rc_proto.py; that Python is
// frozen at RC_VERSION 1 and is NO LONGER a wire oracle -- mabur owns these
// bytes as of RC_VERSION 2, pinned by the goldens in tests/test_rc.cpp.
// All multi-byte fields are little-endian; every frame ends with a u16
// CRC16-CCITT (mabur::crc16_ccitt) over every byte before it.

constexpr uint16_t RC_MAGIC = 0x5243;  // "RC"
// Bumped 1 -> 2 on 2026-08-12: the RCF power byte and the T_TELEM
// applied_off_qdb/derate_qdb fields were removed when runtime TX-power
// control was deleted. Spec 2026-08-12-constant-txpower-design.md.
// Bumped 2 -> 3 on 2026-08-15: RCF lost ack_seq, score and the
// n_layers + layer_delivery tail. maburd read none of the three (rc_agent.cpp
// uses vtx_id/seq/profile/fec_overhead/probe and nothing else), so they were
// write-only ballast; the RCF head is fixed-length now.
// Bumped 3 -> 4 on 2026-08-29: fec_overhead is now the literal air overhead
// in x100 encoding (was a x16 'cmd' scalar the drone scaled 2x); Telem
// applied_ov split per stream; probe flag renamed — the fixed base=mcs−1
// rule means both ends of v4 agree on the split with no extra bytes. Spec
// 2026-08-29-airtime-balance-uep.
// Bumped 4 -> 5 on 2026-08-30: fec_overhead split into per-stream
// fec_overhead_base/fec_overhead_enh (fixed per-rung pairs replace the
// balancer; base rides the scored mcs — same-rate). Spec
// 2026-08-30-same-rate-fixed-pairs-design.md.
// Bumped 5 -> 6 on 2026-09-04: probe_profile is a FIXED head byte (0xFF =
// no probe stream); the RCF_F_PROBE_ENH flag + optional tail and
// CAP_ENH_PROBE are gone. Spec 2026-09-04-probe-stream-design.md.
// Old and new peers reject each other in BOTH directions -- a half-deployed
// pair has no control link and, because DISC_ACK carries CAP_FRAME_WIRE, no
// video either. Recovery is to finish the deploy.
// Bumped 6 -> 7 on 2026-09-10: T_CAL_CMD / T_CAL_RESULT carry the TX-power
// wall calibration session; Telem gained cal_base_ref_idx and flags bit6
// (cal_active). Spec 2026-09-10-tx-power-calibration-design.md.
constexpr uint8_t RC_VERSION = 7;

// RCF probe_profile sentinel: the drone runs no probe stream.
constexpr uint8_t kNoProbeProfile = 0xFF;

constexpr uint8_t T_RCF = 1;
constexpr uint8_t T_DISC = 2;
constexpr uint8_t T_DISC_ACK = 3;
constexpr uint8_t T_TELEM = 4;
constexpr uint8_t T_CAL_CMD = 5;
constexpr uint8_t T_CAL_RESULT = 6;

constexpr uint8_t F_DISCOVERY = 0x04;

// DiscAck.chip_caps bit: VTX's video bodies use the frame wire format
// (8-byte FrameHdr units + 6-byte wide FRAG headers) instead of pre-built
// RTP packets + 4-byte FRAG headers. Spec 2026-07-22 frame-shm ingest.
constexpr uint16_t CAP_FRAME_WIRE = 0x0001;

// DiscAck.chip_caps bit: drone sends T_TELEM frames on its RC uplink.
// Display-grade only (not a safety gate): a GS lacking this bit just never
// sees a T_TELEM frame from an old drone. Spec 2026-07-26 drone-telemetry.
constexpr uint16_t CAP_TELEMETRY = 0x0002;

// DiscAck.chip_caps bit: VTX understands T_CAL_CMD / T_CAL_RESULT and can run
// a TX-power wall calibration. The GS refuses to start a session without it.
constexpr uint16_t CAP_CALIBRATE = 0x0004;

// VRX -> VTX feedback: the GS-authoritative operating point. Every field
// here is one maburd acts on. It used to also carry ack_seq, an alink-style
// score and per-layer delivery percentages; RC_VERSION 3 dropped all three
// because no consumer ever read them off the wire (the GS reports layer
// delivery to operators over its own stats sideport instead).
struct Rcf {
  uint32_t vtx_id = 0;
  uint16_t seq = 0;
  uint8_t profile = 0;
  double fec_overhead_base = 0.5;
  double fec_overhead_enh = 0.5;

  // Probe stream MCS (spec 2026-09-04): encode_profile of the rung the GS
  // wants probed, or kNoProbeProfile. Always present in the head.
  uint8_t probe_profile = kNoProbeProfile;
};

// VRX -> VTX discovery beacon (rendezvous), addressed to a VTX_ID.
struct Disc {
  uint32_t vtx_id = 0;
  uint32_t vrx_nonce = 0;
  uint8_t op_channel = 0;
  uint8_t op_width = 20;
  uint8_t table_ver = 1;
  uint8_t init_profile = 0;
  uint16_t cap_bits = 0;
  uint16_t seq = 0;
};

// VTX -> VRX reply completing rendezvous + agreeing the op channel.
struct DiscAck {
  uint32_t vtx_id = 0;
  uint32_t vrx_nonce = 0;
  uint16_t chip_caps = 0;
  uint8_t agreed_channel = 0;
  uint8_t agreed_width = 20;
  uint16_t seq = 0;
};

// VTX -> VRX drone telemetry: RcAgent/pipeline/queue/radio state for the GS
// DRONE display region. Sent unconditionally, unconditioned on peer caps;
// an old GS ignores the unknown type. Spec 2026-07-26 drone-telemetry.
struct Telem {
  uint16_t tlm_seq = 0;
  uint8_t state = 0;            // RcAgent::State numeric
  uint8_t flags = 0;  // bit0 failsafe_shed, bit1 radio_rx_ok,
                      // bit2 probe stream on (RcAgent::probe_on()),
                      // bit3 rcf_seq_echo valid (link-rtt),
                      // bit4 congestion_shed (RcAgent::run_congestion_guard
                      //      shed_level >= 1: TxQueue pressure / USB failure;
                      //      distinct from bit0 so a bench can count sheds
                      //      and flightreport can attribute an enh gap to
                      //      congestion rather than RF — 2026-09-03),
                      // bit5 air_shed (AirClock enh admission dropped >= 1 enh AU this window — spec 2026-09-06),
                      // bit6 cal_active (drone accepted a calibration command; set on the
                      //      ack Telem for each accepted PHASE -- coarse, then fine -- sent
                      //      BEFORE that phase's first sweep frame, and re-sent on every exact
                      //      retransmission of the phase already running (the GS resends its
                      //      CalCmd every 200 ms into a 30-50%-lossy uplink; CalSession::on_ack
                      //      is a no-op once already out of AwaitAck, so answering a repeat is
                      //      harmless — Task 11 review). Telem is suppressed only WHILE a phase
                      //      is actively sweeping, so the ack(s) and the suppression do not
                      //      conflict. The verify pass has no command and therefore no ack: the
                      //      drone self-initiates it after applying the result — spec 2026-09-10)
  uint32_t generation = 0;
  uint8_t applied_profile = 0;  // encode_profile(mode, mcs, bw)
  double applied_ov_base = 0.0;
  double applied_ov_enh = 0.0;
  uint16_t rcf_age_ms = 0;  // saturating
  // link-rtt (2026-09-02): seq of the RCF rcf_age_ms is aging against, so
  // the GS can subtract the send time of the RIGHT frame (repeats are 10 ms
  // apart — closer than the RTT being measured). Meaningless while
  // rcf_age_ms holds its 65535 never-sentinel.
  uint16_t rcf_seq_echo = 0;
  // Drone pts-domain clock (MI timebase, µs) read at telem build — the t3
  // of the GS's NTP-style offset estimate. A duration-free timestamp is
  // safe here because the GS only ever differences it against its own
  // arrival stamp plus rtt/2; it never treats it as a shared clock.
  uint64_t pts_at_build = 0;
  uint32_t rcf_rx = 0;
  uint32_t enc_frames = 0;
  uint32_t enc_kbytes = 0;
  uint16_t cmd_kbps = 0;
  // RcAgent's ROI QP override as last commanded (actuator.last_roi_qp;
  // encoder.roi_qp_low/normal, e.g. -24 / 0). Signed delta QP. Until
  // 2026-09-03 an unsigned `qp` byte carried this same value under the
  // wrong name; for a few hours that day it carried the encoder's
  // startQual instead, which this firmware never fills, so the byte was
  // dropped (Telem 84 -> 83) — there is no encoder-QP readback on the
  // wire, by design (docs/data-provenance.md).
  int8_t roi_qp = 0;
  uint16_t ring_drops = 0;  // saturating
  uint8_t txq_depth = 0, txq_cap = 0;
  uint32_t txq_drops = 0;
  uint16_t txq_wait_max_ms = 0;  // per-telemetry-window max TxQueue wait (saturating)
  uint32_t radio_sent = 0;
  uint32_t radio_drops = 0;
  uint16_t usb_fail = 0;  // saturating
  uint8_t up_rssi[2] = {0, 0};  // raw, dBm = v - 110
  int8_t up_snr[2] = {0, 0};
  int8_t soc_temp_c = -128;  // -128 = unavailable
  int8_t thermal_delta = 0;
  uint16_t load_x100 = 0;
  uint16_t idr_disagree = 0;      // saturating; spec 2026-07-26 svct-enable
  uint16_t enhance_disagree = 0;  // saturating
  // venc-ring vanish detection (docs/venc-ring-vanish-findings-2026-08-12.md):
  // frames that vanished between waybeam's encoder and maburd's ring read
  // (pts-jump-detected, classified base/enhance from neighbour flags), and
  // base vanishes suppressed by the IDR-adjacency re-seed guard (counted for
  // loop visibility; the self-IDR consumer itself is NOT wired on this
  // build — detection-only port of 65c94fd). All saturating. Counters are
  // zeroed at the FIRST link-establish (encoder bring-up books ~8-9 boot
  // counts that would otherwise need analyzer-side baselining; a mid-flight
  // re-establish does NOT zero), so they read "vanishes since first link".
  uint16_t vanished_base = 0;
  uint16_t vanished_enh = 0;
  uint16_t self_idr_refused = 0;
  // venc encoder ring (spec 2026-08-28 venc-foldin, Task B6): the PRODUCER
  // side of the same shm ring `ring_drops` reports the consumer side of.
  // full_drops counts whole access units the encoder threw away because
  // maburd had not drained the ring — the loss that breaks the decode chain
  // and drives RcAgent's chain-break IDR — and fill_pct is the ring
  // occupancy at the telemetry tick. Together they are the only view a
  // ground operator has into an encoder that is running but outpacing its
  // reader; a *stalled* encoder shows instead as enc_frames not advancing.
  uint16_t venc_full_drops = 0;     // saturating
  uint8_t venc_ring_fill_pct = 0;   // 0..100
  // Drone air clock (spec 2026-09-06 §4.6): per-telemetry-window max of the
  // modelled air backlog, and enh AUs dropped by the admission gate since
  // link-up. Both saturating.
  uint16_t air_backlog_max_ms = 0;
  uint16_t air_shed_drops = 0;
  // Calibration (spec 2026-09-10): this chip's efuse TXAGC reference index,
  // read back at bring-up. power_plan.h derives every per-rate diff as
  // walls[r] - margin*4 - base_ref_idx, so the GS needs this unit's value to
  // range-check a candidate wall table before sending it. 0 = not read.
  // Paired with flags bit6 (cal_active): the drone emits this Telem for
  // each ACCEPTED PHASE (coarse, then fine -- CalSession::on_ack() re-enters
  // AwaitAck for the fine phase and needs a second ack to leave it, see
  // tests/test_cal_session.cpp's fine_phase_sharpens_a_real_dip_and_flags_drift),
  // before that phase starts sweeping -- and again on every exact
  // retransmission of the phase already running, since on_ack() is a
  // no-op once the GS has already left AwaitAck (Task 11 review: a single
  // lost ack must not cost the whole phase). Telem is suppressed only
  // while a phase is actively sweeping, not for the whole session, so a
  // phase boundary's ack Telem(s) and the suppression rule never conflict.
  // The verify pass sends no command and gets no ack -- the drone
  // self-initiates it once it applies the result.
  uint8_t cal_base_ref_idx = 0;
};

// One rate's index range for a calibration phase. idx_step 4 is the coarse
// scan; 1 is the full-resolution fine window.
struct CalWindow {
  uint8_t rate = 0;      // HT MCS 0..7
  uint8_t idx_lo = 0;
  uint8_t idx_hi = 0;
  uint8_t idx_step = 1;
};

constexpr size_t kMaxCalWindows = 8;  // one per HT MCS

// VRX -> VTX: run this sweep. Idempotent by (nonce, phase) so the GS can
// repeat it into the drone's listen window without the drone re-running a
// phase it already started -- the uplink loses 30-50% of frames.
struct CalCmd {
  uint32_t vtx_id = 0;
  uint32_t nonce = 0;
  uint8_t phase = 0;              // cal::kPhaseCoarse / Fine / Verify
  uint16_t frames_per_cell = 20;
  uint16_t settle_ms = 100;
  uint16_t gap_us = 2000;
  std::vector<CalWindow> windows;  // 1..kMaxCalWindows
};

// VRX -> VTX: the measured table. A wall of -1 is UNDETERMINED -- the GS
// could not derive one from the data, and the drone must leave that config
// entry exactly as it found it rather than write a fabricated number.
struct CalResult {
  uint32_t vtx_id = 0;
  uint32_t nonce = 0;
  std::array<int16_t, 8> walls{};   // per-MCS, -1 = undetermined
  int16_t legacy_wall = -1;
  uint32_t flags = 0;               // maburgs::CalFlag bitset, for the log
};

std::vector<uint8_t> pack_rcf(const Rcf& r);
std::optional<Rcf> parse_rcf(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_disc(const Disc& d);
std::optional<Disc> parse_disc(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_disc_ack(const DiscAck& a);
std::optional<DiscAck> parse_disc_ack(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_telem(const Telem& t);
std::optional<Telem> parse_telem(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_cal_cmd(const CalCmd& c);
std::optional<CalCmd> parse_cal_cmd(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_cal_result(const CalResult& r);
std::optional<CalResult> parse_cal_result(const uint8_t* buf, size_t len);

// Peeks the RC frame type without a full parse (no CRC check). Returns -1 if
// the buffer is too short or doesn't carry the RC magic/version.
int frame_type(const uint8_t* buf, size_t len);

// True iff these bytes carry the RC magic but NOT our RC_VERSION -- i.e. a
// peer at a different protocol version. Total and side-effect-free: false for
// a buffer shorter than 4 bytes, false for a non-RC body, false for our own
// version.
//
// Deliberately additive rather than a change to frame_type()'s contract:
// frame_type() returning -1 is the affirmative "this is video" signal on the
// GS ingest path (gs/src/main.cpp), so distinguishing the version case there
// would silently reroute video accounting. This predicate exists so the two
// ingest points can LOG a version mismatch while still dropping/handling the
// frame exactly as before -- a half-deployed pair otherwise fails completely
// silently, presenting as no-video that looks like the stale-caps deadlock.
//
// NOT a CRC check. RC_MAGIC is two bytes, so ~1 in 65536 corrupt bodies match
// it by chance: RX-path callers must gate on their own crc_ok.
bool is_foreign_rc_version(const uint8_t* buf, size_t len);

// Converts a fractional FEC overhead (e.g. 0.25) to the wire's literal x100
// byte encoding, rounded to nearest and clamped to [0.05, 2.0] (5..200).
uint8_t overhead_to_x100(double ov);

}  // namespace mabur::rc
