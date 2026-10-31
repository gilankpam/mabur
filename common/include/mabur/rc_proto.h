#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#include "mabur/cal_wire.h"
#include "mabur/link_key.h"
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
// Bumped 1 -> 2: the RCF power byte and the T_TELEM
// applied_off_qdb/derate_qdb fields were removed when runtime TX-power
// control was deleted.
// Bumped 2 -> 3: RCF lost ack_seq, score and the n_layers + layer_delivery
// tail. maburd read none of the three (rc_agent.cpp uses
// seq/profile/fec_overhead/probe and nothing else), so they were write-only
// ballast; the RCF head is fixed-length now.
// Bumped 3 -> 4: fec_overhead is now the literal air overhead in x100
// encoding (was a x16 'cmd' scalar the drone scaled 2x); Telem applied_ov
// split per stream; probe flag renamed — the fixed base=mcs−1 rule means
// both ends of v4 agree on the split with no extra bytes.
// Bumped 4 -> 5: fec_overhead split into per-stream
// fec_overhead_base/fec_overhead_enh (fixed per-rung pairs replace the
// balancer; base rides the scored mcs — same-rate).
// Bumped 5 -> 6: probe_profile is a FIXED head byte (0xFF = no probe
// stream); the RCF_F_PROBE_ENH flag + optional tail and CAP_ENH_PROBE are
// gone.
// Old and new peers reject each other in BOTH directions -- a half-deployed
// pair has no control link and, because DISC_ACK carries CAP_FRAME_WIRE, no
// video either. Recovery is to finish the deploy.
// Bumped 6 -> 7: T_CAL_CMD / T_CAL_RESULT carry the TX-power wall
// calibration session; Telem gained cal_base_ref_idx and flags bit6
// (cal_active).
// Bumped 7 -> 8: calibration indices are SIGNED and RELATIVE to the chip's
// efuse anchor (CalWindow int8, CalResult walls in [-64,63], sentinel
// -128); Telem drops cal_base_ref_idx.
// Bumped 8 -> 9: RCF gains hop_ch/hop_epoch (in-flight channel hop order,
// present in every RCF), Telem gains channel/hop_epoch (readback).
// Bumped 10 -> 11: RCF gains `rec` (VTX recorder wish), Telem gains
// `rec_status`.
// Bumped 11 -> 12: RCF gains `idr_epoch` (GS-requested IDR), Telem gains
// `idr_gs`.
// Bumped 12 -> 13: Telem drops 25 fields no GS consumer needs (generation,
// encoder/vanish/venc-ring counters, txq depth/cap, radio sent/drops, air
// clock, thermal_delta, channel/hop_epoch, the applied profile/overhead
// echo, idr_gs) and flag bits 1/2/5 -- 98 -> 48 bytes.
// Bumped 13 -> 14: vtx_id deleted from every frame (the link key is the
// identity); the same bump carries the per-frame auth tag, DISC_ACK's
// vtx_nonce + flags and Telem flags bit1. Flag day.
// Bumped 14 -> 15: T_NACK's final layout (GS->drone selective-repeat
// request, counter + sid + repeat flag + up to kMaxNackEntries runs)
// replaces the spike's wire; Telem gains nack_rx/retx_syms/retx_refused.
// Bumped 15 -> 16: RCF gains snr_off (phase lock: the GS's sensor-rate
// steering offset, level-triggered in every RCF).
// Bumped 16 -> 17: T_MAV (MAVLink tunnel, both directions) and CAP_MAVLINK.
// Bumped 17 -> 18: T_CAL_CMD / T_CAL_RESULT deleted (calibration rides the
// MAVLink tunnel's cal lane instead); Telem flags bit6 (cal_active), which
// acked the dedicated command frame, is no longer set.
constexpr uint8_t RC_VERSION = 18;

// RCF probe_profile sentinel: the drone runs no probe stream.
constexpr uint8_t kNoProbeProfile = 0xFF;

// Rcf::rec bits (VTX onboard recorder).
constexpr uint8_t kRecOn = 0x01;     // the operator wants the VTX recording
constexpr uint8_t kRecKnown = 0x02;  // the GS knows the wish; 0 = unknown, drone keeps its state

// Phase lock: the steering range either end accepts, in milli-fps around
// the drone's boot trim (about 28 sensor lines at 60 fps: a 59.94 Hz panel
// needs ~3, a browser panel up to 0.5 Hz off nominal needs up to ~300, the
// rest is headroom for auto-exposure stretching the frame).
constexpr int16_t kSnrOffRangeMfps = 500;

constexpr uint8_t T_RCF = 1;
constexpr uint8_t T_DISC = 2;
constexpr uint8_t T_DISC_ACK = 3;
constexpr uint8_t T_TELEM = 4;
// 5 and 6 were T_CAL_CMD / T_CAL_RESULT, deleted when calibration moved onto
// the MAVLink tunnel's cal lane; unassigned, not reused.
// T_NACK (fec-nack): GS -> drone selective-repeat
// request. Lists source symbols the sliding-window decoder could not
// recover; the drone re-sends them from its retransmit ring at the head of
// the TxQueue.
constexpr uint8_t T_NACK = 7;
// T_MAV (MAVLink tunnel): carries an opaque MAVLink packet. GS -> drone is
// tagged, with the request counter doubling as the tag ctx's seq32 (same
// accept-only-greater rule as T_NACK's counter); drone -> GS is untagged,
// like T_TELEM, and its per-process counter lets a two-card GS drop the
// second card's copy of a frame. rc_proto treats the payload as opaque bytes -- MAVLink
// parsing lives above this layer.
constexpr uint8_t T_MAV = 8;

constexpr uint8_t F_DISCOVERY = 0x04;

// SipHash-24 auth tag (link pairing): the 8 bytes
// immediately before the CRC on every DISC/RCF/NACK frame and every
// GS -> drone MAV frame.
constexpr size_t kTagLen = 8;

constexpr uint8_t kAckKeyMismatch = 0x01;   // DiscAck::flags bit0
constexpr uint8_t kTelemAuthReject = 0x02;  // Telem::flags bit1

// Values hashed into a control frame's tag but never sent on the wire
// (link pairing): the VRX/VTX nonces from the completed
// rendezvous plus a 32-bit sequence (RCF's seq, widened; 0 for DISC
// frames, which carry no seq of their own).
struct TagCtx {
  uint32_t vrx_nonce = 0;
  uint32_t vtx_nonce = 0;
  uint32_t seq32 = 0;
};

// DiscAck.chip_caps bit: VTX's video bodies use the frame wire format
// (8-byte FrameHdr units + 6-byte wide FRAG headers) instead of pre-built
// RTP packets + 4-byte FRAG headers.
constexpr uint16_t CAP_FRAME_WIRE = 0x0001;

// DiscAck.chip_caps bit: drone sends T_TELEM frames on its RC uplink.
// Display-grade only (not a safety gate): a GS lacking this bit just never
// sees a T_TELEM frame from an old drone.
constexpr uint16_t CAP_TELEMETRY = 0x0002;

// DiscAck.chip_caps bit: the peer runs the mabur-dialect calibration (the
// MAVLink tunnel's cal lane). The GS refuses to start a session without it.
constexpr uint16_t CAP_CALIBRATE = 0x0004;

// DiscAck.chip_caps bit: VTX understands T_MAV and tunnels MAVLink both ways.
// Display-only, as CAP_TELEMETRY: the GS logs a peer without it once per
// session and gates nothing on it.
constexpr uint16_t CAP_MAVLINK = 0x0008;

// VRX -> VTX feedback: the GS-authoritative operating point. Every field
// here is one maburd acts on. It used to also carry ack_seq, an alink-style
// score and per-layer delivery percentages; RC_VERSION 3 dropped all three
// because no consumer ever read them off the wire (the GS reports layer
// delivery to operators over its own stats sideport instead).
struct Rcf {
  uint16_t seq = 0;
  uint8_t profile = 0;
  double fec_overhead_base = 0.5;
  double fec_overhead_enh = 0.5;

  // Probe stream MCS: encode_profile of the rung the GS
  // wants probed, or kNoProbeProfile. Always present in the head.
  uint8_t probe_profile = kNoProbeProfile;

  // In-flight hop: the channel the drone must be on,
  // in EVERY RCF (the standing truth, not an event), and the epoch the GS
  // bumps on each order/withdrawal so repeats are idempotent. 0 = no order
  // ever issued (a pre-hop GS); the drone ignores hop_ch 0.
  uint8_t hop_ch = 0;
  uint8_t hop_epoch = 0;

  // VTX recorder wish: kRecKnown | (kRecOn if recording).
  // 0 = unknown (maburgs just started, no player message yet): the drone
  // leaves the recorder alone. Level-triggered, in EVERY RCF; no decay.
  uint8_t rec = 0;

  // GS-requested IDR: the requester bumps this on every
  // request; the drone serves one paced IDR per CHANGE. In EVERY RCF, so a
  // lost RCF loses nothing. 0 from a GS that never requests (maburgs).
  uint8_t idr_epoch = 0;

  // Phase lock: the sensor-rate offset the GS wants on top of the drone's
  // boot trim, in milli-fps (signed; 0 = no steering, the drone runs on its
  // trim). Level-triggered in EVERY RCF; the drone clamps it to
  // kSnrOffRangeMfps and keeps the last value through link loss.
  int16_t snr_off = 0;
};

// VRX -> VTX discovery beacon (rendezvous), addressed to a VTX_ID.
struct Disc {
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
  uint32_t vrx_nonce = 0;
  uint32_t vtx_nonce = 0;
  uint16_t chip_caps = 0;
  uint8_t agreed_channel = 0;
  uint8_t agreed_width = 20;
  uint8_t flags = 0;  // bit0 kAckKeyMismatch: VTX's DISC tag did not verify
                      // against the VTX's own key --
                      // the rendezvous still completes (no tag to check the
                      // ack itself against yet, pre-rendezvous), but the
                      // VRX now knows the pair is running mismatched keys.
  uint16_t seq = 0;
};

// VTX -> VRX drone telemetry: RcAgent/queue/radio state for the GS
// DRONE display region. Sent unconditionally, unconditioned on peer caps;
// an old GS ignores the unknown type.
// Trimmed (RC_VERSION 13) to the fields a GS consumer reads --
// link control, OSD, web UI, flightreport/flightjitter; the maburtop-only
// encoder/queue/air/channel counters, the applied-op echo (a spotter takes
// its width from config) and idr_gs are gone (list in
// docs/data-provenance.md "Removed sideport keys").
struct Telem {
  uint16_t tlm_seq = 0;
  uint8_t state = 0;            // RcAgent::State numeric
  uint8_t flags = 0;  // bit0 failsafe_shed,
                      // bit3 rcf_seq_echo valid (link-rtt),
                      // bit4 congestion_shed (RcAgent::run_congestion_guard
                      //      shed_level >= 1: TxQueue pressure / USB failure;
                      //      distinct from bit0 so a bench can count sheds
                      //      and flightreport can attribute an enh gap to
                      //      congestion rather than RF),
                      // bit6 unused: no frame sets it (it was cal_active,
                      //      the ack of a dedicated calibration command frame;
                      //      calibration acks now ride the MAVLink tunnel),
                      // bit7 low_power (RcAgent::low_power(): pre-arm 1 Mb/s / 15 fps operating point)
                      // bit1 auth_reject: >= 1 GS->drone control frame failed tag/seq
                      //      verification this telemetry period. One period around a
                      //      drone restart is the expected transient; sustained = bug or
                      //      two controllers.
                      // bits 2, 5 unused.
  uint16_t rcf_age_ms = 0;  // saturating
  // link-rtt: seq of the RCF rcf_age_ms is aging against, so
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
  uint16_t cmd_kbps = 0;
  uint32_t txq_drops = 0;
  uint16_t txq_wait_max_ms = 0;  // per-telemetry-window max TxQueue wait (saturating)
  uint16_t usb_fail = 0;  // saturating
  // Uplink (GS -> drone) signal per drone antenna -- the one view of a dead
  // drone chain/antenna the GS's own downlink readings cannot give.
  uint8_t up_rssi[2] = {0, 0};  // raw, dBm = v - 110
  int8_t up_snr[2] = {0, 0};
  int8_t soc_temp_c = -128;  // -128 = unavailable
  // CPU busy percent x100 over the last telemetry tick, from a /proc/stat
  // delta (user+nice+system+irq+softirq+steal over everything). 65535 =
  // unavailable (first tick, unreadable). Not loadavg: on this SoC loadavg
  // counts the SigmaStar SDK's parked D-state workers and reads a flat ~13
  // idle or pegged (docs/findings/2026-08-31-dq-spike.md).
  uint16_t cpu_busy_x100 = 65535;
  // Drone RX-side channel view, per telemetry period (cca-on):
  // every frame the monitor-mode receiver handed the RX callback, split
  // into RC frames from the GS (own), CRC-clean frames that were not ours
  // (foreign -- other 802.11 on our channel) and CRC-failed frames
  // (crcfail -- a preamble was heard, the payload did not decode). With
  // carrier sense back ON, foreign + crcfail is what the drone's
  // transmitter deferred to: the altitude view the GS's ground cards
  // cannot measure. Counted in software on the RX path -- deliberately NOT
  // the chip's CCA/FA registers, whose read is a control-plane transfer
  // that must take the TX gate exclusive and stalled the USB TX pool once
  // a second (795 TxQueue drops in 20 min on the bench). Saturating.
  uint16_t rx_own = 0, rx_foreign = 0, rx_crcfail = 0;

  // VTX recorder: bits 0-1 RecState (0 off, 1 recording,
  // 2 error), bits 2-7 RecErr (drone/src/vtx_recorder.h).
  uint8_t rec_status = 0;

  // Software NACK, per Telem period, saturating:
  // requests verified, source symbols re-sent, symbols refused by the air bucket.
  uint16_t nack_rx = 0, retx_syms = 0, retx_refused = 0;
};

// Calibration indices: relative to the chip's own per-channel TXAGC anchor,
// so they live in the 7-bit signed diff-field range. -1 is a legal wall;
// "undetermined" is this sentinel, outside the range.
constexpr int kRelMin = -64;
constexpr int kRelMax = 63;
constexpr int kRailRel = 63;  // a no-dip row's wall: rail to the top of the range
constexpr int16_t kWallUndetermined = -128;

// Tagged frames (DISC/RCF) carry an 8-byte SipHash tag
// (kTagLen) right before the CRC. pack_* always writes a tag -- keyed by
// `key`, defaulting to kDefaultLinkKey, hashed over the frame's bytes plus
// `ctx` (never sent; see TagCtx). parse_* stays structural and never checks
// it; verify_control is the one place a tag is checked.
std::vector<uint8_t> pack_rcf(const Rcf& r, const LinkKey& key = kDefaultLinkKey,
                              const TagCtx& ctx = {});
std::optional<Rcf> parse_rcf(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_disc(const Disc& d, const LinkKey& key = kDefaultLinkKey);  // ctx all-zero
std::optional<Disc> parse_disc(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_disc_ack(const DiscAck& a);
std::optional<DiscAck> parse_disc_ack(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_telem(const Telem& t);
std::optional<Telem> parse_telem(const uint8_t* buf, size_t len);

// T_NACK: GS -> drone selective-repeat
// request for base-layer source symbols. counter is the GS's per-session
// request counter and the tag ctx seq32; the drone accepts only a counter
// greater than the last one it verified this session.
constexpr uint8_t kNackFlagRepeat = 0x01;  // >= 1 seq inside is on its 2nd try: drone doubles the send
constexpr int kMaxNackEntries = 4;
struct NackEntry {
  uint32_t first_seq = 0;
  uint32_t bitmap = 1;  // bit i = wire seq first_seq + i requested; bit 0 always set on the wire
};
struct Nack {
  uint32_t counter = 0;
  uint8_t sid = 0;      // 0 only today; carried for an enh follow-up
  uint8_t flags = 0;
  uint8_t n = 0;        // 1..kMaxNackEntries
  NackEntry e[kMaxNackEntries];
};
std::vector<uint8_t> pack_nack(const Nack& n, const LinkKey& key = kDefaultLinkKey,
                               const TagCtx& ctx = TagCtx{});
std::optional<Nack> parse_nack(const uint8_t* buf, size_t len);

// T_MAV: an opaque MAVLink packet tunneled over the RC link. The payload is
// never inspected here -- rc_proto just frames it. magic(2) ver type
// counter(4) len(2) = kMavFixedLen (10); payload is len bytes, then
// (tagged ? the tag) then the CRC.
constexpr size_t kMavFixedLen = 2 + 1 + 1 + 4 + 2;  // magic ver type counter len = 10
constexpr size_t kMavMaxPayload = 600;
struct Mav {
  uint32_t counter = 0;
  std::vector<uint8_t> payload;
};
// tagged = true: GS->drone layout (tag before CRC); false: drone->GS, no
// tag. The frame itself carries no direction bit -- the caller knows which
// way it is sending/expecting and must pass the same `tagged` on both ends.
// Caller keeps payload.size() <= kMavMaxPayload (asserted, not refused).
std::vector<uint8_t> pack_mav(const Mav& m, bool tagged, const LinkKey& key = kDefaultLinkKey,
                              const TagCtx& ctx = {});
// Structural parse. `tagged` must match the sender's layout (the frame does
// not say); the wrong value fails the CRC check.
std::optional<Mav> parse_mav(const uint8_t* buf, size_t len, bool tagged);

// Recomputes the tag of any tagged frame (DISC/RCF/NACK/MAV): the 8 bytes
// at the frame's STRUCTURAL tag offset -- derived from its type (DISC_LEN,
// RCF_HEAD_LEN, kNackFixedLen + n*kNackEntryLen, or kMavFixedLen + len16) --
// must equal SipHash(key, bytes-before-tag || ctx). Bytes past tag+CRC are
// ignored: on hardware the drone's body still carries the 4-byte 802.11 FCS.
// Constant-time compare. False for any other type, a NACK whose n is 0 or
// > kMaxNackEntries, a MAV whose len16 > kMavMaxPayload, or a buffer shorter
// than structural + tag + CRC.
bool verify_control(const uint8_t* buf, size_t len, const LinkKey& key, const TagCtx& ctx);

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
