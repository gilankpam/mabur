#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
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
// Old and new peers reject each other in BOTH directions -- a half-deployed
// pair has no control link and, because DISC_ACK carries CAP_FRAME_WIRE, no
// video either. Recovery is to finish the deploy.
constexpr uint8_t RC_VERSION = 5;

constexpr uint8_t T_RCF = 1;
constexpr uint8_t T_DISC = 2;
constexpr uint8_t T_DISC_ACK = 3;
constexpr uint8_t T_TELEM = 4;

constexpr uint8_t F_DISCOVERY = 0x04;

// RCF flags bit: one probe_profile byte follows the head — layer 3 (s3)
// transmits at that MCS while everything else stays on Rcf.profile. This is
// the only bit an RCF ever sets; the flags byte carries nothing else.
constexpr uint8_t RCF_F_PROBE_ENH = 0x08;

// DiscAck.chip_caps bit: VTX's video bodies use the frame wire format
// (8-byte FrameHdr units + 6-byte wide FRAG headers) instead of pre-built
// RTP packets + 4-byte FRAG headers. Spec 2026-07-22 frame-shm ingest.
constexpr uint16_t CAP_FRAME_WIRE = 0x0001;

// DiscAck.chip_caps bit: drone sends T_TELEM frames on its RC uplink.
// Display-grade only (not a safety gate): a GS lacking this bit just never
// sees a T_TELEM frame from an old drone. Spec 2026-07-26 drone-telemetry.
constexpr uint16_t CAP_TELEMETRY = 0x0002;

// DiscAck.chip_caps bit: drone accepts RCF_F_PROBE_ENH (s3-only MCS probe).
// Spec 2026-08-05 s3-probe-promote.
constexpr uint16_t CAP_ENH_PROBE = 0x0004;

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

  // s3 probe (spec 2026-08-05): when true, pack appends probe_profile after
  // the head (inside the CRC) and sets RCF_F_PROBE_ENH in the flags byte.
  bool probe3 = false;
  uint8_t probe_profile = 0;
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
  uint8_t flags = 0;            // bit0 failsafe_shed, bit1 radio_rx_ok, bit2 probing
  uint32_t generation = 0;
  uint8_t applied_profile = 0;  // encode_profile(mode, mcs, bw)
  double applied_ov_base = 0.0;
  double applied_ov_enh = 0.0;
  uint16_t rcf_age_ms = 0;  // saturating
  uint32_t rcf_rx = 0;
  uint32_t enc_frames = 0;
  uint32_t enc_kbytes = 0;
  uint16_t cmd_kbps = 0;
  uint8_t qp = 0;
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
};

std::vector<uint8_t> pack_rcf(const Rcf& r);
std::optional<Rcf> parse_rcf(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_disc(const Disc& d);
std::optional<Disc> parse_disc(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_disc_ack(const DiscAck& a);
std::optional<DiscAck> parse_disc_ack(const uint8_t* buf, size_t len);

std::vector<uint8_t> pack_telem(const Telem& t);
std::optional<Telem> parse_telem(const uint8_t* buf, size_t len);

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
