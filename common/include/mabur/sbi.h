#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
namespace mabur {

// Hard cap on one SBI body (one injected air frame's payload). No 802.11
// constant enforces this in code — the chip accepts well past the 2304B
// MSDU nominal via injection — 2900 covers every geometry proven on air
// (2848B linkbench bodies 2026-07-13; 2652-2680B big-symbol probes
// 2026-07-15) while rejecting configs that would silently change the
// airtime/PER envelope.
//
// NOTE: the config-load guard that enforces this (drone/src/config.cpp)
// measures only bpb*(kSwHeaderLen + symbol_size) — the per-block SW-header
// envelope — NOT the real air body. It excludes the 11B SBI header and the
// 2B per-block CRC that SbiPacker actually adds on top, so the true body
// size is 11 + bpb*(16 + symbol_size) — 11 + 2*bpb bytes more than what the
// guard computes (43B at bpb 16). The 2900 constant carries slack for exactly
// this gap: proven bodies on air measured 2887B actual against a
// 2848B-by-the-guard-formula config, comfortably under 2900 either way (deployed
// geometry: 11 + 4*(16+332) = 1403 B).
inline constexpr int kMaxBodyBytes = 2900;

// Sub-Block Integrity (SBI) framing constants. Byte-exact port of devourer's
// tools/precoder/fec_subblock.py (SBI_MAGIC, SBI_HDR_LEN, SBI_HDR_STRUCT
// "<HBBHBHH" = MAGIC, VER, STREAM_ID, BLOCK_PAYLOAD, N_BLOCKS, Q_MS, ENC_US).
constexpr uint16_t SBI_MAGIC = 0xF5B0;
constexpr int SBI_HDR_LEN = 11;
constexpr uint8_t SBI_VER = 1;  // ver 0 (7-byte header) is hard-rejected:
// parsing a ver-0 body with the 11-byte stride would silently mis-partition
// every sub-block, so there is no compat path (flag-day deploy).
constexpr int SBI_Q_MS_OFF = 7;   // u16 LE: TxQueue wait, ms, saturating
constexpr int SBI_ENC_US_OFF = 9; // u16 LE: encoder latency, µs, saturating
// Post-hoc patchers for the two duration fields. The header sits OUTSIDE
// the FEC envelopes and per-block CRCs — that is the only reason a
// submit-time measurement can exist on this wire; everything inside the
// envelope is frozen at encode time.
void sbi_set_q_ms(uint8_t* body, size_t len, uint16_t ms);
void sbi_set_enc_us(uint8_t* body, size_t len, uint16_t us);

// Reserved SBI stream_id for the MSP DisplayPort OSD side-channel (video uses
// 0..3). Bodies tagged with this id route to the GS MspSink, not the video
// decoder.
constexpr uint8_t kMspStreamId = 4;

// Reserved SBI stream_id for the probe stream (spec 2026-09-04): a
// video-body-sized canary at the candidate rung's MCS, one per enh AU.
// Routed to the GS ProbeTrack, never the video decoder.
constexpr uint8_t kProbeStreamId = 5;

// Packs fixed-size FEC envelopes into SBI radio bodies, each sub-block
// guarded by its own CRC16-CCITT so a corrupted body still yields its
// surviving sub-blocks as usable symbols. Byte-exact port of
// devourer/tools/precoder/fec_subblock.py's SubBlockPacker (crc_bytes fixed
// at 2, matching the default used throughout the precoder toolchain).
//
// Wire body: header <u16 MAGIC LE, u8 ver, u8 stream_id, u16 block_payload
// LE, u8 n_blocks, u16 q_ms LE, u16 enc_us LE> followed by, per accumulated envelope,
// <u16 crc16_ccitt(payload) LE, payload>.
class SbiPacker {
 public:
  SbiPacker(int block_payload, int blocks_per_body, uint8_t stream_id);

  // Feeds one fixed-size FEC envelope in. Returns a freshly completed body
  // once blocks_per_body envelopes have accumulated (zero or more bodies;
  // in practice at most one per call). A deviation from the Python
  // reference: an envelope whose length != block_payload just returns
  // empty, instead of raising (hot path can't throw).
  std::vector<std::vector<uint8_t>> add(const uint8_t* env, size_t len);

  // Emits a short final body with whatever envelopes are pending. Returns
  // empty if nothing is pending.
  std::vector<std::vector<uint8_t>> flush();

  // Per-block wire size: crc16 (2 bytes) + block_payload.
  int block_stride() const;

 private:
  std::vector<uint8_t> build_body(const std::vector<std::vector<uint8_t>>& batch);

  int block_payload_;
  int blocks_per_body_;
  uint8_t stream_id_;
  std::vector<std::vector<uint8_t>> pending_;
};

// Receiver-side split of a radio body into CRC-surviving sub-blocks. Port of
// fec_subblock.py's unpack(): block_payload comes from the receiver's CONFIG
// and is authoritative — the body's header is sanity-checked (header_ok) but
// never trusted to drive partitioning, so a corrupted header cannot desync
// the scan. Works identically on clean and kept-corrupt bodies.
struct SbiUnpackResult {
  std::vector<std::vector<uint8_t>> survivors;  // CRC-valid sub-block payloads
  int n_blocks = 0;                             // sub-blocks scanned
  int n_failed = 0;                             // CRC-mismatched (erasures)
  bool header_ok = false;
  uint8_t stream_id = 0;                        // 0 when the header is short
  uint16_t q_ms = 0;                            // TxQueue wait, ms; 0 = unknown
  uint16_t enc_us = 0;                          // encoder latency, µs; 0 = unknown
};
SbiUnpackResult sbi_unpack(const uint8_t* body, size_t len, int block_payload);

// SBI STREAM_ID peek for routing (fixed 11-byte header, independent of
// block_payload), or -1 on a short/bad-magic/bad-version header. A corrupt
// header may misroute a body, but the wrong stream's decoder then rejects
// the mismatched sub-blocks — a dropped body, never a mis-decode.
int sbi_peek_stream_id(const uint8_t* body, size_t len);

}  // namespace mabur
