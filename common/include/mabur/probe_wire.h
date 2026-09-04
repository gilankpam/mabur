#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mabur::probe {

// SBI magic for probe stream bodies (spec 2026-09-04). One 9-byte probe header
// repeats in every sub-block of a probe body, each aligned after a 2-byte CRC
// for FEC symbol recovery. The magic distinguishes probe stream patterns (seed,
// profile, enh_fid) from unrelated payload and validates block integrity.
constexpr uint16_t kProbeMagic = 0xF5A5;

// Per-block probe header length: magic(u16) + seq(u32) + profile(u8) + enh_fid(u16).
constexpr size_t kProbeHdrLen = 9;

// Probe stream sub-block header: enh frame's candidate rung parameters.
struct ProbeHdr {
  uint32_t seq = 0;      // enh AU sequence number
  uint8_t profile = 0;   // codec profile
  uint16_t enh_fid = 0;  // enh frame id
};

// Packs a probe header into its 9-byte wire format (little-endian, magic first).
void pack_hdr(uint8_t out[kProbeHdrLen], const ProbeHdr& h);

// Parses a probe header from wire format. Returns false on short buffer or
// bad magic. Otherwise, h is filled and returns true.
bool parse_hdr(const uint8_t* p, size_t len, ProbeHdr* h);

// Computes the full SBI-framed probe body size: 11-byte SBI header plus per-block
// 2-byte CRC and block_payload bytes.
size_t probe_body_len(int bpb, int block_payload);

// Builds an SBI-framed probe body with the probe header repeated in every sub-block,
// and the remaining payload filled deterministically from the header's seq number
// via splitmix64 pseudo-random fill. The body is routed to stream_id 5 (kProbeStreamId).
// Layout: 11-byte SBI header, then bpb x [crc16 LE | kProbeHdrLen-byte header + fill].
std::vector<uint8_t> build_probe_body(const ProbeHdr& h, int bpb, int block_payload);

// Receives a parsed probe body. The caller's own per-block CRC scan (sbi_unpack
// drops block indices, so the parser must walk the body directly). Surviving blocks
// are validated for consistency (seq, profile, enh_fid must agree across all CRC-clean blocks).
struct ProbeRx {
  ProbeHdr hdr;          // header extracted from the first surviving block
  int n_blocks = 0;      // blocks the body carried (from its SBI header, capped at 32)
  uint32_t survivors = 0; // bit i set = block i CRC-clean and header-consistent
  int n_ok = 0;          // popcount(survivors)
};

// Parses an SBI-framed probe body, validating the stream_id and performing a
// per-block CRC scan. Returns false when no block survived, blocks disagree on
// (seq, profile, enh_fid), or the body is too short. Otherwise, ProbeRx is filled
// with the merged results and returns true.
bool parse_probe_body(const uint8_t* body, size_t len, int block_payload, ProbeRx* out);

}  // namespace mabur::probe
