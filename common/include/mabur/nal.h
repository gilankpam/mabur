#pragma once
#include <cstddef>
#include <cstdint>
namespace mabur {

// Result of parsing a single HEVC NAL unit header (first two bytes).
// Byte-exact port of devourer's examples/svctx/svc_tx.h parse_hevc_nal.
struct NalInfo {
  uint8_t tid = 0;
  bool critical = false;
  uint8_t type = 0;
};

// Parses a bare HEVC NAL unit (start-code / length prefix already stripped).
// nal[0] bits 6..1 give the NAL unit type; nal[1] bits 2..0 give
// nuh_temporal_id_plus1 (tid = max(that - 1, 0)). critical marks VPS/SPS/PPS
// (32-34) and the IRAP/IDR/BLA/CRA range (16-23), matching devourer's SVC
// layer policy. len < 2 -> default NalInfo (tid 0, type 0, not critical) —
// malformed input is treated as base layer, never as critical.
NalInfo parse_hevc_nal(const uint8_t* nal, size_t len);

// Classifies a whole encoded Annex-B frame (as delivered by maburd's venc
// ring) into a stream id in {0, 1} for the 2-stream link. Walks 00 00 01
// start codes (a 4-byte 00 00 00 01 code contains one): any critical NAL
// (VPS/SPS/PPS 32-34, IRAP 16-23) -> 0 immediately; otherwise the first
// VCL NAL (type < 16) picks the layer — TRAIL_N (type 0, non-referenced;
// maburd's SVC-T enhance marker) -> 1, all others (TRAIL_R, etc.) -> 0.
// No parseable VCL NAL -> 0.
//
// The unparseable -> 0 fallback is a deliberate protect-up policy: maburd is
// a trusted producer, so misclassifying a non-critical frame as critical only
// costs extra airtime, while misclassifying a critical frame (e.g. an IDR) as
// non-critical risks losing it under adverse link conditions.
int classify_frame(const uint8_t* annexb, size_t len);

// True when the frame's first VCL NAL (type < 16) is TRAIL_N (type 0) —
// maburd's SVC-T enhance marker (the star6e TRAIL_R->TRAIL_N rewrite).
// Walks the same start-code scan as classify_frame; a critical NAL seen
// BEFORE the first VCL NAL returns false. A critical NAL after the first
// VCL is never reached by this scan (it returns as soon as it finds a VCL
// NAL) — harmless, since classify_frame already maps such frames to sid 0
// regardless, and agreement demotion (this function feeding into it) only
// ever lowers sid, never raises it. Only genuine TRAIL_N participates in the
// producer-flag agreement check (spec 2026-07-26 svct-enable).
bool frame_is_trail_n(const uint8_t* annexb, size_t len);

}  // namespace mabur
