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

// Classifies an RTP packet carrying HEVC (RFC 7798) payload into a stream id
// in [0, 3] for waybeam's layered link. stream_id mapping: critical NAL ->
// 0; otherwise 1 + min(tid, 2). Anything unparseable (short/truncated
// packet, bad RTP version, malformed HEVC payload) -> 0. This is a
// deliberate protect-up policy: waybeam is a trusted producer, so
// misclassifying a non-critical NAL as critical only costs extra airtime,
// while misclassifying a critical NAL (e.g. an IDR) as non-critical risks
// losing it under adverse link conditions.
int classify_rtp(const uint8_t* pkt, size_t len);

// Classifies a whole encoded Annex-B frame (as delivered by waybeam's
// frame-shm ring) into a stream id in [0, 3]. Walks 00 00 01 start codes
// (a 4-byte 00 00 00 01 code contains one): any critical NAL (VPS/SPS/PPS
// 32-34, IRAP 16-23) -> 0 immediately; otherwise the first VCL NAL
// (type < 16) picks 1 + min(tid, 2). No parseable NAL -> 0 — the same
// protect-up policy as classify_rtp. Replaces per-packet FU spelunking on
// the frame-shm ingest path (spec 2026-07-22).
int classify_frame(const uint8_t* annexb, size_t len);

}  // namespace mabur
