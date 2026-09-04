#pragma once
#include <cstdint>
#include "mabur/probe_wire.h"
#include "mabur/sbi.h"
#include "mabur/uep_encoder.h"

namespace mabur {
// Drone-side probe stream producer (spec 2026-09-04): one video-body-sized
// SBI body on kProbeStreamId per enh AU, at the RCF-commanded probe MCS.
// Pure — the caller decides WHEN (right after the enh AU's last body is
// pushed) and stamps enqueued_ms/pushed_us like any other body. Random
// initial seq like SwEncoder: a restarted daemon must not replay seqs.
class ProbeSource {
 public:
  ProbeSource(int bpb, int block_payload, uint32_t initial_seq)
      : bpb_(bpb), block_payload_(block_payload), seq_(initial_seq) {}
  UepBody build(uint8_t profile, uint16_t enh_fid) {
    UepBody b;
    b.stream_id = kProbeStreamId;
    b.body = probe::build_probe_body(probe::ProbeHdr{seq_++, profile, enh_fid},
                                     bpb_, block_payload_);
    ++built_;
    return b;
  }
  uint32_t next_seq() const { return seq_; }
  uint64_t built() const { return built_; }
 private:
  int bpb_, block_payload_;
  uint32_t seq_;
  uint64_t built_ = 0;
};
}  // namespace mabur
