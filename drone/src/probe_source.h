#pragma once
#include <cstdint>
#include "mabur/probe_wire.h"
#include "mabur/sbi.h"
#include "mabur/uep_encoder.h"

namespace mabur {
// Does a probe trail the AU just pushed? Every video AU that went on air
// -- base or enh, so the probe cadence is the AU rate, 60/s at 60 fps
// SVC-T (probe per AU, 2026-09-16; was enh-only, 30/s) -- while a probe
// is commanded. A shed layer's AU never went on air, so no probe: the GS
// books no expectation for an AU it never sees, and a probe carrying that
// fid would count as arrived-without-expected. A non-video body (MSP,
// the probe itself) is not an AU.
inline bool probe_follows(int au_sid, bool probe_commanded, bool layer_shed) {
  return probe_commanded && !layer_shed && au_sid >= 0 &&
         au_sid < UepEncoder::kNumStreams;
}

// Drone-side probe stream producer (spec 2026-09-04): one video-body-sized
// SBI body on kProbeStreamId per video AU (base and enh since 2026-09-16),
// at the RCF-commanded probe MCS. Pure — the caller decides WHEN (right
// after the AU's last body is pushed, see probe_follows) and stamps
// enqueued_ms/pushed_us like any other body. Random initial seq like
// SwEncoder: a restarted daemon must not replay seqs.
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
