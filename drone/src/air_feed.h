#pragma once
#include <cstddef>
#include "rc_agent.h"  // AirFeedOut

namespace mabur {

// The measurement half of the deleted air-time balancer (Task 7, spec
// 2026-08-30-same-rate-fixed-pairs §... — the solver that redistributed
// FEC overhead between the two streams is gone: the per-rung overhead
// PAIR is now fixed and applied directly by apply_op_to_uep, Task 6).
// AirFeed keeps the per-stream EWMAs (frame-unit bytes in, emitted body
// bytes out) that run_bitrate_policy's blended-rate target consumes via
// AirFeedOut, and reports how far actual emitted bytes diverge from the
// anchor overhead currently in effect (excess_base/enh) purely for
// observability plus the bitrate policy's framing-excess term.
class AirFeed {
 public:
  explicit AirFeed(AirFeedOut* out);  // out may be nullptr (tests)

  // One shipped frame: sid 0/1, frame-unit bytes in, body bytes out.
  // IDR frames are excluded by the caller (spec: 2-10x outliers). Updates
  // the EWMAs, then publishes share_base/excess_base/excess_enh/ov_base/
  // ov_enh into the AirFeedOut (no-op if out is null).
  void on_frame(int sid, size_t len_in, size_t emitted);

  // The overhead anchor each stream is CURRENTLY flying — called from the
  // hot loop whenever apply_op_to_uep applies a new op (mirrors that call
  // site 1:1). excess_base/enh measure actual emitted bytes against this
  // anchor. If the debug-HTTP override (AirFeedOut::ovr_*_pct) is armed,
  // on_frame's publish uses the override values as the anchor instead —
  // the override wins in main.cpp's hot loop, so the published anchor must
  // track what's ACTUALLY flying, not the stale op pair.
  void set_applied(double ovb, double ove);

  bool seeded() const { return len_[0] > 0.0 && len_[1] > 0.0; }

  // The AirFeedOut this feed publishes into (never null in main.cpp — the
  // hot loop reads the debug-HTTP override atomics through this instead of
  // keeping a second pointer to the same struct around).
  AirFeedOut& out() const { return *out_; }

 private:
  double len_[2] = {0.0, 0.0};    // EWMA frame-unit bytes, alpha 1/16
  double emit_[2] = {0.0, 0.0};   // EWMA emitted body bytes, alpha 1/16
  double applied_[2] = {-1.0, -1.0};  // ov anchor set by set_applied(); <0 = none yet
  AirFeedOut* out_;
};

}  // namespace mabur
