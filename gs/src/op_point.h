#pragma once

namespace maburgs {

// Snapshot of a commanded operating point: PHY mode/rate and FEC overhead.
// Historically owned by the model-driven link-table resolver (deleted
// 2026-07-27, SDD ladder-controller Task 5, along with its two output
// fields that had zero live readers anywhere in the tree); the ladder
// controller (ladder_controller.h) only ever produces {mcs, overhead} via
// Rung, so VrxController fills the rest with 0 and leaves this struct's
// shape otherwise unchanged so stats_exporter and the wire encoding
// (rc_proto profile/overhead fields) keep compiling untouched. snr_req is
// kept — stats_exporter still serializes it. Power is constant and is not
// part of the operating point (spec 2026-08-12-constant-txpower).
// Same-rate-fixed-pairs (Task 4): overhead is a base/enh pair, mirroring
// Rung — the two sids share one MCS (same-rate) but each gets its own
// literal FEC command overhead, straight onto the RCF pair.
struct OpPoint {
  bool vht = false;
  int mcs = 0;
  int bw = 20;
  bool sgi = false;
  double overhead_base = 1.0;
  double overhead_enh = 1.0;
  double snr_req = 0.0;
};

}  // namespace maburgs
