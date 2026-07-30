#pragma once

namespace maburgs {

// Snapshot of a commanded operating point: PHY mode/rate, FEC overhead, and
// power offset. Historically owned by the model-driven link-table resolver
// (deleted 2026-07-27, SDD ladder-controller Task 5, along with its two
// output fields that had zero live readers anywhere in the tree); the
// ladder controller (ladder_controller.h) only ever produces {mcs,
// overhead} via Rung, so VrxController fills the rest with 0 and leaves
// this struct's shape otherwise unchanged so stats_exporter and the wire
// encoding (rc_proto profile/overhead/offset fields) keep compiling
// untouched. snr_req is kept — stats_exporter still serializes it.
struct OpPoint {
  bool vht = false;
  int mcs = 0;
  int bw = 20;
  bool sgi = false;
  int pwr_offset_qdb = 0;
  double overhead = 1.0;
  double snr_req = 0.0;
};

}  // namespace maburgs
