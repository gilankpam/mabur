#pragma once

namespace maburgs {

// Snapshot of a commanded operating point: PHY mode/rate, FEC overhead, and
// power offset. Historically owned by op_table.h's LinkTable resolver
// (snr_req/e_bit/p_deliver are that model's outputs); the ladder controller
// (ladder_controller.h) only ever produces {mcs, overhead} via Rung, so
// VrxController fills the rest with 0 and leaves this struct's shape
// unchanged so stats_exporter and the wire encoding (rc_proto profile/
// overhead/offset fields) keep compiling untouched. Moved out of op_table.h
// (SDD 2026-07-27 ladder-controller Task 4) so it survives op_table.h's
// deletion in Task 5.
struct OpPoint {
  bool vht = false;
  int mcs = 0;
  int bw = 20;
  bool sgi = false;
  int pwr_offset_qdb = 0;
  double overhead = 1.0;
  double snr_req = 0.0;
  double e_bit;
  double p_deliver = 0.0;
};

}  // namespace maburgs
