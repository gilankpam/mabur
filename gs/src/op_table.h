#pragma once

#include <optional>
#include <vector>

#include "op_point.h"

// Port of devourer's op_table.py: LinkTable with 1 dB bucketing via
// std::nearbyint (banker's rounding) to replicate Python's
// round(snr/bucket)*bucket exactly. snr_required scans [lo=-5, hi=40, step=0.5]
// and returns hi+step when target never met, matching link_model.py exactly.

namespace maburgs {

struct LinkRow {
  bool vht;
  int mcs;
  int bw;
  bool sgi;
  double overhead;
  double snr_req;
};

class LinkTable {
 public:
  // Look up p_deliver at 1-dB bucketed SNR, clamped to [mcs_min, mcs_max],
  // using the generated grid (gen/gen_tables.h).
  double p_deliver(double snr_db, int mcs, double overhead) const;

  // Scan SNR from -5.0 dB to 40.0 dB (step 0.5), return first SNR meeting
  // target delivery, or 40.5 if never met (mirrors link_model.py exactly).
  double snr_required(int mcs, double overhead, double target) const;
};

// Generate link table rows for all (mcs, overhead) in the cross-product of
// mcs_set x overhead_set x widths, where widths = bw_set if non-empty else
// [bw]. Each row holds the snr_required for that mode + overhead at the
// specified target, plus BW-specific noise penalty.
std::vector<LinkRow> build_link_rows(const LinkTable& lt, double target,
                                     const std::vector<int>& mcs_set,
                                     const std::vector<double>& overhead_set,
                                     int bw, bool sgi,
                                     const std::vector<int>& bw_set, bool vht);

// Resolve a LinkRow at given path_loss_db; compute the required qdB power
// offset, received SNR, p_deliver, and energy/bit. Return std::nullopt if
// even max_offset_qdb can't supply the needed gain.
std::optional<OpPoint> resolve(const LinkRow& row, double path_loss_db,
                               const LinkTable& lt, int payload_bytes,
                               double src_bitrate_bps, double margin_db,
                               int min_offset_qdb, int max_offset_qdb,
                               int base_ref_idx);

// Sentinel: HT MCS0 BW20, max_offset_qdb, overhead 1.0, 0 delivery (max range).
OpPoint max_range(int max_offset_qdb);

}  // namespace maburgs
