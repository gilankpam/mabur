#include "op_table.h"

#include <cmath>
#include <limits>

#include "energy.h"
#include "gen/gen_tables.h"

namespace maburgs {
namespace {
int overhead_index(double ov) {
  for (int i = 0; i < gen::kOverheadCount; ++i)
    if (std::fabs(gen::kOverheads[i] - ov) < 1e-9) return i;
  // Defensive guard (pure safety, never called out-of-range by current callers):
  // nearest known overhead fallback when ov not in table set.
  int best = 0;
  for (int i = 1; i < gen::kOverheadCount; ++i)
    if (std::fabs(gen::kOverheads[i] - ov) < std::fabs(gen::kOverheads[best] - ov))
      best = i;
  return best;
}
}  // namespace

double LinkTable::p_deliver(double snr_db, int mcs, double overhead) const {
  // Defensive guard (pure safety, never called out-of-range by current callers):
  if (mcs < 0) mcs = 0;
  if (mcs >= gen::kMcsCount) mcs = gen::kMcsCount - 1;
  // Python: bucket = round(snr / 1.0) * 1.0 (banker's rounding) — nearbyint
  // under the default FE_TONEAREST mode is the same tie-to-even.
  int bi = static_cast<int>(std::nearbyint((snr_db - gen::kSnrLo) / gen::kSnrBucket));
  // Python's _snr_bucket is unbounded (it simulates a fresh cell for out-of-range
  // SNR); we clamp to the grid edges instead. These agree ONLY because the grid
  // endpoints saturate (0.0 at the low end, 1.0 at the high end). A future grid
  // regen that left a non-saturated endpoint would silently diverge — the
  // edges_pdeliver golden vectors (snr=-50->0.0, snr=100->1.0) guard this.
  if (bi < 0) bi = 0;
  if (bi >= gen::kSnrBuckets) bi = gen::kSnrBuckets - 1;
  const int oi = overhead_index(overhead);
  return gen::kPDeliver[(mcs * gen::kOverheadCount + oi) * gen::kSnrBuckets + bi];
}

double LinkTable::snr_required(int mcs, double overhead, double target) const {
  // Mirror link_model.snr_required: lo=-5, hi=40, step=0.5 (all exact in
  // binary), return hi+step when the target is never met.
  const double lo = -5.0, hi = 40.0, step = 0.5;
  double snr = lo;
  while (snr <= hi) {
    if (p_deliver(snr, mcs, overhead) >= target) return snr;
    snr += step;
  }
  return hi + step;
}

std::vector<LinkRow> build_link_rows(const LinkTable& lt, double target,
                                     const std::vector<int>& mcs_set,
                                     const std::vector<double>& overhead_set,
                                     int bw, bool sgi,
                                     const std::vector<int>& bw_set, bool vht) {
  std::vector<LinkRow> rows;
  std::vector<int> widths = bw_set.empty() ? std::vector<int>{bw} : bw_set;
  for (int w : widths)
    for (int mcs : mcs_set)
      for (double ov : overhead_set) {
        const double req = lt.snr_required(mcs, ov, target);
        rows.push_back(LinkRow{vht, mcs, w, sgi, ov, req + bw_noise_db(w)});
      }
  return rows;
}

std::optional<OpPoint> resolve(const LinkRow& row, double path_loss_db,
                               const LinkTable& lt, int payload_bytes,
                               double src_bitrate_bps, double margin_db) {
  const double need_gain = (row.snr_req + margin_db) - path_loss_db;
  int txagc = 0;
  if (need_gain > 0) {
    txagc = min_txagc_for_gain(need_gain);
    if (txagc < 0) return std::nullopt;
  }
  const double recv = path_loss_db + gain_db(txagc);
  const double pdel = lt.p_deliver(recv - bw_noise_db(row.bw), row.mcs, row.overhead);
  const TxPoint pt{row.vht, row.mcs, row.bw, row.sgi, txagc};
  const double eb = energy_per_delivered_bit(pt, src_bitrate_bps, row.overhead,
                                             payload_bytes, pdel);
  return OpPoint{row.vht, row.mcs, row.bw, row.sgi, txagc,
                 row.overhead, row.snr_req, eb, pdel};
}

OpPoint max_range() {
  return OpPoint{false, 0, 20, false, 63, 1.00, 0.0,
                 std::numeric_limits<double>::infinity(), 0.0};
}

}  // namespace maburgs
