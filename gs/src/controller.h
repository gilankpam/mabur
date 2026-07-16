#pragma once

#include <map>
#include <optional>
#include <vector>

#include "op_table.h"

namespace maburgs {

struct ControllerConfig {
  double target = 0.99;
  int payload_bytes = 1024;
  double src_bitrate_bps = 4e6;
  std::vector<int> mcs_set = {0, 1, 2, 3, 4, 5, 6, 7};
  std::vector<double> overhead_set = {0.10, 0.25, 0.50, 0.75, 1.00};
  int bw = 20;
  std::vector<int> bw_set;              // empty = single-bw rows
  bool vht = false;
  double ema_alpha = 0.3, ema_alpha_down = 0.8;
  double margin_db = 2.0;
  int min_between_changes_ms = 150;
  int hold_after_downgrade_ms = 4000;
  double improve_frac = 0.03;
  int feedback_timeout_ms = 1000;
  bool allow_shed = false;
  double rung_block_delta = 0.15;
  int rung_block_hold_ms = 5000;
  int rung_min_samples = 8;
  // qdB offset rail (RCF wire semantics, rc_proto bias-64). max_offset_qdb
  // is ZERO by construction: the wall-equalized diffs already park every
  // rate at wall - margin at offset 0, so no legal offset can exceed it
  // (docs/txagc-calibration.md). min_offset_qdb is the deployable floor.
  int min_offset_qdb = -40;
  int max_offset_qdb = 0;
  int base_ref_idx = 53;
};

class Controller {
 public:
  Controller(const LinkTable& lt, ControllerConfig cfg);
  void report_rung_delivery(const std::map<int, std::pair<double, int>>& stats,
                            double now_ms);
  std::optional<OpPoint> update(double reported_snr, int reported_offset_qdb, double now_ms);
  std::optional<OpPoint> on_tick(double now_ms);
  bool shed() const;
  bool primary_dirty() const;

 private:
  const LinkTable& lt_;
  ControllerConfig cfg_;
  std::vector<LinkRow> rows_;
  bool has_ema_ = false;
  double snr_ema_ = 0;
  std::optional<OpPoint> cur_;
  bool shed_ = false;
  double last_change_ms_ = -1e18;
  double last_downgrade_ms_ = -1e18;
  double last_feedback_ms_ = -1e18;
  double now_ms_ = -1e18;
  std::map<int, double> rung_block_;
  bool primary_dirty_ = false;

  double path_loss(double reported_snr, int reported_offset_qdb) const;
  bool rung_blocked(int bw) const;
  std::optional<OpPoint> best(double path_loss, double margin) const;
  std::optional<OpPoint> decide(double path_loss, double now_ms);
  std::optional<OpPoint> commit(const OpPoint& op, double now_ms);
};

}  // namespace maburgs
