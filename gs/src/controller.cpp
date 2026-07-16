#include "controller.h"

#include <algorithm>
#include <cmath>

#include "energy.h"

namespace maburgs {

Controller::Controller(const LinkTable& lt, ControllerConfig cfg)
    : lt_(lt), cfg_(std::move(cfg)) {
  rows_ = build_link_rows(lt_, cfg_.target, cfg_.mcs_set, cfg_.overhead_set,
                          cfg_.bw, false, cfg_.bw_set, cfg_.vht);
}

double Controller::path_loss(double reported_snr, int reported_offset_qdb) const {
  return reported_snr - gain_db(reported_offset_qdb);
}

bool Controller::rung_blocked(int bw) const {
  auto it = rung_block_.find(bw);
  return it != rung_block_.end() && now_ms_ < it->second;
}

void Controller::report_rung_delivery(
    const std::map<int, std::pair<double, int>>& stats, double now_ms) {
  std::map<int, double> usable;
  for (const auto& [bw, dn] : stats)
    if (dn.second >= cfg_.rung_min_samples) usable[bw] = dn.first;
  if (usable.empty()) return;
  double best = -1.0;
  for (const auto& [bw, d] : usable) best = std::max(best, d);
  const int narrowest = usable.begin()->first;
  for (const auto& [bw, d] : usable)
    if (bw != narrowest && d < best - cfg_.rung_block_delta)
      rung_block_[bw] = now_ms + cfg_.rung_block_hold_ms;
  primary_dirty_ = usable.size() >= 2;
  if (primary_dirty_)
    for (const auto& [bw, d] : usable)
      if (d >= cfg_.target - cfg_.rung_block_delta) { primary_dirty_ = false; break; }
}

std::optional<OpPoint> Controller::best(double path_loss, double margin) const {
  std::optional<OpPoint> out;
  for (const auto& r : rows_) {
    if (rung_blocked(r.bw)) continue;
    auto op = resolve(r, path_loss, lt_, cfg_.payload_bytes,
                      cfg_.src_bitrate_bps, margin, cfg_.min_offset_qdb,
                      cfg_.max_offset_qdb, cfg_.base_ref_idx);
    if (!op || op->p_deliver < cfg_.target || std::isinf(op->e_bit)) continue;
    if (!out || op->e_bit < out->e_bit) out = op;
  }
  return out;
}

std::optional<OpPoint> Controller::update(double reported_snr,
                                          int reported_offset_qdb, double now_ms) {
  last_feedback_ms_ = now_ms;
  now_ms_ = now_ms;
  const double pl_inst = path_loss(reported_snr, reported_offset_qdb);
  if (!has_ema_) {
    snr_ema_ = pl_inst;
    has_ema_ = true;
  } else {
    const double a = pl_inst < snr_ema_ ? cfg_.ema_alpha_down : cfg_.ema_alpha;
    snr_ema_ = (1 - a) * snr_ema_ + a * pl_inst;
  }
  return decide(snr_ema_, now_ms);
}

std::optional<OpPoint> Controller::decide(double path_loss, double now_ms) {
  bool cur_ok = false;
  if (cur_.has_value() && !shed_) {
    const LinkRow row{cur_->vht, cur_->mcs, cur_->bw, cur_->sgi,
                      cur_->overhead, cur_->snr_req};
    auto cur_now = resolve(row, path_loss, lt_, cfg_.payload_bytes,
                           cfg_.src_bitrate_bps, 0.0, cfg_.min_offset_qdb,
                           cfg_.max_offset_qdb, cfg_.base_ref_idx);
    cur_ok = cur_now.has_value() && cur_now->p_deliver >= cfg_.target &&
             !rung_blocked(cur_->bw);
    if (cur_ok) cur_ = cur_now;  // refresh txagc/e_bit at the new path loss
  }

  auto cand = best(path_loss, cfg_.margin_db);

  if (!cand) {
    if (cur_ok) return cur_;
    if (cfg_.allow_shed) {
      shed_ = true;
      cur_.reset();
      return std::nullopt;
    }
    cur_ = max_range(cfg_.max_offset_qdb);
    return cur_;
  }

  shed_ = false;
  if (!cur_.has_value()) return commit(*cand, now_ms);

  if (now_ms - last_change_ms_ < cfg_.min_between_changes_ms && cur_ok) return cur_;
  const bool is_upgrade = cand->e_bit < cur_->e_bit;
  if (is_upgrade) {
    if (cur_ok && cand->e_bit > cur_->e_bit * (1 - cfg_.improve_frac)) return cur_;
    if (cur_ok && now_ms - last_downgrade_ms_ < cfg_.hold_after_downgrade_ms)
      return cur_;
    return commit(*cand, now_ms);
  }
  if (!cur_ok) {
    last_downgrade_ms_ = now_ms;
    return commit(*cand, now_ms);
  }
  return cur_;
}

std::optional<OpPoint> Controller::on_tick(double now_ms) {
  if (now_ms - last_feedback_ms_ > cfg_.feedback_timeout_ms) {
    shed_ = false;
    cur_ = max_range(cfg_.max_offset_qdb);
  }
  return cur_;
}

std::optional<OpPoint> Controller::commit(const OpPoint& op, double now_ms) {
  cur_ = op;
  last_change_ms_ = now_ms;
  return cur_;
}

bool Controller::shed() const { return shed_; }
bool Controller::primary_dirty() const { return primary_dirty_; }

}  // namespace maburgs
