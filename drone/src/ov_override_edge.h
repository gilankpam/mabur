#pragma once

namespace mabur {

// Edge tracker for the debug-HTTP per-layer overhead override, stepped once
// per hot-loop tick. The override is armed when both percentages are >= 0:
// every armed tick re-applies it. The first tick after it clears must
// re-apply the current op's overheads exactly once, because clearing has NO
// bounded re-assert to fall back on -- run_bitrate_policy's 5 s reassert
// (kReassertMs) only re-sends the bitrate/roi_qp verbs and never touches the
// UEP layers' overhead, so without this the encoder would stay pinned at the
// last override value until the next genuine op change.
class OvOverrideEdge {
 public:
  enum class Action { None, ApplyOverride, ReapplyOp };

  Action step(int base_pct, int enh_pct) {
    if (base_pct >= 0 && enh_pct >= 0) {
      was_armed_ = true;
      return Action::ApplyOverride;
    }
    if (was_armed_) {
      was_armed_ = false;
      return Action::ReapplyOp;
    }
    return Action::None;
  }

 private:
  bool was_armed_ = false;
};

}  // namespace mabur
