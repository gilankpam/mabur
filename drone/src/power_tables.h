#pragma once
// Per-width power plans built from RadioCfg's rate_walls_rel_20/_40, and the
// cal_flags_20/_40 string -> wire mask mapping. See power_plan.h for what a
// PowerPlan actually means; this file is the per-width wrapper over it plus
// the fallback a unit calibrated on only one width needs.

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "config.h"
#include "power_plan.h"
#include "mabur/cal_wire.h"

namespace mabur {

// One PowerPlan per width, built straight from RadioCfg -- nullopt when that
// width has no wall table of its own (RadioCfg::walls_for() already folds
// the cross-width fallback in at load time; plan_for() below applies the
// same fallback to the built plans, so either entry point gives the same
// answer).
struct WidthPlans {
  std::optional<PowerPlan> p20, p40;
};

inline WidthPlans make_width_plans(const RadioCfg& r, double margin_db,
                                   int anchor_idx) {
  WidthPlans wp;
  if (r.rate_walls_rel_20)
    wp.p20 = make_power_plan(*r.rate_walls_rel_20, r.legacy_wall_rel,
                             margin_db, anchor_idx);
  if (r.rate_walls_rel_40)
    wp.p40 = make_power_plan(*r.rate_walls_rel_40, r.legacy_wall_rel,
                             margin_db, anchor_idx);
  return wp;
}

// The plan for `bw`, falling back to the other width's when `bw`'s own is
// absent (same fallback rule as RadioCfg::walls_for()). nullptr when
// neither width has a plan.
inline const PowerPlan* plan_for(const WidthPlans& p, uint8_t bw) {
  const auto& own = (bw == 40) ? p.p40 : p.p20;
  const auto& other = (bw == 40) ? p.p20 : p.p40;
  if (own) return &*own;
  if (other) return &*other;
  return nullptr;
}

// "ldpc+stbc" -> kFlagLdpc|kFlagStbc, "stbc" -> kFlagStbc, "ldpc" ->
// kFlagLdpc, "none" -> 0. nullopt for anything else -- callers that read
// this from config.cpp never see anything else, since parse_radio already
// rejects an unknown string at load; this stays total so a caller
// constructing a string some other way isn't tempted to assume it is
// always one of the four.
inline std::optional<uint8_t> cal_flags_mask(const std::string& s) {
  if (s == "ldpc+stbc")
    return static_cast<uint8_t>(mabur::cal::kFlagLdpc | mabur::cal::kFlagStbc);
  if (s == "stbc") return mabur::cal::kFlagStbc;
  if (s == "ldpc") return mabur::cal::kFlagLdpc;
  if (s == "none") return static_cast<uint8_t>(0);
  return std::nullopt;
}

}  // namespace mabur
