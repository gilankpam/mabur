#pragma once
// Power plans per (channel, width), built from the wall cache's measured
// curves. See power_plan.h for what a PowerPlan actually means; this file is
// the per-(channel, width) wrapper over it and its lookup.

#include <cstdint>
#include <map>
#include <utility>

#include "config.h"
#include "power_plan.h"
#include "wall_cache.h"
#include "wall_curve.h"

namespace mabur {

struct ChannelPlans {
  std::map<std::pair<uint8_t, uint8_t>, PowerPlan> by;  // key (channel, width)
};

// (channel, bw)'s own plan, else none: the chip's own table. The other
// width's plan is never borrowed: at 40 MHz a 20 MHz plan lost a quarter to
// half of the frames at mcs4 on a channel where the chip's table was clean.
inline const PowerPlan* plan_for(const ChannelPlans& p, uint8_t channel, uint8_t bw) {
  auto own = p.by.find({channel, bw});
  return own == p.by.end() ? nullptr : &own->second;
}

// Empty unless power_mode is "offset". One plan per cache entry, its walls
// derived from the stored curve under the link's LDPC flag now, so an ldpc
// or margin change re-derives every plan without a re-measure. An entry
// whose curve fails the measurement's validity checks (a hand-edited or
// damaged file can still parse) is never flown: its walls could read 63,
// full power.
inline ChannelPlans make_channel_plans(const WallCache& cache, const RadioCfg& r,
                                       double margin_db, bool ldpc) {
  ChannelPlans p;
  if (r.power_mode != "offset") return p;
  for (const auto& [key, curve] : cache.entries)
    if (check_curve(curve) == cal::WallResult::Ok)
      p.by[key] = make_power_plan(walls_from_curve(curve, ldpc), r.legacy_wall_rel, margin_db,
                                curve.anchor);
  return p;
}

}  // namespace mabur
