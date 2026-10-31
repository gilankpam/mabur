#include "cal_link.h"

#include <cstdio>

#include "mabur/cal_park.h"
#include "mabur/cal_wire.h"
#include "mabur/rc_proto.h"
#include "power_plan.h"
#include "power_tables.h"

namespace mabur {

CalWrite cal_write_for(const cal::ApplyCmd& a, uint8_t stamp_flags) {
  CalWrite w;
  w.bw = a.bw;
  w.has_walls = a.has_walls;
  w.has_eff = a.has_eff;
  for (size_t r = 0; r < 8; ++r) {
    w.walls[r] = a.walls[r] == cal::kWallLeave ? rc::kWallUndetermined : a.walls[r];
    w.eff[r] = a.eff_x1000[r] == 0 ? 0.0 : a.eff_x1000[r] / 1000.0;
  }
  w.legacy_wall = a.legacy == cal::kWallLeave ? rc::kWallUndetermined : a.legacy;
  w.cal_flags = cal_flags_name(stamp_flags);
  return w;
}

std::string cal_flags_name(uint8_t flags) {
  const bool ldpc = (flags & cal::kFlagLdpc) != 0;
  const bool stbc = (flags & cal::kFlagStbc) != 0;
  if (ldpc && stbc) return "ldpc+stbc";
  if (stbc) return "stbc";
  if (ldpc) return "ldpc";
  return "none";
}

std::array<int, 8> verify_parks(const cal::ApplyCmd& a, double margin_db) {
  std::array<int, 8> park;
  park.fill(cal::kWallLeave);
  if (!a.has_walls) return park;
  return park_rel_for(a.walls, margin_db);
}

std::optional<cal::Applied> CalApplyLedger::cached(uint32_t nonce, uint8_t seq) const {
  if (!have_nonce_ || nonce != nonce_) return std::nullopt;
  const auto it = by_seq_.find(seq);
  if (it == by_seq_.end()) return std::nullopt;
  return it->second;
}

bool CalApplyLedger::want_backup(uint32_t nonce) const {
  return !(have_nonce_ && nonce == nonce_ && backed_up_);
}

void CalApplyLedger::record(const cal::Applied& a) {
  if (!have_nonce_ || a.nonce != nonce_) {
    have_nonce_ = true;
    nonce_ = a.nonce;
    backed_up_ = false;
    by_seq_.clear();
  }
  if (a.ok) backed_up_ = true;
  by_seq_[a.seq] = a;
}

namespace {
// 20 always (control frames, and 20 rungs fly on a 40-tuned radio's
// primary); 40 only on a radio tuned to 40.
bool width_flown(const RadioCfg& r, int bw) { return bw == 20 || r.width == 40; }
}  // namespace

std::vector<std::string> ldpc_mismatch_warnings(const RadioCfg& r) {
  std::vector<std::string> out;
  if (r.power_mode != "offset") return out;
  for (const int bw : {20, 40}) {
    if (!width_flown(r, bw)) continue;
    if (!r.has_walls(static_cast<uint8_t>(bw))) continue;
    const auto mask = cal_flags_mask(bw == 40 ? r.cal_flags_40 : r.cal_flags_20);
    if (!mask) continue;
    const bool measured_ldpc = (*mask & cal::kFlagLdpc) != 0;
    if (measured_ldpc == r.ldpc) continue;
    char line[128];
    std::snprintf(line, sizeof line,
                  "radio: %d MHz walls were measured with ldpc %s; radio.ldpc is %s "
                  "\xe2\x80\x94 re-run maburcal",
                  bw, measured_ldpc ? "on" : "off", r.ldpc ? "on" : "off");
    out.emplace_back(line);
  }
  return out;
}

std::vector<std::string> missing_wall_notes(const RadioCfg& r) {
  std::vector<std::string> out;
  if (r.power_mode != "offset") return out;
  for (const int bw : {20, 40}) {
    const int other = bw == 40 ? 20 : 40;
    if (!width_flown(r, bw)) continue;
    if (r.has_walls(static_cast<uint8_t>(bw)) || !r.has_walls(static_cast<uint8_t>(other)))
      continue;
    // Walls at 40 MHz sit at or above the 20 MHz walls, so 20's table at 40
    // only gives power away; 40's table at 20 overdrives, and the note says so.
    char line[160];
    std::snprintf(line, sizeof line, "radio: no %d MHz walls, flying the %d MHz table at %d%s", bw,
                  other, bw,
                  bw == 20 ? " \xe2\x80\x94 the overdriving direction, re-run maburcal at 20 MHz"
                           : "");
    out.emplace_back(line);
  }
  return out;
}

}  // namespace mabur
