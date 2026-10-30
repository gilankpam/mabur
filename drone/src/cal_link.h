#pragma once
// The drone-side glue between the calibration protocol (mabur/cal_proto.h)
// and the drone's config and power path, kept out of main.cpp so it can be
// tested on the host: what an ApplyCmd writes, where the Verify sweep that
// follows it parks each rate, the APPLIED cache that answers the GS's
// APPLY resends, and the boot/parameter-set warnings about the per-width
// wall tables.

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "cal_apply.h"
#include "config.h"
#include "mabur/cal_proto.h"

namespace mabur {

// The CalWrite an ApplyCmd asks for: the GS's kWallLeave and eff 0 both
// become cal_apply's "leave the config's value", eff_x1000 is scaled to a
// fraction, and cal_flags records `stamp_flags` (CalSweep::stamp_flags(),
// the flags the sweep's frames flew with).
CalWrite cal_write_for(const cal::ApplyCmd& a, uint8_t stamp_flags);

// "ldpc+stbc", "stbc", "ldpc" or "none" for a kFlagLdpc|kFlagStbc mask:
// the inverse of power_tables.h's cal_flags_mask.
std::string cal_flags_name(uint8_t flags);

// The parked relative index of each rate for the Verify sweep that
// follows an applied ApplyCmd: wall - margin_steps(margin_db) for every
// wall the apply wrote, clamped to the stamp's [kRelMin, kRelMax] range,
// and kWallLeave for every wall it left (and for all eight when the apply
// carried no walls). The GS predicts the same parks from the same walls
// and margin, so this deliberately ignores the drone-internal anchor cap
// make_power_plan applies.
std::array<int, 8> verify_parks(const cal::ApplyCmd& a, double margin_db);

// Per-run bookkeeping for applies, owned by the TX writer thread.
//
// The GS resends MABUR_CAL_APPLY every 200 ms until it hears
// MABUR_CAL_APPLIED, so the APPLIED of every (nonce, bw) already applied
// is kept and re-sent for each resend instead of applying again. The
// "<config>.pre-cal" backup belongs to the first successful apply of a
// run only, so it holds the file as it was before the run. A nonce other
// than the one recorded starts a fresh ledger.
class CalApplyLedger {
 public:
  // The APPLIED already sent for (nonce, bw), if any.
  std::optional<cal::Applied> cached(uint32_t nonce, uint8_t bw) const;
  // True until an apply of `nonce` has succeeded.
  bool want_backup(uint32_t nonce) const;
  // Records the outcome of one apply.
  void record(const cal::Applied& a);

 private:
  bool have_nonce_ = false;
  uint32_t nonce_ = 0;
  bool backed_up_ = false;
  std::optional<cal::Applied> by_bw_[2];  // [0] = 20 MHz, [1] = 40 MHz
};

// One line per width whose own wall table the drone flies (power_mode
// "offset") while radio.ldpc disagrees with the ldpc flag the table was
// measured under (cal_flags_<bw>). Flags-off walls sit below flags-on
// ones, so an ldpc-on table flown with ldpc off is the overdriving
// direction. Empty when nothing disagrees, or when power_mode is "none"
// and no table is flown at all.
std::vector<std::string> ldpc_mismatch_warnings(const RadioCfg& r);

// One line per width that has no wall table of its own and so flies the
// other width's (RadioCfg::walls_for's fallback), under power_mode
// "offset". Empty when both widths have a table, or neither does.
std::vector<std::string> missing_wall_notes(const RadioCfg& r);

}  // namespace mabur
