#pragma once
// The drone-side glue between the calibration protocol (mabur/cal_proto.h)
// and the drone's config and power path, kept out of maburd so it can be
// tested on the host: what an ApplyCmd writes, the APPLIED cache that
// answers the GS's APPLY resends, and the restore a refused run owes.

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "cal_apply.h"
#include "config.h"
#include "mabur/cal_proto.h"

namespace mabur {

// The CalWrite an ApplyCmd asks for: eff_x1000 scaled to a fraction, the
// GS's 0 kept as cal_apply's "leave the config's value".
CalWrite cal_write_for(const cal::ApplyCmd& a);

// Per-run bookkeeping for applies, owned by the TX writer thread.
//
// The GS resends MABUR_CAL_APPLY every 200 ms until it hears
// MABUR_CAL_APPLIED, so the APPLIED of every (nonce, seq) already applied
// is kept and re-sent for each resend instead of applying again. Keyed on
// the sweep's seq, not the width: a run may apply two sweeps at the same
// width, and the second must be applied. The
// "<config>.pre-cal" backup belongs to the first successful apply of a
// run only, so it holds the file as it was before the run. A nonce other
// than the one recorded starts a fresh ledger.
class CalApplyLedger {
 public:
  // The APPLIED already sent for (nonce, seq), if any.
  std::optional<cal::Applied> cached(uint32_t nonce, uint8_t seq) const;
  // True until an apply of `nonce` has succeeded.
  bool want_backup(uint32_t nonce) const;
  // Records the outcome of one apply.
  void record(const cal::Applied& a);

 private:
  bool have_nonce_ = false;
  uint32_t nonce_ = 0;
  bool backed_up_ = false;
  std::map<uint8_t, cal::Applied> by_seq_;
};

// The TX writer's "calibration touched the device, the operating power is
// owed back" latch. Noted at a sweep's first device write; due once no run
// is open -- however briefly the run lived. A run can open and close inside
// one drain of the calibration queue (a SWEEP and an ABORT together), never
// once reading as active between passes, and a refused sweep never opens
// one at all; both leave the diffs zeroed and the override parked.
class CalRestoreOwed {
 public:
  void note_device_touched() { owed_ = true; }
  // True once, clearing the latch, when a restore is owed and no run is open.
  bool take_due(bool run_active) {
    if (!owed_ || run_active) return false;
    owed_ = false;
    return true;
  }

 private:
  bool owed_ = false;
};

}  // namespace mabur
