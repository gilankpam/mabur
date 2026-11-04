#include "mtest.h"
#include "width_resync.h"
using namespace maburgs;

// Gate: closed while a scan holds the scout card (the boot pick open, or a
// calibration measure round), open otherwise or when there is no scout.
TEST(gate_closed_while_the_boot_pick_is_open) {
  CHECK(!width_resync_open(/*has_scout=*/true, /*pick_open=*/true, /*round_open=*/false));
}

TEST(gate_closed_while_a_measure_round_runs) {
  CHECK(!width_resync_open(true, false, true));
}

TEST(gate_open_once_the_pick_froze_or_without_a_scout) {
  CHECK(width_resync_open(true, false, false));    // frozen: a searching scout keeps only its own card
  CHECK(width_resync_open(false, true, false));    // scan off: no scout at all
}

TEST(card_needs_fix_only_when_ready_idle_untried_and_off_width) {
  WidthCard c{/*ready=*/true, /*width=*/20, /*tried=*/false, /*busy=*/false, /*scouting=*/false};
  CHECK(needs_width_fix(c, 40));
  CHECK(!needs_width_fix(c, 20));                 // already there
  WidthCard notready = c; notready.ready = false;
  CHECK(!needs_width_fix(notready, 40));
  WidthCard tried = c; tried.tried = true;
  CHECK(!needs_width_fix(tried, 40));             // once per bring-up
  WidthCard busy = c; busy.busy = true;
  CHECK(!needs_width_fix(busy, 40));              // in-flight dwell owns it now; retry next tick
  WidthCard scouting = c; scouting.scouting = true;
  CHECK(!needs_width_fix(scouting, 40));          // the search owns it; fixed once it lets go
  WidthCard at40 = c; at40.width = 40;
  CHECK(!needs_width_fix(at40, 40));
}
MTEST_MAIN
