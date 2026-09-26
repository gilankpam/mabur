#include "mtest.h"
#include "width_resync.h"
using namespace maburgs;

// Gate: the boot scout owns its card until joined; the fix-up opens once no
// scout thread runs AND the scan is over (frozen), or when there is no scout.
TEST(gate_closed_while_the_scout_thread_runs) {
  CHECK(!width_resync_open(/*scout_joined=*/false, /*has_scout=*/true, /*frozen=*/false));
  CHECK(!width_resync_open(false, true, true));   // frozen but not yet joined
}

TEST(gate_closed_before_the_scout_thread_ever_started) {
  // Joined (no thread yet) but not frozen: the scan has not even begun.
  CHECK(!width_resync_open(true, true, false));
}

TEST(gate_open_after_join_or_when_frozen_without_a_run) {
  CHECK(width_resync_open(true, true, true));     // joined after run(), or frozen before it ran
  CHECK(width_resync_open(true, false, false));   // scan off: no scout at all
}

TEST(card_needs_fix_only_when_ready_idle_untried_and_off_width) {
  WidthCard c{/*ready=*/true, /*width=*/20, /*tried=*/false, /*busy=*/false};
  CHECK(needs_width_fix(c, 40));
  CHECK(!needs_width_fix(c, 20));                 // already there
  WidthCard notready = c; notready.ready = false;
  CHECK(!needs_width_fix(notready, 40));
  WidthCard tried = c; tried.tried = true;
  CHECK(!needs_width_fix(tried, 40));             // once per bring-up
  WidthCard busy = c; busy.busy = true;
  CHECK(!needs_width_fix(busy, 40));              // in-flight dwell owns it now; retry next tick
  WidthCard at40 = c; at40.width = 40;
  CHECK(!needs_width_fix(at40, 40));
}
MTEST_MAIN
