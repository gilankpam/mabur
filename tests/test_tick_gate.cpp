#include "../drone/src/tick_gate.h"
#include "mtest.h"

using mabur::TickGate;

TEST(tick_gate_cadence_invariant_under_fast_wakes) {
  // 5 ms wakes over 1 s with period 100: fires exactly as often as 100 ms
  // wakes do — the deadline, not the wake rate, owns the cadence.
  TickGate fast(0, 100);
  int fires_fast = 0;
  for (uint64_t now = 0; now <= 1000; now += 5)
    if (fast.due(now)) ++fires_fast;
  TickGate slow(0, 100);
  int fires_slow = 0;
  for (uint64_t now = 0; now <= 1000; now += 100)
    if (slow.due(now)) ++fires_slow;
  CHECK(fires_fast == fires_slow);
  CHECK(fires_fast == 11);  // t=0 inclusive, then every 100 ms
}

TEST(tick_gate_first_call_fires_immediately) {
  TickGate g(500, 100);
  CHECK(g.due(500));
  CHECK(!g.due(599));
  CHECK(g.due(600));
}

TEST(tick_gate_wake_geq_period_is_legacy) {
  // rc_drain_ms >= tick_ms degenerates to firing on every wake — the
  // legacy loop shape (body every iteration).
  TickGate g(0, 100);
  int fires = 0;
  for (uint64_t now = 0; now <= 1000; now += 100)
    if (g.due(now)) ++fires;
  CHECK(fires == 11);
}

MTEST_MAIN
