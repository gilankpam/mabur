#include "../drone/src/ov_override_edge.h"
#include "mtest.h"

using mabur::OvOverrideEdge;
using Action = OvOverrideEdge::Action;

TEST(disarmed_does_nothing) {
  OvOverrideEdge e;
  CHECK(e.step(-1, -1) == Action::None);
  CHECK(e.step(5, -1) == Action::None);
  CHECK(e.step(-1, 5) == Action::None);
}

TEST(armed_applies_every_tick) {
  OvOverrideEdge e;
  CHECK(e.step(0, 0) == Action::ApplyOverride);
  CHECK(e.step(10, 20) == Action::ApplyOverride);
}

TEST(clearing_reapplies_op_once) {
  OvOverrideEdge e;
  e.step(10, 20);
  CHECK(e.step(-1, -1) == Action::ReapplyOp);
  CHECK(e.step(-1, -1) == Action::None);
  CHECK(e.step(10, -1) == Action::None);
}

MTEST_MAIN
