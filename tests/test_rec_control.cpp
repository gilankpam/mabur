#include "mtest.h"
#include "rec_control.h"
#include "mabur/rc_proto.h"

using maburgs::RecControl;

TEST(rec_control_starts_unknown) {
  RecControl c;
  CHECK(c.wire() == 0);   // unknown: the drone keeps whatever it is doing
}

TEST(rec_control_on_off_and_holds_without_decay) {
  RecControl c;
  CHECK(c.apply("vtx_rec on"));
  CHECK(c.wire() == (mabur::rc::kRecKnown | mabur::rc::kRecOn));
  CHECK(!c.apply("vtx_rec on\n"));   // same wish: no change
  CHECK(c.wire() == (mabur::rc::kRecKnown | mabur::rc::kRecOn));  // held, no timer
  CHECK(c.apply("vtx_rec off"));
  CHECK(c.wire() == mabur::rc::kRecKnown);
}

TEST(rec_control_ignores_garbage) {
  RecControl c;
  CHECK(!c.apply(""));
  CHECK(!c.apply("vtx_rec maybe"));
  CHECK(!c.apply("start"));
  CHECK(c.wire() == 0);
}

MTEST_MAIN
