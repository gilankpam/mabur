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

// `refresh <reason>` lines are queued verbatim and never touch the VTX
// record wish; malformed lines are ignored.
TEST(rec_control_refresh_lines) {
  maburgs::RecControl c;
  CHECK(!c.apply("refresh start"));         // not a wish change ...
  CHECK(!c.apply("refresh dvr\n"));         // ... trailing newline tolerated
  CHECK(!c.apply("refresh"));               // no reason: ignored
  CHECK(!c.apply("refresh  resync"));       // two spaces: ignored, the line is exact
  CHECK(!c.apply("refreshstart"));
  auto got = c.take_refresh();
  REQUIRE(got.size() == 2);
  CHECK(got[0] == "start" && got[1] == "dvr");
  CHECK(c.take_refresh().empty());
  CHECK(c.wire() == 0);                     // the VTX wish is still unknown
  CHECK(c.apply("vtx_rec on"));
  CHECK(!c.apply("refresh decoder"));
  CHECK(c.wire() == (mabur::rc::kRecKnown | mabur::rc::kRecOn));
}

MTEST_MAIN
