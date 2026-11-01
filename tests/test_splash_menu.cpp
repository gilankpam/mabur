// Host test for gs/player/src/splash_menu.h: the pure open/close -> allow
// OSD over the splash / restore the splash decision rule.
#include "splash_menu.h"
#include "mtest.h"

using maburplay::SplashMenuGate;

TEST(open_before_video_allows) {
  SplashMenuGate g;
  SplashMenuGate::Decision d = g.tick(/*menu_open=*/true, /*have_frame=*/false);
  CHECK(d.allow_over_splash);
  CHECK(!d.restore_splash);
}

TEST(close_before_video_restores_once) {
  SplashMenuGate g;
  g.tick(true, false);   // open, no frame yet
  SplashMenuGate::Decision d = g.tick(false, false);  // close, still no frame
  CHECK(!d.allow_over_splash);
  CHECK(d.restore_splash);
  // Repeated ticks in the closed state don't re-fire.
  for (int i = 0; i < 5; ++i) {
    SplashMenuGate::Decision d2 = g.tick(false, false);
    CHECK(!d2.allow_over_splash);
    CHECK(!d2.restore_splash);
  }
}

TEST(open_then_video_arrives_no_restore_on_close) {
  SplashMenuGate g;
  g.tick(true, false);           // open before video
  SplashMenuGate::Decision mid = g.tick(true, true);  // video arrives, still open
  CHECK(!mid.allow_over_splash);  // nothing special once a frame exists
  CHECK(!mid.restore_splash);
  SplashMenuGate::Decision close = g.tick(false, true);  // close, after video
  CHECK(!close.allow_over_splash);
  CHECK(!close.restore_splash);  // the ordinary hand-off already ran
}

TEST(menu_never_opened_never_allows) {
  SplashMenuGate g;
  for (int i = 0; i < 10; ++i) {
    SplashMenuGate::Decision d = g.tick(false, false);
    CHECK(!d.allow_over_splash);
    CHECK(!d.restore_splash);
  }
}

TEST(repeated_ticks_while_open_dont_refire_or_flicker) {
  SplashMenuGate g;
  for (int i = 0; i < 10; ++i) {
    SplashMenuGate::Decision d = g.tick(true, false);
    CHECK(d.allow_over_splash);
    CHECK(!d.restore_splash);
  }
}

TEST(repeated_open_close_cycles_before_video_each_restore_once) {
  SplashMenuGate g;
  for (int cycle = 0; cycle < 3; ++cycle) {
    SplashMenuGate::Decision open = g.tick(true, false);
    CHECK(open.allow_over_splash);
    CHECK(!open.restore_splash);
    SplashMenuGate::Decision close = g.tick(false, false);
    CHECK(!close.allow_over_splash);
    CHECK(close.restore_splash);
  }
}

MTEST_MAIN
