#ifndef MABUR_PLAYER_SPLASH_MENU_H_
#define MABUR_PLAYER_SPLASH_MENU_H_

namespace maburplay {

// Whether the settings menu should be allowed to take the splash's plane
// before any video has ever been shown, and when the splash should come
// back. The GS runs the splash (drm_presenter.h) until the first decoded
// frame; with no drone linked that can be forever, and the menu must be
// usable the whole time (docs/gs-menu.md). Pure decision logic, driven
// once per housekeeping tick from the menu's own open/closed state and
// whether a video frame has ever been decoded -- main.cpp acts on the
// two outputs (DrmPresenter::set_osd_over_splash(), and re-running the
// `show_splash` lambda), DrmPresenter owns the actual plane hand-off.
//
// `allow_over_splash` is a level, true for exactly the span the menu is
// open with no frame yet: set every tick, so DrmPresenter's gate always
// matches the current state even across a presenter rebuild in between
// (display-follow, a replug) that forgets the override.
//
// `restore_splash` is an edge, true for one tick only: the tick the menu
// closes having never seen a frame while it was open. A menu that closes
// AFTER a frame arrived does not fire it (the ordinary first-frame
// hand-off already ran), and it does not repeat on later ticks -- one
// splash->OSD switch on open, one OSD->splash switch on close, however
// many times the menu is opened and closed before video ever shows up.
class SplashMenuGate {
 public:
  struct Decision {
    bool allow_over_splash = false;
    bool restore_splash = false;
  };

  Decision tick(bool menu_open, bool have_frame) {
    Decision d;
    d.allow_over_splash = menu_open && !have_frame;
    d.restore_splash = was_open_ && !menu_open && !have_frame;
    was_open_ = menu_open;
    return d;
  }

 private:
  bool was_open_ = false;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_SPLASH_MENU_H_
