// Host test for gs/player/src/display_watch.h: when a display that went
// away under a live presenter has come back and the presenter is rebuilt,
// and the uevent filter that wakes the check.
#include <cstdint>
#include <utility>
#include <vector>
#include <cstddef>

#include "display_watch.h"
#include "mtest.h"

using maburplay::DisplayWatch;
using maburplay::is_drm_hotplug_uevent;
using Action = DisplayWatch::Action;

namespace {

// A literal's bytes without the terminator the compiler appends, so the
// embedded NULs separate fields exactly as in a datagram.
template <size_t N>
bool hotplug(const char (&s)[N]) {
  return is_drm_hotplug_uevent(s, N - 1);
}

// The display as the kernel reports it, plus a tally of what the watch
// said: one housekeeping iteration every `step` ms from `t` to `until`.
struct Sim {
  DisplayWatch w;
  uint64_t t = 100000;
  bool connected = true;
  int reads = 0;
  int unplugged = 0;
  int rebuilds = 0;
  int capped = 0;
  uint64_t last_rebuild_ms = 0;
  std::vector<std::pair<uint64_t, bool>> read_log;  // (time, connected)
  // A sink that answers every modeset by pulsing HPD: down `pulse_delay_ms`
  // after a rebuild, back 400 ms later.
  bool pulses_on_modeset = false;
  uint64_t pulse_delay_ms = 100;
  uint64_t pulse_down_ms = 0;
  uint64_t pulse_up_ms = 0;

  explicit Sim(bool uevents) : w(uevents) { w.on_acquired(t); }

  void tick(bool uevent = false) {
    if (w.due(t, uevent)) {
      ++reads;
      read_log.emplace_back(t, connected);
      switch (w.on_status(t, connected)) {
        case Action::kUnplugged: ++unplugged; break;
        case Action::kCapped: ++capped; break;
        case Action::kRebuild:
          ++rebuilds;
          last_rebuild_ms = t;
          w.on_acquired(t);  // what the caller's rebuild does
          if (pulses_on_modeset) {
            pulse_down_ms = t + pulse_delay_ms;
            pulse_up_ms = t + pulse_delay_ms + 400;
          }
          break;
        case Action::kNone: break;
      }
    }
  }
  // A hotplug edge: the kernel updates the status, then sends the uevent.
  void edge(bool now_connected) {
    connected = now_connected;
    tick(/*uevent=*/true);
  }
  void run_for(uint64_t ms, uint64_t step = 2) {
    const uint64_t end = t + ms;
    while (t < end) {
      t += step;
      if (pulse_down_ms != 0 && t >= pulse_down_ms) {
        pulse_down_ms = 0;
        edge(false);
      } else if (pulse_up_ms != 0 && t >= pulse_up_ms) {
        pulse_up_ms = 0;
        edge(true);
      } else {
        tick();
      }
    }
  }
};

}  // namespace

TEST(uevent_filter_matches_a_drm_hotplug) {
  CHECK(hotplug("change@/devices/platform/display-subsystem/drm/card0\0"
                "ACTION=change\0DEVPATH=/devices/platform/display-subsystem/drm/card0\0"
                "SUBSYSTEM=drm\0HOTPLUG=1\0DEVNAME=dri/card0\0SEQNUM=2140\0"));
  // Field order is free, and the last field need not be NUL-terminated.
  CHECK(hotplug("change@/x\0HOTPLUG=1\0SUBSYSTEM=drm"));
}

TEST(uevent_filter_rejects_everything_else) {
  CHECK(!is_drm_hotplug_uevent("", 0));
  // A drm change that is not a hotplug.
  CHECK(!hotplug("change@/x\0SUBSYSTEM=drm\0DEVNAME=dri/card0\0"));
  // A hotplug from another subsystem, or one whose name only starts "drm".
  CHECK(!hotplug("change@/x\0SUBSYSTEM=usb\0HOTPLUG=1\0"));
  CHECK(!hotplug("change@/x\0SUBSYSTEM=drm_dp_aux_dev\0HOTPLUG=1\0"));
  // Whole fields only: neither key may match as a prefix or a suffix.
  CHECK(!hotplug("change@/x\0XSUBSYSTEM=drm\0HOTPLUG=1\0"));
  CHECK(!hotplug("change@/x\0SUBSYSTEM=drm\0HOTPLUG=10\0"));
  CHECK(!hotplug("change@/x\0SUBSYSTEM=drm\0HOTPLUG=0\0"));
  // A field cut short by the datagram's length does not match.
  const char cut[] = "change@/x\0SUBSYSTEM=drm\0HOTPLUG=1\0";
  CHECK(!is_drm_hotplug_uevent(cut, sizeof(cut) - 3));
}

TEST(with_uevents_nothing_is_read_while_nothing_happens) {
  Sim s(true);
  s.run_for(60000);
  CHECK(s.reads == 0);
  CHECK(s.rebuilds == 0);
}

TEST(nothing_is_due_before_the_first_acquire) {
  DisplayWatch w(false);
  CHECK(!w.due(100000, true));
  CHECK(!w.due(200000, false));
}

TEST(unplug_then_replug_rebuilds_once_after_the_settle) {
  Sim s(true);
  s.run_for(10000);  // out of the startup guard
  s.edge(false);
  CHECK(s.unplugged == 1);
  s.run_for(3000);
  CHECK(s.unplugged == 1);  // logged once per episode, not per read
  CHECK(s.rebuilds == 0);
  s.edge(true);
  const uint64_t replug = s.t;
  s.run_for(DisplayWatch::kSettleMs - 10);
  CHECK(s.rebuilds == 0);  // still settling
  s.run_for(30000);
  CHECK(s.rebuilds == 1);
  CHECK(s.last_rebuild_ms >= replug + DisplayWatch::kSettleMs);
  CHECK(s.last_rebuild_ms <= replug + DisplayWatch::kSettleMs + 10);
}

TEST(a_burst_of_hotplug_edges_coalesces_into_one_rebuild) {
  Sim s(true);
  s.run_for(10000);
  s.edge(false);
  s.run_for(300);
  s.edge(true);
  s.run_for(400);
  s.edge(false);  // DDC/HPD flap mid-replug
  s.run_for(200);
  s.edge(true);
  s.run_for(500);
  s.tick(/*uevent=*/true);  // an EDID re-read: still connected
  const uint64_t quiet = s.t;
  s.run_for(30000);
  CHECK(s.unplugged == 1);
  CHECK(s.rebuilds == 1);
  CHECK(s.last_rebuild_ms >= quiet + DisplayWatch::kSettleMs);
}

TEST(two_replug_cycles_far_apart_rebuild_twice) {
  Sim s(true);
  s.run_for(10000);
  s.edge(false);
  s.run_for(500);
  s.edge(true);
  s.run_for(18000);
  s.edge(false);
  s.run_for(500);
  s.edge(true);
  s.run_for(18000);
  CHECK(s.unplugged == 2);
  CHECK(s.rebuilds == 2);
}

TEST(a_uevent_without_a_disconnect_never_rebuilds) {
  Sim s(true);
  s.run_for(10000);
  for (int i = 0; i < 20; ++i) {
    s.tick(/*uevent=*/true);
    s.run_for(1500);
  }
  CHECK(s.reads >= 20);
  CHECK(s.rebuilds == 0);
  CHECK(s.unplugged == 0);
}

TEST(the_rebuilds_own_hpd_pulse_does_not_rebuild_again) {
  Sim s(true);
  s.pulses_on_modeset = true;
  s.run_for(10000);
  s.edge(false);
  s.run_for(500);
  s.edge(true);
  // Were the pulse taken for a replug, each rebuild would cause the next:
  // one every ~1.5 s for the whole minute.
  s.run_for(60000);
  CHECK(s.rebuilds == 1);
  CHECK(s.unplugged == 1);
}

TEST(an_unplug_inside_the_guard_is_seen_when_the_guard_ends) {
  Sim s(true);
  s.run_for(1000);  // inside the startup guard
  s.edge(false);
  CHECK(s.unplugged == 0);
  s.run_for(DisplayWatch::kGuardMs);  // no uevent at the guard's end
  CHECK(s.unplugged == 1);
  s.run_for(5000);
  s.edge(true);
  s.run_for(DisplayWatch::kSettleMs + 100);
  CHECK(s.rebuilds == 1);
}

TEST(a_pulse_inside_the_guard_ends_quietly) {
  Sim s(true);
  s.run_for(500);
  s.edge(false);
  s.run_for(300);
  s.edge(true);
  s.run_for(60000);
  CHECK(s.unplugged == 0);
  CHECK(s.rebuilds == 0);
}

TEST(without_uevents_the_status_is_polled_once_a_second) {
  Sim s(false);
  s.run_for(10000);
  CHECK(s.reads >= 9 && s.reads <= 11);
  s.connected = false;
  s.run_for(3000);
  CHECK(s.unplugged == 1);
  s.connected = true;
  const uint64_t replug = s.t;
  s.run_for(30000);
  CHECK(s.rebuilds == 1);
  // The first connected poll starts the settle (it is the reconnect: no
  // uevent marks it); the rebuild runs kSettleMs after it, never at it.
  uint64_t first_connected = 0;
  for (const auto& r : s.read_log)
    if (r.first > replug && r.second) {
      first_connected = r.first;
      break;
    }
  REQUIRE(first_connected != 0);
  CHECK(first_connected <= replug + DisplayWatch::kPollMs);
  CHECK(s.last_rebuild_ms >= first_connected + DisplayWatch::kSettleMs);
  CHECK(s.last_rebuild_ms <= first_connected + DisplayWatch::kSettleMs + 10);
}

TEST(without_uevents_a_flapping_connector_waits_for_quiet) {
  Sim s(false);
  s.run_for(10000);
  for (int i = 0; i < 6; ++i) {
    s.connected = (i % 2) == 1;
    s.run_for(1000);
  }
  s.connected = true;
  s.run_for(30000);
  CHECK(s.unplugged == 1);
  CHECK(s.rebuilds == 1);
}

TEST(an_acquire_cancels_a_pending_timed_read) {
  Sim s(true);
  s.run_for(10000);
  s.edge(false);
  s.run_for(500);
  s.edge(true);  // settling: a read is scheduled kSettleMs from now
  s.run_for(200);
  s.w.on_acquired(s.t);  // a live screen_mode set rebuilt the presenter
  const int reads = s.reads;
  s.run_for(60000);
  CHECK(s.reads == reads);  // nothing happened since: nothing to read
  CHECK(s.rebuilds == 0);
}

// A sink whose HPD pulse answers the modeset later than the guard covers:
// each rebuild would cause the next, one every few seconds, for ever.
TEST(rebuilds_are_capped_per_window) {
  Sim s(true);
  s.pulses_on_modeset = true;
  s.pulse_delay_ms = DisplayWatch::kGuardMs + 500;
  s.run_for(10000);
  s.edge(false);
  s.run_for(500);
  s.edge(true);
  s.run_for(600000);
  CHECK(s.rebuilds == DisplayWatch::kMaxRebuilds);
  CHECK(s.capped == 1);  // logged once
}

TEST(after_the_cap_window_a_fresh_replug_rebuilds_again) {
  Sim s(true);
  s.pulses_on_modeset = true;
  s.pulse_delay_ms = DisplayWatch::kGuardMs + 500;
  s.run_for(10000);
  s.edge(false);
  s.run_for(500);
  s.edge(true);
  s.run_for(30000);  // three rebuilds, then capped
  REQUIRE(s.capped == 1);
  REQUIRE(s.rebuilds == DisplayWatch::kMaxRebuilds);
  s.pulses_on_modeset = false;
  // A replug inside the window is ignored ...
  s.edge(false);
  s.run_for(500);
  s.edge(true);
  s.run_for(5000);
  CHECK(s.rebuilds == DisplayWatch::kMaxRebuilds);
  // ... one after it rebuilds.
  s.run_for(DisplayWatch::kCapWindowMs);
  s.edge(false);
  s.run_for(500);
  s.edge(true);
  s.run_for(DisplayWatch::kSettleMs + 100);
  CHECK(s.rebuilds == DisplayWatch::kMaxRebuilds + 1);
}

TEST(an_unplug_standing_at_the_cap_windows_end_is_seen) {
  Sim s(true);
  s.pulses_on_modeset = true;
  s.pulse_delay_ms = DisplayWatch::kGuardMs + 500;
  s.run_for(10000);
  s.edge(false);
  s.run_for(500);
  s.edge(true);
  s.run_for(30000);
  REQUIRE(s.capped == 1);
  s.pulses_on_modeset = false;
  const int unplugged = s.unplugged;
  s.edge(false);  // and it stays out past the window, with no further uevent
  s.run_for(DisplayWatch::kCapWindowMs);
  CHECK(s.unplugged == unplugged + 1);
  s.edge(true);
  s.run_for(DisplayWatch::kSettleMs + 100);
  CHECK(s.rebuilds == DisplayWatch::kMaxRebuilds + 1);
}

MTEST_MAIN
