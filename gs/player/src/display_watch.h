#ifndef MABUR_PLAYER_DISPLAY_WATCH_H_
#define MABUR_PLAYER_DISPLAY_WATCH_H_

#include <cstddef>
#include <cstdint>

namespace maburplay {

// Whether one kernel uevent datagram (NETLINK_KOBJECT_UEVENT: an
// "action@devpath" header, then KEY=VALUE fields, all NUL-separated) is a
// DRM hotplug: it carries both SUBSYSTEM=drm and HOTPLUG=1 as whole fields.
bool is_drm_hotplug_uevent(const char* buf, size_t len);

// Notices that the display a live presenter drives was unplugged and came
// back, and says when to rebuild the presenter for it.
//
// After a replug the sink can stay dark although KMS re-applies the
// retained mode and the player keeps flipping into it; a fresh presenter's
// full modeset brings the picture back. So a replug under a live presenter
// is answered with a rebuild.
//
// Pure logic, driven by the housekeeping thread: each iteration it asks
// due() whether to read the connector's status (a cheap no-probe read) and
// feeds the answer to on_status(). With hotplug uevents the read happens
// only when one arrived or a timer below is due; without them (no netlink
// socket) it is a 1 s poll.
//
// The rebuild rule: the connector must have been READ as disconnected and
// then as connected, and stay quiet (no uevent, no disconnected read) for
// kSettleMs, before one rebuild runs. A uevent alone never rebuilds -- the
// kernel also sends HOTPLUG=1 for changes that leave the sink lit (an EDID
// re-read, a property change, another connector on the card), and every
// rebuild blanks the screen for a moment. Each uevent wakes a read within
// one housekeeping iteration (a few ms), so an unplug/replug is missed only
// if both edges land between two reads (with uevents, inside that one
// iteration; polled, inside one kPollMs). The settle coalesces a burst --
// HPD bounces and DDC flaps on the connector -- into a single rebuild.
//
// A rebuild's own modeset can make a sink pulse HPD. Two limits keep that
// from rebuilding over and over:
//  - The guard: everything read during kGuardMs after an acquire
//    (on_acquired) is not an unplug. It runs from the acquire, not from the
//    modeset, which can come later (a rebuild with no splash modesets at
//    the first present()), so it covers a pulse that follows the modeset
//    promptly, not every pulse.
//  - The cap: at most kMaxRebuilds rebuilds per kCapWindowMs. One more
//    within the window is refused (kCapped, logged once) and the watch goes
//    blind until the window ends; after that only a fresh disconnected then
//    connected read rebuilds. A sink that pulses HPD late on every modeset
//    therefore costs kMaxRebuilds rebuilds and then stays as it is.
// A disconnected read while blind (guard or cap window) only schedules
// another read at the blind spell's end, so a display that really went away
// is still noticed once it ends; an unplug AND replug both inside one go
// unseen.
class DisplayWatch {
 public:
  enum class Action {
    kNone,
    kUnplugged,  // first disconnected read of an episode: log it
    kRebuild,    // reconnected and settled: rebuild the presenter now
    kCapped,     // would rebuild, but the cap is reached: log it
  };
  static constexpr uint64_t kPollMs = 1000;
  static constexpr uint64_t kSettleMs = 1000;
  static constexpr uint64_t kGuardMs = 5000;
  static constexpr int kMaxRebuilds = 3;
  static constexpr uint64_t kCapWindowMs = 60000;

  // `uevents`: whether hotplug uevents arrive (a netlink socket is open).
  // Without them the status is polled every kPollMs.
  explicit DisplayWatch(bool uevents);

  // A presenter was just (re)built: forget any episode and start the guard.
  void on_acquired(uint64_t now_ms);

  // Whether to read the connector's status now. `uevent`: a DRM hotplug
  // uevent arrived since the last call. Call once per iteration while a
  // presenter is live; nothing is due before the first on_acquired().
  bool due(uint64_t now_ms, bool uevent);

  // The status read due() asked for. `connected` is false only for a
  // definite disconnected reading.
  Action on_status(uint64_t now_ms, bool connected);

 private:
  bool uevents_;
  bool acquired_ = false;
  uint64_t acquired_ms_ = 0;
  uint64_t next_poll_ms_ = 0;
  bool check_pending_ = false;  // a timed read is scheduled at check_at_ms_
  uint64_t check_at_ms_ = 0;
  bool armed_ = false;           // read disconnected since the acquire
  bool last_connected_ = true;   // the previous read's answer
  uint64_t last_activity_ms_ = 0;  // newest uevent, disconnected read or reconnect
  // The newest kMaxRebuilds rebuilds this watch asked for, oldest at
  // rebuild_next_ once all are filled; and the end of a cap window.
  uint64_t rebuild_ms_[kMaxRebuilds] = {};
  int rebuild_count_ = 0;
  int rebuild_next_ = 0;
  uint64_t capped_until_ms_ = 0;
};

// A non-blocking NETLINK_KOBJECT_UEVENT socket on the kernel's uevent
// group. drain() empties it without ever blocking.
class HotplugSocket {
 public:
  HotplugSocket() = default;
  HotplugSocket(const HotplugSocket&) = delete;
  HotplugSocket& operator=(const HotplugSocket&) = delete;
  ~HotplugSocket();

  bool open();
  bool is_open() const { return fd_ >= 0; }
  // Reads every queued datagram; true when one was a DRM hotplug, or when
  // the socket overflowed (a dropped uevent may have been one).
  bool drain();

 private:
  int fd_ = -1;
};

}  // namespace maburplay

#endif  // MABUR_PLAYER_DISPLAY_WATCH_H_
