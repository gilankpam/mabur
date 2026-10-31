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
// A replug after a successful acquire does not reliably re-light the sink
// on its own: the kernel re-applies the retained mode, yet the sink can
// stay dark while the player keeps flipping into it -- the replug path does
// not redo everything a full modeset tells the sink (likely the HDMI 2.0
// scrambling setup sent over DDC at high pixel clocks). A fresh presenter's
// full modeset re-lights it.
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
// rebuild blanks the screen for a moment. A fast unplug/replug is not
// missed for want of that: the uevent wakes a read within one housekeeping
// iteration (a few ms), while a hand replug and the kernel's own HPD
// debounce both take far longer. The settle coalesces a burst -- HPD
// bounces and DDC flaps on the connector -- into a single rebuild.
//
// No loop: a rebuild's own modeset can make a sink pulse HPD. Everything
// read during kGuardMs after an acquire (on_acquired) is therefore not an
// unplug; a disconnected read there only schedules another read at the
// guard's end, so a display that really did go away during the guard is
// still noticed once it ends. The cost is that an unplug AND replug both
// inside the guard go unseen.
class DisplayWatch {
 public:
  enum class Action {
    kNone,
    kUnplugged,  // first disconnected read of an episode: log it
    kRebuild,    // reconnected and settled: rebuild the presenter now
  };
  static constexpr uint64_t kPollMs = 1000;
  static constexpr uint64_t kSettleMs = 1000;
  static constexpr uint64_t kGuardMs = 5000;

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
