#pragma once
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "display_lock_core.h"
#include "frame_length.h"

namespace maburplay {

class DrmPresenter;

// display.refresh_sync's actuator: a thread that wakes at each of the
// presenter's vblanks, steps the DisplayLockCore and writes the frame
// length it asks for into the display controller (FrameLength; on the
// RK356x the VOP2's DSP_VTOTAL_VS_END through /dev/mem). One per presenter:
// built after its init(), stopped before its retire(), never carried over.
//
// Writes: the core's lines go in as an immediate write of vtotal + lines,
// only in the first half of the frame that started at the vblank -- a total
// set while the scan-out is already past it runs the frame to the counter's
// end, one very long frame. A late wake skips the write and leaves the core
// believing the register unchanged. A release goes back to the mode's own
// value with the immediate bit clear, so it latches at a frame boundary.
//
// The mode's own register value is read once, at the first vblank: a total
// that is not this mode's (another program's, or a mode the driver set
// otherwise) leaves the lock disengaged for this presenter's life.
//
// Threads: the setters and note_* come from any thread and go straight to
// the core, which is thread-safe; the snapshot getters read the core.
class DisplayLock {
 public:
  struct Cfg {
    RefreshSync mode = RefreshSync::Off;
    int target_margin_ms = 0;
    int camera_fps = 0;
    // The release thread's SCHED_FIFO priority; this thread asks for one
    // less (at least 1), below the release thread it must never delay.
    // 0 = SCHED_OTHER.
    int rt_priority = 0;
  };

  DisplayLock(DrmPresenter& presenter, Cfg cfg);
  ~DisplayLock();  // stop()
  DisplayLock(const DisplayLock&) = delete;
  DisplayLock& operator=(const DisplayLock&) = delete;

  // The frame-length register mapped; when not, why() names the reason.
  bool available() const { return fl_ != nullptr; }
  // Unavailable: the mapping's reason. Otherwise why the lock is not
  // holding the vblank now (the mode's total not taken, or the core's
  // Waiting reason), or nullptr.
  const char* why() const;

  void start();  // no-op when !available() or already started
  // Joins the thread; a register it left off the mode's own is brought
  // back (0 lines written for up to 5 vblanks, one more waited, then the
  // mode's own value). Safe when never started.
  void stop();

  // The register is ours and refresh sync is on: available, the mode's
  // total taken at the first vblank, and the mode not Off.
  bool engaged() const;

  void set_mode(RefreshSync m) { core_.set_mode(m); }
  void set_paused(bool paused) { core_.set_paused(paused); }
  void set_camera_fps(int fps) {
    fps_.store(fps, std::memory_order_relaxed);
    core_.set_camera_fps(fps);
  }
  void note_margin(double margin_us, uint64_t t_us) { core_.on_margin(margin_us, t_us); }
  void note_pts(uint32_t pts_us) { core_.on_pts(pts_us); }
  void on_discontinuity() { core_.on_discontinuity(); }

  // Snapshot for the logs and the OSD.
  DisplayLockCore::State state() const { return core_.state(); }
  bool locked() const { return core_.locked(); }
  RefreshSync mode() const { return core_.mode(); }
  double error_ms() const { return core_.error_ms(); }
  int lines() const { return core_.lines(); }
  double grid_lines() const { return core_.grid_lines(); }
  double camera_period_us() const { return core_.camera_period_us(); }

 private:
  void run_();
  void log_(const ModeTiming& t, uint64_t now_us);
  void exit_restore_(const ModeTiming& t);

  DrmPresenter& presenter_;
  const Cfg cfg_;
  DisplayLockCore core_;
  std::unique_ptr<FrameLength> fl_;
  std::string map_why_;
  std::thread th_;
  std::atomic<bool> running_{false};
  std::atomic<bool> took_mode_{false};
  std::atomic<bool> take_failed_{false};
  std::atomic<int> fps_;

  // The thread's own state.
  bool dirty_ = false;  // written off the mode's own value since the last restore
  double moved_us_ = 0;  // the vblank's shift over the grid since the last 5 s line
  bool engaged_said_ = false;
  RefreshSync mode_said_ = RefreshSync::Off;
  // The reason last said, "" for none; starts as one the core never gives,
  // so the first is said.
  std::string why_said_ = "-";
  int dir_said_ = 0;  // NUDGE: 1 longer, -1 shorter, 0 not said
  bool locked_said_ = false;
  uint64_t log_t_us_ = 0;
};

}  // namespace maburplay
