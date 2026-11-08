// The refresh-sync thread (HW build only): DisplayLockCore driven at the
// presenter's vblanks, its frame lengths written into the display
// controller.

#include "display_lock.h"

#include <pthread.h>
#include <sched.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <ctime>

#include "drm_presenter.h"
#include "worker_thread.h"

namespace maburplay {

namespace {

constexpr uint64_t kLogEveryUs = 5'000'000;
constexpr int kExitZeroVblanks = 5;
constexpr const char* kTakeModeWhy = "the display controller's total is not the mode's";

uint64_t mono_us() {
  timespec ts;
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1'000'000ull +
         static_cast<uint64_t>(ts.tv_nsec) / 1000;
}

// Early enough in the frame that started at vb_us for an immediate write to
// land in it: the first half, as a total set past the scan-out's line runs
// the frame to the counter's end.
bool early_in_frame(uint64_t vb_us, double period_us) {
  const uint64_t now = mono_us();
  return now >= vb_us && static_cast<double>(now - vb_us) < period_us / 2;
}

}  // namespace

DisplayLock::DisplayLock(DrmPresenter& presenter, Cfg cfg)
    : presenter_(presenter), cfg_(cfg), core_(cfg.target_margin_ms) {
  core_.set_mode(cfg.mode);
  core_.set_camera_fps(cfg.camera_fps);
  fl_ = map_frame_length(presenter_.crtc_index(), &map_why_);
}

DisplayLock::~DisplayLock() { stop(); }

const char* DisplayLock::why() const {
  if (!available()) return map_why_.c_str();
  if (take_failed_.load(std::memory_order_relaxed)) return kTakeModeWhy;
  return core_.why();
}

bool DisplayLock::engaged() const {
  return available() && took_mode_.load(std::memory_order_relaxed) &&
         core_.mode() != RefreshSync::Off;
}

void DisplayLock::start() {
  if (!available() || th_.joinable()) return;
  running_.store(true);
  th_ = start_worker_thread([this] { run_(); });
  if (cfg_.rt_priority <= 0) {
    std::fprintf(stderr, "maburplay: display-lock thread: SCHED_OTHER (rt_priority 0)\n");
    return;
  }
  sched_param sp{};
  sp.sched_priority = std::max(1, cfg_.rt_priority - 1);
  const int rc = ::pthread_setschedparam(th_.native_handle(), SCHED_FIFO, &sp);
  if (rc != 0) {
    std::fprintf(stderr,
                 "maburplay: display-lock thread: SCHED_FIFO refused (%s), running at normal "
                 "priority\n",
                 std::strerror(rc));
    return;
  }
  int policy = 0;
  sched_param got{};
  ::pthread_getschedparam(th_.native_handle(), &policy, &got);
  std::fprintf(stderr, "maburplay: display-lock thread: %s priority %d\n",
               policy == SCHED_FIFO ? "SCHED_FIFO" : "SCHED_OTHER", got.sched_priority);
}

void DisplayLock::stop() {
  running_.store(false);
  if (th_.joinable()) th_.join();
}

void DisplayLock::run_() {
  // The presenter's mode is fixed for its life, and so for this lock's.
  const ModeTiming t = presenter_.mode_timing();
  bool have_prev = false;
  uint32_t seq_prev = 0;

  while (running_.load(std::memory_order_relaxed)) {
    uint32_t seq = 0;
    uint64_t vb = 0;
    if (!presenter_.wait_vblank(&seq, &vb)) {
      // The core's last seq is left as it was: the immediate total in force
      // holds through the gap, so the next step counting the frames missed
      // at the register's lines (when fewer than 100) is what the display
      // ran.
      ::usleep(100'000);
      have_prev = false;
      continue;
    }
    if (take_failed_.load(std::memory_order_relaxed)) continue;  // idles until stop()

    if (!took_mode_.load(std::memory_order_relaxed)) {
      FrameLength::Seen seen;
      if (!fl_->take_mode(t.vtotal, t.vsync_lines, &seen)) {
        std::fprintf(stderr,
                     "maburplay: refresh sync: the display controller's total (0x%08x) is not "
                     "the mode's (%u lines) -- off\n",
                     seen.value, static_cast<unsigned>(t.vtotal));
        take_failed_.store(true);
        continue;
      }
      // A total a little off, or an immediate write still held, is ours,
      // left by a run that did not get to put it back: the core brings it
      // back (0 lines early in a frame, the release a vblank later). A mode
      // it cannot count in is put back at once.
      if (seen.held || seen.total != t.vtotal) {
        dirty_ = true;
        if (!core_.found_register(seen.total - t.vtotal, seen.held, t)) {
          fl_->restore();
          dirty_ = false;
        }
      }
      took_mode_.store(true);
    }

    // The frames since the last vblank each ran the register's lines over
    // the grid's: the vblank's shift, for the 5 s line.
    if (have_prev) {
      const uint32_t n = seq - seq_prev;
      if (n >= 1 && n < 100) moved_us_ += (core_.lines() - core_.grid_lines()) * t.line_us * n;
    }
    have_prev = true;
    seq_prev = seq;

    const DisplayLockCore::Step s = core_.step(seq, vb, mono_us(), t);
    if (s.release) {
      // The core marks itself released only once a release it asked for
      // is applied.
      fl_->restore();
      dirty_ = false;
      core_.applied(0);
    } else if (s.lines != core_.lines() && early_in_frame(vb, t.period_us)) {
      fl_->set_total(t.vtotal + s.lines);
      dirty_ = true;
      core_.applied(s.lines);
    }
    // A late wake writes nothing and tells the core nothing: it keeps the
    // register's lines as they were and asks again at the next vblank.

    log_(mono_us());
  }
  exit_restore_(t);
}

void DisplayLock::log_(uint64_t now_us) {
  const RefreshSync mode = core_.mode();
  const bool on = mode != RefreshSync::Off;
  if (on && (!engaged_said_ || mode != mode_said_)) {
    std::fprintf(stderr, "refresh-sync: on (%s)\n",
                 mode == RefreshSync::Follow ? "follow: each frame as long as the camera's"
                                             : "nudge: one line at a time");
    dir_said_ = 0;
    locked_said_ = false;
    log_t_us_ = now_us;
    moved_us_ = 0;
  }
  engaged_said_ = on;
  mode_said_ = mode;

  // Compared by text, not pointer: no allocation on this SCHED_FIFO thread.
  const char* why = core_.why();
  const bool same = why == why_said_ || (why && why_said_ && std::strcmp(why, why_said_) == 0);
  if (!same) {
    if (why) std::fprintf(stderr, "refresh-sync: %s\n", why);
    why_said_ = why;
  }
  if (!on) return;

  if (mode == RefreshSync::Nudge && !why) {
    const bool shorter = core_.nudge_shorter();
    const int dir = shorter ? -1 : 1;
    if (dir != dir_said_) {
      dir_said_ = dir;
      std::fprintf(stderr, "refresh-sync: frames made only %s\n",
                   shorter ? "shorter (the display slower)"
                           : "longer (the display no slower than the camera)");
    }
  }
  if (!locked_said_ && core_.locked()) {
    locked_said_ = true;
    std::fprintf(stderr, "refresh-sync: in step\n");
  }

  if (now_us - log_t_us_ >= kLogEveryUs) {
    std::fprintf(stderr,
                 "refresh-sync: err=%.2fms lines=%+d (grid %.1f, trim %+.2f) moved=%+.0fus/5s "
                 "cam=%.3fus\n",
                 core_.error_ms(), core_.lines(), core_.grid_lines(), core_.trim_lines(),
                 moved_us_, core_.camera_period_us());
    log_t_us_ = now_us;
    moved_us_ = 0;
  }
}

// Leaving: an immediate total still in force goes back to the mode's own
// first (0 lines, as early in a frame as the writes always are), then the
// mode's value with the immediate bit clear a frame later, so the driver's
// own writes latch at frame boundaries again.
void DisplayLock::exit_restore_(const ModeTiming& t) {
  if (!dirty_) return;
  uint32_t seq = 0;
  uint64_t vb = 0;
  for (int i = 0; i < kExitZeroVblanks && core_.lines() != 0; ++i) {
    if (!presenter_.wait_vblank(&seq, &vb)) break;
    if (!early_in_frame(vb, t.period_us)) continue;
    fl_->set_total(t.vtotal);
    core_.applied(0);
  }
  presenter_.wait_vblank(&seq, &vb);
  fl_->restore();
  dirty_ = false;
}

}  // namespace maburplay
