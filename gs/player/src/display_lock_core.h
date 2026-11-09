#ifndef MABUR_PLAYER_DISPLAY_LOCK_CORE_H_
#define MABUR_PLAYER_DISPLAY_LOCK_CORE_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "pi_mutex.h"

namespace maburplay {

// display.refresh_sync: how the GS bends the display's own frame length to
// hold its vblank at the camera's frames.
//  Off    -- the mode's own frame length: nothing written, except to bring
//            back a register found off the mode's own value.
//  Nudge  -- one line a frame, one way only; the drift between refresh and
//            camera brings the vblank back the other way. Holds only while
//            the camera drifts less than a line a frame against the refresh
//            (kNudgeReachLines of one). For panels that blank on larger
//            changes.
//  Follow -- each frame as long as one camera frame: lines added to the
//            mode's total for a slower camera, taken off it (up to 0.5 %)
//            for a faster one, and the phase brought back with up to 2 % of
//            a frame more or less on top. For panels that take any frame
//            length.
enum class RefreshSync : uint8_t { Off, Nudge, Follow };
bool parse_refresh_sync(const std::string& s, RefreshSync* out);  // "off"/"nudge"/"follow"
const char* refresh_sync_name(RefreshSync m);

// The display mode the lock steps: total lines, sync width, one line's time
// and the mode's own frame period (µs).
struct ModeTiming {
  int vtotal = 0;
  int vsync_lines = 0;
  double line_us = 0;
  double period_us = 0;
};

// The refresh-sync controller: from the regulator's release margins, where
// the vblank should be, and from that the display frame length to write at
// each vblank. Pure logic: no I/O, no clock; the caller owns the register.
//
// Reference frame: G (µs, mod one camera frame Pt) is the vblank shift the
// written lines have applied so far, over a reference that turns once per
// camera frame -- so the frames' samples stand still on it. FOLLOW holds
// that frame length in the register (its grid); NUDGE keeps the mode's and
// counts the drift. Each margin m becomes a sample
// u = G_t + wrap(M - m): where G should have been for that frame to leave
// the target margin M (positive: the release later, frames longer). The
// target is the circular median of the last 2 s of samples, the error
// e = wrap(target - G); NUDGE closes it with line_step(), FOLLOW with whole
// lines on top of the grid's.
//
// The camera's period comes from the decoded frames' capture stamps: a
// 10 s reading every 10 s, adopted after two in a row agree (until then the
// nominal fps names it). It decides NUDGE's direction (shorter when the
// screen is the slower) and FOLLOW's grid.
//
// Threads: the inputs (on_margin, on_pts) and the setters may come from any
// thread, step/applied from the display thread; one mutex guards all of it.
class DisplayLockCore {
 public:
  enum class State : uint8_t { Waiting, Acquiring, Locked };

  static constexpr uint64_t kWindowUs = 2'000'000, kQuietUs = 2'000'000, kJudgeUs = 1'000'000,
                            kCamWindowUs = 10'000'000, kCamEveryUs = 10'000'000;
  static constexpr size_t kMinSamples = 50, kCamMinStamps = 500;
  static constexpr double kLockInMs = 1.0, kLockOutMs = 3.0;
  static constexpr int kLockInJudgements = 3, kLockOutJudgements = 2;
  static constexpr double kNudgeRateTol = 0.01, kNudgeReachLines = 0.8, kFreeShorter = 0.005,
                          kFreeLonger = 0.01, kFollowCap = 0.02;
  static constexpr double kTrimGain = 0.002, kTrimStepMax = 2.0, kTrimMax = 3.0;
  static constexpr double kCamSteadyPpm = 100, kCamReplacePpm = 20, kCamNominalTol = 0.01;
  // NUDGE's Waiting reason when the camera drifts past its reach.
  static constexpr const char* kWhyBeyondReach = "the camera drifts more than a line a frame";
  static constexpr const char* kWhyNotRate = "the refresh is not the camera's frame rate";
  static constexpr const char* kWhyFaster = "the camera is faster than the display's mode";
  static constexpr const char* kWhyMeasuring = "measuring the camera's frame rate";

  explicit DisplayLockCore(double target_margin_ms);

  // Window, trim, moving, judgements cleared; state Waiting. From Off to a
  // display mode the camera's period is measured afresh as well.
  void set_mode(RefreshSync m);
  void set_paused(bool paused);  // rate mismatch: release and wait
  void set_camera_fps(int fps);  // the nominal rate; a change forgets the measured period
  // A period measured by the lock before this one (a display rebuild at the
  // same camera rate), put in use at once: within kCamNominalTol of the
  // nominal rate, and only with a display mode on (under the camera the
  // drone is steered off its own rate). False, and nothing changed,
  // otherwise. The measurement goes on and replaces it as usual.
  bool seed_camera_period(double pc);
  double camera_period_in_use() const;  // the adopted (or seeded) period; 0 = none
  // One frame's release margin (µs) from the regulator, taken at t_us.
  void on_margin(double margin_us, uint64_t t_us);
  void on_pts(uint32_t pts_us);  // every decoded frame's capture stamp, µs mod 2^32
  void on_discontinuity();       // window and judgements cleared; G and lines kept

  struct Step {
    int lines;
    bool release;
  };
  // One vblank: seq/vb_us from the display, now_us the caller's clock.
  // lines: what the register should hold from this frame on (0 = the
  // mode's own); a lines other than lines() is a write of it, the same is
  // none. release: the register holds 0 lines already and should go back
  // to the mode's own value, immediate bit clear -- a register off 0 lines
  // is first asked for 0 (a write early in a frame), the release a step
  // later.
  Step step(uint32_t seq, uint64_t vb_us, uint64_t now_us, const ModeTiming& m);
  // The register now holds `lines` (the write landed). Not called = unchanged.
  // applied(0) after a release asked is the release; after a write of 0, the
  // register holds 0 lines as an immediate write, still to be released.
  void applied(int lines);
  // The register as the display thread found it before the first step:
  // `lines` over m's total, `held` an immediate write still in force (a
  // run that did not put it back). The core counts it as the register's
  // and brings it back as it leaves any. False when m cannot be counted in.
  bool found_register(int lines, bool held, const ModeTiming& m);

  State state() const;
  bool locked() const;
  RefreshSync mode() const;
  double error_ms() const;           // wrap(target - G), ms; 0 without a target
  int lines() const;                 // in the register now
  double grid_lines() const;         // FOLLOW: B + trim; NUDGE: 0
  double trim_lines() const;         // FOLLOW: the learnt trim in B; NUDGE: 0
  double camera_period_us() const;   // the latest 10 s reading; 0 until measured
  bool nudge_shorter() const;        // NUDGE's one direction
  // Waiting's reason, a string literal: "off", "rate mismatch", "waiting
  // for frames", kWhyNotRate, kWhyMeasuring, kWhyBeyondReach,
  // kWhyFaster;
  // nullptr otherwise.
  const char* why() const;
  // Waiting with kWhyBeyondReach: NUDGE cannot follow the camera's drift.
  bool beyond_reach() const;
  // The display cannot hold the phase at this camera rate: Waiting with
  // kWhyNotRate, kWhyBeyondReach or kWhyFaster, that reason; nullptr
  // otherwise (measuring, waiting for frames, paused, off, or holding).
  const char* cannot_hold() const;

 private:
  struct Mark {  // a vblank and the shift G as of it
    uint64_t vb_us;
    double g;
  };
  struct Pending {
    double margin_us;
    uint64_t t_us;
  };
  struct Sample {
    uint64_t t_us;
    double u;
  };

  void start_mode(const ModeTiming& m);
  void enter(State s);
  void clear_window();
  void clear_camera();
  void measure_camera(uint64_t now_us);
  const char* unusable(double Pm, double Pc, double line) const;
  double g_at(uint64_t t_us) const;
  double circular_median(double P);
  void judge(uint64_t now_us);

  // Priority-inheriting: the SCHED_FIFO display thread steps under it while
  // the decode output and housekeeping feed it at normal priority.
  mutable PiMutex mu_;
  const double target_us_;
  RefreshSync mode_ = RefreshSync::Off;
  bool paused_ = false;
  int fps_ = 0;

  // The display mode the shift is counted in; a new one starts over.
  int mode_vtotal_ = 0;
  double mode_line_ = 0;

  double G_ = 0;
  bool have_seq_ = false;
  uint32_t seq_prev_ = 0;
  std::deque<Mark> marks_;  // the last 8 vblanks

  int lines_ = 0;           // in the register now
  bool released_ = true;    // the register at the mode's own value
  bool asked_release_ = false;

  std::vector<Pending> pending_;  // margins not yet turned into samples
  std::deque<Sample> win_;
  std::vector<double> med_buf_;  // circular_median's scratch, kept to spare the step an allocation
  uint64_t last_sample_us_ = 0;
  bool have_target_ = false;
  double err_us_ = 0;

  double B_ = 0;            // grid lines over the mode's (FOLLOW)
  double trim_ = 0;         // FOLLOW: lines learnt on top of the period's
  bool shorter_ = false;
  bool moving_ = false;

  State state_ = State::Waiting;
  const char* why_ = "off";
  int in_count_ = 0, out_count_ = 0;
  uint64_t judged_us_ = 0;

  // Camera stamps, unwrapped to 64 bits, the last kCamWindowUs of them.
  std::deque<uint64_t> caps_;
  bool have_pts_ = false;
  uint64_t pts_hi_ = 0, pts_last_ = 0;
  bool cam_timer_ = false;
  uint64_t cam_t_us_ = 0;
  double cam_reading_ = 0;  // the latest reading
  double cam_cand_ = 0;     // the last reading, if it was a candidate
  double cam_use_ = 0;      // the period in use; 0 = the nominal
  std::vector<uint64_t> gaps_;  // measure_camera's scratch
};

const char* display_lock_state_name(DisplayLockCore::State s);

}  // namespace maburplay

#endif  // MABUR_PLAYER_DISPLAY_LOCK_CORE_H_
