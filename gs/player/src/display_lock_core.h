#ifndef MABUR_PLAYER_DISPLAY_LOCK_CORE_H_
#define MABUR_PLAYER_DISPLAY_LOCK_CORE_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace maburplay {

// display.refresh_sync: how the GS bends the display's own frame length to
// hold its vblank at the camera's frames.
//  Off    -- never touches the frame length.
//  Nudge  -- one line at a time, one way only, with the refresh the camera's
//            frame rate (within 1 %): the drift brings the vblank back the
//            other way. For panels that blank on larger changes.
//  Follow -- each frame as long as one camera frame (lines added to the
//            mode, or taken off one up to 0.5 % slower than the camera), the
//            phase moved up to 2 % of a frame per frame. For panels that take
//            any frame length.
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
  static constexpr double kNudgeRateTol = 0.01, kFreeShorter = 0.005, kFreeLonger = 0.01,
                          kFollowCap = 0.02;
  static constexpr double kTrimGain = 0.002, kTrimStepMax = 2.0, kTrimMax = 3.0;
  static constexpr double kCamSteadyPpm = 100, kCamReplacePpm = 20, kCamNominalTol = 0.01;

  explicit DisplayLockCore(double target_margin_ms);

  void set_mode(RefreshSync m);  // window, trim, moving, judgements cleared; state Waiting
  void set_paused(bool paused);  // rate mismatch: release and wait
  void set_camera_fps(int fps);  // the nominal rate; a change forgets the measured period
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
  // mode's own); release: lines is 0 and the register should go back to the
  // mode's own value.
  Step step(uint32_t seq, uint64_t vb_us, uint64_t now_us, const ModeTiming& m);
  // The register now holds `lines` (the write landed). Not called = unchanged.
  void applied(int lines);

  State state() const;
  bool locked() const;
  RefreshSync mode() const;
  double error_ms() const;           // wrap(target - G), ms; 0 without a target
  int lines() const;                 // in the register now
  double grid_lines() const;         // FOLLOW: B + trim; NUDGE: 0
  double camera_period_us() const;   // the latest 10 s reading; 0 until measured
  bool nudge_shorter() const;        // NUDGE's one direction
  // Waiting's reason: "off", "rate mismatch", "waiting for frames",
  // "the refresh is not the camera's frame rate", "the camera is faster
  // than the display's mode"; nullptr otherwise.
  const char* why() const;

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

  void enter(State s);
  void clear_window();
  void clear_camera();
  void measure_camera(uint64_t now_us);
  const char* unusable(double Pm, double Pc) const;
  double g_at(uint64_t t_us) const;
  double circular_median(double P) const;
  void judge(uint64_t now_us);

  mutable std::mutex mu_;
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
};

const char* display_lock_state_name(DisplayLockCore::State s);

}  // namespace maburplay

#endif  // MABUR_PLAYER_DISPLAY_LOCK_CORE_H_
