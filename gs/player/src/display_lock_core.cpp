#include "display_lock_core.h"

#include <algorithm>
#include <cmath>

#include "line_step.h"

namespace maburplay {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr size_t kMarks = 8;
constexpr size_t kPendingCap = 1024;
constexpr size_t kCapsCap = 4096;
constexpr uint64_t kWrap = 1ull << 32, kHalfWrap = 1ull << 31;

double wrap(double x, double P) {  // into [-P/2, P/2)
  x = std::fmod(x, P);
  if (x >= P / 2) x -= P;
  if (x < -P / 2) x += P;
  return x;
}
double mod(double x, double P) {
  x = std::fmod(x, P);
  return x < 0 ? x + P : x;
}

}  // namespace

bool parse_refresh_sync(const std::string& s, RefreshSync* out) {
  for (RefreshSync m : {RefreshSync::Off, RefreshSync::Nudge, RefreshSync::Follow}) {
    if (s == refresh_sync_name(m)) {
      *out = m;
      return true;
    }
  }
  return false;
}

const char* refresh_sync_name(RefreshSync m) {
  switch (m) {
    case RefreshSync::Off: return "off";
    case RefreshSync::Nudge: return "nudge";
    case RefreshSync::Follow: return "follow";
  }
  return "?";
}

const char* display_lock_state_name(DisplayLockCore::State s) {
  switch (s) {
    case DisplayLockCore::State::Waiting: return "wait";
    case DisplayLockCore::State::Acquiring: return "acq";
    case DisplayLockCore::State::Locked: return "lock";
  }
  return "?";
}

DisplayLockCore::DisplayLockCore(double target_margin_ms) : target_us_(target_margin_ms * 1000.0) {
  med_buf_.reserve(kPendingCap);
  gaps_.reserve(kCapsCap);
}

void DisplayLockCore::set_mode(RefreshSync m) {
  std::lock_guard<PiMutex> lk(mu_);
  mode_ = m;
  trim_ = 0;
  moving_ = false;
  clear_window();
  enter(State::Waiting);
  why_ = m == RefreshSync::Off ? "off" : "waiting for frames";
}

void DisplayLockCore::set_paused(bool paused) {
  std::lock_guard<PiMutex> lk(mu_);
  paused_ = paused;
}

void DisplayLockCore::set_camera_fps(int fps) {
  std::lock_guard<PiMutex> lk(mu_);
  if (fps == fps_) return;
  fps_ = fps;
  clear_camera();
  cam_reading_ = cam_cand_ = cam_use_ = 0;
  cam_timer_ = false;  // the next reading a whole window of the new rate on
  trim_ = 0;
  // The samples were taken modulo the old camera frame and the judgements
  // made on them: acquire afresh. G and the register are physical and stay.
  moving_ = false;
  clear_window();
  enter(State::Waiting);
  why_ = mode_ == RefreshSync::Off ? "off" : "waiting for frames";
}

void DisplayLockCore::on_margin(double margin_us, uint64_t t_us) {
  std::lock_guard<PiMutex> lk(mu_);
  if (pending_.size() >= kPendingCap) pending_.erase(pending_.begin());
  pending_.push_back({margin_us, t_us});
}

void DisplayLockCore::on_pts(uint32_t pts_us) {
  std::lock_guard<PiMutex> lk(mu_);
  uint64_t v = pts_hi_ + pts_us;
  // A step back of more than half the range is the 32-bit stamp wrapping.
  if (have_pts_ && v < pts_last_ && pts_last_ - v > kHalfWrap) {
    pts_hi_ += kWrap;
    v += kWrap;
  }
  if (!caps_.empty() && v <= caps_.back()) {
    // A repeat or a straggler is skipped; a step back further than the
    // window is a new timeline (the drone restarted), measured afresh.
    if (caps_.back() - v <= kCamWindowUs) return;
    caps_.clear();
  }
  have_pts_ = true;
  pts_last_ = v;
  caps_.push_back(v);
  while (caps_.front() + kCamWindowUs < v || caps_.size() > kCapsCap) caps_.pop_front();
}

void DisplayLockCore::on_discontinuity() {
  std::lock_guard<PiMutex> lk(mu_);
  clear_window();
  clear_camera();  // the stamps are of the old timeline; the period still holds
  enter(State::Waiting);
  if (mode_ != RefreshSync::Off) why_ = "waiting for frames";
}

void DisplayLockCore::applied(int lines) {
  std::lock_guard<PiMutex> lk(mu_);
  released_ = lines == 0 && asked_release_;
  lines_ = lines;
}

bool DisplayLockCore::found_register(int lines, bool held, const ModeTiming& m) {
  std::lock_guard<PiMutex> lk(mu_);
  if (m.period_us <= 0 || m.line_us <= 0 || m.vtotal <= 0) return false;
  if (m.vtotal != mode_vtotal_ || m.line_us != mode_line_) start_mode(m);
  lines_ = lines;
  released_ = lines == 0 && !held;
  asked_release_ = false;
  return true;
}

// A new display mode: the modeset wrote the register's own value, and the
// shift is counted in other lines. Start over.
void DisplayLockCore::start_mode(const ModeTiming& m) {
  mode_vtotal_ = m.vtotal;
  mode_line_ = m.line_us;
  G_ = 0;
  have_seq_ = false;
  marks_.clear();
  lines_ = 0;
  released_ = true;
  moving_ = false;
  clear_window();
  enter(State::Waiting);
}

void DisplayLockCore::enter(State s) {
  state_ = s;
  in_count_ = out_count_ = 0;
}

void DisplayLockCore::clear_window() {
  pending_.clear();
  win_.clear();
  have_target_ = false;
  err_us_ = 0;
}

void DisplayLockCore::clear_camera() {
  caps_.clear();
  have_pts_ = false;
  pts_hi_ = pts_last_ = 0;
}

// The span of the stamps over the frames in it, counted from the median gap
// (a lost frame is a gap of two): a stamp's error over some 600 frames.
// Every kCamEveryUs; a reading within kCamNominalTol of the nominal rate is
// this camera's, and two such in a row within kCamSteadyPpm replace the
// period in use when they differ from it by more than kCamReplacePpm (a
// single reading over a window with a step in it is never adopted).
void DisplayLockCore::measure_camera(uint64_t now_us) {
  if (!cam_timer_) {
    cam_timer_ = true;
    cam_t_us_ = now_us;
    return;
  }
  if (now_us - cam_t_us_ < kCamEveryUs) return;
  cam_t_us_ = now_us;
  double pc = 0;
  if (caps_.size() >= kCamMinStamps) {
    std::vector<uint64_t>& g = gaps_;
    g.clear();
    for (size_t i = 1; i < caps_.size(); ++i) g.push_back(caps_[i] - caps_[i - 1]);
    std::nth_element(g.begin(), g.begin() + g.size() / 2, g.end());
    const double med = static_cast<double>(g[g.size() / 2]);
    const double span = static_cast<double>(caps_.back() - caps_.front());
    const double n = med > 0 ? std::round(span / med) : 0;
    if (n > 0) pc = span / n;
  }
  if (pc > 0) cam_reading_ = pc;
  const bool cand = pc > 0 && fps_ > 0 && std::fabs(pc * fps_ - 1e6) <= kCamNominalTol * 1e6;
  if (cand && cam_cand_ > 0 && std::fabs(pc - cam_cand_) < kCamSteadyPpm * 1e-6 * pc &&
      std::fabs(pc - cam_use_) > kCamReplacePpm * 1e-6 * pc) {
    cam_use_ = pc;
    trim_ = 0;
  }
  cam_cand_ = cand ? pc : 0;
}

// Why the lock cannot hold the vblank now; nullptr when it can. NUDGE
// moves the vblank one line a frame against a drift of Pc - Pm a frame, so
// it pulls in at (line - drift) a frame: a refresh more than kNudgeRateTol
// off the camera's frame rate is another rate, and a drift past
// kNudgeReachLines of a line (on the period in use) leaves too little to
// pull in with: a line or more never comes back, just under one takes
// minutes. FOLLOW can take frames at most kFreeShorter shorter than the
// mode's.
const char* DisplayLockCore::unusable(double Pm, double Pc, double line) const {
  if (mode_ == RefreshSync::Off) return "off";
  if (paused_) return "rate mismatch";
  if (mode_ == RefreshSync::Nudge && std::fabs(Pc - Pm) > kNudgeRateTol * Pc)
    return "the refresh is not the camera's frame rate";
  if (mode_ == RefreshSync::Nudge && std::fabs(Pc - Pm) > kNudgeReachLines * line)
    return kWhyBeyondReach;
  if (mode_ == RefreshSync::Follow && Pc < Pm * (1 - kFreeShorter))
    return "the camera is faster than the display's mode";
  return nullptr;
}

// The shift as of the last vblank at or before t_us (the oldest kept if
// none): the reference the frame's margin was taken against.
double DisplayLockCore::g_at(uint64_t t_us) const {
  for (auto it = marks_.rbegin(); it != marks_.rend(); ++it)
    if (it->vb_us <= t_us) return it->g;
  return marks_.empty() ? G_ : marks_.front().g;
}

// The median of the samples on the circle of period P: pivoted on their mean
// angle, so a window straddling the 0/P seam is one continuous run.
double DisplayLockCore::circular_median(double P) {
  double sx = 0, sy = 0;
  for (const Sample& s : win_) {
    const double a = 2 * kPi * s.u / P;
    sx += std::cos(a);
    sy += std::sin(a);
  }
  const double pivot = std::atan2(sy, sx) * P / (2 * kPi);
  std::vector<double>& d = med_buf_;
  d.clear();
  for (const Sample& s : win_) d.push_back(wrap(s.u - pivot, P));
  std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
  return mod(pivot + d[d.size() / 2], P);
}

void DisplayLockCore::judge(uint64_t now_us) {
  if (now_us - judged_us_ < kJudgeUs) return;
  judged_us_ = now_us;
  const double ae = std::fabs(err_us_) / 1000.0;
  if (state_ == State::Acquiring) {
    if (ae < kLockInMs) {
      if (++in_count_ >= kLockInJudgements) enter(State::Locked);
    } else {
      in_count_ = 0;
    }
  } else if (state_ == State::Locked) {
    if (ae > kLockOutMs) {
      if (++out_count_ >= kLockOutJudgements) enter(State::Acquiring);
    } else {
      out_count_ = 0;
    }
  }
}

DisplayLockCore::Step DisplayLockCore::step(uint32_t seq, uint64_t vb_us, uint64_t now_us,
                                            const ModeTiming& m) {
  std::lock_guard<PiMutex> lk(mu_);
  const double Pm = m.period_us, line = m.line_us;
  if (Pm <= 0 || line <= 0 || m.vtotal <= 0) return {lines_, false};

  if (m.vtotal != mode_vtotal_ || line != mode_line_) start_mode(m);

  measure_camera(now_us);
  const double Pc = cam_use_ > 0 ? cam_use_ : fps_ > 0 ? 1e6 / fps_ : Pm;
  const bool follow = mode_ == RefreshSync::Follow;
  const double vt = static_cast<double>(m.vtotal);

  // FOLLOW's grid, held in the register: one camera frame, B lines over
  // the mode's, fewer by up to kFreeShorter where the camera is a little
  // faster.
  double B = 0;
  if (follow) B = std::clamp(Pc / line - vt + trim_, -kFreeShorter * vt, kFreeLonger * vt);
  B_ = B;
  // NUDGE has no grid in the register, but its reference still turns at the
  // camera's period: the frames arrive every Pc, so against a reference at
  // Pm every sample drifts by Pc - Pm a frame and the window's median lags
  // that drift by half the window (200 us at 59.988 fps on 60 Hz). Counted
  // against Bref lines a frame (the camera's frame over the mode's), the
  // samples stand still. Until the period is adopted Pc is the nominal and
  // the lag remains.
  const double Bref = follow ? B : Pc / line - vt;
  const double Pt = (vt + Bref) * line;
  const bool shorter = Pm > Pc * (1 + 1e-6);
  if (shorter != shorter_) moving_ = false;
  shorter_ = shorter;

  // The frames since the last vblank each ran lines_ (a write holds until
  // the next), Bref of them the reference's own.
  if (have_seq_) {
    const uint32_t n = seq - seq_prev_;
    if (n >= 1 && n < 100) G_ = mod(G_ + (lines_ - Bref) * line * n, Pt);
  }
  have_seq_ = true;
  seq_prev_ = seq;
  marks_.push_back({vb_us, G_});
  if (marks_.size() > kMarks) marks_.pop_front();

  if (const char* why = unusable(Pm, Pc, line)) {
    clear_window();
    enter(State::Waiting);
    why_ = why;
    moving_ = false;
    // Off the mode's own: 0 lines first, an immediate write early in a
    // frame like any other, and the release only once that landed. The
    // release clears the immediate bit wherever in the frame the thread
    // is, so it must find the frame already at the mode's total.
    asked_release_ = lines_ == 0 && !released_;
    return {0, asked_release_};
  }

  // The margins since the last vblank, onto the reference.
  const bool fresh = !pending_.empty();
  for (const Pending& p : pending_) {
    const double delta = wrap(target_us_ - p.margin_us, Pt);
    win_.push_back({p.t_us, mod(g_at(p.t_us) + delta, Pt)});
  }
  pending_.clear();
  while (!win_.empty() && win_.front().t_us + kWindowUs < vb_us) win_.pop_front();

  if (fresh) {
    last_sample_us_ = now_us;
    if (state_ == State::Waiting) enter(State::Acquiring);
  } else if (state_ != State::Waiting && now_us - last_sample_us_ >= kQuietUs) {
    clear_window();
    enter(State::Waiting);
  }
  why_ = state_ == State::Waiting ? "waiting for frames" : nullptr;

  have_target_ = win_.size() >= kMinSamples;
  err_us_ = have_target_ ? wrap(circular_median(Pt) - G_, Pt) : 0;
  if (have_target_) judge(now_us);

  int want = 0;
  if (!have_target_) {
    want = follow ? static_cast<int>(std::lround(B)) : 0;
    moving_ = false;
  } else if (follow) {
    // As long as a camera frame, and the error brought back in whole lines
    // on top -- never shorter than the grid's own (a pull-in goes the
    // long way round instead), nor shorter than the mode's when the grid
    // is longer.
    const int c = std::fabs(err_us_) < 0.5 * line ? 0 : static_cast<int>(std::lround(err_us_ / line));
    const int cap = static_cast<int>(std::floor(kFollowCap * vt));
    want = std::clamp(static_cast<int>(std::lround(B)) + c, -cap, cap);
    want = std::max(want, std::min(0, static_cast<int>(std::floor(B))));
    // Frames that keep coming out longer or shorter than the grid's mean
    // the camera's frame is not quite the period in use: learn that slowly
    // into the grid, once in step and a little per frame (a pull-in's
    // lines would wind it up).
    if (state_ == State::Locked)
      trim_ = std::clamp(trim_ + kTrimGain * std::clamp(want - B, -kTrimStepMax, kTrimStepMax),
                         -kTrimMax, kTrimMax);
  } else {
    const LineStep s = line_step(err_us_, Pt, line, shorter, moving_);
    moving_ = s.moving;
    want = s.lines;
  }
  asked_release_ = want == 0 && lines_ == 0 && !released_;
  return {want, asked_release_};
}

DisplayLockCore::State DisplayLockCore::state() const {
  std::lock_guard<PiMutex> lk(mu_);
  return state_;
}

bool DisplayLockCore::locked() const {
  std::lock_guard<PiMutex> lk(mu_);
  return state_ == State::Locked;
}

RefreshSync DisplayLockCore::mode() const {
  std::lock_guard<PiMutex> lk(mu_);
  return mode_;
}

double DisplayLockCore::error_ms() const {
  std::lock_guard<PiMutex> lk(mu_);
  return have_target_ ? err_us_ / 1000.0 : 0.0;
}

int DisplayLockCore::lines() const {
  std::lock_guard<PiMutex> lk(mu_);
  return lines_;
}

double DisplayLockCore::grid_lines() const {
  std::lock_guard<PiMutex> lk(mu_);
  return mode_ == RefreshSync::Follow ? B_ : 0.0;
}

double DisplayLockCore::trim_lines() const {
  std::lock_guard<PiMutex> lk(mu_);
  return mode_ == RefreshSync::Follow ? trim_ : 0.0;
}

double DisplayLockCore::camera_period_us() const {
  std::lock_guard<PiMutex> lk(mu_);
  return cam_reading_;
}

bool DisplayLockCore::nudge_shorter() const {
  std::lock_guard<PiMutex> lk(mu_);
  return shorter_;
}

const char* DisplayLockCore::why() const {
  std::lock_guard<PiMutex> lk(mu_);
  return state_ == State::Waiting ? why_ : nullptr;
}

bool DisplayLockCore::beyond_reach() const {
  std::lock_guard<PiMutex> lk(mu_);
  return state_ == State::Waiting && why_ == kWhyBeyondReach;
}

}  // namespace maburplay
