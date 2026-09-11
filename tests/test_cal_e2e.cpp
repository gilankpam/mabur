// In-process loopback of the WHOLE calibration kit: the GS's CalSession
// and the drone's CalSweep, driven against each other through the real
// wire codecs, over a channel that can drop frames in either direction.
//
// Why this file exists (review finding I5): both state machines were
// thoroughly unit-tested in isolation, and the ~500 lines of glue across
// the two main.cpp files had no coverage at all. Every defect that
// actually escaped this build was of that class -- CAP_CALIBRATE never
// wired, a watchdog aborting 3 s into every sweep, a result frame sent
// exactly once over an uplink that loses 30-50% of frames, and a
// base_ref_idx read back through a parked TXAGC override. None of them
// were visible from either side alone.
//
// The harness below is deliberately a TRANSCRIPTION of the two main.cpp
// call sites, not an idealized protocol:
//
//   GS  (gs/src/main.cpp, the "Calibration uplink" block): due_cmd() is
//       gated on radio_silent(); due_result() is NOT (it must repeat into
//       the verify window -- cal_session.h); on_cal_frame() is fed from
//       the RX path; on_ack() from a Telem with flags bit6.
//   Drone (drone/src/main.cpp, the TX writer thread): parse the frame,
//       on_cmd(), send the ack Telem SYNCHRONOUSLY before the next pump(),
//       take_pending_result() -> apply_calibration() -> self-initiated
//       verify CalCmd (whose ack is discarded), pump() on a ~200 us tick.
//
// If a fix has to change one of those call sites, it has to change here
// too -- which is the point.
#include "mtest.h"

#include "cal_analysis.h"
#include "cal_apply.h"
#include "cal_plan.h"
#include "cal_session.h"
#include "cal_sweep.h"
#include "mabur/cal_wire.h"
#include "mabur/rc_proto.h"
#include "radio_tx.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {

// --- the unit under simulation ---------------------------------------

// Per-rate true PA compression wall on the simulated vtx: delivery is
// clean at and below it and collapses above it. Rates 0-2 never dip
// inside the sweep range (BPSK/QPSK on real hardware), so they take the
// no_dip path through the RSSI knee.
constexpr int kTrueWall[8] = {127, 127, 127, 95, 73, 54, 51, 49};

// The TXAGC transfer curve of docs/txagc-calibration.md: flat floor, a
// ~0.3 dB/idx ramp, then a flat ceiling. The knee at the top is what a
// no_dip row's wall is derived from.
constexpr int kTestEvm = -30;  // raw half-dB (-15 dB); nonzero = sampled

int ramp_rssi(int idx) {
  if (idx <= 28) return -80;
  if (idx <= 91) return -80 + (idx - 28) * 3 / 10;
  return -80 + (91 - 28) * 3 / 10;
}

// The minimal-but-real config apply_calibration() is pointed at -- the
// same shape tests/test_cal_apply.cpp uses, which matters because
// apply_calibration verifies its candidate with mabur::load_config(), the
// function maburd actually boots with.
const char* kConfig = R"(# a comment that must survive
[radio]
usb_vid    = 3034
power_mode = "none"      # set to offset to use rate_walls_idx

rate_walls_idx  = [91, 91, 91, 91, 73, 56, 51, 49]
legacy_wall_idx = 91
wall_margin_db  = 1.0
base_ref_idx    = 53

[fec]
symbol_size = 332
)";

std::string fresh_config(const char* name) {
  const std::string dir = std::string(MABUR_TEST_SCRATCH_DIR);
  ::mkdir(dir.c_str(), 0777);
  const std::string path = dir + "/" + name + ".toml";
  // The scratch directory outlives the process: a leftover .pre-cal from an
  // earlier run would make "no backup was created" pass or fail on history
  // rather than on this run.
  ::unlink((path + ".pre-cal").c_str());
  ::unlink((path + ".new").c_str());
  std::ofstream f(path);
  f << kConfig;
  return path;
}

std::string read_file(const std::string& p) {
  std::ifstream f(p);
  return std::string((std::istreambuf_iterator<char>(f)),
                      std::istreambuf_iterator<char>());
}

// --- drone-side seams -------------------------------------------------

// Models devourer's TX power state closely enough for the two things the
// kit reads back: an index override masks the anchor (GetTxPowerState
// reports chip truth "for the current moment"), and the anchor is only
// visible with no override live.
struct SimPower : mabur::CalSweep::PowerCtl {
  int anchor = 53;
  int override_idx = -1;
  int zero_calls = 0;
  bool set_index_override(int idx) override {
    override_idx = idx;
    return true;
  }
  bool zero_rate_diffs() override {
    ++zero_calls;
    return true;
  }
  int read_base_ref_idx() override {
    // The hazard C2 exists for: a readback taken while an override is
    // parked reports the override, not the efuse anchor.
    return override_idx >= 0 ? override_idx : anchor;
  }
};

// Collects the drone's built frames so the harness can hand them to the
// GS's RX path.
struct SimSink : mabur::FrameSink {
  std::vector<std::vector<uint8_t>> frames;
  bool send(const uint8_t* f, size_t n) override {
    frames.emplace_back(f, f + n);
    return true;
  }
};

// Frames are radiotap | dot11 | body; find the calibration payload by its
// magic rather than by a header length the builder owns.
bool payload_of(const std::vector<uint8_t>& frame,
                mabur::cal::CalFrameInfo* out) {
  for (size_t i = 0; i + mabur::cal::kCalPayloadLen <= frame.size(); ++i)
    if (mabur::cal::parse_cal_payload(frame.data() + i,
                                      mabur::cal::kCalPayloadLen, out))
      return true;
  return false;
}

// --- the pair ---------------------------------------------------------

struct Pair {
  // Loss knobs, per direction, as a percentage of frames dropped. The
  // uplink figure is the documented real one (rcf-uplink-loss: 30-50%).
  int drop_uplink_pct = 0;
  int drop_ack_pct = 0;
  int drop_downlink_pct = 0;
  // Blacks out T_CAL_RESULT alone, leaving the commands to get through --
  // the C1 failure in isolation. Applied on top of drop_uplink_pct.
  int drop_result_pct = 0;
  // Simulated per-frame link quality on the sweep itself: a frame sent
  // above its rate's true wall is lost to PA compression.
  bool model_compression = true;

  maburgs::CalSessionCfg gs_cfg;
  maburgs::CalSession gs{gs_cfg};

  mabur::CalSweep drone{mabur::CalSweepCfg{}};
  SimPower pwr;
  SimSink sink;
  mabur::RadioTx tx{sink};
  std::string cfg_path;
  double drone_margin_db = 1.0;

  std::mt19937 rng{12345};

  // Instrumentation the assertions read.
  int uplink_cmds = 0;
  int uplink_results = 0;
  int applies_ok = 0;
  int applies_failed = 0;
  // Invariant 1, watched continuously rather than asserted once: once the
  // GS has HEARD a frame of the phase the drone is sweeping, it must not
  // transmit again until that phase ends. (Stated against what the GS has
  // heard, not against the drone's private state, because that is the
  // most the GS can know -- and it is exactly the window in which a send
  // both contaminates the measurement and blanks the GS's own RX cards.)
  bool gs_talked_during_sweep = false;
  bool heard_sweep_frame_ = false;
  // Virtual time, microseconds. The drone's writer thread ticks every
  // loop_sleep_us (its sleep, 200 us in production -- drone/src/main.cpp);
  // the GS core loop every 1 ms.
  uint64_t t_us = 0;
  uint32_t loop_sleep_us = 200;
  // What one tx.send_body() costs the writer thread. The measured USB
  // round-trip is ~0.385 ms/body (dq-streaming-push notes), and charging
  // it is what makes this a test of the PACING rather than of an
  // infinitely fast loop: with the pre-fix `next_send_ms_ = now_ms + gap`
  // form, every microsecond spent here was added to the inter-frame gap
  // and accumulated across the phase's 5120 frames.
  uint32_t send_cost_us = 400;
  uint64_t next_gs_us_ = 0;
  // Realized wall-clock span of each completed drone phase, us.
  std::vector<uint64_t> phase_span_us;

  bool drop(int pct) {
    if (pct <= 0) return false;
    return static_cast<int>(rng() % 100) < pct;
  }

  uint64_t t_ms() const { return t_us / 1000; }

  explicit Pair(const char* name) : cfg_path(fresh_config(name)) {
    gs.set_peer(/*linked=*/true, /*cal_capable=*/true);
  }

  // --- drone side, transcribed from drone/src/main.cpp ----------------

  void drone_rx(const std::vector<uint8_t>& body) {
    const int type = mabur::rc::frame_type(body.data(), body.size());
    if (type == mabur::rc::T_CAL_CMD) {
      if (auto c = mabur::rc::parse_cal_cmd(body.data(), body.size())) {
        drone.on_cmd(*c, t_ms(), pwr);
        if (auto base_ref = drone.take_ack_base_ref()) {
          // The ack rides a Telem, and Telem is lost like anything else.
          if (!drop(drop_ack_pct)) gs.on_ack(c->nonce, *base_ref, t_ms());
        }
      }
    } else if (type == mabur::rc::T_CAL_RESULT) {
      if (auto r = mabur::rc::parse_cal_result(body.data(), body.size()))
        drone.on_result(*r, t_ms());
    }
  }

  // main.cpp's apply_result_and_arm_verify(), including the real
  // apply_calibration() against a real config file and the self-initiated
  // verify plan whose parameters must match gs/src/cal_plan.h.
  void drone_apply(const mabur::rc::CalResult& result) {
    const int m = static_cast<int>(std::lround(drone_margin_db * 4.0));
    mabur::CalWrite w;
    w.base_ref_idx = drone.base_ref_idx();
    for (int r = 0; r < 8; ++r) w.walls[r] = result.walls[r];
    w.legacy_wall = result.legacy_wall;

    std::string err;
    if (mabur::apply_calibration(cfg_path, w, drone_margin_db, &err) !=
        mabur::ApplyResult::Ok) {
      ++applies_failed;
      return;  // NOTE: verify is never armed -- exactly as in main.cpp
    }
    ++applies_ok;

    mabur::rc::CalCmd verify;
    verify.vtx_id = result.vtx_id;
    verify.nonce = result.nonce;
    verify.phase = mabur::cal::kPhaseVerify;
    verify.frames_per_cell = maburgs::kVerifyFrames;
    verify.settle_ms = maburgs::kSettleMs;
    verify.gap_us = maburgs::kGapUs;
    for (uint8_t r = 0; r < 8; ++r) {
      if (w.walls[r] == -1) continue;
      const int idx = w.walls[r] - m;
      if (idx < 0 || idx > 127) continue;
      verify.windows.push_back(
          {r, static_cast<uint8_t>(idx), static_cast<uint8_t>(idx), 1});
    }
    if (!verify.windows.empty()) {
      drone.on_cmd(verify, t_ms(), pwr);
      (void)drone.take_ack_base_ref();  // the drone's own command: no ack
    }
  }

  // --- the channel ----------------------------------------------------

  void deliver_sweep_frames() {
    for (const auto& f : sink.frames) {
      mabur::cal::CalFrameInfo info;
      if (!payload_of(f, &info)) continue;
      // PA compression: above the true wall the waveform is mush.
      const bool compressed =
          model_compression && info.idx > kTrueWall[info.rate];
      const bool lost = drop(drop_downlink_pct);
      if (lost) continue;
      // Two cards; card 1 hears a little less, which is also what makes
      // "best single card" a real choice rather than a formality.
      gs.on_cal_frame(0, info, ramp_rssi(info.idx), kTestEvm, /*crc_ok=*/!compressed,
                      t_ms());
      heard_sweep_frame_ = true;
      if (!compressed && (rng() % 100) < 80)
        gs.on_cal_frame(1, info, ramp_rssi(info.idx) - 3, kTestEvm, true, t_ms());
    }
    sink.frames.clear();
  }

  // --- one tick -------------------------------------------------------

  void tick() {
    if (drone.state() != mabur::CalSweep::State::Sweeping)
      heard_sweep_frame_ = false;

    // GS core loop, 1 kHz.
    if (t_us >= next_gs_us_) {
      next_gs_us_ = t_us + 1000;
      std::vector<std::vector<uint8_t>> uplink;
      if (!gs.radio_silent(t_ms())) {
        if (auto cmd = gs.due_cmd(t_ms())) {
          uplink.push_back(mabur::rc::pack_cal_cmd(*cmd));
          ++uplink_cmds;
        }
      }
      // Deliberately outside the radio_silent() gate -- see
      // gs/src/main.cpp and cal_session.h's due_result().
      if (auto res = gs.due_result(t_ms())) {
        uplink.push_back(mabur::rc::pack_cal_result(*res));
        ++uplink_results;
      }
      for (auto& body : uplink) {
        if (heard_sweep_frame_) gs_talked_during_sweep = true;
        const bool is_result =
            mabur::rc::frame_type(body.data(), body.size()) ==
            mabur::rc::T_CAL_RESULT;
        if (drop(drop_uplink_pct)) continue;
        if (is_result && drop(drop_result_pct)) continue;
        drone_rx(body);
      }
    }

    // Drone TX writer thread.
    if (auto result = drone.take_pending_result()) drone_apply(*result);
    const size_t frames_before = sink.frames.size();
    if (drone.active()) {
      const bool was_sweeping =
          drone.state() == mabur::CalSweep::State::Sweeping;
      if (was_sweeping && phase_started_us_ == 0) phase_started_us_ = t_us;
      drone.pump(t_ms(), tx, pwr);
      if (was_sweeping &&
          drone.state() != mabur::CalSweep::State::Sweeping &&
          phase_started_us_ != 0) {
        phase_span_us.push_back(t_us - phase_started_us_);
        phase_started_us_ = 0;
      }
    }
    // Every frame this pump built cost the writer thread a real USB
    // round-trip before it could loop again.
    t_us += send_cost_us * (sink.frames.size() - frames_before);
    deliver_sweep_frames();

    t_us += loop_sleep_us;  // the writer thread's sleep, drone/src/main.cpp
  }

  // Runs until the GS reaches a terminal state or the budget is spent.
  void run(uint64_t budget_ms = 300000) {
    const uint64_t end = t_us + budget_ms * 1000;
    while (t_us < end) {
      tick();
      if (gs.state() == maburgs::CalSession::State::Done ||
          gs.state() == maburgs::CalSession::State::Failed)
        break;
    }
  }

  // Keeps ticking after the GS is done, until the drone's own session
  // closes (await_next_ms, or the hard cap). The two ends finish at
  // different times by design -- the drone's sweep is open-loop -- so a
  // test that wants to assert "power restored" has to wait for it.
  void drain_drone(uint64_t budget_ms = 60000) {
    const uint64_t end = t_us + budget_ms * 1000;
    while (t_us < end && drone.active()) tick();
  }

  uint64_t phase_started_us_ = 0;
};

}  // namespace

using maburgs::CalSession;

TEST(full_cycle_over_a_clean_channel) {
  Pair p("cal_e2e_clean");
  std::string err;
  REQUIRE(p.gs.start(/*vtx_id=*/0, /*nonce=*/4242, p.t_ms(), &err));
  p.run();

  CHECK(p.gs.state() == CalSession::State::Done);
  CHECK(p.applies_ok == 1);
  CHECK(p.applies_failed == 0);

  // Invariant 1: the GS transmits nothing during a sweep phase.
  CHECK(!p.gs_talked_during_sweep);

  // The measured table. Rates 3-7 have a real dip and must land on their
  // true wall; 0-2 never dip and take the RSSI knee (88, where the ramp
  // first reaches its ceiling), flagged no_dip -- never 127.
  const auto& w = p.gs.walls();
  for (int r = 0; r < 3; ++r) {
    CHECK((w[r].flags & maburgs::kCalNoDip) != 0);
    CHECK(w[r].wall == 88);
  }
  for (int r = 3; r < 8; ++r) {
    CHECK((w[r].flags & maburgs::kCalUndetermined) == 0);
    CHECK(w[r].wall == kTrueWall[r]);
  }

  // Invariant: margin is applied exactly once, on the drone. The config
  // on disk carries the RAW walls, and power_mode flipped to offset.
  const std::string cfg = read_file(p.cfg_path);
  CHECK(cfg.find("rate_walls_idx  = [88, 88, 88, 95, 73, 54, 51, 49]") !=
        std::string::npos);
  CHECK(cfg.find("legacy_wall_idx = 88") != std::string::npos);
  CHECK(cfg.find("base_ref_idx    = 53") != std::string::npos);
  CHECK(cfg.find("power_mode = \"offset\"") != std::string::npos);
  // ...and every comment survived the line-surgical patch.
  CHECK(cfg.find("# a comment that must survive") != std::string::npos);
  // The backup is the original, byte for byte.
  CHECK(read_file(p.cfg_path + ".pre-cal") == std::string(kConfig));

  // The two ends finish at different times by design: the drone's sweep
  // is open-loop, so it closes on its own await_next_ms timer after the
  // GS has already declared the run done.
  p.drain_drone();
  // The drone left the session with the anchor restored, not a swept cell.
  CHECK(!p.drone.active());
  CHECK(p.pwr.override_idx == 53);
}

TEST(realized_phase_duration_fits_the_gs_listen_window) {
  // I4: the drone's TX writer thread now sleeps 200 us per iteration
  // instead of spinning a core for the whole 72-180 s session. Nothing in
  // the tree pinned what that sleep is allowed to cost, and it is
  // load-bearing: plan_duration_ms() is what the GS sizes both its
  // radio-silence window and its phase-end deadline from, and a phase
  // that overruns it has its top-of-range cells -- the ones the wall is
  // derived from -- analyzed as dead air.
  //
  // The budget is settle + frames*gap per cell; the drone realizes
  // settle + (frames-1)*gap plus whatever the loop costs, so the slack is
  // one gap (2 ms) per 140 ms cell and nothing more.
  Pair p("cal_e2e_timing");
  std::string err;
  REQUIRE(p.gs.start(0, 77, p.t_ms(), &err));
  p.run();
  REQUIRE(p.gs.state() == CalSession::State::Done);
  REQUIRE(p.phase_span_us.size() >= 2);   // coarse, fine, verify

  const uint32_t coarse_budget_ms =
      maburgs::plan_duration_ms(maburgs::make_coarse_plan(0, 77));
  const uint64_t coarse_realized_ms = p.phase_span_us[0] / 1000;
  // At the shipped 200 us sleep (plus a charged ~400 us USB round-trip per
  // frame) the phase lands ~0.5 s inside a 35.8 s window.
  CHECK(coarse_realized_ms + 300 <= coarse_budget_ms);
  // ...and not absurdly under, which would mean the pacing collapsed and
  // frames went out back to back, bleeding cells into each other.
  CHECK(coarse_realized_ms > coarse_budget_ms * 9 / 10);

  // Why 200 us and not the obvious 1 ms: at 1 ms the SAME run finishes
  // within a millisecond of the ceiling. It still fits -- CalSweep's
  // pacing is deadline-based (next_send_ms_ += gap_ms_), so a late wake
  // fires the next frame immediately instead of pushing every subsequent
  // deadline out -- but there is no margin left for anything else the
  // writer thread might have to do. This second run is what says so, and
  // what will notice if a future change spends that millisecond.
  Pair slow("cal_e2e_timing_slow");
  slow.loop_sleep_us = 1000;
  REQUIRE(slow.gs.start(0, 78, slow.t_ms(), &err));
  slow.run();
  REQUIRE(slow.gs.state() == CalSession::State::Done);
  REQUIRE(!slow.phase_span_us.empty());
  const uint64_t slow_ms = slow.phase_span_us[0] / 1000;
  CHECK(slow_ms <= coarse_budget_ms);
  CHECK(slow_ms + 300 > coarse_budget_ms);   // the slack really is gone
  // The table is still right, which is the outcome that actually matters:
  // an overrun surfaces as missing cells, never as an error.
  const auto& w = slow.gs.walls();
  for (int r = 3; r < 8; ++r) CHECK(w[r].wall == kTrueWall[r]);
}

TEST(a_lossy_control_plane_still_produces_the_right_table) {
  // The documented uplink: 30-50% of frames lost, and the drone's ack
  // rides a Telem that is lost too. Every control frame in the protocol
  // therefore has to survive loss on its own -- T_CAL_CMD by repeating
  // until acked, T_CAL_RESULT by repeating until the verify sweep acks it
  // implicitly, and a lost ack by the arriving sweep frames themselves
  // standing in for it. The sweep frames are clean here on purpose: this
  // is a test of the control plane, and the measurement must come out
  // bit-identical to the clean run.
  Pair p("cal_e2e_lossy_ctl");
  p.drop_uplink_pct = 40;
  p.drop_ack_pct = 40;
  std::string err;
  REQUIRE(p.gs.start(0, 909, p.t_ms(), &err));
  p.run();

  REQUIRE(p.gs.state() == CalSession::State::Done);
  CHECK(p.applies_ok == 1);
  // Repeated, not sent once -- and bounded, because the verify sweep
  // acked it rather than the cap running out.
  CHECK(p.uplink_results >= 1);
  CHECK(p.uplink_results <= 16);
  // Invariant 1 holds even with a lost ack: the arriving sweep frames are
  // the implicit ack, so the session leaves AwaitAck (and stops repeating
  // T_CAL_CMD into the live sweep) on the first frame it hears rather than
  // on a Telem that may never come.
  CHECK(!p.gs_talked_during_sweep);
  const auto& w = p.gs.walls();
  for (int r = 0; r < 3; ++r) CHECK(w[r].wall == 88);
  for (int r = 3; r < 8; ++r) CHECK(w[r].wall == kTrueWall[r]);
  CHECK(read_file(p.cfg_path).find("power_mode = \"offset\"") !=
        std::string::npos);
}

TEST(a_lossy_measurement_never_reads_a_wall_HIGHER_than_the_truth) {
  // Loss on the sweep frames themselves is a different animal: it IS the
  // measurement, and at 20 frames per coarse cell against a 90% threshold
  // even a few percent of random loss drops cells below the line and cuts
  // the first contiguous run short. That is not a defect to fix but a
  // property to pin, because the direction matters: truncating a run can
  // only move the wall DOWN (under-power, the safe direction), never up
  // past the true compression point. If this ever fails, some path is
  // extending a run across a failed cell -- landing inside the comb and
  // overdriving the PA, which is the one outcome this kit exists to
  // prevent.
  const int kKneeWall[8] = {88, 88, 88, 95, 73, 54, 51, 49};
  Pair p("cal_e2e_lossy_meas");
  p.drop_downlink_pct = 5;
  std::string err;
  REQUIRE(p.gs.start(0, 910, p.t_ms(), &err));
  p.run();

  REQUIRE(p.gs.state() == CalSession::State::Done);
  const auto& w = p.gs.walls();
  for (int r = 0; r < 8; ++r) {
    if (w[r].flags & maburgs::kCalUndetermined) continue;  // no wall claimed
    CHECK(w[r].wall <= kKneeWall[r]);
  }
  // And whatever it concluded, it either applied it or it did not -- never
  // "reported an apply that did not happen".
  CHECK(p.applies_failed == 0);
  CHECK(p.applies_ok == 1);
}

TEST(a_result_frame_that_never_arrives_reports_no_apply) {
  // The C1 failure, reproduced: the drone hears the commands and sweeps
  // fine, but every copy of T_CAL_RESULT is lost. The run must end with
  // the config UNTOUCHED and with nothing in the record that could be
  // read as a successful apply -- no verify data at all, which is what
  // maburcal keys its "written:" line on.
  Pair p("cal_e2e_result_lost");
  std::string err;
  // The commands get through; every copy of the result is lost. (Blacking
  // out the whole uplink instead would just fail the coarse ack and never
  // reach the state under test.)
  p.drop_result_pct = 100;
  REQUIRE(p.gs.start(0, 5150, p.t_ms(), &err));
  p.run();

  CHECK(p.gs.state() == CalSession::State::Done);
  CHECK(p.applies_ok == 0);
  CHECK(p.applies_failed == 0);
  // Repeated, and bounded -- it did not fill the whole verify window.
  CHECK(p.uplink_results >= 2);
  CHECK(p.uplink_results <= 16);
  // Nothing written, no backup created.
  CHECK(read_file(p.cfg_path) == std::string(kConfig));
  std::ifstream backup(p.cfg_path + ".pre-cal");
  CHECK(!backup.good());
  // The drone still gets itself home: session closed, anchor restored,
  // on its own timers with nothing further from the GS.
  p.drain_drone();
  CHECK(!p.drone.active());
  CHECK(p.pwr.override_idx == 53);
}

TEST(a_second_run_in_the_same_session_window_anchors_correctly) {
  // C2 end to end: a second `maburcal start` inside the drone's
  // await_next_ms (15 s) window reaches CalSweep::on_cmd() with the
  // previous session still live and the TXAGC parked at its last swept
  // cell. SimPower::read_base_ref_idx() reports the override in that
  // state, exactly as devourer does -- so a base_ref_idx of 53 in the
  // second run's config is the whole assertion.
  Pair p("cal_e2e_second_run");
  std::string err;
  REQUIRE(p.gs.start(0, 61, p.t_ms(), &err));
  p.run();
  REQUIRE(p.gs.state() == CalSession::State::Done);
  const std::string first = read_file(p.cfg_path);
  CHECK(first.find("base_ref_idx    = 53") != std::string::npos);

  // Deliberately do NOT let the drone's session time out: start again
  // immediately, which is the reachable case (an operator re-running
  // after seeing a flag, a GS restart, an abort).
  CHECK(p.drone.active());
  p.applies_ok = 0;
  REQUIRE(p.gs.start(0, 62, p.t_ms(), &err));
  p.run();
  CHECK(p.gs.state() == CalSession::State::Done);
  CHECK(p.applies_ok == 1);
  CHECK(read_file(p.cfg_path).find("base_ref_idx    = 53") !=
        std::string::npos);
}

MTEST_MAIN
