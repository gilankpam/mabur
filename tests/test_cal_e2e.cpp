// In-process loopback of the WHOLE calibration kit: the GS's CalSession and
// the drone's CalSweep, driven against each other over the real mabur
// MAVLink dialect codec (cal::encode / decode_all), with a simulated radio
// in between, until the drone's real config file holds the result.
//
// Both engines are unit-tested in isolation; what only this file covers is
// the glue across the two main.cpp files -- ack before pump, the
// capacity-plan-after-accepted-ack rule, the APPLIED cache, one backup per
// run, the verify park both ends must agree on, the run-pair latch that
// keeps a run alive after the drone's session lapses mid-sweep, and the
// power restore however the run ends.
//
// The harness below is deliberately a TRANSCRIPTION of the two call sites,
// not an idealized protocol:
//
//   GS  (gs/src/main.cpp core loop): set_peer every tick; due_mav() ->
//       cal::encode(.., 255, 190, kChanGsLocal) -> the tunnel's cal lane,
//       never gated on radio_silent(); inbound packets -> feed_cal_packet();
//       stamped frames -> on_cal_frame(card, info, rssi, crc_ok,
//       24 + body + 4, now).
//   Drone (drone/src/main.cpp): the MAV thread's check_mav verdict (kOk /
//       kOkLatched under the open run / rejected) -> decode_all -> the cal
//       queue; the TX writer thread drains the queue (latched SWEEPs only
//       for the open run's nonce; begin_cal + no_agg for a starting
//       non-capacity sweep; on_cmd; take_ack; program_for_cal only for an
//       accepted starting capacity sweep; the ack sent before the next
//       pump; the ledger answering APPLY repeats), then
//       take_pending_apply -> apply_calibration (backup per the ledger) ->
//       APPLIED -> arm_verify(verify_parks(..)), then mirrors
//       cal_active/cal_sweeping, pumps, sends SweepDone, and once the run
//       is closed restores operating power through CalRestoreOwed.
//
// If a fix has to change one of those call sites, it has to change here
// too -- which is the point.
#include "mtest.h"

#include "cal_apply.h"
#include "cal_link.h"
#include "cal_picker.h"
#include "cal_session.h"
#include "cal_sweep.h"
#include "cal_wiring.h"
#include "config.h"
#include "mav_component.h"
#include "mabur/cal_mav.h"
#include "mabur/cal_park.h"
#include "mabur/cal_proto.h"
#include "mabur/cal_wire.h"
#include "mabur/mav.h"
#include "mabur/profile.h"
#include "radio_tx.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <map>
#include <random>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>
#include <variant>
#include <vector>

namespace {

namespace cal = mabur::cal;
using maburgs::CalSession;
using State = maburgs::CalSession::State;

// --- the simulated unit ------------------------------------------------

// Per-width true PA compression wall, RELATIVE to the chip's anchor:
// delivery is clean at and below it and every frame above it arrives
// corrupt. 127 = this rate never compresses inside the swept range (the
// top cell is kRailRel), so it must come out no_dip at the rail. The 40 MHz
// walls sit lower on mcs5-7, as on every unit measured.
constexpr int kTrueWallRel20[8] = {127, 127, 127, 127, 45, 23, 22, 21};
constexpr int kTrueWallRel40[8] = {127, 127, 127, 127, 45, 19, 18, 16};
int true_wall(uint8_t bw, int r) { return bw == 40 ? kTrueWallRel40[r] : kTrueWallRel20[r]; }
// What the analysis must write for a true wall: the wall itself, or the
// rail for a rate that never dips.
int expected_wall(uint8_t bw, int r) { return std::min(true_wall(bw, r), maburgs::kRailRel); }

// Per-(mcs, bw) true air efficiency at saturation -- deliberately different
// from the fixture's values so a write is visible.
constexpr double kTrueEff20[8] = {0.91, 0.86, 0.82, 0.79, 0.73, 0.71, 0.69, 0.66};
constexpr double kTrueEff40[8] = {0.81, 0.79, 0.78, 0.72, 0.70, 0.68, 0.66, 0.62};
double true_eff(uint8_t bw, int r) { return bw == 40 ? kTrueEff40[r] : kTrueEff20[r]; }

// The chip's own TXAGC anchor (mcs7_index with diffs zeroed).
constexpr int kAnchor = 53;

// A unit calibrated at 20 MHz once, under STBC only, and never at 40: it
// has rate_walls_rel_20 and both efficiency tables but no
// rate_walls_rel_40 / cal_flags_40 line, so a 40 MHz apply has to INSERT
// them. The 20 MHz walls and both efficiency tables differ from what the
// simulated unit measures, and cal_flags_20 differs from what the sweep
// flies (ldpc+stbc), so every rewrite is visible.
const char* kFixture = R"(# a comment that must survive
[radio]
usb_vid    = 3034
power_mode = "none"      # set to offset to use the calibrated walls

rate_walls_rel_20 = [63, 63, 63, 63, 40, 20, 18, 16]
cal_flags_20      = "stbc"
legacy_wall_rel   = 63
wall_margin_db    = 1.0

[fec]
symbol_size = 332

[air_clock]
shed_ms       = 25
efficiency_20 = [0.93, 0.88, 0.84, 0.8, 0.76, 0.76, 0.78, 0.76]
efficiency_40 = [0.77, 0.75, 0.75, 0.75, 0.74, 0.77, 0.76, 0.75]
body_us       = 0
)";
constexpr double kFixtureEff20[8] = {0.93, 0.88, 0.84, 0.8, 0.76, 0.76, 0.78, 0.76};
constexpr double kFixtureEff40[8] = {0.77, 0.75, 0.75, 0.75, 0.74, 0.77, 0.76, 0.75};

std::string read_file(const std::string& p) {
  std::ifstream f(p);
  return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

bool file_exists(const std::string& p) {
  struct stat st;
  return ::stat(p.c_str(), &st) == 0;
}

// The drone's /etc/mabur.toml, in the build tree's scratch directory.
// Unique per process and test; every sibling the apply path can leave
// behind is removed on both ends so a leftover .pre-cal from an earlier run
// can never pass or fail an assertion on history.
struct DroneConfigFile {
  std::string path;
  explicit DroneConfigFile(const char* name) {
    ::mkdir(MABUR_TEST_SCRATCH_DIR, 0777);
    path = std::string(MABUR_TEST_SCRATCH_DIR) + "/cal_e2e-" + name + "-" +
           std::to_string(::getpid()) + ".toml";
    clean();
    std::ofstream f(path);
    f << kFixture;
  }
  ~DroneConfigFile() {
    clean();
    ::unlink(path.c_str());
  }
  void clean() {
    ::unlink((path + ".pre-cal").c_str());
    ::unlink((path + ".new").c_str());
  }
};

// --- drone-side seams --------------------------------------------------

// devourer's TX power state as far as the kit can see it: a flat index
// override that masks the anchor while live, and a rate-diff table that is
// either the operating plan or zeroed.
struct SimPower : mabur::CalSweep::PowerCtl {
  int override_idx = -1;  // -1 = no override
  bool diffs_zeroed = false;
  bool set_index_override(int idx) override {
    override_idx = idx;
    return true;
  }
  bool zero_rate_diffs() override {
    diffs_zeroed = true;
    return true;
  }
  int read_anchor_idx() override {
    // GetTxPowerState reports chip truth for the current moment: a parked
    // override reads back as itself, not the anchor.
    return override_idx >= 0 ? override_idx : kAnchor;
  }
  // The index a frame actually flies at right now.
  int on_air_idx() const { return override_idx >= 0 ? override_idx : kAnchor; }
};

// Every frame RadioTx builds (radiotap | dot11 | body), for the harness to
// put on the simulated air.
struct SimSink : mabur::FrameSink {
  std::vector<std::vector<uint8_t>> frames;
  bool send(const uint8_t* f, size_t n) override {
    frames.emplace_back(f, f + n);
    return true;
  }
};

struct Kit;

// The capacity actuator: per-(mcs, bw) true capacity. blast() is where the
// simulated clock moves during a dwell -- the TX writer is blocked inside
// it for the whole dwell, while the GS's core loop keeps running and the
// drone's cal queue keeps filling.
struct FakeCapacityCtl : mabur::CalSweep::CapacityCtl {
  Kit* kit = nullptr;
  bool set_point(uint8_t mcs, uint8_t bw) override;
  mabur::BlastResult blast(const uint8_t* frame, size_t len, uint32_t dwell_ms) override;
  // (mcs, bw) points where the drone cannot feed the air: blocked_pct low,
  // fewer frames offered than the air would carry.
  std::vector<std::pair<uint8_t, uint8_t>> tx_bound;
  int points = 0;
};

// A pinned channel: no measure round runs.
struct FakePicker : maburgs::CalPicker {
  Mode mode() const override { return Mode::Pinned; }
  uint8_t op() const override { return 36; }
  void begin_measure(uint64_t) override {}
  std::optional<uint8_t> measured_best() override { return op(); }
  bool order_relocate(uint8_t) override { return false; }
  std::optional<bool> relocate_settled() override { return std::nullopt; }
  void cancel() override {}
};

// One calibration message from the MAV thread to the TX writer
// (main.cpp's CalInbound).
struct CalInbound {
  cal::CalMsg msg;
  bool latched = false;
};

// --- the pair ------------------------------------------------------------

struct Kit {
  // Knobs.
  int drop_gs_to_drone_pct = 0;   // control plane, GS -> drone
  int drop_drone_to_gs_pct = 0;   // control plane, drone -> GS
  // The drone's published session lapses 3 s into each sweep (the GS is
  // radio-silent for the whole sweep, longer than the link failsafe).
  bool lapse_session = false;

  // Simulated clock, ms. Both loops tick once per ms; a capacity dwell
  // advances it from inside blast().
  uint64_t t = 0;
  std::mt19937 rng{12345};

  // --- GS ---
  FakePicker picker;
  CalSession gs{maburgs::CalSessionCfg{}, nullptr, &picker};

  // --- drone ---
  DroneConfigFile file;
  mabur::Config boot_cfg;
  double live_margin_db = 1.0;
  mabur::CalSweep sweep;
  SimPower pwr;
  SimSink sink;
  mabur::RadioTx tx{sink};
  FakeCapacityCtl cap;
  mabur::CalApplyLedger ledger;
  mabur::CalRestoreOwed restore_owed;
  std::deque<CalInbound> cal_queue;
  bool cal_active = false;     // main.cpp's mirrored atomics
  bool cal_sweeping = false;
  bool session_ok = true;      // RcAgent::has_published_session()
  uint64_t sweeping_since = 0;

  // LivePowerTable, as far as the TX writer drives it.
  bool cal_owns_table = false;
  std::vector<uint8_t> programmed_for_cal;  // program_for_cal(bw) calls
  int no_agg_calls = 0;
  int set_plans_calls = 0;
  int restores = 0;
  int reload_failures = 0;

  // Instrumentation.
  int applies_ok = 0, applies_failed = 0;
  int latched_sweeps = 0, latched_applies = 0, latched_aborts = 0;
  int latch_drops = 0, rejected = 0, non_cal_frames = 0;
  int gs_msgs_sent = 0;
  int dropped_gs_to_drone = 0, dropped_drone_to_gs = 0;
  // The relative index each verify frame was stamped with, per [bw][rate]
  // (kWallLeave = none seen; -1000 = seen at more than one index).
  std::map<std::pair<uint8_t, uint8_t>, int> verify_idx_on_air;
  bool gs_talked_while_silent = false;
  size_t frames_on_air = 0;
  std::vector<std::string> apply_log;  // "<bw> walls|eff ok|fail"

  static mabur::CalSweepCfg make_cal_cfg(const mabur::Config& c) {
    mabur::CalSweepCfg s;
    s.ldpc = true;  // radio.ldpc as the link flies it
    s.capacity_frame_len = 13 + static_cast<size_t>(c.fec.blocks_per_body[1]) *
                                    (16 + static_cast<size_t>(c.fec.symbol_size[1]));
    return s;
  }

  explicit Kit(const char* name)
      : file(name),
        boot_cfg(mabur::load_config(file.path)),
        live_margin_db(boot_cfg.radio.wall_margin_db),
        sweep(make_cal_cfg(boot_cfg)) {
    cap.kit = this;
    gs.set_peer(true, true);
  }

  bool drop(int pct) { return pct > 0 && static_cast<int>(rng() % 100) < pct; }

  // --- GS core loop tick ----------------------------------------------

  void gs_tick() {
    gs.set_peer(true, true);
    if (auto m = gs.due_mav(t)) {
      ++gs_msgs_sent;
      if (gs.radio_silent(t)) gs_talked_while_silent = true;
      const auto bytes = cal::encode(*m, 255, 190, mabur::mav::kChanGsLocal);
      if (drop(drop_gs_to_drone_pct))
        ++dropped_gs_to_drone;
      else
        drone_mav_rx(bytes);
    }
  }

  // --- drone MAV thread: check_mav's verdict, then the cal queue ------

  void drone_mav_rx(const std::vector<uint8_t>& payload) {
    if (!cal::all_packets_are_cal(payload.data(), payload.size())) {
      ++non_cal_frames;  // would go to the parameter component, not here
      return;
    }
    bool latched;
    if (session_ok) {
      latched = false;  // kOk
    } else if (cal_active) {
      latched = true;   // kOkLatched: verified under the open run's pair
    } else {
      ++rejected;       // kRejected: no session and no run to latch onto
      return;
    }
    for (auto& msg : cal::decode_all(payload.data(), payload.size()))
      cal_queue.push_back(CalInbound{std::move(msg), latched});
  }

  void send_cal(const cal::CalMsg& msg) {
    const auto bytes = cal::encode(msg, mabur::MavComponent::sys_id(),
                                   mabur::MavComponent::comp_id(), mabur::mav::kChanDroneRx);
    if (drop(drop_drone_to_gs_pct))
      ++dropped_drone_to_gs;
    else
      maburgs::feed_cal_packet(gs, bytes.data(), bytes.size(), t);
  }

  // --- drone TX writer, LivePowerTable and config reload stand-ins ----

  mabur::Config reload_config() {
    try {
      return mabur::load_config(file.path);
    } catch (const std::exception&) {
      ++reload_failures;
      return boot_cfg;
    }
  }

  void cal_no_agg() { ++no_agg_calls; }

  void restore_operating_power() {
    (void)reload_config();
    // replace(plans, bw, zero_offset, clear_override): the rung's plan back
    // on the diffs, then the override cleared, in one hold.
    pwr.diffs_zeroed = false;
    pwr.override_idx = -1;
    cal_owns_table = false;
    ++restores;
  }

  void apply_and_arm_verify(const cal::ApplyCmd& a) {
    const double margin_db = live_margin_db;
    const mabur::CalWrite w = mabur::cal_write_for(a, sweep.stamp_flags());
    std::string err;
    const mabur::ApplyResult ar =
        mabur::apply_calibration(file.path, w, margin_db, ledger.want_backup(a.nonce), &err);
    cal::Applied out;
    out.nonce = a.nonce;
    out.seq = a.seq;
    out.bw = a.bw;
    out.ok = ar == mabur::ApplyResult::Ok;
    if (out.ok) {
      (void)reload_config();
      ++set_plans_calls;
      ++applies_ok;
    } else {
      out.reason = err.substr(0, 49);
      ++applies_failed;
    }
    apply_log.push_back(std::to_string(a.bw) + (a.has_walls ? " walls" : "") +
                        (a.has_eff ? " eff" : "") + (out.ok ? " ok" : " fail: " + err));
    ledger.record(out);
    send_cal(out);
    if (out.ok && a.has_walls) {
      cal_no_agg();
      sweep.arm_verify(mabur::verify_parks(a, margin_db), a.bw, t);
    }
  }

  void drone_tick() {
    CalInbound in;
    while (!cal_queue.empty()) {
      in = std::move(cal_queue.front());
      cal_queue.pop_front();
      if (const auto* c = std::get_if<cal::SweepCmd>(&in.msg)) {
        if (in.latched && !(sweep.active() && c->nonce == sweep.run_nonce())) {
          ++latch_drops;
          continue;
        }
        if (in.latched) ++latched_sweeps;
        const bool starting = sweep.starts_sweep(*c);
        if (starting) {
          cal_owns_table = true;  // power_table.begin_cal()
          restore_owed.note_device_touched();
          if (c->kind != cal::Kind::Capacity) cal_no_agg();
        }
        sweep.on_cmd(*c, t, pwr);
        const auto ack = sweep.take_ack();
        if (starting && c->kind == cal::Kind::Capacity && ack &&
            ack->result == cal::AckResult::Accepted)
          programmed_for_cal.push_back(c->bw);
        if (ack) send_cal(*ack);
        (void)sweep.take_refused();
      } else if (const auto* ap = std::get_if<cal::ApplyCmd>(&in.msg)) {
        if (in.latched) ++latched_applies;
        if (auto hit = ledger.cached(ap->nonce, ap->seq))
          send_cal(*hit);
        else
          sweep.on_apply(*ap, t);
      } else if (const auto* ab = std::get_if<cal::Abort>(&in.msg)) {
        if (in.latched) ++latched_aborts;
        sweep.on_abort(*ab, t, pwr);
      }
    }

    if (auto a = sweep.take_pending_apply()) apply_and_arm_verify(*a);

    cal_active = sweep.active();
    const bool sweeping = sweep.state() == mabur::CalSweep::State::Sweeping;
    if (sweeping && !cal_sweeping) sweeping_since = t;
    cal_sweeping = sweeping;
    // The failsafe: 3 s of a radio-silent GS into a sweep and the drone
    // drops its published session. Nothing re-pairs while the run is open
    // -- the GS is silent through every sweep and verify and only sends
    // cal traffic in between -- so from here on every cal message rides
    // the run-pair latch, until the run closes and the link re-pairs.
    if (lapse_session && cal_sweeping && t - sweeping_since >= 3000) session_ok = false;

    if (sweep.active()) {
      sweep.pump(t, tx, pwr, cap);
      put_sweep_frames_on_air();
      if (auto d = sweep.take_done()) send_cal(*d);
      return;
    }
    if (restore_owed.take_due(sweep.active())) restore_operating_power();
    if (lapse_session) session_ok = true;
  }

  // --- the air ----------------------------------------------------------

  // Power/verify frames: one stamped frame per send_body, heard by both GS
  // cards. Above the rate's true wall (at the index actually programmed,
  // not the one stamped) the waveform is compressed and both cards get it
  // CRC-bad -- maburgs keeps corrupted frames, so they still reach
  // on_cal_frame as corrupt.
  void put_sweep_frames_on_air() {
    for (const auto& f : sink.frames) {
      REQUIRE(f.size() >= 4);
      const size_t rt_len = static_cast<size_t>(f[2]) | (static_cast<size_t>(f[3]) << 8);
      const size_t hdr = rt_len + 26;  // radiotap | QoS-data dot11 (24 + QoS control)
      REQUIRE(f.size() > hdr);
      const uint8_t* body = f.data() + hdr;
      const size_t body_len = f.size() - hdr;
      cal::CalFrameInfo info;
      if (!cal::parse_cal_payload(body, body_len, &info)) continue;
      ++frames_on_air;
      if (info.kind == cal::Kind::Verify) {
        const auto key = std::make_pair(info.bw, info.rate);
        const auto it = verify_idx_on_air.find(key);
        if (it == verify_idx_on_air.end())
          verify_idx_on_air[key] = info.idx;
        else if (it->second != info.idx)
          it->second = -1000;
      }
      const int rel = pwr.on_air_idx() - kAnchor;
      const bool compressed = rel > true_wall(info.bw, info.rate);
      // A plausible RSSI: the TXAGC ramp (~0.3 dB/idx) well clear of the
      // saturation threshold; card 1 hears a little less.
      const int rssi = -80 + std::clamp(pwr.on_air_idx() - 28, 0, 63) * 3 / 10;
      gs.on_cal_frame(0, info, rssi, !compressed, 24 + body_len + 4, t);
      gs.on_cal_frame(1, info, rssi - 3, !compressed, 24 + body_len + 4, t);
    }
    sink.frames.clear();
  }

  // --- driving ------------------------------------------------------------

  void step() {
    gs_tick();
    drone_tick();
    ++t;
  }

  bool gs_terminal() const { return gs.state() == State::Done || gs.state() == State::Failed; }

  void run(uint64_t budget_ms = 300000) {
    const uint64_t end = t + budget_ms;
    while (t < end && !gs_terminal()) step();
    if (gs.state() != State::Done)
      std::printf("  run ended %s at %.1f s: %s\n", maburgs::cal_state_name(gs.state()), t / 1000.0,
                  gs.snapshot().fail_reason.c_str());
  }

  // The GS's closing Abort, and (if that is lost) the drone's own await
  // timer, close the drone's run after the GS is done; the restore follows.
  void drain(uint64_t budget_ms = 30000) {
    const uint64_t end = t + budget_ms;
    while (t < end && (sweep.active() || cal_owns_table)) step();
  }

  bool start(std::vector<cal::SweepSpec> sweeps, uint32_t nonce = 4242) {
    std::string err;
    const bool ok = gs.start(nonce, std::move(sweeps), t, &err);
    if (!ok) std::printf("start refused: %s\n", err.c_str());
    return ok;
  }

  mabur::Config loaded() const { return mabur::load_config(file.path); }
  std::string read() const { return read_file(file.path); }
};

bool FakeCapacityCtl::set_point(uint8_t mcs, uint8_t bw) {
  (void)mcs;
  (void)bw;
  // DeviceCapacityCtl clears the override so the width's plan is on the air.
  kit->pwr.override_idx = -1;
  return true;
}

mabur::BlastResult FakeCapacityCtl::blast(const uint8_t* frame, size_t len, uint32_t dwell_ms) {
  ++points;
  cal::CalFrameInfo info;
  REQUIRE(cal::parse_cal_payload(frame, len, &info));
  REQUIRE(info.kind == cal::Kind::Capacity);
  const size_t air_bytes = 24 + len + 4;
  const double nominal_bps =
      mabur::rc::phy_rate_mbps({mabur::rc::PhyMode::HT, info.rate, info.bw}) * 1e6;
  // Frames the air carries in the dwell at this point's true efficiency.
  const uint32_t air_frames = static_cast<uint32_t>(std::lround(
      true_eff(info.bw, info.rate) * (dwell_ms / 1000.0) * nominal_bps / (air_bytes * 8.0)));
  const bool bound =
      std::find(tx_bound.begin(), tx_bound.end(), std::make_pair(info.rate, info.bw)) !=
      tx_bound.end();
  // A tx_bound point: the drone's CPU/USB, not the air, is the limit.
  const uint32_t delivered = bound ? air_frames * 6 / 10 : air_frames;
  mabur::BlastResult r;
  r.offered = delivered;
  r.blocked_pct = bound ? 20 : 85;
  // The dwell, a millisecond at a time: frames land on both cards as they
  // are sent and the GS's core loop runs alongside.
  uint32_t sent = 0;
  for (uint32_t ms = 0; ms < dwell_ms; ++ms) {
    const uint32_t due = static_cast<uint32_t>(uint64_t{delivered} * (ms + 1) / dwell_ms);
    for (; sent < due; ++sent) {
      kit->gs.on_cal_frame(0, info, -58, true, air_bytes, kit->t);
      kit->gs.on_cal_frame(1, info, -61, true, air_bytes, kit->t);
    }
    kit->gs_tick();
    ++kit->t;
  }
  return r;
}

const std::vector<cal::SweepSpec> kFourSweeps = {{cal::Kind::Power, 20},
                                                 {cal::Kind::Capacity, 20},
                                                 {cal::Kind::Power, 40},
                                                 {cal::Kind::Capacity, 40}};

void check_walls(const std::array<int, 8>& got, uint8_t bw) {
  for (int r = 0; r < 8; ++r) {
    // Err low, never high -- and on a clean channel, exactly on the truth.
    CHECK(got[r] <= expected_wall(bw, r));
    CHECK(got[r] == expected_wall(bw, r));
  }
}

void check_eff(const std::array<double, 8>& got, uint8_t bw) {
  for (int r = 0; r < 8; ++r) CHECK(std::fabs(got[r] - true_eff(bw, r)) <= 0.01);
}

}  // namespace

TEST(four_sweeps_write_both_width_tables) {
  Kit k("four");
  REQUIRE(k.start(kFourSweeps));
  k.run();
  REQUIRE(k.gs.state() == State::Done);
  CHECK(k.applies_ok == 4);
  CHECK(k.applies_failed == 0);
  CHECK(!k.gs_talked_while_silent);
  // The capacity sweeps fly each width's production plan, programmed
  // after the accepted ack -- once per capacity sweep, never on a repeat.
  CHECK((k.programmed_for_cal == std::vector<uint8_t>{20, 40}));
  // Aggregation off for each power sweep and each verify that follows its
  // apply; the drone's plans re-read from the file after every apply.
  CHECK(k.no_agg_calls == 4);
  CHECK(k.set_plans_calls == 4);

  // The GS's own record agrees with the truth, per width.
  const auto snap = k.gs.snapshot();
  for (int wi = 0; wi < 2; ++wi) {
    const uint8_t bw = wi ? 40 : 20;
    const auto& w = snap.width[wi];
    CHECK(w.have_walls && w.have_eff && w.applied);
    for (int r = 0; r < 8; ++r) {
      CHECK(w.walls[r].wall == expected_wall(bw, r));
      CHECK(((w.walls[r].flags & maburgs::kCalNoDip) != 0) == (true_wall(bw, r) == 127));
    }
  }

  // The file, through the loader maburd boots with.
  const mabur::Config c = k.loaded();
  CHECK(c.radio.power_mode == "offset");
  REQUIRE(c.radio.rate_walls_rel_20.has_value());
  REQUIRE(c.radio.rate_walls_rel_40.has_value());
  check_walls(*c.radio.rate_walls_rel_20, 20);
  check_walls(*c.radio.rate_walls_rel_40, 40);
  CHECK(c.radio.cal_flags_20 == "ldpc+stbc");
  CHECK(c.radio.cal_flags_40 == "ldpc+stbc");
  CHECK(c.radio.legacy_wall_rel == 63);
  check_eff(c.air_clock.efficiency_20, 20);
  check_eff(c.air_clock.efficiency_40, 40);
  // The 40 MHz keys were inserted (the loader's default for cal_flags_40 is
  // the same string, so the text is what proves the line exists), and the
  // rest of the file -- comments included -- survived.
  const std::string text = k.read();
  CHECK(text.find("rate_walls_rel_40") != std::string::npos);
  CHECK(text.find("cal_flags_40") != std::string::npos);
  CHECK(text.find("# a comment that must survive") != std::string::npos);
  CHECK(text.find("usb_vid    = 3034") != std::string::npos);
  // One backup for the run: the file as it was before the first apply,
  // byte for byte -- a later apply re-backing-up would hold a mid-run state.
  CHECK(read_file(k.file.path + ".pre-cal") == std::string(kFixture));

  // The GS's closing Abort closes the drone's run at once; operating power
  // comes back from the file the run just wrote.
  k.drain();
  CHECK(!k.sweep.active());
  CHECK(k.restores == 1);
  CHECK(k.pwr.override_idx == -1);
  CHECK(!k.pwr.diffs_zeroed);
  CHECK(k.reload_failures == 0);
}

TEST(walls_only_20_leaves_the_40_table_alone) {
  Kit k("walls20");
  REQUIRE(k.start({{cal::Kind::Power, 20}}));
  k.run();
  REQUIRE(k.gs.state() == State::Done);
  CHECK(k.applies_ok == 1);
  CHECK(k.programmed_for_cal.empty());

  const mabur::Config c = k.loaded();
  CHECK(c.radio.power_mode == "offset");
  REQUIRE(c.radio.rate_walls_rel_20.has_value());
  check_walls(*c.radio.rate_walls_rel_20, 20);
  CHECK(c.radio.cal_flags_20 == "ldpc+stbc");
  // Nothing at 40 was measured, so nothing at 40 is written -- not even
  // an inserted line -- and neither efficiency table moves.
  CHECK(!c.radio.rate_walls_rel_40.has_value());
  const std::string text = k.read();
  CHECK(text.find("rate_walls_rel_40") == std::string::npos);
  CHECK(text.find("cal_flags_40") == std::string::npos);
  for (int r = 0; r < 8; ++r) {
    CHECK(c.air_clock.efficiency_20[r] == kFixtureEff20[r]);
    CHECK(c.air_clock.efficiency_40[r] == kFixtureEff40[r]);
  }
  k.drain();
  CHECK(k.restores == 1);
}

TEST(a_tx_bound_point_keeps_the_old_efficiency_entry) {
  Kit k("txbound");
  k.cap.tx_bound = {{5, 20}};
  REQUIRE(k.start({{cal::Kind::Capacity, 20}}));
  k.run();
  REQUIRE(k.gs.state() == State::Done);
  CHECK(k.applies_ok == 1);
  CHECK(k.cap.points == 8);
  CHECK((k.gs.snapshot().width[0].cap[5].flags & maburgs::kCalTxBound) != 0);

  const mabur::Config c = k.loaded();
  for (int r = 0; r < 8; ++r) {
    if (r == 5)
      CHECK(c.air_clock.efficiency_20[r] == kFixtureEff20[r]);  // left alone
    else
      CHECK(std::fabs(c.air_clock.efficiency_20[r] - true_eff(20, r)) <= 0.01);
    CHECK(c.air_clock.efficiency_40[r] == kFixtureEff40[r]);
  }
  // A capacity-only run writes no walls: power_mode, the wall table and its
  // flags stay as they were.
  CHECK(c.radio.power_mode == "none");
  REQUIRE(c.radio.rate_walls_rel_20.has_value());
  CHECK((*c.radio.rate_walls_rel_20 == std::array<int, 8>{63, 63, 63, 63, 40, 20, 18, 16}));
  CHECK(c.radio.cal_flags_20 == "stbc");
  k.drain();
  CHECK(k.restores == 1);
  CHECK(k.pwr.override_idx == -1);
}

TEST(session_lapse_mid_run_does_not_stop_the_apply) {
  Kit k("lapse");
  k.lapse_session = true;
  REQUIRE(k.start(kFourSweeps));
  k.run();
  REQUIRE(k.gs.state() == State::Done);
  CHECK(k.applies_ok == 4);
  // The lapse really happened, and everything after it came in on the
  // latch: three later SWEEPs, every APPLY (and its repeats), the Abort.
  CHECK(k.latched_sweeps >= 3);
  CHECK(k.latched_applies >= 4);
  CHECK(k.latch_drops == 0);
  CHECK(k.rejected == 0);

  const mabur::Config c = k.loaded();
  CHECK(c.radio.power_mode == "offset");
  REQUIRE(c.radio.rate_walls_rel_20.has_value());
  REQUIRE(c.radio.rate_walls_rel_40.has_value());
  check_walls(*c.radio.rate_walls_rel_20, 20);
  check_walls(*c.radio.rate_walls_rel_40, 40);
  check_eff(c.air_clock.efficiency_20, 20);
  check_eff(c.air_clock.efficiency_40, 40);

  k.drain();
  CHECK(k.latched_aborts == 1);
  CHECK(!k.sweep.active());
  CHECK(k.restores == 1);
}

TEST(abort_mid_sweep_restores_power_and_writes_nothing) {
  Kit k("abort");
  REQUIRE(k.start(kFourSweeps));
  // 5 s into the first (power, 20 MHz) sweep: the drone is parked at some
  // swept cell with its diffs zeroed.
  while (k.t < 5000) k.step();
  REQUIRE(k.gs.state() == State::Sweep);
  REQUIRE(k.sweep.state() == mabur::CalSweep::State::Sweeping);
  REQUIRE(k.pwr.diffs_zeroed);
  REQUIRE(k.pwr.override_idx >= 0);

  k.gs.abort("operator abort");
  CHECK(k.gs.state() == State::Idle);
  for (int i = 0; i < 50; ++i) k.step();
  CHECK(!k.sweep.active());
  CHECK(k.restores == 1);
  CHECK(k.pwr.override_idx == -1);
  CHECK(!k.pwr.diffs_zeroed);
  // Nothing more goes on the air, and nothing was written: no apply, no
  // backup, the file byte for byte.
  const size_t frames = k.frames_on_air;
  for (int i = 0; i < 20000; ++i) k.step();
  CHECK(k.frames_on_air == frames);
  CHECK(k.applies_ok == 0);
  CHECK(k.applies_failed == 0);
  CHECK(k.read() == std::string(kFixture));
  CHECK(!file_exists(k.file.path + ".pre-cal"));
  CHECK(k.gs.snapshot().fail_reason == "operator abort");
}

TEST(the_run_fits_the_budget) {
  {
    Kit k("budget4");
    REQUIRE(k.start(kFourSweeps));
    k.run();
    REQUIRE(k.gs.state() == State::Done);
    std::printf("  four sweeps: Done at %.1f s\n", k.t / 1000.0);
    CHECK(k.t < 100000);
  }
  {
    Kit k("budget1");
    REQUIRE(k.start({{cal::Kind::Power, 20}}));
    k.run();
    REQUIRE(k.gs.state() == State::Done);
    std::printf("  power20: Done at %.1f s\n", k.t / 1000.0);
    CHECK(k.t < 30000);
  }
}

TEST(verify_column_reads_for_every_parked_rate) {
  Kit k("verify");
  REQUIRE(k.start(kFourSweeps));
  k.run();
  REQUIRE(k.gs.state() == State::Done);
  const auto snap = k.gs.snapshot();
  const int m = mabur::margin_steps(k.live_margin_db);
  for (int wi = 0; wi < 2; ++wi) {
    const uint8_t bw = wi ? 40 : 20;
    const auto& w = snap.width[wi];
    for (int r = 0; r < 8; ++r) {
      // Every rate got a wall, so every rate was parked at wall - margin;
      // the drone flew exactly the park the GS predicted, and the GS heard
      // it there cleanly.
      CHECK(w.park[r] == expected_wall(bw, r) - m);
      const auto it = k.verify_idx_on_air.find({bw, static_cast<uint8_t>(r)});
      REQUIRE(it != k.verify_idx_on_air.end());
      CHECK(it->second == w.park[r]);
      CHECK(w.verify_pct[r] >= 90);
    }
  }
}

TEST(a_lossy_link_still_completes) {
  // The clean run's file is the reference.
  std::string clean_text;
  {
    Kit k("lossy_ref");
    REQUIRE(k.start(kFourSweeps));
    k.run();
    REQUIRE(k.gs.state() == State::Done);
    clean_text = k.read();
  }
  // 10 % of control messages lost each way. The sweep frames themselves
  // are clean: they ARE the measurement, and this is a test of the control
  // plane. A run carries only a dozen or so control messages each way, so
  // one seed loses one or two of them at most; thirty seeds between them
  // lose acks, SWEEP and APPLY repeats, APPLIEDs, SweepDones and closing
  // Aborts.
  //
  // One loss the protocol accepts by design: SweepDone goes out once, and
  // without it a capacity sweep cannot tell a tx_bound point from a real
  // one, so it writes nothing at that width (WidthResult::eff_unknown).
  // Such a width must leave the old efficiency entries alone; every other
  // byte matches the clean run.
  int lost_down = 0, lost_up = 0, exact = 0, eff_unknown = 0, slowest_ms = 0;
  for (uint32_t seed = 1; seed <= 30; ++seed) {
    Kit k("lossy");
    k.rng.seed(seed);
    k.drop_gs_to_drone_pct = 10;
    k.drop_drone_to_gs_pct = 10;
    REQUIRE(k.start(kFourSweeps, 1000 + seed));
    k.run();
    CHECK(k.gs.state() == State::Done);
    lost_down += k.dropped_gs_to_drone;
    lost_up += k.dropped_drone_to_gs;
    slowest_ms = std::max(slowest_ms, static_cast<int>(k.t));
    const auto snap = k.gs.snapshot();
    const mabur::Config c = k.loaded();
    REQUIRE(c.radio.rate_walls_rel_20.has_value());
    REQUIRE(c.radio.rate_walls_rel_40.has_value());
    check_walls(*c.radio.rate_walls_rel_20, 20);
    check_walls(*c.radio.rate_walls_rel_40, 40);
    CHECK(c.radio.power_mode == "offset");
    const bool unknown20 = snap.width[0].eff_unknown, unknown40 = snap.width[1].eff_unknown;
    if (!unknown20 && !unknown40) {
      CHECK(k.read() == clean_text);
      ++exact;
    } else {
      ++eff_unknown;
    }
    if (unknown20)
      for (int r = 0; r < 8; ++r) CHECK(c.air_clock.efficiency_20[r] == kFixtureEff20[r]);
    else
      check_eff(c.air_clock.efficiency_20, 20);
    if (unknown40)
      for (int r = 0; r < 8; ++r) CHECK(c.air_clock.efficiency_40[r] == kFixtureEff40[r]);
    else
      check_eff(c.air_clock.efficiency_40, 40);
    // However the closing Abort fared, the drone gets home: the Abort, or
    // its own await timer if the Abort was lost.
    k.drain();
    CHECK(!k.sweep.active());
    CHECK(k.restores == 1);
    CHECK(k.pwr.override_idx == -1);
    CHECK(!k.pwr.diffs_zeroed);
  }
  std::printf("  30 seeds: lost %d GS->drone, %d drone->GS; %d byte-identical, %d with a "
              "capacity SweepDone lost; slowest Done %.1f s\n",
              lost_down, lost_up, exact, eff_unknown, slowest_ms / 1000.0);
  CHECK(lost_down >= 10);
  CHECK(lost_up >= 10);
  // The byte-identical branch carries the test, not the eff_unknown one.
  CHECK(exact >= 15);
  CHECK(slowest_ms < 100000);
}

MTEST_MAIN
