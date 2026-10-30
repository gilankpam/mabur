#pragma once
// The glue between maburgs' core loop and its CalSession that is worth a
// test of its own: the drone's calibration replies out of the MAVLink
// tunnel into the session, the sideport's link.cal block, and the stderr
// lines that narrate a run in /tmp/maburgs.log. maburgs-only (the web GS
// does not run calibration), but free of any socket or radio so the tests
// drive it with plain snapshots.
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "cal_session.h"
#include "mabur/cal_proto.h"
#include "stats_exporter.h"

namespace maburgs {

// The state name `maburcal status` prints ("await_ack", "sweep", ...).
const char* cal_state_name(CalSession::State st);

// "verify" during the verify pass, else the running sweep's own kind:
// "power" or "capacity".
const char* cal_kind_name(const CalSnapshot& snap);

// The operator's token for one sweep: power20, power40, cap20, cap40.
std::string cal_sweep_token(const mabur::cal::SweepSpec& spec);

// One whole MAVLink packet the tunnel delivered from the drone: every
// calibration message in it goes to the session (anything else is not the
// session's and is skipped). The drone sends its replies whatever the
// session's state; the session itself ignores what does not fit.
void feed_cal_packet(CalSession& s, const uint8_t* bytes, size_t n, uint64_t now_ms);

// The sideport's link.cal input: present only while the run owns the air.
StatsCalIn cal_sideport(const CalSession& s);

// Turns successive snapshots (one per core tick) into the run's milestone
// lines: the start (at the moment the run first commands the drone, after
// any channel pick), each apply's outcome, each finished sweep and the
// terminal state. Lines come back without a prefix.
class CalNarrator {
 public:
  // `log_path` is where cal.log resolved to, named on the start line.
  explicit CalNarrator(std::string log_path) : log_path_(std::move(log_path)) {}

  std::vector<std::string> observe(const CalSnapshot& snap,
                                   const std::vector<mabur::cal::SweepSpec>& sweeps);

 private:
  std::string log_path_;
  CalSnapshot prev_{};
};

}  // namespace maburgs
