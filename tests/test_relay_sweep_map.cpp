#include <cmath>

#include "mtest.h"
#include "relay_sweep_map.h"
using namespace maburgs;

TEST(sweep_visit_uses_raw_busy_and_marks_relay) {
  SweepEntry e; e.ch = 165; e.valid = true; e.active_ms = 20; e.busy_ms = 14; e.rx_ms = 2;
  e.foreign = 3; e.ofdm_err = 9;
  const HopVisit v = sweep_visit(e, 500);
  CHECK(v.ch == 165 && v.t_ms == 500);
  CHECK(v.src == VisitSrc::Relay);
  CHECK(v.fa == 9 && v.foreign == 3 && v.cca == 0 && v.own == 0);
  CHECK(v.busy_valid && std::abs(v.busy_pct - 70.0) < 1e-9);   // raw, NOT (busy - rx)
}

TEST(sweep_visit_without_air_time_has_no_busy) {
  SweepEntry e; e.ch = 40; e.valid = true;
  CHECK(!sweep_visit(e, 1).busy_valid);
}

TEST(sweep_dwell_fills_the_d_record) {
  SweepEntry e; e.ch = 64; e.valid = true; e.active_ms = 20; e.busy_ms = 10; e.rx_ms = 5; e.foreign = 4; e.ofdm_err = 7;
  const ScoutDwell d = sweep_dwell(e, 3);
  CHECK(d.survey.def.primary == 64 && d.survey.round == 3 && d.survey.observe_ms == 20);
  CHECK(d.survey.fa_ofdm == 7 && d.survey.frames == 4 && d.survey.dvr_frames == 0);
  CHECK(d.in_session && d.busy_valid && std::abs(d.busy_pct - 50.0) < 1e-9);
  CHECK(d.rx_valid && std::abs(d.rx_pct - 25.0) < 1e-9);
}

MTEST_MAIN
