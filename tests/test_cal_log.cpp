#include "mtest.h"
#include "cal_log.h"
#include "cal_analysis.h"
#include "mabur/cal_wire.h"
#include "scratch.h"

#include <cstdlib>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <vector>

using namespace maburgs;

namespace {

std::vector<std::string> lines_of(const std::string& dir) {
  std::ifstream f(dir + "/cal.log");
  std::vector<std::string> out;
  for (std::string l; std::getline(f, l);) out.push_back(l);
  return out;
}

// rm -rf still needs a shell (no direct syscall for a recursive tree
// delete); mkdir does not, so it gets the real syscall rather than a
// forked "mkdir -p" that would also hide a mid-path failure behind "&&"
// (matches tests/test_ctl_log.cpp's reset_dir / test_probe_log.cpp). The
// cleanup is genuinely best-effort (a fresh scratch dir has nothing to
// remove on a clean run), so the exit status is deliberately unchecked --
// but GCC's warn_unused_result on system() is NOT silenced by a plain
// (void) cast (only by actually consuming the value), so it goes through
// an if with an empty body rather than a cast that would still warn.
std::string fresh_dir(const char* name) {
  const std::string d = std::string(MABUR_TEST_SCRATCH_DIR) + "/" + name;
  if (std::system(("rm -rf " + d).c_str()) != 0) { /* best-effort */ }
  ::mkdir(d.c_str(), 0755);
  return d;
}

}  // namespace

TEST(header_is_callog_4) {
  // The marker is what versions this FILE, independent of any run's data --
  // the schema itself is free to change provided maburcal changes in the
  // same commit.
  const auto d = fresh_dir("callog1");
  { CalLog l(d); l.header(); }
  const auto ls = lines_of(d);
  REQUIRE(!ls.empty());
  CHECK(ls[0] == "callog 4");
}

TEST(run_record_carries_channel_pick_and_ldpc) {
  const auto d = fresh_dir("callog_run");
  { CalLog l(d); l.header(); l.run(42, 1.0, 149, "auto", true); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 2);
  CHECK(ls[1] == "R 42 1.00 149 auto 1");
}

TEST(run_record_prints_ldpc_off_and_the_pick_word_verbatim) {
  const auto d = fresh_dir("callog_run2");
  { CalLog l(d); l.header(); l.run(7, 0.5, 64, "onecard", false); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 2);
  CHECK(ls[1] == "R 7 0.50 64 onecard 0");
}

TEST(two_runs_in_one_session_append_two_run_records) {
  // The normal retry path (start, see a narrow/saturated flag, move the
  // drone, start again) shares one session directory across two runs --
  // nothing about starting a run rotates the session. Each run gets its
  // own R line so a reader can tell which nonce/margin/channel/pick a
  // later C/W/P/V line belongs to, and where one run's lines end.
  const auto d = fresh_dir("callog_tworuns");
  {
    CalLog l(d);
    l.header();
    l.run(42, 1.0, 149, "auto", true);
    l.verify(0, 10, 90);
    l.run(43, 1.5, 149, "stayed", true);
    l.verify(0, 12, 95);
  }
  const auto ls = lines_of(d);
  int header_count = 0;
  std::vector<std::string> run_lines;
  for (const auto& line : ls) {
    if (line == "callog 4") ++header_count;
    if (line.rfind("R ", 0) == 0) run_lines.push_back(line);
  }
  CHECK(header_count == 1);
  REQUIRE(run_lines.size() == 2);
  CHECK(run_lines[0] == "R 42 1.00 149 auto 1");
  CHECK(run_lines[1] == "R 43 1.50 149 stayed 1");
}

TEST(sweep_record_scopes_the_rows_below) {
  const auto d = fresh_dir("callog_sweep");
  { CalLog l(d); l.header(); l.run(1, 1.0, 149, "auto", true);
    l.sweep(0, mabur::cal::Kind::Power, 20, 20);
    l.sweep(1, mabur::cal::Kind::Capacity, 40, 2000); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 4);
  CHECK(ls[2] == "S 0 power 20 20");
  CHECK(ls[3] == "S 1 capacity 40 2000");
}

TEST(a_verify_sweep_shares_the_power_word) {
  // A verify pass is a frame-count measurement same as an ordinary power
  // sweep -- cal::Kind::Verify is not given its own word in the log; the
  // C-vs-V rows that follow are what tell the two apart.
  const auto d = fresh_dir("callog_sweep_verify");
  { CalLog l(d); l.header(); l.run(1, 1.0, 149, "auto", true);
    l.sweep(2, mabur::cal::Kind::Verify, 20, 20); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 3);
  CHECK(ls[2] == "S 2 power 20 20");
}

TEST(cell_record_round_trips_every_field) {
  const auto d = fresh_dir("callog2");
  CalCell c;
  c.idx = 56;
  c.expected = 100;
  c.received = {97, 12};
  c.corrupt = 3;
  c.rssi_dbm = {-67, kRssiNone};
  c.have_rssi = {true, false};
  { CalLog l(d); l.header(); l.run(1, 1.0, 149, "auto", true);
    l.cell(7, 56, c); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 3);
  CHECK(ls[2] == "C 7 56 100 97 12 3 -67 -999");
}

TEST(a_minus_one_wall_is_written_verbatim) {
  // Relative walls: -1 is an ORDINARY wall one index below the chip's
  // anchor, not a sentinel -- mcs7 measured -4 on the bench.
  const auto d = fresh_dir("callog3");
  RateWall w;
  w.wall = -1;
  w.floor_idx = -1;
  w.best_card = 0;
  w.flags = 0;
  { CalLog l(d); l.header(); l.run(1, 1.0, 149, "auto", true); l.wall(7, w); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 3);
  CHECK(ls[2] == "W 7 -1 -1 0 0");
}

TEST(negative_indices_and_the_sentinel_print_verbatim) {
  const auto d = fresh_dir("callog_neg");
  {
    CalLog l(d); l.header(); l.run(1, 1.0, 149, "auto", true);
    RateWall w; w.wall = -4; w.floor_idx = -40; w.best_card = 0; w.flags = 0;
    l.wall(7, w);
    RateWall u;  // default: kNoWall
    l.wall(3, u);
    l.verify(7, -8, 97);
  }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 5);
  CHECK(ls[2] == "W 7 -4 -40 0 0");
  CHECK(ls[3] == "W 3 -128 -128 -1 0");
  CHECK(ls[4] == "V 7 -8 97");
}

TEST(cap_record_round_trips_every_field) {
  const auto d = fresh_dir("callog_cap");
  CapCell c;
  c.offered = 4000;
  c.blocked_pct = 72;
  c.frames = {3950, 1200};
  c.air_bits = {58000000ull, 20000000ull};
  c.rssi_dbm = {-50, -60};
  c.have_rssi = {true, true};
  CapResult r;
  r.eff = 0.73;
  r.best_card = 0;
  r.flags = kCalTxBound;
  { CalLog l(d); l.header(); l.run(1, 1.0, 149, "auto", true);
    l.sweep(0, mabur::cal::Kind::Capacity, 40, 2000);
    l.cap(5, c, r); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 4);
  CHECK(ls[3] == "P 5 4000 3950 58000000 730 72 64");
}

TEST(cap_record_uses_the_other_card_when_it_is_best) {
  const auto d = fresh_dir("callog_cap2");
  CapCell c;
  c.offered = 500;
  c.blocked_pct = 10;
  c.frames = {100, 480};
  c.air_bits = {1000ull, 9000ull};
  CapResult r;
  r.eff = 0.5;
  r.best_card = 1;
  r.flags = 0;
  { CalLog l(d); l.header(); l.run(1, 1.0, 149, "auto", true);
    l.sweep(0, mabur::cal::Kind::Capacity, 20, 2000);
    l.cap(2, c, r); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 4);
  CHECK(ls[3] == "P 2 500 480 9000 500 10 0");
}

TEST(verify_record) {
  const auto d = fresh_dir("callog4");
  { CalLog l(d); l.header(); l.run(1, 1.0, 149, "auto", true);
    l.verify(5, 50, 97); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 3);
  CHECK(ls[2] == "V 5 50 97");
}

TEST(applied_record_keeps_the_reason_text) {
  const auto d = fresh_dir("callog_applied");
  { CalLog l(d); l.header(); l.run(1, 1.0, 149, "auto", true);
    l.applied(20, true, "ok");
    l.applied(40, false, "out of range table"); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 4);
  CHECK(ls[2] == "A 20 1 ok");
  CHECK(ls[3] == "A 40 0 out of range table");
}

TEST(applied_record_replaces_control_characters_with_spaces) {
  // A newline (or any other control byte) embedded in `reason` would
  // otherwise split this one A record across two lines in the file --
  // LogWriter appends exactly one '\n' of its own and writes everything
  // handed to it verbatim.
  const auto d = fresh_dir("callog_applied_ctrl");
  { CalLog l(d); l.header(); l.run(1, 1.0, 149, "auto", true);
    l.applied(20, false, std::string("line one\nline two\ttab")); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 3);
  CHECK(ls[2] == "A 20 0 line one line two tab");
}

TEST(reopening_appends_without_a_second_header) {
  // A wrapper respawn rejoins the same session directory; two headers would
  // make maburcal read the second run's start as a corrupt record.
  const auto d = fresh_dir("callog5");
  { CalLog l(d); l.header(); l.verify(0, 87, 100); }
  { CalLog l(d); l.verify(1, 87, 99); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 3);
  CHECK(ls[0] == "callog 4");
  CHECK(ls[1].rfind("V ", 0) == 0);
  CHECK(ls[2].rfind("V ", 0) == 0);
}

MTEST_MAIN

TEST(header_is_due_only_until_the_file_itself_exists) {
  // cal_log_header_due is the single source of truth for "is this
  // construction the FILE's first writer", and it answers that by asking
  // the filesystem rather than by inferring it from anything else.
  const auto d = fresh_dir("callog_due");
  CHECK(cal_log_header_due(d));
  { CalLog l(d); l.header(); l.run(7, 1.0, 149, "auto", true); }
  CHECK(!cal_log_header_due(d));
}

TEST(a_rejoined_session_directory_with_no_cal_log_is_still_due_a_marker) {
  // Regression (hardware): maburgs keyed header() on
  // DebugSession::rejoined(), so the first calibration in a session
  // directory that maburgs had merely RESTARTED into -- the ordinary case,
  // since deploying a new binary restarts the daemon and it rejoins the
  // live session via the marker file -- wrote a cal.log with no `callog N`
  // line. `maburcal report` then refused the file outright and `maburcal
  // start`'s own end-of-run table never rendered, losing the entire
  // operator-facing result of a run that had already measured and applied
  // walls. A rejoined session says nothing about whether cal.log exists:
  // here the directory is old and populated, and the marker is still due.
  const auto d = fresh_dir("callog_rejoin");
  std::ofstream(d + "/ctl.log") << "ctllog 11\n";
  CHECK(cal_log_header_due(d));
}

TEST(a_cal_log_from_an_older_format_is_retired_rather_than_appended_to) {
  // The once-per-FILE marker rule means a cal.log can only ever hold one
  // format's C rows, and nothing else enforces that: cal_log_header_due
  // alone says "file exists, no header needed", so a maburgs carrying a
  // newer schema would append its rows under the older file's marker and
  // leave a file that lies about its own shape. Rotating the old one aside
  // keeps its data readable as what it actually is.
  const auto d = fresh_dir("callog_stale");
  { std::ofstream f(d + "/cal.log"); f << "callog 3\nR 1 1.00\n"; }
  CHECK(cal_log_prepare(d));
  CHECK(cal_log_header_due(d));  // the current file is gone, so it is due
  std::ifstream retired(d + "/cal.log.callog3");
  REQUIRE(retired.good());
  std::string first;
  std::getline(retired, first);
  CHECK(first == "callog 3");
}

TEST(a_current_format_cal_log_is_left_alone) {
  const auto d = fresh_dir("callog_current");
  { CalLog l(d); l.header(); l.run(1, 1.0, 149, "auto", true); }
  CHECK(!cal_log_prepare(d));
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 2);
  CHECK(ls[0] == "callog 4");
}

TEST(prepare_on_a_directory_with_no_cal_log_reports_the_header_is_due) {
  const auto d = fresh_dir("callog_absent");
  CHECK(cal_log_prepare(d));
}
