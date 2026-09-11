#include "mtest.h"
#include "cal_log.h"
#include "cal_analysis.h"
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

TEST(header_is_the_format_marker) {
  // The marker is what versions this FILE, independent of any run's data --
  // the schema itself is free to change provided maburcal changes in the
  // same commit.
  const auto d = fresh_dir("callog1");
  { CalLog l(d); l.header(); }
  const auto ls = lines_of(d);
  REQUIRE(!ls.empty());
  CHECK(ls[0] == "callog 2");
}

TEST(run_record_carries_the_per_run_parameters) {
  const auto d = fresh_dir("callog_run");
  { CalLog l(d); l.header(); l.run(42, 53, 1.0); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 2);
  CHECK(ls[1] == "R 42 53 1.00");
}

TEST(two_runs_in_one_session_append_two_run_records) {
  // The normal retry path (start, see a narrow/saturated flag, move the
  // drone, start again) shares one session directory across two runs --
  // nothing about starting a run rotates the session. Each run gets its
  // own R line so a reader can tell which nonce/base_ref/margin a later
  // C/W/V line belongs to, and where one run's lines end.
  const auto d = fresh_dir("callog_tworuns");
  {
    CalLog l(d);
    l.header();
    l.run(42, 53, 1.0);
    l.verify(0, 10, 90);
    l.run(43, 53, 1.5);
    l.verify(0, 12, 95);
  }
  const auto ls = lines_of(d);
  int header_count = 0;
  std::vector<std::string> run_lines;
  for (const auto& line : ls) {
    if (line == "callog 2") ++header_count;
    if (line.rfind("R ", 0) == 0) run_lines.push_back(line);
  }
  CHECK(header_count == 1);
  REQUIRE(run_lines.size() == 2);
  CHECK(run_lines[0] == "R 42 53 1.00");
  CHECK(run_lines[1] == "R 43 53 1.50");
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
  c.evm_dbh = {-31, kEvmNone};
  c.have_evm = {true, false};
  { CalLog l(d); l.header(); l.run(1, 53, 1.0); l.cell(1, 7, 56, c); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 3);
  CHECK(ls[2] == "C 1 7 56 100 97 12 3 -67 -999 -31 -999");
}

TEST(header_marker_is_version_2_since_evm_joined_the_cell_record) {
  // callog 2 adds the two trailing EVM columns to C. The marker is what
  // tells a reader which C shape it is looking at -- a v1 file on the DVR
  // has nine fields, not eleven, and gs/bundle/maburcal reads both.
  const auto d = fresh_dir("callog_v2");
  { CalLog l(d); l.header(); }
  CHECK(lines_of(d)[0] == "callog 2");
}

TEST(a_cell_with_no_evm_sample_writes_the_sentinel_not_a_zero) {
  // The chip encodes "not sampled" as raw 0, which is also a legal (absurd)
  // EVM value, so the log must not pass that ambiguity on: an unsampled
  // card writes kEvmNone exactly as an unheard card writes kRssiNone.
  const auto d = fresh_dir("callog_evmnone");
  CalCell c;
  c.idx = 8;
  c.expected = 20;
  c.received = {20, 20};
  c.rssi_dbm = {-60, -62};
  c.have_rssi = {true, true};
  c.have_evm = {false, false};
  { CalLog l(d); l.header(); l.run(1, 53, 1.0); l.cell(1, 0, 8, c); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 3);
  CHECK(ls[2] == "C 1 0 8 20 20 20 0 -60 -62 -999 -999");
}

TEST(undetermined_wall_writes_minus_one) {
  const auto d = fresh_dir("callog3");
  RateWall w;
  w.wall = -1;
  w.floor_idx = -1;
  w.best_card = 0;
  w.flags = kCalUndetermined;
  { CalLog l(d); l.header(); l.run(1, 53, 1.0); l.wall(7, w); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 3);
  CHECK(ls[2] == "W 7 -1 -1 0 2");
}

TEST(verify_record) {
  const auto d = fresh_dir("callog4");
  { CalLog l(d); l.header(); l.run(1, 53, 1.0); l.verify(5, 50, 97); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 3);
  CHECK(ls[2] == "V 5 50 97");
}

TEST(reopening_appends_without_a_second_header) {
  // A wrapper respawn rejoins the same session directory; two headers would
  // make maburcal read the second run's start as a corrupt record.
  const auto d = fresh_dir("callog5");
  { CalLog l(d); l.header(); l.verify(0, 87, 100); }
  { CalLog l(d); l.verify(1, 87, 99); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 3);
  CHECK(ls[0] == "callog 2");
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
  { CalLog l(d); l.header(); l.run(7, 39, 1.0); }
  CHECK(!cal_log_header_due(d));
}

TEST(a_rejoined_session_directory_with_no_cal_log_is_still_due_a_marker) {
  // Regression (hardware, 2026-09-11): maburgs keyed header() on
  // DebugSession::rejoined(), so the first calibration in a session
  // directory that maburgs had merely RESTARTED into -- the ordinary case,
  // since deploying a new binary restarts the daemon and it rejoins the
  // live session via the marker file -- wrote a cal.log with no `callog 1`
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
  { std::ofstream f(d + "/cal.log"); f << "callog 1\nR 1 53 1.00\n"; }
  CHECK(cal_log_prepare(d));
  CHECK(cal_log_header_due(d));  // the current file is gone, so it is due
  std::ifstream retired(d + "/cal.log.callog1");
  REQUIRE(retired.good());
  std::string first;
  std::getline(retired, first);
  CHECK(first == "callog 1");
}

TEST(a_current_format_cal_log_is_left_alone) {
  const auto d = fresh_dir("callog_current");
  { CalLog l(d); l.header(); l.run(1, 53, 1.0); }
  CHECK(!cal_log_prepare(d));
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 2);
  CHECK(ls[0] == "callog 2");
}

TEST(prepare_on_a_directory_with_no_cal_log_reports_the_header_is_due) {
  const auto d = fresh_dir("callog_absent");
  CHECK(cal_log_prepare(d));
}
