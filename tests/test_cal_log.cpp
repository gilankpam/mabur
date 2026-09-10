#include "mtest.h"
#include "cal_log.h"
#include "cal_analysis.h"
#include "scratch.h"

#include <fstream>
#include <string>
#include <vector>

using namespace maburgs;

namespace {

std::vector<std::string> lines_of(const std::string& dir) {
  std::ifstream f(dir + "/cal.log");
  std::vector<std::string> out;
  for (std::string l; std::getline(f, l);) out.push_back(l);
  return out;
}

std::string fresh_dir(const char* name) {
  const std::string d = std::string(MABUR_TEST_SCRATCH_DIR) + "/" + name;
  ::system(("rm -rf " + d + " && mkdir -p " + d).c_str());
  return d;
}

}  // namespace

TEST(header_is_the_format_marker) {
  // The marker is what versions this file -- the schema itself is free to
  // change provided maburcal changes in the same commit.
  const auto d = fresh_dir("callog1");
  { CalLog l(d); l.header(42, 53, 1.0); }
  const auto ls = lines_of(d);
  REQUIRE(!ls.empty());
  CHECK(ls[0] == "callog 1 nonce=42 base_ref=53 margin_db=1.00");
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
  { CalLog l(d); l.header(1, 53, 1.0); l.cell(1, 7, 56, c); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 2);
  CHECK(ls[1] == "C 1 7 56 100 97 12 3 -67 -999");
}

TEST(undetermined_wall_writes_minus_one) {
  const auto d = fresh_dir("callog3");
  RateWall w;
  w.wall = -1;
  w.floor_idx = -1;
  w.best_card = 0;
  w.flags = kCalUndetermined;
  { CalLog l(d); l.header(1, 53, 1.0); l.wall(7, w); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 2);
  CHECK(ls[1] == "W 7 -1 -1 0 2");
}

TEST(verify_record) {
  const auto d = fresh_dir("callog4");
  { CalLog l(d); l.header(1, 53, 1.0); l.verify(5, 50, 97); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 2);
  CHECK(ls[1] == "V 5 50 97");
}

TEST(reopening_appends_without_a_second_header) {
  // A wrapper respawn rejoins the same session directory; two headers would
  // make maburcal read the second run as a corrupt first record.
  const auto d = fresh_dir("callog5");
  { CalLog l(d); l.header(1, 53, 1.0); l.verify(0, 87, 100); }
  { CalLog l(d); l.verify(1, 87, 99); }
  const auto ls = lines_of(d);
  REQUIRE(ls.size() == 3);
  CHECK(ls[0].rfind("callog 1", 0) == 0);
  CHECK(ls[1].rfind("V ", 0) == 0);
  CHECK(ls[2].rfind("V ", 0) == 0);
}

MTEST_MAIN
