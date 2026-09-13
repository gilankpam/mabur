#include "scan_log.h"
#include "log_writer.h"
#include "mtest.h"
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

static std::string read_all(const std::string& p) {
  std::ifstream f(p); std::stringstream ss; ss << f.rdbuf(); return ss.str();
}
static void reset_dir(const std::string& dir) {
  (void)std::system(("rm -rf " + dir).c_str());
  mkdir(dir.c_str(), 0755);
}

TEST(scan_log_records_are_byte_exact) {
  std::string dir = "build_scan_log_test";
  reset_dir(dir);
  maburgs::LogWriter w;
  maburgs::ScanLog log(w, dir, "home=136 candidates=149,161 dwell_ms=250");
  REQUIRE(log.ok());
  maburgs::CardCaps c; c.valid = true; c.chip = "RTL8822E"; c.gen = "jaguar3"; c.tx_chains = 2; c.rx_chains = 2;
  c.bw_mask = 0x1f; c.tune5g_lo = 5080; c.tune5g_hi = 6165; c.fast_retune = true;
  c.fa_ok = true; c.igi_ok = true; c.nhm_ok = true; c.floor_ok = false;
  log.caps(1000, 1, c);
  // A card whose GetAdapterCaps came back unsupported: chip/gen must still
  // occupy their columns, so every later field keeps its position.
  maburgs::CardCaps bad;  // valid=false, everything else default-constructed
  log.caps(1001, 0, bad);
  maburgs::ScoutDwell d; d.survey.def.primary = 161; d.survey.round = 2; d.survey.observe_ms = 250;
  d.survey.cca_ofdm = 812; d.survey.fa_ofdm = 790; d.survey.dvr_frames = 3; d.survey.frames = 5;
  d.survey.valid_igi = true; d.survey.igi = 0x2a; d.survey.flags = 0x10; d.floor_valid = true; d.floor_dbm = -93;
  log.dwell(1300, 1, d);
  maburgs::ScoutDwell d2; d2.survey.def.primary = 149; d2.survey.observe_ms = 250; d2.survey.valid_igi = false;
  log.dwell(1600, 1, d2);
  std::vector<maburgs::RankEntry> all = {{136, 4, 3, false, 0}, {149, 0, 3, true, -96}, {161, 812, 2, true, -93}};
  log.pick(2000, 149, 3, all, 3);
  log.pick(2001, std::nullopt, 0, all, 3);
  log.move(maburgs::MoveEvent{2100, -1, 136, 149, maburgs::MoveReason::Commit});
  log.move(maburgs::MoveEvent{9000, 0, 149, 136, maburgs::MoveReason::SplitHome});
  maburgs::ScoutEnergy e; e.fa_valid = true; e.cca_ofdm = 61; e.fa_ofdm = 2; e.igi_valid = true; e.igi = 40;
  log.energy(3000, 0, 149, e, 59, 1);
  e.igi_valid = false;
  log.energy(4000, 1, 149, e, 59, 1);
  w.flush_now();
  std::string text = read_all(log.path());
  CHECK(text.rfind("scanlog 1 home=136 candidates=149,161 dwell_ms=250\n", 0) == 0);
  CHECK(text.find("\nC 1000 1 RTL8822E jaguar3 2x2 1f 5080-6165 1 1 1 1 0\n") != std::string::npos);
  CHECK(text.find("\nC 1001 0 ? ? 0x0 0 0-0 0 0 0 0 0\n") != std::string::npos);
  CHECK(text.find("\nD 1300 1 161 2 250 812 790 3 2 42 -93 10\n") != std::string::npos);
  CHECK(text.find("\nD 1600 1 149 0 250 0 0 0 0 - nan 0\n") != std::string::npos);
  CHECK(text.find("\nK 2000 149 3 136:4 149:0:-96\n") != std::string::npos);
  CHECK(text.find("\nK 2001 none 0\n") != std::string::npos);
  CHECK(text.find("\nM 2100 all 136 149 commit\n") != std::string::npos);
  CHECK(text.find("\nM 9000 0 149 136 split_home\n") != std::string::npos);
  CHECK(text.find("\nA 3000 0 149 61 2 59 1 40\n") != std::string::npos);
  CHECK(text.find("\nA 4000 1 149 61 2 59 1 -\n") != std::string::npos);
  CHECK(log.path() == dir + "/scan.log");
}

TEST(scan_log_bad_dir_is_nonfatal) {
  maburgs::LogWriter w;
  maburgs::ScanLog log(w, "/nonexistent/dir/for/scan", "x");
  CHECK(!log.ok());
  log.move(maburgs::MoveEvent{1, -1, 1, 2, maburgs::MoveReason::Reunite});  // must not crash
}
MTEST_MAIN
