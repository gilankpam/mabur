#include "stats_sink.h"
#include "mtest.h"

#include <string>
#include <vector>

TEST(file_only_still_reports_accepted) {
  std::vector<std::string> filed;
  auto sink = maburgs::make_stats_sink(
      nullptr, [&](const std::string& s) { filed.push_back(s); });
  CHECK(sink("{\"v\":1}"));  // debug_log on, stats.enable off
  CHECK(filed.size() == 1);
  CHECK(filed[0] == "{\"v\":1}");
}

TEST(udp_only_passes_through_its_verdict) {
  auto ok = maburgs::make_stats_sink(
      [](const std::string&) { return true; }, nullptr);
  auto dead = maburgs::make_stats_sink(
      [](const std::string&) { return false; }, nullptr);
  CHECK(ok("{}"));
  CHECK(!dead("{}"));  // every UDP consumer dead and no file
}

TEST(a_dead_udp_consumer_does_not_blind_the_file) {
  std::vector<std::string> filed;
  auto sink = maburgs::make_stats_sink(
      [](const std::string&) { return false; },
      [&](const std::string& s) { filed.push_back(s); });
  CHECK(sink("{}"));  // the file accepted it
  CHECK(filed.size() == 1);
}

TEST(no_destinations_is_not_a_crash) {
  auto sink = maburgs::make_stats_sink(nullptr, nullptr);
  CHECK(!sink("{}"));
}

MTEST_MAIN
