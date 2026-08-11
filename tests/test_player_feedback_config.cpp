#include <cstdio>
#include <fstream>
#include <string>
#include "mtest.h"
#include "player_config.h"

namespace {
std::string write_cfg(const char* name, const std::string& body) {
  const std::string path = std::string(MABUR_TEST_SCRATCH_DIR) + "/" + name;
  std::ofstream f(path);
  f << body;
  return path;
}
}  // namespace

TEST(feedback_defaults_when_block_absent) {
  const auto p = write_cfg("fb_absent.json", R"({"backend":"null"})");
  const auto c = maburplay::load_config(p);
  CHECK(c.feedback.enable == true);
  CHECK(c.feedback.gs_host == "127.0.0.1");
  CHECK(c.feedback.gs_port == 8303);
  CHECK(c.feedback.report_ms == 500);
}

TEST(feedback_values_are_read) {
  const auto p = write_cfg("fb_set.json", R"({
    "backend":"null",
    "feedback":{"enable":false,"gs_host":"127.0.0.2","gs_port":9999,"report_ms":250}
  })");
  const auto c = maburplay::load_config(p);
  CHECK(c.feedback.enable == false);
  CHECK(c.feedback.gs_host == "127.0.0.2");
  CHECK(c.feedback.gs_port == 9999);
  CHECK(c.feedback.report_ms == 250);
}

TEST(unknown_feedback_key_fails_boot) {
  const auto p = write_cfg("fb_bad.json",
                           R"({"backend":"null","feedback":{"nope":1}})");
  bool threw = false;
  try {
    maburplay::load_config(p);
  } catch (const std::exception&) {
    threw = true;
  }
  CHECK(threw);
}

TEST(out_of_range_report_ms_fails_boot) {
  const auto p = write_cfg("fb_range.json",
                           R"({"backend":"null","feedback":{"report_ms":0}})");
  bool threw = false;
  try {
    maburplay::load_config(p);
  } catch (const std::exception&) {
    threw = true;
  }
  CHECK(threw);
}

MTEST_MAIN
