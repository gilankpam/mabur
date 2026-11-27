#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <string>

#include "config.h"
#include "config_migrate.h"
#include "mabur/config_write.h"
#include "mabur/toml.h"
#include "mtest.h"
#include "scratch.h"

using namespace mabur;

namespace {

std::string fixture() {
  std::ifstream f(std::string(MABUR_FIXTURE_DIR) + "/mabur.full.toml");
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

const char* const kVenc = "[venc]\nsensor_bin = \"/x.bin\"\n";

MigrateResult migrate(const std::string& text) {
  ScratchFile f{"test_config_migrate", ".toml"};
  return migrate_config_text(text, f.path);
}

Config load_text(const std::string& text) {
  ScratchFile f{"test_config_migrate_load", ".toml"};
  REQUIRE(write_file_fsync(f.path, text));
  return load_config(f.path);
}

// Dotted paths of every non-table value in `text`.
void collect(const toml::Value& t, const std::string& prefix, std::set<std::string>* out) {
  for (auto it = t.begin(); it != t.end(); ++it) {
    const std::string path = prefix.empty() ? it.key() : prefix + "." + it.key();
    if ((*it).is_object())
      collect(*it, path, out);
    else
      out->insert(path);
  }
}

std::set<std::string> key_paths(const std::string& text) {
  std::set<std::string> out;
  collect(toml::parse_toml_string(text, "migrated"), "", &out);
  return out;
}

bool logged(const MigrateResult& r, const std::string& line) {
  for (const std::string& l : r.log)
    if (l == line) return true;
  return false;
}

}  // namespace

TEST(full_bundle_migrates_to_per_drone_keys) {
  const MigrateResult r = migrate(fixture());
  REQUIRE(r.ok);
  CHECK(key_paths(r.text) ==
        (std::set<std::string>{"radio.channels", "radio.legacy_wall_rel", "link.key_file",
                               "venc.sensor_bin"}));
  CHECK(logged(r, "removed default venc.min_iqp = 34"));
}

TEST(migrated_file_loads_equal_to_input) {
  const std::string in = fixture();
  const MigrateResult r = migrate(in);
  REQUIRE(r.ok);
  CHECK(load_text(r.text) == load_text(in));
}

TEST(unknown_key_and_table_removed) {
  const MigrateResult r = migrate(std::string("[radio]\nfoo = 1\ncca = false\n[bogus]\nx = 2\n") + kVenc);
  REQUIRE(r.ok);
  CHECK(r.text == std::string("[radio]\ncca = false\n") + kVenc);
  CHECK(logged(r, "removed unknown radio.foo"));
  CHECK(logged(r, "removed unknown bogus"));
}

TEST(changed_value_survives_with_comment) {
  const std::string in = std::string(kVenc) + "fps = 90  # chosen\n";
  const MigrateResult r = migrate(in);
  REQUIRE(r.ok);
  CHECK(r.text == in);
}

TEST(default_in_other_spelling_is_pruned) {
  const MigrateResult r =
      migrate(std::string(kVenc) + "fps=60\n[encoder]\nairtime_budget = 0.650\n");
  REQUIRE(r.ok);
  CHECK(r.text == std::string(kVenc) + "[encoder]\n");
}

TEST(default_channels_kept) {
  const std::string in = std::string("[radio]\nchannels = [40, 64, 112, 144]\n") + kVenc;
  const MigrateResult r = migrate(in);
  REQUIRE(r.ok);
  CHECK(r.text == in);
}

// The walls live in the wall cache now: a file from before keeps its
// channel set and loses the wall keys.
TEST(the_removed_wall_keys_are_pruned) {
  const MigrateResult r = migrate(
      std::string("[radio]\nchannels = [40, 64, 112, 144]\n"
                  "rate_walls_rel_20 = [60, 60, 60, 45, 22, 6, 9, 5]\ncal_flags_40 = \"stbc\"\n") +
      kVenc);
  REQUIRE(r.ok);
  CHECK(r.text == std::string("[radio]\nchannels = [40, 64, 112, 144]\n") + kVenc);
  CHECK(logged(r, "removed unknown radio.rate_walls_rel_20"));
  CHECK(logged(r, "removed unknown radio.cal_flags_40"));
}

TEST(bad_value_fails_without_output) {
  const MigrateResult r = migrate(std::string("[fec]\nwindow = 9999\n") + kVenc);
  CHECK(!r.ok);
  CHECK(r.error.find("fec.window") != std::string::npos);
}

// A line the parser refuses (a quoted key) is not an unknown key the
// migration can drop: it stops with the parser's reason, never loops.
TEST(unparseable_key_fails_cleanly) {
  const MigrateResult r = migrate(std::string("[radio]\n\"foo\" = 1\n") + kVenc);
  CHECK(!r.ok);
  CHECK(r.error.find("quoted keys") != std::string::npos);
}

// The legacy wall is per card: kept even when it equals today's default,
// so a later default change never rewrites a measured unit.
TEST(calibration_results_kept_even_at_default_value) {
  const std::string in =
      "[radio]\npower_mode = \"offset\"\nlegacy_wall_rel = 63\n" + std::string(kVenc);
  const MigrateResult r = migrate(in);
  REQUIRE(r.ok);
  CHECK(r.text == in);
}

MTEST_MAIN
