#pragma once
// Trims a drone config to what this binary knows and does not already
// default. An over-the-air update runs the NEW maburd's migration on the
// drone's current file before the new build first boots on it:
//
//   - a key the loader rejects as unknown is removed (its whole table when
//     the unknown name is a table);
//   - a key whose line can go without changing what the file loads to (an
//     equal Config) is removed: its value is this binary's default;
//   - per-drone keys (the channel set, the calibration results -- wall
//     tables, cal_flags, legacy_wall_rel -- the key file, the sensor file)
//     are never removed, even when they equal the default;
//   - every other line is kept byte for byte, comments included.
//
// The loader is load_config itself, so "known key" and "default value"
// can never drift from what the binary boots with.
#include <string>
#include <vector>

namespace mabur {

struct MigrateResult {
  bool ok = false;
  std::string text;               // the migrated file, valid when ok
  std::vector<std::string> log;   // "removed unknown radio.foo", "removed default venc.min_iqp = 34"
  std::string error;              // set when !ok
};

// `scratch_path` is a writable path the loader runs against; it is removed
// before returning.
MigrateResult migrate_config_text(const std::string& text, const std::string& scratch_path);

// maburd --migrate-config <in> <out>: writes <out> (via <out>.new + rename)
// only on success, never writes <in>, logs "migrate: ..." to stderr.
// Returns the process exit code: 0 ok, 1 failure.
int migrate_config_cli(const std::string& in, const std::string& out);

}  // namespace mabur
