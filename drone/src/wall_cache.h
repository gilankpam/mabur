#pragma once
#include "wall_curve.h"

#include <cstdint>
#include <map>
#include <string>
#include <utility>

namespace mabur {

constexpr const char* kWallCacheFile = "/etc/mabur.walls";

struct WallCache {
  std::map<std::pair<uint8_t, uint8_t>, WallCurve> entries;  // key (channel, width)
};

// Never fails: a missing, unreadable, malformed or wrong-version file gives an
// empty cache and a one-line reason in *note (may be null); a good file leaves *note empty.
WallCache load_wall_cache(const std::string& path, std::string* note);
// <path>.tmp, fsync, rename. False (and no file) on any failure.
bool save_wall_cache(const std::string& path, const WallCache& c);

}  // namespace mabur
