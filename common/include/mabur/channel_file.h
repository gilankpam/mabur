#pragma once
// The remembered channel (spec 2026-10-03-auto-channel-set §2 "State
// files"): one decimal channel number and a newline. Written via a temp
// file in the same directory + rename() so a power cut mid-write leaves
// the old value, never a torn one. Paths are compiled-in: there is nothing
// to configure.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <string>

namespace mabur {

constexpr const char* kDroneChannelFile = "/etc/mabur.channel";
constexpr const char* kGsChannelFile = "/etc/maburgs.channel";

// nullopt on a missing/unreadable file or anything that is not a channel
// number in [1,177]. Membership in the set is the CALLER's check.
inline std::optional<uint8_t> read_channel_file(const std::string& path) {
  std::ifstream f(path);
  if (!f) return std::nullopt;
  long v = 0;
  if (!(f >> v)) return std::nullopt;
  if (v < 1 || v > 177) return std::nullopt;
  return static_cast<uint8_t>(v);
}

inline bool write_channel_file(const std::string& path, uint8_t ch) {
  const std::string tmp = path + ".tmp";
  {
    std::FILE* f = std::fopen(tmp.c_str(), "w");
    if (!f) return false;
    const bool ok = std::fprintf(f, "%u\n", static_cast<unsigned>(ch)) > 0 && std::fflush(f) == 0;
    std::fclose(f);
    if (!ok) { std::remove(tmp.c_str()); return false; }
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) { std::remove(tmp.c_str()); return false; }
  return true;
}

}  // namespace mabur
