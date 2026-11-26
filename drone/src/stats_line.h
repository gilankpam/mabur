// The MAV suffix of the agent's periodic `stats:` line. The MAV counters are
// quiet for most of a flight, so the fields are shown only when a counter
// moved since they were last shown; an unchanged set yields an empty string
// and keeps the line short.
#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <string>

namespace mabur {

// Order: rx frames/packets, tx frames/packets, drop, reject, cal_late,
// update_dropped. Copies `now` into `last` when it differs.
inline std::string mav_stats_suffix(const uint64_t (&now)[8], uint64_t (&last)[8]) {
  if (std::equal(std::begin(now), std::end(now), std::begin(last))) return {};
  std::copy(std::begin(now), std::end(now), std::begin(last));
  char buf[256];
  std::snprintf(buf, sizeof buf,
                " mav rx=%llu/%llu tx=%llu/%llu drop=%llu reject=%llu cal_late=%llu"
                " update_dropped=%llu",
                static_cast<unsigned long long>(now[0]), static_cast<unsigned long long>(now[1]),
                static_cast<unsigned long long>(now[2]), static_cast<unsigned long long>(now[3]),
                static_cast<unsigned long long>(now[4]), static_cast<unsigned long long>(now[5]),
                static_cast<unsigned long long>(now[6]), static_cast<unsigned long long>(now[7]));
  return buf;
}

}  // namespace mabur
