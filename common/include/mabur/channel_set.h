#pragma once
// The channel set (spec 2026-10-03-auto-channel-set §2): the members both
// ends agree the link may live on. Pure, header-only, shared by the drone
// and GS config loaders and by RcAgent/ChannelPlan membership checks.
#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mabur/ht40.h"

namespace mabur {

struct ChannelSetIssue {
  std::string field, why;
};

inline bool channel_set_member(const std::vector<uint8_t>& set, uint8_t ch) {
  return ch != 0 && std::find(set.begin(), set.end(), ch) != set.end();
}

// nullopt = valid. 1-8 unique members in [1,177]; at 40 MHz every member is
// a standard HT40 pair primary and all share one ht40_offset (FastRetune
// keeps the offset; docs/bw40.md "Channels").
inline std::optional<ChannelSetIssue> channel_set_issue(const std::vector<uint8_t>& set,
                                                        uint8_t width_mhz,
                                                        const std::string& field) {
  if (set.empty() || set.size() > 8)
    return ChannelSetIssue{field, "must list 1 to 8 channels"};
  for (size_t i = 0; i < set.size(); ++i) {
    const uint8_t ch = set[i];
    if (ch < 1 || ch > 177)
      return ChannelSetIssue{field, "channel " + std::to_string(ch) + " must be in [1,177]"};
    for (size_t j = 0; j < i; ++j)
      if (set[j] == ch)
        return ChannelSetIssue{field, "duplicate channel " + std::to_string(ch)};
  }
  if (width_mhz == 40) {
    const uint8_t off0 = ht40_offset(set[0]);
    for (uint8_t ch : set) {
      const uint8_t off = ht40_offset(ch);
      if (off == 0)
        return ChannelSetIssue{field, "channel " + std::to_string(ch) +
                                          " has no 40 MHz pair (docs/bw40.md)"};
      if (off != off0)
        return ChannelSetIssue{
            field, "channel " + std::to_string(ch) +
                       " is on the other side of the pair grid from channel " +
                       std::to_string(set[0]) +
                       "; FastRetune keeps the offset, every member must share one (docs/bw40.md)"};
    }
  }
  return std::nullopt;
}

}  // namespace mabur
