#ifndef MABUR_PLAYER_GS_LIVE_VIEW_H_
#define MABUR_PLAYER_GS_LIVE_VIEW_H_
// Parses maburgs' live-key `get` dump (docs/gs-live-config.md) into a
// struct the goggles settings menu can read without re-running the
// listener's own exchange. The text comes straight from
// GsConfigApplier::render_live_value (gs/src/gs_config_apply.cpp): ints
// bare, reals shortest round-trip, bools "true"/"false", radio.channel as
// "auto" or the number, the ladder as one inline-array line. A dump from a
// maburgs build this does not fully recognise -- an unknown key, a line it
// cannot split, a ladder it cannot read -- never throws and never aborts
// the rest of the dump: whatever it understood lands, the rest is skipped.
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace maburplay {

// One link.ladder rung. mabur::toml has no inline-table support, so this
// is read and written by hand, the same reason maburgs' own ladder check
// (gs_config_apply.cpp split_ladder) is.
struct Rung {
  int mcs = 0;
  int bw = 40;
  double base = 0.5;
  double enh = 0.25;
};

// maburgs' nine live keys, as last read from a `get` dump.
struct GsLiveView {
  bool have = false;
  int static_mcs = -1;
  int static_bw = 20;
  double static_base = 0.5, static_enh = 0.5;
  std::vector<Rung> ladder;
  bool nack = false, debug_log = false, msp = false;
  std::optional<int> pinned_channel;  // nullopt = auto
  uint64_t read_ms = 0;
};

// Parses a bare `get`'s whole reply -- "ok" then one "<key> = <value>"
// line per live key -- into `*out`. False only when `reply` does not
// start with "ok"; `*out` is left untouched in that case. Otherwise
// `out->have` is set and `out->read_ms = now_ms`, and every line is read
// on its own: an unknown key, or a line this cannot split or whose value
// this cannot parse, is skipped rather than failing the whole dump.
bool parse_get_all(const std::string& reply, GsLiveView* out, uint64_t now_ms);

// The inline-array form maburgs renders and its loader hand-checks:
// `[{mcs = .., bw = .., overhead_base = .., overhead_enh = ..}, ...]`,
// the four keys in any order, no duplicates, no nested table. False,
// `*out` untouched, if the text cannot be split that way or any rung is
// missing one of the four keys.
bool parse_ladder_value(const std::string& text, std::vector<Rung>* out);

// The form maburgs' `set link.ladder = ...` parser (gs_config_apply.cpp
// split_ladder, reached through patch_live_key) accepts: reals via `%g`.
std::string render_ladder_value(const std::vector<Rung>& l);

enum class LadderWidth { k20, k40, kMixed };

// k20/k40 when every rung shares that bandwidth, kMixed otherwise
// (including an empty ladder, which has no bandwidth to agree on).
LadderWidth ladder_width(const std::vector<Rung>& l);

// The two default width ladders the Width row loads: 40 -> MCS 0,1,2,3,4
// at 0.50/0.25, all bw 40; anything else -> MCS 0,2,4,5 at 0.50/0.25, all
// bw 20.
const std::vector<Rung>& default_ladder(int bw);

// Whether `l` is exactly one of the two default_ladder() tables.
bool is_default_ladder(const std::vector<Rung>& l);

}  // namespace maburplay

#endif  // MABUR_PLAYER_GS_LIVE_VIEW_H_
