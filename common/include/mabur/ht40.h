#pragma once
// 5 GHz HT40 pairing: which side of a 20 MHz primary its 40 MHz secondary
// sits on, per the standard grid (36+40, 44+48, ... 132+136, 140+144,
// 149+153, 157+161). Returned in devourer's SelectedChannel.ChannelOffset
// convention: 1 = HT40+ (primary is the lower 20), 2 = HT40- (primary is the
// upper 20), 0 = no 40 MHz pair (165, or off the 5 GHz grid). A fixed
// "always above" choice lands off-grid (136+140 centres on 138, straddling
// the real 132+136 and 140+144), where every other 40 MHz network and
// devourer's spur table (keyed on standard centres) disagree with us
// (docs/bw40-sweep-findings-2026-09-23.md).
#include <cstdint>

namespace mabur {

constexpr uint8_t ht40_offset(uint8_t ch) {
  if (ch >= 36 && ch <= 144 && (ch - 36) % 4 == 0)
    return ((ch - 36) / 4) % 2 == 0 ? 1 : 2;
  if (ch >= 149 && ch <= 161 && (ch - 149) % 4 == 0)
    return ((ch - 149) / 4) % 2 == 0 ? 1 : 2;
  return 0;
}

static_assert(ht40_offset(36) == 1 && ht40_offset(40) == 2, "36+40");
static_assert(ht40_offset(132) == 1 && ht40_offset(136) == 2, "132+136");
static_assert(ht40_offset(140) == 1 && ht40_offset(144) == 2, "140+144");
static_assert(ht40_offset(116) == 1 && ht40_offset(120) == 2, "116+120");
static_assert(ht40_offset(149) == 1 && ht40_offset(153) == 2, "149+153");
static_assert(ht40_offset(157) == 1 && ht40_offset(161) == 2, "157+161");
static_assert(ht40_offset(165) == 0 && ht40_offset(6) == 0, "no pair");

// The other 20 MHz half of ch's standard pair (136 -> 132, 144 -> 140,
// 40 -> 36); 0 when ch has no pair. The boot scout at 40 MHz dwells on
// both halves of every pair (docs/bw40.md).
constexpr uint8_t ht40_pair_other(uint8_t ch) {
  const uint8_t off = ht40_offset(ch);
  if (off == 1) return static_cast<uint8_t>(ch + 4);
  if (off == 2) return static_cast<uint8_t>(ch - 4);
  return 0;
}

static_assert(ht40_pair_other(136) == 132 && ht40_pair_other(132) == 136, "132+136");
static_assert(ht40_pair_other(149) == 153 && ht40_pair_other(165) == 0, "149+153 / no pair");

}  // namespace mabur
