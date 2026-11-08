#include <cstdint>

#include "mtest.h"
#include "vop2_frame_length.h"

using namespace maburplay::vop2;

TEST(fields_and_is_mode) {
  // A 1080p120 mode: vtotal 1125, sync 5 lines. vtotal in [28:16], sync
  // width in [12:0].
  const uint32_t reg = (1125u << 16) | 5;
  CHECK(vtotal_of(reg) == 1125);
  CHECK(sync_of(reg) == 5);
  CHECK(is_mode(reg, 1125, 5));
  CHECK(!is_mode(reg, 1125, 6));             // another mode's sync
  CHECK(!is_mode(reg, 500, 5));              // more than twice a 500-line mode
  CHECK(is_mode((1124u << 16) | 5, 1125, 5)); // one short: ours, left behind
  CHECK(!is_mode((1123u << 16) | 5, 1125, 5));
  CHECK(is_mode((2250u << 16) | 5, 1125, 5));
  CHECK(!is_mode((2251u << 16) | 5, 1125, 5));
}

TEST(mode_value_recovers_the_modes_own_total) {
  // A total left at 1130 by an earlier run comes back as the mode's 1125.
  const uint32_t orig = mode_value((1130u << 16) | 5, 1125);
  CHECK(orig == ((1125u << 16) | 5));
}

TEST(total_value_sets_total_keeps_sync_marks_immediate) {
  const uint32_t orig = mode_value((1130u << 16) | 5, 1125);
  const uint32_t w = total_value(orig, 1145);
  CHECK(vtotal_of(w) == 1145);
  CHECK(sync_of(w) == 5);
  CHECK((w & kImd) != 0);
  CHECK((orig & kImd) == 0);  // restoring clears it

  CHECK(vtotal_of(total_value(orig, 1105)) == 1105);
  // A total too big for the 13-bit field is masked, not spilled into sync.
  CHECK((total_value(orig, 0x2000 + 7) & ~(0x1fffu << 16) & ~kImd) == 5);
}

TEST(on_this_soc) {
  CHECK(on_this_soc("rockchip,rk3568-evb1-ddr4-v10 rockchip,rk3568 "));
  CHECK(on_this_soc("rockchip,rk3566 "));
  CHECK(!on_this_soc("rockchip,rk3588 "));
  CHECK(!on_this_soc("allwinner,sun50i-h618 "));
  CHECK(!on_this_soc(""));
}

MTEST_MAIN
