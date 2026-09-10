#include "mtest.h"
#include "cal_analysis.h"

#include <vector>

using namespace maburgs;

namespace {

// Builds one cell. rssi is the best card's median; -999 = unmeasured.
CalCell cell(int idx, int pct, int rssi = -70, int expected = 100) {
  CalCell c;
  c.idx = static_cast<uint8_t>(idx);
  c.expected = static_cast<uint16_t>(expected);
  c.received[0] = static_cast<uint16_t>(expected * pct / 100);
  c.received[1] = 0;
  c.rssi_dbm[0] = rssi;
  c.have_rssi[0] = rssi != kRssiNone;
  c.have_rssi[1] = false;
  return c;
}

}  // namespace

// mcs7 as measured 2026-07-29: a sensitivity floor below idx 34, a clean run
// to 49, then the comb -- 2% at 56 but 95% at 57. The wall is the END OF THE
// FIRST CONTIGUOUS RUN (49). The island at 57 is not headroom.
TEST(mcs7_first_dip_not_last_good) {
  std::vector<CalCell> cells;
  for (int i = 0; i <= 33; ++i) cells.push_back(cell(i, 10));   // floor
  for (int i = 34; i <= 49; ++i) cells.push_back(cell(i, 98));  // clean run
  for (int i = 50; i <= 55; ++i) cells.push_back(cell(i, 30));  // dip
  cells.push_back(cell(56, 2));
  // An island ABOVE the threshold: the measured comb produces these (mcs6
  // reads 63% at idx 52 but 95-97% at 53-57). This is the cell that
  // discriminates -- a "last index >= 90%" implementation returns 57 here,
  // the correct first-contiguous-run implementation returns 49.
  cells.push_back(cell(57, 95));
  for (int i = 58; i <= 63; ++i) cells.push_back(cell(i, 0));

  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK(w.wall == 49);
  CHECK(w.floor_idx == 34);
  CHECK((w.flags & kCalNoDip) == 0);
  CHECK((w.flags & kCalUndetermined) == 0);
}

// mcs0: never dips. The wall is the saturation knee from the RSSI curve --
// the index where radiated power stops rising -- NOT 127.
TEST(mcs0_no_dip_uses_saturation_knee) {
  std::vector<CalCell> cells;
  // Flat floor to 28, ~0.3 dB/idx ramp to 91, flat ceiling to 127.
  for (int i = 0; i <= 127; i += 4) {
    int rssi;
    if (i <= 28) rssi = -80;
    else if (i <= 91) rssi = -80 + (i - 28) * 3 / 10;
    else rssi = -80 + (91 - 28) * 3 / 10;
    cells.push_back(cell(i, 100, rssi));
  }
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK((w.flags & kCalNoDip) != 0);
  CHECK((w.flags & kCalUndetermined) == 0);
  // Coarse step is 4, so the knee lands within +/-2 of 91. It must never be
  // 127: parking above the knee radiates nothing extra and burns diff range.
  CHECK(w.wall >= 88 && w.wall <= 92);
}

TEST(row_never_reaching_threshold_is_undetermined) {
  std::vector<CalCell> cells;
  for (int i = 0; i <= 127; i += 4) cells.push_back(cell(i, 40));
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK((w.flags & kCalUndetermined) != 0);
  CHECK(w.wall == -1);  // caller must leave the config entry alone
}

// Diversity is not protection against PA compression: both cards see the same
// degraded waveform. Scoring the union inflates delivery and pushes the wall
// UP, which is the overdriven direction.
TEST(uses_best_single_card_not_union) {
  std::vector<CalCell> cells;
  for (int i = 40; i <= 60; ++i) {
    CalCell c;
    c.idx = static_cast<uint8_t>(i);
    c.expected = 100;
    // Card 0 is the good card and walls at 50.
    c.received[0] = static_cast<uint16_t>(i <= 50 ? 98 : 20);
    // Card 1 hears nothing below the wall and 70% above it. A summed/union
    // implementation would therefore score 20+70 = 90% past idx 50, find no
    // dip at all, and fall through to the knee -- so this fixture fails
    // against a union scorer and passes only for best-single-card. Card 1's
    // total volume (700) stays under card 0's (1278), so card 0 is still the
    // card selected.
    c.received[1] = static_cast<uint16_t>(i <= 50 ? 0 : 70);
    c.rssi_dbm[0] = -70;
    c.rssi_dbm[1] = -85;
    c.have_rssi[0] = c.have_rssi[1] = true;
    cells.push_back(c);
  }
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK(w.wall == 50);
  CHECK(w.best_card == 0);
  CHECK((w.flags & kCalNoDip) == 0);  // where a union scorer would land
}

TEST(flags_saturation_from_peak_rssi) {
  std::vector<CalCell> cells;
  for (int i = 0; i <= 60; ++i) cells.push_back(cell(i, 98, -40));
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK((w.flags & kCalSaturated) != 0);
}

TEST(flags_narrow_window) {
  std::vector<CalCell> cells;
  for (int i = 0; i <= 43; ++i) cells.push_back(cell(i, 10));
  for (int i = 44; i <= 46; ++i) cells.push_back(cell(i, 98));  // 3-wide
  for (int i = 47; i <= 60; ++i) cells.push_back(cell(i, 5));
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK(w.wall == 46);
  CHECK((w.flags & kCalNarrow) != 0);
}

TEST(flags_card_disagreement) {
  std::vector<CalCell> cells;
  for (int i = 40; i <= 60; ++i) {
    CalCell c;
    c.idx = static_cast<uint8_t>(i);
    c.expected = 100;
    c.received[0] = static_cast<uint16_t>(i <= 55 ? 98 : 10);
    c.received[1] = static_cast<uint16_t>(i <= 45 ? 98 : 10);
    c.rssi_dbm[0] = c.rssi_dbm[1] = -70;
    c.have_rssi[0] = c.have_rssi[1] = true;
    cells.push_back(c);
  }
  const auto w = analyze_rate(cells, CalThresholds{});
  CHECK((w.flags & kCalCardDisagree) != 0);
}

TEST(empty_input_is_undetermined) {
  const auto w = analyze_rate({}, CalThresholds{});
  CHECK(w.wall == -1);
  CHECK((w.flags & kCalUndetermined) != 0);
}

MTEST_MAIN
