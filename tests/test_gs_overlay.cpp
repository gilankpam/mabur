#include "mtest.h"
#include "gs_overlay.h"
#include <string>
using namespace maburplay;

// --- thresholds -------------------------------------------------------
// Boundaries are inclusive at the OK end: "ok >= -65 dBm" means -65 is ok.

TEST(rssi_status_boundaries) {
  CHECK(rssi_status(-40.0) == Status::kOk);
  CHECK(rssi_status(-65.0) == Status::kOk);
  CHECK(rssi_status(-65.1) == Status::kCaution);
  CHECK(rssi_status(-80.0) == Status::kCaution);
  CHECK(rssi_status(-80.1) == Status::kCritical);
  CHECK(rssi_status(-120.0) == Status::kCritical);
}

TEST(snr_status_boundaries) {
  CHECK(snr_status(30.0) == Status::kOk);
  CHECK(snr_status(12.0) == Status::kOk);
  CHECK(snr_status(11.9) == Status::kCaution);
  CHECK(snr_status(6.0) == Status::kCaution);
  CHECK(snr_status(5.9) == Status::kCritical);
}

// Worst-of wins: a strong signal with terrible SNR is not a good link.
TEST(card_status_takes_the_worst_of_rssi_and_snr) {
  GsCard c;
  c.heard = true;
  c.rssi_dbm = -40.0;   // ok
  c.snr_db = 4.0;       // critical
  CHECK(card_status(c) == Status::kCritical);
  c.snr_db = 8.0;       // caution
  CHECK(card_status(c) == Status::kCaution);
  c.snr_db = 20.0;      // ok
  CHECK(card_status(c) == Status::kOk);
  c.rssi_dbm = -90.0;   // critical
  CHECK(card_status(c) == Status::kCritical);
}

// An unheard card has no status to report and must not read as critical --
// it renders unlit with "never heard" instead.
TEST(unheard_card_is_ok_status_and_zero_bars) {
  GsCard c;
  c.heard = false;
  CHECK(card_status(c) == Status::kOk);
  CHECK(card_bars(c) == 0);
}

TEST(rssi_bars_map_six_steps_over_minus90_to_minus45) {
  CHECK(rssi_bars(-95.0) == 0);
  CHECK(rssi_bars(-90.0) == 0);
  CHECK(rssi_bars(-40.0) == 6);
  CHECK(rssi_bars(-45.0) == 6);
  // -58 is (90-58)/45 = 0.711 of the range -> 4 of 6
  CHECK(rssi_bars(-58.0) == 4);
  // -71 is (90-71)/45 = 0.422 -> 3 of 6 (the handoff's caution sample)
  CHECK(rssi_bars(-71.0) == 3);
  // Monotone, and never out of range.
  int prev = -1;
  for (int d = -100; d <= -30; ++d) {
    const int b = rssi_bars((double)d);
    CHECK(b >= 0 && b <= 6);
    CHECK(b >= prev);
    prev = b;
  }
}

TEST(airtime_status_cautions_at_the_ceiling) {
  CHECK(airtime_status(0.0) == Status::kOk);
  CHECK(airtime_status(74.9) == Status::kOk);
  CHECK(airtime_status(75.0) == Status::kCaution);
  CHECK(airtime_status(120.0) == Status::kCaution);
}

// Zero post-FEC loss is the only OK state -- any unrecovered loss is
// picture damage, and >1% is where it becomes unflyable.
TEST(post_loss_status_treats_any_loss_as_at_least_caution) {
  CHECK(post_loss_status(0.0) == Status::kOk);
  CHECK(post_loss_status(0.01) == Status::kCaution);
  CHECK(post_loss_status(1.0) == Status::kCaution);
  CHECK(post_loss_status(1.01) == Status::kCritical);
}

// Critical uses the caution hue: the essential OSD has no third colour,
// escalation beyond it is the alert layer's job.
TEST(critical_renders_in_the_caution_colour) {
  CHECK(status_rgb(Status::kOk) == tok::kStatusOk);
  CHECK(status_rgb(Status::kCaution) == tok::kStatusCaution);
  CHECK(status_rgb(Status::kCritical) == tok::kStatusCaution);
}

// --- formatting -------------------------------------------------------

TEST(fmt_int_rounds_to_nearest) {
  CHECK(fmt_int(0.0) == "0");
  CHECK(fmt_int(60.4) == "60");
  CHECK(fmt_int(60.5) == "61");
  CHECK(fmt_int(-0.4) == "0");
}

TEST(fmt_one_dp_always_keeps_the_decimal) {
  CHECK(fmt_one_dp(0.0) == "0.0");
  CHECK(fmt_one_dp(2.14) == "2.1");
  CHECK(fmt_one_dp(2.15) == "2.2");
  CHECK(fmt_one_dp(24.6) == "24.6");
  CHECK(fmt_one_dp(100.0) == "100.0");
}

// RSSI uses a TRUE minus sign (U+2212), never a hyphen -- the design is
// explicit, and the hyphen is visually much shorter in this typeface.
TEST(fmt_signed_int_uses_a_true_minus_sign) {
  CHECK(fmt_signed_int(-58.0) == std::string(kMinus) + "58");
  CHECK(fmt_signed_int(-58.4) == std::string(kMinus) + "58");
  CHECK(fmt_signed_int(0.0) == "0");
  CHECK(fmt_signed_int(12.0) == "12");
  CHECK(fmt_signed_int(-58.0).find('-') == std::string::npos);
}

TEST(fmt_clock_is_mm_ss_and_saturates) {
  CHECK(fmt_clock(0) == "00:00");
  CHECK(fmt_clock(9) == "00:09");
  CHECK(fmt_clock(767) == "12:47");
  CHECK(fmt_clock(3599) == "59:59");
  // Past an hour it keeps counting minutes rather than wrapping to 00:00,
  // which would read as "recording just started" mid-flight.
  CHECK(fmt_clock(3600) == "60:00");
  CHECK(fmt_clock(5999) == "99:59");
  CHECK(fmt_clock(6000) == "99:59");   // saturate, never widen the box
  CHECK(fmt_clock(-5) == "00:00");
}

TEST(em_dash_pair_is_the_never_received_rendering) {
  CHECK(std::string(kEmDashPair) == "——");
}

MTEST_MAIN
