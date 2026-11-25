#include <deque>
#include <stdexcept>

#include "../drone/src/busy_window.h"
#include "mtest.h"

using mabur::BusyWindow;

namespace {
struct FakeDev : BusyWindow::Dev {
  std::deque<int> readings;
  bool arm_ok = true, throw_read = false, throw_arm = false;
  int arms = 0, reads = 0;
  uint32_t last_window = 0;
  int read_busy_pct() override {
    ++reads;
    if (throw_read) throw std::runtime_error("usb");
    if (readings.empty()) return -1;
    const int v = readings.front();
    readings.pop_front();
    return v;
  }
  bool arm(uint32_t w) override {
    ++arms;
    last_window = w;
    if (throw_arm) throw std::runtime_error("usb");
    return arm_ok;
  }
};
}  // namespace

TEST(first_eligible_tick_arms_no_reading) {
  BusyWindow b;
  FakeDev d;
  bool u = false;
  CHECK(b.step(5000, true, d, &u) == -1);
  CHECK(d.arms == 1 && d.reads == 0 && !u);
  CHECK(d.last_window == 240000);
}

TEST(reads_after_a_second) {
  BusyWindow b;
  FakeDev d;
  d.readings = {37};
  bool u = false;
  b.step(5000, true, d, &u);
  CHECK(b.step(5900, true, d, &u) == -1);  // too early: nothing read, no new arm
  CHECK(d.reads == 0 && d.arms == 1);
  CHECK(b.step(6000, true, d, &u) == 37);
  CHECK(d.reads == 1 && d.arms == 2);
}

TEST(reading_expires_after_two_seconds) {
  BusyWindow b;
  FakeDev d;
  d.readings = {37};
  d.arm_ok = true;
  bool u = false;
  b.step(5000, true, d, &u);
  CHECK(b.step(6000, true, d, &u) == 37);
  CHECK(b.step(8000, true, d, &u) == -1);  // second read has no valid value
  d.readings = {50};
  CHECK(b.step(9000, true, d, &u) == 50);
}

TEST(reading_holds_between_reads) {
  BusyWindow b;
  FakeDev d;
  d.readings = {37};
  bool u = false;
  b.step(5000, true, d, &u);
  CHECK(b.step(6000, true, d, &u) == 37);
  CHECK(b.step(6999, true, d, &u) == 37);  // no read due yet; value 999 ms old
}

TEST(ineligible_resets) {
  BusyWindow b;
  FakeDev d;
  d.readings = {37, 40};
  bool u = false;
  b.step(5000, true, d, &u);
  CHECK(b.step(5100, false, d, &u) == -1);
  CHECK(d.arms == 1 && d.reads == 0);
  // Disarmed and read time cleared: next eligible tick arms, does not read.
  CHECK(b.step(5200, true, d, &u) == -1);
  CHECK(d.arms == 2 && d.reads == 0);
}

TEST(unsupported_reports_once_and_stays_off) {
  BusyWindow b;
  FakeDev d;
  d.arm_ok = false;
  bool u = false;
  CHECK(b.step(5000, true, d, &u) == -1);
  CHECK(u);
  u = false;
  CHECK(b.step(7000, true, d, &u) == -1);
  CHECK(!u && d.arms == 1);
}

TEST(throw_drops_reading_and_rearms_next_second) {
  BusyWindow b;
  FakeDev d;
  d.readings = {37};
  bool u = false;
  b.step(5000, true, d, &u);
  d.throw_read = true;
  CHECK(b.step(6000, true, d, &u) == -1);
  CHECK(d.arms == 1 && !u);  // read threw before re-arm
  d.throw_read = false;
  CHECK(b.step(7000, true, d, &u) == -1);  // disarmed: arms, no read
  CHECK(d.arms == 2 && d.reads == 1);
  d.throw_arm = true;
  CHECK(b.step(8000, true, d, &u) == -1);
  CHECK(!u);
}

MTEST_MAIN
