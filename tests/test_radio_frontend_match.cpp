// usb_id_matches (gs/src/radio_frontend.h): the device-identity rule
// open_and_start() applies, without libusb.
#include "mtest.h"
#include "radio_frontend.h"
using namespace maburgs;

TEST(default_cfg_scans_realtek_pids) {
  RadioFrontend::Cfg c;                       // vid 0bda, pid 0 = scan list
  CHECK(usb_id_matches(c, 0x0bda, 0xa81a));
  CHECK(usb_id_matches(c, 0x0bda, 0x8812));
  CHECK(!usb_id_matches(c, 0x0bda, 0x0001));
  CHECK(!usb_id_matches(c, 0x2357, 0xa81a));  // other vendor
}
TEST(explicit_pid_is_exact) {
  RadioFrontend::Cfg c; c.usb_pid = 0x881a;
  CHECK(usb_id_matches(c, 0x0bda, 0x881a));
  CHECK(!usb_id_matches(c, 0x0bda, 0xa81a));
}
TEST(ids_list_overrides_vid_pid) {
  RadioFrontend::Cfg c; c.ids = {{0x0bda, 0xa81a}, {0x2357, 0x0120}};
  CHECK(usb_id_matches(c, 0x2357, 0x0120));
  CHECK(usb_id_matches(c, 0x0bda, 0xa81a));
  CHECK(!usb_id_matches(c, 0x0bda, 0x8812));  // not in the list, scan list ignored
}
MTEST_MAIN
