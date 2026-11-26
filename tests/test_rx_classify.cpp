// rx_classify.h: route, cleanliness and foreign-version flag of one RX frame.
#include <vector>
#include "mtest.h"
#include "rx_classify.h"
using namespace mabur;

// Only the 4-byte head matters to the classifier; pad to a plausible body.
static std::vector<uint8_t> body(uint8_t ver, uint8_t type) {
  std::vector<uint8_t> b(24, 0);
  b[0] = rc::RC_MAGIC & 0xff;
  b[1] = rc::RC_MAGIC >> 8;
  b[2] = ver;
  b[3] = type;
  return b;
}
static RxVerdict run(const std::vector<uint8_t>& b, bool crc_err = false) {
  return classify_rx(b.data(), b.size(), crc_err);
}

TEST(nack_routes_to_nack) {
  const auto v = run(body(rc::RC_VERSION, rc::T_NACK));
  CHECK(v.route == RxRoute::Nack);
  CHECK(v.clean_own);
}
TEST(mav_routes_to_mav) {
  CHECK(run(body(rc::RC_VERSION, rc::T_MAV)).route == RxRoute::Mav);
}
TEST(mav_of_other_version_routes_to_mav) {
  CHECK(run(body(rc::RC_VERSION + 1, rc::T_MAV)).route == RxRoute::Mav);
}
TEST(disc_of_other_version_routes_to_rc) {
  CHECK(run(body(rc::RC_VERSION + 1, rc::T_DISC)).route == RxRoute::Rc);
}
TEST(rcf_routes_to_rc) {
  CHECK(run(body(rc::RC_VERSION, rc::T_RCF)).route == RxRoute::Rc);
}
TEST(crc_failed_own_frame_still_routes_but_is_not_clean) {
  const auto v = run(body(rc::RC_VERSION, rc::T_RCF), true);
  CHECK(v.route == RxRoute::Rc);
  CHECK(!v.clean_own);
}
TEST(foreign_clean_frame_counts_foreign) {
  const std::vector<uint8_t> b(24, 0x5a);
  const auto v = run(b);
  CHECK(v.route == RxRoute::Foreign);
  CHECK(!v.foreign_version);
  CHECK(!v.clean_own);
}
TEST(foreign_rc_version_flags_version) {
  const auto v = run(body(rc::RC_VERSION + 1, rc::T_RCF));
  CHECK(v.route == RxRoute::Foreign);
  CHECK(v.foreign_version);
}
TEST(crc_failed_foreign_is_dropped) {
  const std::vector<uint8_t> b(24, 0x5a);
  CHECK(run(b, true).route == RxRoute::Drop);
  CHECK(run(body(rc::RC_VERSION + 1, rc::T_RCF), true).route == RxRoute::Drop);
}

MTEST_MAIN
