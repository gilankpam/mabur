#include "mtest.h"
#include "mabur/crc16.h"
TEST(crc16_known_value) {
  // CRC-16-CCITT-FALSE("123456789") == 0x29B1 (standard check value)
  const uint8_t d[] = {'1','2','3','4','5','6','7','8','9'};
  CHECK(mabur::crc16_ccitt(d, 9) == 0x29B1);
}
MTEST_MAIN
