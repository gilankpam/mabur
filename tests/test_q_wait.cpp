#include "../drone/src/q_wait.h"
#include "mtest.h"

using mabur::q_wait_ms;

TEST(unstamped_is_zero) { CHECK(q_wait_ms(0, 500) == 0); }

TEST(pop_not_after_push_is_zero) {
  CHECK(q_wait_ms(500, 500) == 0);
  CHECK(q_wait_ms(500, 400) == 0);
}

TEST(plain_wait) { CHECK(q_wait_ms(100, 130) == 30); }

TEST(clamped_to_u16) { CHECK(q_wait_ms(1, 100000) == 65535); }

MTEST_MAIN
