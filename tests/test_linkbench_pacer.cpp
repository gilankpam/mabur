#include "mtest.h"
#include "pacer.h"
using namespace linkbench;

TEST(token_bucket_converges_to_rate) {
  // 1 MB/s, 62-byte packets, driven in 1 ms steps for 10 simulated seconds.
  TokenBucket tb(1'000'000.0, 4096.0);
  uint64_t sent = 0;
  for (uint64_t ms = 1; ms <= 10'000; ++ms) {
    tb.advance(ms * 1000);
    while (tb.spend(62)) ++sent;
  }
  const uint64_t expect = 10'000'000ull / 62;  // 161290
  CHECK(sent >= expect - 100);
  CHECK(sent <= expect + 100);
}

TEST(token_bucket_burst_bounded) {
  TokenBucket tb(1'000'000.0, 4096.0);
  tb.advance(5'000'000);  // 5 s idle — tokens must cap at the burst, not 5 MB
  uint64_t burst = 0;
  while (tb.spend(62)) ++burst;
  CHECK(burst <= 4096 / 62 + 1);
}

TEST(token_bucket_spend_refuses_when_broke) {
  TokenBucket tb(1000.0, 100.0);
  tb.advance(1000);  // 1 ms at 1000 B/s = 1 byte
  CHECK(!tb.spend(62));
}

TEST(token_bucket_time_going_backwards_is_ignored) {
  TokenBucket tb(1'000'000.0, 4096.0);
  tb.advance(2000);
  const double before = tb.tokens();
  tb.advance(1000);  // non-monotonic input must not mint or burn tokens
  CHECK(tb.tokens() == before);
}

MTEST_MAIN
