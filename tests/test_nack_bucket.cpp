// TokenBucket: the air cap on NACK re-sends (spec 2026-10-05 fec-nack §4.3),
// in symbols. A request beyond the tokens is refused, never queued; a
// repeat-flagged (doubled) send draws two tokens per symbol.
#include "nack_bucket.h"
#include "mtest.h"

using mabur::TokenBucket;

TEST(bucket_refuses_beyond_depth) {
  TokenBucket b(64);
  b.refill(0, 1000.0);           // first call seeds the clock; tokens start full
  CHECK(b.tokens() == 64);
  CHECK(b.take(24) && b.take(24));
  CHECK(!b.take(24));            // 16 left: refused whole, nothing taken
  CHECK(b.tokens() == 16);
  b.refill(1000000, 1000.0);     // 1 s at 1000/s -> clamp at depth
  CHECK(b.tokens() == 64);
}

TEST(bucket_refills_at_rate) {
  TokenBucket b(64);
  b.refill(0, 1000.0);
  CHECK(b.take(64));
  CHECK(b.tokens() == 0);
  b.refill(10000, 1000.0);       // 10 ms at 1000/s = 10 tokens
  CHECK(b.tokens() > 9.999 && b.tokens() < 10.001);
}

TEST(doubled_send_draws_two) {
  TokenBucket b(4);
  b.refill(0, 0.0);
  CHECK(b.take(2));              // one doubled symbol = 2 tokens
  CHECK(b.take(2));
  CHECK(!b.take(2));
}

MTEST_MAIN
