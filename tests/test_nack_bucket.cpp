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

TEST(depth_follows_the_rung_and_clamps_tokens) {
  // Depth is burst_ms of air at the CURRENT op (spec: a demote to a slow
  // rung must not burst a fast rung's worth of symbols), so the handler
  // re-sets it on every refill. Growing it leaves the tokens alone (they
  // refill toward the new ceiling); shrinking it clamps what is held.
  TokenBucket b(64);
  b.refill(0, 1000.0);
  CHECK(b.take(64));
  b.set_depth(146);
  b.refill(1000000, 1000.0);     // 1 s at 1000/s -> clamp at the NEW depth
  CHECK(b.tokens() == 146);
  b.set_depth(44);               // slower rung: a full bucket shrinks with it
  CHECK(b.tokens() == 44);
  CHECK(b.depth() == 44);
}

TEST(depth_for_is_burst_ms_of_symbols_at_the_rate) {
  // rung 0 (mcs0/20) delivers ~2185 sym/s: 20 ms of it is ~44 symbols;
  // rung 5 (mcs4/40) ~22500 sym/s -> ~450. Never below one symbol.
  CHECK(TokenBucket::depth_for(2185.0, 20) > 43.6 && TokenBucket::depth_for(2185.0, 20) < 43.8);
  CHECK(TokenBucket::depth_for(22500.0, 20) == 450.0);
  CHECK(TokenBucket::depth_for(0.0, 20) == 1.0);
}

MTEST_MAIN
