// PtsUnwrap: the u32 wire pts extended to the monotonic 64-bit pts MppBackend
// hands MPP (the h265d stream hal refuses appends whose pts is not newer than
// the newest picture it opened -- a raw u32 goes "late" at every 71.6-min
// wrap). Half-range rule: a step back of more than 2^31 is a forward wrap.
#include <cstdint>

#include "mtest.h"
#include "pts_unwrap.h"

using maburplay::PtsUnwrap;

namespace {
constexpr uint64_t k32 = uint64_t{1} << 32;
}

TEST(first_value_is_seeded_well_above_zero) {
  PtsUnwrap u;
  CHECK(u(0) == k32);
  PtsUnwrap v;
  CHECK(v(123456u) == k32 + 123456u);
}

TEST(monotonic_across_the_u32_wrap) {
  PtsUnwrap u;
  const uint32_t step = 16667;                   // one 60 fps frame
  uint32_t pts = 0xFFFFFFFFu - 3 * step;          // a few frames before the wrap
  uint64_t prev = u(pts);
  for (int i = 0; i < 10; ++i) {                  // straddles 0xFFFFFFFF -> 0
    pts += step;                                  // u32 arithmetic: wraps
    const uint64_t x = u(pts);
    CHECK(x > prev);
    CHECK(x - prev == step);
    prev = x;
  }
  CHECK(pts < 0x10000000u);                       // the input really wrapped
  CHECK(prev > k32 + 0xFFFFFFFFull);              // and the output carried
}

TEST(exact_boundary_values) {
  PtsUnwrap u;
  const uint64_t a = u(0xFFFFFFFFu);
  const uint64_t b = u(0u);
  CHECK(b == a + 1);
  const uint64_t c = u(1u);
  CHECK(c == a + 2);
}

TEST(small_step_back_is_not_a_forward_wrap) {
  PtsUnwrap u;
  const uint64_t a = u(1000000u);
  const uint64_t b = u(1016667u);
  const uint64_t back = u(1000000u);              // a re-sent earlier picture
  CHECK(back == a);                               // behind, NOT 2^32 ahead
  CHECK(back < b);
  const uint64_t c = u(1033334u);                 // and forward again from there
  CHECK(c == b + 16667u);
}

TEST(step_back_across_the_wrap_stays_behind) {
  PtsUnwrap u;
  const uint64_t a = u(0xFFFFFF00u);
  const uint64_t b = u(0x00000100u);              // forward over the wrap
  CHECK(b == a + 0x200u);
  const uint64_t back = u(0xFFFFFF00u);           // a re-send from before it
  CHECK(back == a);
}

TEST(half_range_rule_edges) {
  PtsUnwrap u;
  const uint64_t a = u(0u);
  const uint64_t fwd = u(0x7FFFFFFFu);            // +2^31-1: forward
  CHECK(fwd == a + 0x7FFFFFFFull);
  PtsUnwrap v;
  const uint64_t x = v(0x80000000u);
  const uint64_t y = v(0u);                       // delta exactly -2^31: treated as back
  CHECK(y + 0x80000000ull == x);
  const uint64_t z = v(0x80000001u);              // +2^31+1 from 0: a step back of 2^31-1
  CHECK(z < y + k32);
  CHECK(z + 0x7FFFFFFFull == y);
}

TEST(low_32_bits_round_trip) {
  PtsUnwrap u;
  const uint32_t seq[] = {5u, 70000u, 0x7FFFFFFFu, 0xFFFFFFF0u, 3u, 2u, 0x90000000u, 0xFFFFFFFFu, 0u};
  for (uint32_t p : seq) CHECK(static_cast<uint32_t>(u(p)) == p);
  // across many wraps, and as RK_S64 (what MPP stores) cast back to u32
  PtsUnwrap v;
  uint32_t p = 0xF0000000u;
  for (int i = 0; i < 20; ++i, p += 0x40000000u) {
    const int64_t s = static_cast<int64_t>(v(p));
    CHECK(s > 0);
    CHECK(static_cast<uint32_t>(s) == p);
  }
}

MTEST_MAIN
