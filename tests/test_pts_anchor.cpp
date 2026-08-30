#include <cstdint>

#include "mtest.h"
#include "pts_anchor.h"

using maburgs::PtsAnchor;

namespace {

constexpr int64_t kFrame = 16'684;  // 59.94 fps pts step, µs
constexpr uint64_t kT0 = 5'000'000;  // arbitrary mono anchor

}  // namespace

// (a) unwrap: feed pts 0xFFFF0000 then 0x00010000 (wraps forward) -> pts64
//     advances by 0x20000, no discont.
TEST(unwrap_forward_wrap) {
  PtsAnchor a;
  const uint32_t p0 = 0xFFFF0000u;
  const auto o0 = a.observe(p0, kT0);
  CHECK(!o0.discont);
  CHECK(o0.pts64 == p0);

  const uint32_t p1 = 0x00010000u;  // wrapped forward by 0x20000
  const auto o1 = a.observe(p1, kT0 + 0x20000);
  CHECK(!o1.discont);
  CHECK(o1.pts64 == static_cast<uint64_t>(p0) + 0x20000u);
}

// (b) snap-down: base follows the minimum of (mono - pts64) instantly.
TEST(snap_down_to_minimum_offset) {
  PtsAnchor a;
  // First sample seeds base = mono - pts64 = kT0 - 0 = kT0.
  a.observe(0, kT0);
  CHECK(a.base_valid());
  CHECK(a.base_us() == static_cast<int64_t>(kT0));

  // Next frame arrives with a SMALLER offset (faster arrival): base must
  // snap down instantly to the new minimum.
  const uint64_t mono1 = kT0 + kFrame - 3'000;  // 3ms faster than provisional
  const auto o1 = a.observe(static_cast<uint32_t>(kFrame), mono1);
  CHECK(!o1.discont);
  const int64_t expect_base = static_cast<int64_t>(mono1) - kFrame;
  CHECK(a.base_us() == expect_base);
  CHECK(a.base_us() < static_cast<int64_t>(kT0));

  // A subsequent frame at the SAME pts (d == 0, so the leak does not fire)
  // but with a LARGER offset must NOT move the base back up (no snap-up).
  const uint64_t mono2 = kT0 + kFrame + 5'000;  // slower than the floor
  a.observe(static_cast<uint32_t>(kFrame), mono2);
  CHECK(a.base_us() == expect_base);
}

// (c) leak: two samples 1 s apart with off > base -> base rises by exactly
//     kLeakPpm (integer math d * 60 / 1'000'000 per sample, same as the
//     regulator's line 38 -- assert the exact integer result, not ~=).
TEST(leak_exact_integer_arithmetic) {
  PtsAnchor a;
  a.observe(0, kT0);  // seeds base = kT0
  CHECK(a.base_us() == static_cast<int64_t>(kT0));

  // Second sample 1s later in pts AND mono, but mono offset stays exactly
  // at the floor + the leak would want to apply (base_valid_ && d > 0).
  const int64_t d = 1'000'000;  // 1s pts delta
  const uint64_t mono1 = kT0 + static_cast<uint64_t>(d) + 10'000;  // off > base
  const auto o1 = a.observe(static_cast<uint32_t>(d), mono1);
  CHECK(!o1.discont);

  // Expected: leak applied BEFORE the min-update.
  //   base_us_ += d * kLeakPpm / 1'000'000  =>  base += 1'000'000*60/1'000'000 = 60
  //   off = mono1 - pts64(=d) = kT0 + 10'000
  //   off (kT0+10000) > base (kT0+60) so no snap-down; base stays at kT0+60.
  const int64_t expect_base =
      static_cast<int64_t>(kT0) + (d * PtsAnchor::kLeakPpm / 1'000'000);
  CHECK(expect_base == static_cast<int64_t>(kT0) + 60);
  CHECK(a.base_us() == expect_base);

  // Ordering pin: a third sample whose `off` lands STRICTLY BETWEEN the
  // pre-leak base (kT0+60) and the post-leak base (kT0+60+60=kT0+120).
  // This is the case the previous two assertions cannot distinguish --
  // both "leak then min" and "min then leak" pass them, because in both
  // those samples `off` was either below both candidate bases or above
  // both. Here the two orderings diverge:
  //   correct (leak BEFORE min-check): base becomes kT0+120 first, then
  //     off(kT0+90) < kT0+120 is true -> snaps DOWN to off: base=kT0+90.
  //   swapped (min-check BEFORE leak): off(kT0+90) < kT0+60 is false (no
  //     snap), THEN leak applies: base = kT0+60+60 = kT0+120.
  // The two orders disagree (kT0+90 vs kT0+120), so this sample alone
  // pins the order. Verified by mutation: swapping the leak/min-check
  // statements in pts_anchor.h makes only this assertion fail.
  const int64_t off3 = static_cast<int64_t>(kT0) + 90;
  const uint64_t pts64_before3 = static_cast<uint64_t>(d);  // 1'000'000
  const uint32_t pts32_3 =
      static_cast<uint32_t>(pts64_before3 + static_cast<uint64_t>(d));
  const uint64_t mono3 =
      static_cast<uint64_t>(off3) + pts64_before3 + static_cast<uint64_t>(d);
  const auto o2 = a.observe(pts32_3, mono3);
  CHECK(!o2.discont);
  CHECK(a.base_us() == off3);
}

// (d) discont: delta-pts of 3s -> discont=true, base_valid()==false, usable()
//     false until 32 more frames.
TEST(discont_resets_and_requires_warmup) {
  PtsAnchor a;
  a.observe(0, kT0);
  CHECK(a.base_valid());
  CHECK(!a.usable());  // only 1 sample so far, kWarmFrames == 32

  const uint32_t jumped = 3'000'000u;  // 3s jump, beyond kResyncUs (2s)
  const auto o1 = a.observe(jumped, kT0 + 2'000);
  CHECK(o1.discont);
  CHECK(!a.base_valid());
  CHECK(!a.usable());

  // The next sample re-seeds base (matches the first-sample path) but
  // usable() must stay false until kWarmFrames samples have been observed
  // since the reset.
  a.observe(jumped, kT0 + 20'000);
  CHECK(a.base_valid());
  CHECK(!a.usable());

  for (uint32_t i = 1; i < PtsAnchor::kWarmFrames; ++i) {
    const uint32_t pts = jumped + static_cast<uint32_t>(i * kFrame);
    a.observe(pts, kT0 + 20'000 + static_cast<uint64_t>(i) * kFrame);
  }
  CHECK(a.usable());
}

MTEST_MAIN
