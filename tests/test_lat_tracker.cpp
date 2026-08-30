#include <cstdint>

#include "au_ring.h"
#include "lat_tracker.h"
#include "mtest.h"

using maburgs::AuRecordMeta;
using maburplay::LatTracker;

namespace {

constexpr uint32_t kFrameStep = 16'684;  // 59.94 fps pts step
constexpr uint64_t kC = 100'000;         // warm-up's pinned anchor offset (mono - pts64)
constexpr int kWarmFrames = 33;          // > PtsAnchor::kWarmFrames (32)

// Warms the tracker's internal PtsAnchor with kWarmFrames synthetic
// submissions whose arrival sits exactly on a constant offset (kC) from
// pts -- see lat_tracker.cpp's on_submit(): the offset pins the anchor's
// floor to exactly kC (each step leaks it up, then the next sample proves
// the same floor again and snaps it back down), so every later real frame
// in a test can compute its expected span by hand. Warm frames are never
// completed (no decode/present/flip): they simply occupy inflight slots,
// well under kMaxInflight, and never enter flush_line()'s window.
void warm(LatTracker& lat) {
  for (int i = 0; i < kWarmFrames; ++i) {
    AuRecordMeta m;
    m.pts_us = static_cast<uint32_t>(i * kFrameStep);
    m.t_first_us = m.pts_us + kC;
    m.t_complete_us = m.t_first_us + 1000;
    m.drone_q_ms = 0;
    m.enc_us = 0;
    lat.on_submit(m, /*mono_us=*/0);
  }
}

}  // namespace

TEST(full_chain_segments_and_e2e) {
  LatTracker lat;
  warm(lat);

  // pts64 unwraps to 1000 (see warm()'s comment): the real frame's pts is
  // far below the warm loop's last pts32 but within PtsAnchor::kResyncUs,
  // so it is treated as an ordinary backward step, not a discontinuity,
  // and pts64_ becomes exactly 1000. map_us(1000) == kC + 1000 == 101000.
  AuRecordMeta m;
  m.pts_us = 1000;
  m.t_first_us = 106'000;  // span = 106000 - 101000 = 5000
  m.t_complete_us = m.t_first_us + 3000;  // fec = 3000
  m.drone_q_ms = 2;                       // dq_us = 2000
  m.enc_us = 1500;                        // enc_us = 1500

  lat.on_submit(m, /*mono_us=*/0);
  lat.on_decoded(1000, /*mono_us=*/111'000);  // dec = 111000 - 109000 = 2000
  lat.on_present(1000, /*mono_us=*/112'000);  // reg = 112000 - 111000 = 1000
  lat.on_flip(1000, /*flip_mono_us=*/116'000, /*exact=*/true);  // dsp = 4000

  const auto L = lat.flush_line();
  CHECK(L.n == 1);
  CHECK(L.anchor_ok);
  CHECK(L.dsp_exact);
  CHECK(L.p50[0] == 1500);  // enc
  CHECK(L.p50[1] == 2000);  // dq
  CHECK(L.p50[2] == 1500);  // air (span 5000 - enc 1500 - dq 2000)
  CHECK(L.p50[3] == 3000);  // fec
  CHECK(L.p50[4] == 2000);  // dec
  CHECK(L.p50[5] == 1000);  // reg
  CHECK(L.p50[6] == 4000);  // dsp
  const uint32_t sum = L.p50[0] + L.p50[1] + L.p50[2] + L.p50[3] + L.p50[4] + L.p50[5] + L.p50[6];
  CHECK(L.p50[7] == sum);  // e2e == sum of the other 7
  CHECK(sum == 15'000);
  // p99 == p50 with a single sample.
  for (int i = 0; i < 8; ++i) CHECK(L.p99[i] == L.p50[i]);
}

TEST(slow_encode_fast_transit_attributes_to_enc_not_air) {
  // Enc-excess semantics: the anchor consumes the encode/queue-CORRECTED
  // arrival (t_first - enc - dq), so a frame that spent 8 ms in the encoder
  // and queue yet arrived only 1 ms past the old floor proves its TRANSIT
  // was faster than the floor holder's -- it reports its full enc/dq, reads
  // air 0, and legitimately snaps the transit floor down. The old clamped
  // formula would have laundered the 8 ms into a truncated enc and pinned
  // every later frame's air at 0.
  LatTracker lat;
  warm(lat);

  AuRecordMeta m;
  m.pts_us = 1000;
  m.t_first_us = 102'000;  // adj = 102000 - 5000 - 3000 = 94000
  m.t_complete_us = m.t_first_us + 500;
  m.drone_q_ms = 3;
  m.enc_us = 5000;

  lat.on_submit(m, /*mono_us=*/0);
  lat.on_decoded(1000, m.t_complete_us + 100);
  lat.on_present(1000, m.t_complete_us + 200);
  lat.on_flip(1000, m.t_complete_us + 300, /*exact=*/true);

  const auto L = lat.flush_line();
  CHECK(L.n == 1);
  CHECK(L.p50[0] == 5000);  // enc: full wire value, never clamped away
  CHECK(L.p50[1] == 3000);  // dq: full wire value
  CHECK(L.p50[2] == 0);     // air: this frame IS the new transit floor
  // enc + dq + air still equals t_first - map(pts) -- additive invariant:
  CHECK(L.p50[0] + L.p50[1] + L.p50[2] == 102'000 - 94'000);

  // A follow-up plain frame (enc/dq unknown) arriving at the ORIGINAL
  // cadence now reads its transit excess against the corrected floor.
  AuRecordMeta f;
  f.pts_us = 2000;
  f.t_first_us = 102'000 + 1000;  // adj = 103000; map(2000) = 93000+2000
  f.t_complete_us = f.t_first_us + 400;
  lat.on_submit(f, /*mono_us=*/0);
  lat.on_decoded(2000, f.t_complete_us + 100);
  lat.on_present(2000, f.t_complete_us + 200);
  lat.on_flip(2000, f.t_complete_us + 300, /*exact=*/true);
  const auto L2 = lat.flush_line();
  CHECK(L2.n == 1);
  CHECK(L2.p50[2] == 8000);  // air = 103000 - 95000: the counterfactual floor
}

TEST(implausible_wire_durations_do_not_feed_anchor_or_window) {
  // A dq beyond the TxQueue's own drop-oldest bound cannot be real (the
  // wire bytes are outside the sub-block CRCs and FCS-corrupt bodies are
  // processed by design) -- such a frame must neither drag the snap-down
  // floor nor join the window. kMaxAnchorAdjustUs is the plausibility cap.
  LatTracker lat;
  warm(lat);

  AuRecordMeta g;
  g.pts_us = 1000;
  g.t_first_us = 106'000;
  g.t_complete_us = g.t_first_us + 500;
  g.drone_q_ms = 400;  // 400 ms "queue wait": impossible, cap is ~150 ms
  g.enc_us = 0;
  lat.on_submit(g, /*mono_us=*/0);
  lat.on_decoded(1000, g.t_complete_us + 100);
  lat.on_present(1000, g.t_complete_us + 200);
  lat.on_flip(1000, g.t_complete_us + 300, /*exact=*/true);
  const auto Lg = lat.flush_line();
  CHECK(Lg.n == 0);  // excluded, exactly like an unknown t_first

  // Floor unmoved: a plain frame still reads its honest span.
  AuRecordMeta f;
  f.pts_us = 2000;
  f.t_first_us = 104'500;  // adj = 104500; map(2000) = 100000+2000
  f.t_complete_us = f.t_first_us + 400;
  lat.on_submit(f, /*mono_us=*/0);
  lat.on_decoded(2000, f.t_complete_us + 100);
  lat.on_present(2000, f.t_complete_us + 200);
  lat.on_flip(2000, f.t_complete_us + 300, /*exact=*/true);
  const auto L2 = lat.flush_line();
  CHECK(L2.n == 1);
  CHECK(L2.p50[2] == 2500);  // 104500 - 102000, against the UNPOISONED floor
}

TEST(unknown_wire_values_subtract_nothing) {
  LatTracker lat;
  warm(lat);

  AuRecordMeta m;
  m.pts_us = 1000;
  m.t_first_us = 103'500;  // span = 103500 - 101000 = 2500
  m.t_complete_us = m.t_first_us + 700;
  m.drone_q_ms = 0;  // unknown (wire 0)
  m.enc_us = 0;      // unknown (wire 0)

  lat.on_submit(m, /*mono_us=*/0);
  lat.on_decoded(1000, m.t_complete_us + 10);
  lat.on_present(1000, m.t_complete_us + 20);
  lat.on_flip(1000, m.t_complete_us + 30, /*exact=*/true);

  const auto L = lat.flush_line();
  CHECK(L.n == 1);
  CHECK(L.p50[0] == 0);     // enc: unknown -> subtract nothing
  CHECK(L.p50[1] == 0);     // dq: unknown -> subtract nothing
  CHECK(L.p50[2] == 2500);  // air: the full span
}

TEST(p99_frame_is_the_real_outliers_own_tuple) {
  LatTracker lat;
  warm(lat);

  // 99 baseline frames whose "air" segment climbs (20ms..118ms, distinct
  // per frame) but whose e2e never approaches the outlier's, plus ONE
  // outlier with a small air but a huge dsp -- e2e ranks it p99, but its
  // OWN air (5ms) is far below the per-column p99 air (118ms, from a
  // baseline frame). A "sum independently-ranked percentiles" bug would
  // report the baseline's 118ms; the real-frame rule must report 5ms.
  // pts must climb in small steps (< kLeakPpm's ~16.7 ms/step threshold)
  // from warm()'s last pts (32 * kFrameStep) so the anchor's floor stays
  // pinned exactly at kC with zero leak -- see warm()'s comment. Each
  // frame's t_first is then set as kC + pts + <desired span>, which
  // on_submit's span = t_first - anchor.map_us(pts64) resolves back to
  // exactly <desired span>, independent of the actual pts value.
  const uint32_t base_pts = 32 * kFrameStep;
  for (int i = 0; i < 99; ++i) {
    AuRecordMeta m;
    m.pts_us = base_pts + static_cast<uint32_t>((i + 1) * 2000);
    const uint64_t air = 20'000 + static_cast<uint64_t>(i) * 1000;
    // span = enc(10000) + dq(50000) + air; both comfortably under their
    // own budgets, so the clamp never engages. enc_us is a wire uint16_t
    // (max 65535), so it stays small here.
    const uint64_t span = 10'000 + 50'000 + air;
    m.t_first_us = kC + m.pts_us + span;
    m.t_complete_us = m.t_first_us + 30'000;  // fec = 30ms
    m.drone_q_ms = 50;                        // dq_us = 50000
    m.enc_us = 10'000;                        // enc_us = 10000
    lat.on_submit(m, 0);
    lat.on_decoded(m.pts_us, m.t_complete_us + 10'000);  // dec = 10ms
    lat.on_present(m.pts_us, m.t_complete_us + 15'000);  // reg = 5ms
    lat.on_flip(m.pts_us, m.t_complete_us + 20'000, true);  // dsp = 5ms
  }

  AuRecordMeta o;
  o.pts_us = base_pts + 100 * 2000;
  {
    const uint64_t span = 10'000 + 50'000 + 5'000;  // air = 5000 (5ms)
    o.t_first_us = kC + o.pts_us + span;
  }
  o.t_complete_us = o.t_first_us + 30'000;  // fec = 30ms
  o.drone_q_ms = 50;                        // dq_us = 50000
  o.enc_us = 10'000;                        // enc_us = 10000
  lat.on_submit(o, 0);
  lat.on_decoded(o.pts_us, o.t_complete_us + 10'000);       // dec = 10ms
  lat.on_present(o.pts_us, o.t_complete_us + 15'000);       // reg = 5ms
  lat.on_flip(o.pts_us, o.t_complete_us + 515'000, true);   // dsp = 500ms

  const auto L = lat.flush_line();
  CHECK(L.n == 100);

  const auto B = lat.p99_frame();
  CHECK(B.valid);
  CHECK(B.ms[0] == 10);   // enc
  CHECK(B.ms[1] == 50);   // dq
  CHECK(B.ms[2] == 5);    // air -- the OUTLIER's own 5ms, not the 118ms column p99
  CHECK(B.ms[3] == 30);   // fec
  CHECK(B.ms[4] == 10);   // dec
  CHECK(B.ms[5] == 5);    // reg
  CHECK(B.ms[6] == 500);  // dsp
  CHECK(B.ms[7] == 610);  // e2e
}

TEST(chk_is_zero_on_clean_synthetic_data) {
  LatTracker lat;
  warm(lat);

  for (int i = 0; i < 10; ++i) {
    AuRecordMeta m;
    m.pts_us = static_cast<uint32_t>(500'000 + i * kFrameStep);
    m.t_first_us = 101'000 + 4000;  // span = 4000 (well under enc+dq budget)
    m.t_complete_us = m.t_first_us + 3000;
    m.drone_q_ms = 1;    // dq_us = 1000
    m.enc_us = 1000;     // enc_us = 1000
    lat.on_submit(m, 0);
    lat.on_decoded(m.pts_us, m.t_complete_us + 2000);
    lat.on_present(m.pts_us, m.t_complete_us + 3000);
    lat.on_flip(m.pts_us, m.t_complete_us + 4000, true);
  }

  const auto L = lat.flush_line();
  CHECK(L.n == 10);
  CHECK(L.chk_ms > -0.001 && L.chk_ms < 0.001);
}

TEST(dropped_frame_is_excluded_and_forgotten) {
  LatTracker lat;
  warm(lat);

  AuRecordMeta m;
  m.pts_us = 42;
  m.t_first_us = 105'000;
  m.t_complete_us = m.t_first_us + 1000;
  m.drone_q_ms = 1;
  m.enc_us = 500;
  lat.on_submit(m, 0);
  lat.on_decoded(42, m.t_complete_us + 100);
  lat.on_drop(42);  // e.g. displaced by the regulator's mailbox

  // A dropped frame that later still reports a flip (should not happen,
  // but the map no longer knows about pts 42) must be a silent no-op.
  lat.on_present(42, m.t_complete_us + 200);
  lat.on_flip(42, m.t_complete_us + 300, true);

  const auto L = lat.flush_line();
  CHECK(L.n == 0);
}

TEST(flush_all_resets_anchor_warmup) {
  LatTracker lat;
  warm(lat);
  lat.flush_all();  // discont/flush point: anchor goes cold, window clears

  // Anchor is cold immediately after the reset: this frame's on_submit
  // cannot compute a real span (anchor_ok_at_submit stays false), so
  // on_flip must exclude it from the window ENTIRELY -- not count it with
  // fabricated zero head-segments blended into the aggregates.
  AuRecordMeta cold;
  cold.pts_us = 1000;
  cold.t_first_us = 106'000;
  cold.t_complete_us = cold.t_first_us + 3000;
  cold.drone_q_ms = 2;
  cold.enc_us = 1500;
  lat.on_submit(cold, 0);
  lat.on_decoded(cold.pts_us, cold.t_complete_us + 2000);
  lat.on_present(cold.pts_us, cold.t_complete_us + 3000);
  lat.on_flip(cold.pts_us, cold.t_complete_us + 4000, true);

  auto L = lat.flush_line();
  CHECK(L.n == 0);  // the cold frame contributed nothing, not zeros

  // Re-warm the anchor post-reset (kWarmFrames samples, same pinned-floor
  // construction as warm()'s own comment), then submit one real frame:
  // the window must report it with its actual, non-zero segments.
  warm(lat);

  const uint32_t base_pts = 32 * kFrameStep;
  AuRecordMeta m2;
  m2.pts_us = base_pts + 2000;             // small step past warm()'s last pts
  m2.t_first_us = kC + m2.pts_us + 4000;   // span = 4000
  m2.t_complete_us = m2.t_first_us + 3000; // fec = 3000
  m2.drone_q_ms = 1;                       // dq_us = 1000
  m2.enc_us = 500;                         // enc_us = 500
  lat.on_submit(m2, 0);
  lat.on_decoded(m2.pts_us, m2.t_complete_us + 2000);   // dec = 2000
  lat.on_present(m2.pts_us, m2.t_complete_us + 3000);   // reg = 1000
  lat.on_flip(m2.pts_us, m2.t_complete_us + 4000, true);  // dsp = 1000

  L = lat.flush_line();
  CHECK(L.n == 1);
  CHECK(L.anchor_ok);
  CHECK(L.p50[0] == 500);   // enc
  CHECK(L.p50[1] == 1000);  // dq
  CHECK(L.p50[2] == 2500);  // air (span 4000 - enc 500 - dq 1000)
  CHECK(L.p50[3] == 3000);  // fec
}

// Pins the exact ingredients an OSD caller (maburplay/src/main.cpp's 1 Hz
// fill, Task 12 fix round 1) MUST AND together: p99_frame() alone is not
// enough to tell a fresh frame from a stale one, because p99_frame_ is a
// member that survives flush_all() -- flush_all() only clears map_/
// anchor_/completed_/the chk+dsp accumulators, not p99_frame_ itself (see
// its own comment). So immediately after a reset, p99_frame().valid can
// still read true from BEFORE the reset while flush_line().anchor_ok
// (same call) correctly reads false. A caller that reads bd.valid alone
// would keep showing the pre-reset breakdown as current.
TEST(p99_frame_stays_stale_after_reset_while_anchor_ok_goes_false) {
  LatTracker lat;
  warm(lat);

  AuRecordMeta m;
  m.pts_us = 1000;
  m.t_first_us = 106'000;                 // span = 5000
  m.t_complete_us = m.t_first_us + 3000;  // fec = 3000
  m.drone_q_ms = 2;                       // dq_us = 2000
  m.enc_us = 1500;                        // enc_us = 1500
  lat.on_submit(m, 0);
  lat.on_decoded(1000, 111'000);
  lat.on_present(1000, 112'000);
  lat.on_flip(1000, 116'000, true);

  const auto L1 = lat.flush_line();
  REQUIRE(L1.n == 1);
  REQUIRE(L1.anchor_ok);
  const auto B1 = lat.p99_frame();
  REQUIRE(B1.valid);  // the real ingredient the next flush_all() leaves behind

  lat.flush_all();  // discont/session reset: anchor goes cold, window clears

  // Nothing submitted since the reset, so this window is empty -- but the
  // point under test is anchor_ok, not n.
  const auto L2 = lat.flush_line();
  CHECK(L2.n == 0);
  CHECK(!L2.anchor_ok);  // the ingredient a caller must gate on

  const auto B2 = lat.p99_frame();
  CHECK(B2.valid);              // ...still true: p99_frame_ was NOT reset
  CHECK(B2.ms[7] == B1.ms[7]);  // ...and it's the SAME stale frame, unchanged
}

// t_first_us == 0 is an unknown stamp, not a real timestamp of zero (same
// convention as gs/src/main.cpp's own `if (lat.t_first_us)` gate). on_submit
// must skip anchor_.observe() entirely for such frames -- feeding a run of
// increasing pts with mono_us pinned at 0 into the anchor would otherwise
// warm it up (PtsAnchor::usable() only needs kWarmFrames samples, however
// bogus) after exactly this many frames, so this count is chosen deliberately
// to catch that regression.
TEST(zero_t_first_us_does_not_warm_anchor_or_join_window) {
  LatTracker lat;
  lat.flush_all();

  for (int i = 0; i < kWarmFrames; ++i) {
    AuRecordMeta m;
    m.pts_us = static_cast<uint32_t>(i * kFrameStep);
    m.t_first_us = 0;  // unknown stamp
    m.t_complete_us = 1000;
    m.drone_q_ms = 2;
    m.enc_us = 1500;
    lat.on_submit(m, /*mono_us=*/0);
    lat.on_decoded(m.pts_us, m.t_complete_us + 2000);
    lat.on_present(m.pts_us, m.t_complete_us + 3000);
    lat.on_flip(m.pts_us, m.t_complete_us + 4000, /*exact=*/true);
  }

  const auto L = lat.flush_line();
  CHECK(L.n == 0);          // none of the 33 frames joined the window
  CHECK(!L.anchor_ok);      // the anchor never warmed up on bogus stamps
}

MTEST_MAIN
