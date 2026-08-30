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

TEST(enc_larger_than_span_clamps_to_span) {
  LatTracker lat;
  warm(lat);

  AuRecordMeta m;
  m.pts_us = 1000;
  m.t_first_us = 102'000;  // span = 102000 - 101000 = 1000
  m.t_complete_us = m.t_first_us + 500;
  m.drone_q_ms = 3;    // would want dq_us = 3000, but nothing is left
  m.enc_us = 5000;     // larger than the whole span

  lat.on_submit(m, /*mono_us=*/0);
  lat.on_decoded(1000, m.t_complete_us + 100);
  lat.on_present(1000, m.t_complete_us + 200);
  lat.on_flip(1000, m.t_complete_us + 300, /*exact=*/true);

  const auto L = lat.flush_line();
  CHECK(L.n == 1);
  CHECK(L.p50[0] == 1000);  // enc clamped to the full span
  CHECK(L.p50[1] == 0);     // dq: nothing left
  CHECK(L.p50[2] == 0);     // air: nothing left, never negative
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

MTEST_MAIN
