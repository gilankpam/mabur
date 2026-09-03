// Covers what survives in drone/venc/venc_cfg.c after venc.resilience was
// decomposed into its components (2026-09-04): the absent-key defaults and
// the SuperFrame byte math. The preset table, venc_cfg_expand_preset() and
// VencPresetOut are gone — the pipeline reads VencCfg directly.
#include <cstdio>
#include <string>

#include "mtest.h"
#include "venc_cfg.h"

// venc_cfg_defaults() is the one table of truth for "key absent from the
// config". The intra-refresh + SVC-T defaults reproduce, at 1080p60,
// exactly what the deleted "rally" preset expanded to: a 150 ms sweep is
// ceil(34 CTU rows / 9 frames) = 4 rows per P-frame, its stripe QP was 36,
// and rally ran 1:1 SVC-T with enhance->base prediction on. Deploying this
// change therefore does not move the encoder.
TEST(defaults_reproduce_the_old_rally_preset) {
  VencCfg c{};
  venc_cfg_defaults(&c);
  CHECK(c.intra_refresh_rows == 4);
  CHECK(c.intra_refresh_qp == 36);
  CHECK(c.ref_base == 1);
  CHECK(c.ref_enhance == 1);
  CHECK(c.ref_pred == true);
  CHECK(c.gop_s == 2.0);
}

// venc_cfg_ctu_rows(): H.265 CTU is 32x32, so a picture is ceil(height/32)
// CTU rows. This is the bound the config loader enforces on
// venc.intra_refresh_rows and the divisor that turns rows-per-P into a
// sweep length, so its boundaries matter at every supported picture size.
// (The deleted test_venc_intra_refresh.cpp pinned the 1080 and 720 cases
// through intra_refresh_compute(); this is the same arithmetic, direct.)
TEST(ctu_rows_is_ceil_height_over_32) {
  CHECK(venc_cfg_ctu_rows(1080) == 34);  // 33.75, rounds up
  CHECK(venc_cfg_ctu_rows(720) == 23);   // 22.5
  CHECK(venc_cfg_ctu_rows(1024) == 32);  // exact multiple
  CHECK(venc_cfg_ctu_rows(1056) == 33);  // exact multiple
  CHECK(venc_cfg_ctu_rows(1088) == 34);  // exact multiple, same as 1080
  CHECK(venc_cfg_ctu_rows(1) == 1);      // never rounds to zero rows
  CHECK(venc_cfg_ctu_rows(0) == 0);
}

// venc_superframe_p_bytes(): P-frame SuperFrame threshold as a percentage
// of the per-frame budget at the PROGRAMMED rate (kbps x 1024), so the cap
// follows the rung. Rung 5 at 60 fps: 16000 kbps -> 16.384 Mbit/s -> 34133 B
// per frame; 200 % -> 68266 B. The 2026-09-03 bench saw scene-cut frames of
// 195 kB against that budget.
TEST(superframe_p_bytes_follows_the_rung_budget) {
  CHECK(venc_superframe_p_bytes(100, 16000, 60) == 34133);
  CHECK(venc_superframe_p_bytes(200, 16000, 60) == 68266);
  CHECK(venc_superframe_p_bytes(150, 3900, 60) == 12480);  // rung 1: 66560 bits/frame x 1.5 / 8
  CHECK(venc_superframe_p_bytes(0, 16000, 60) == 0);       // off
  CHECK(venc_superframe_p_bytes(200, 16000, 0) == 0);      // no fps yet
  CHECK(venc_superframe_p_bytes(1000, 200000, 1) > 0);     // no overflow at the rails
}

MTEST_MAIN
