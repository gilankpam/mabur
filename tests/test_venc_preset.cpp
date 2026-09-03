// Fresh test (waybeam had no isolated preset test): covers
// venc_cfg_expand_preset() against the resilience preset table in
// drone/venc/venc_cfg.c. Expected numbers copied from that table.
#include <cstdio>
#include <string>

#include "mtest.h"
#include "venc_cfg.h"

TEST(rally_preset_expands) {
  // Table row: { "rally", "fast", ref_base=1, ref_enhance=1, gop_sec=2.0 }
  VencCfg c{};
  std::snprintf(c.resilience, sizeof c.resilience, "rally");
  VencPresetOut out{};
  REQUIRE(venc_cfg_expand_preset(&c, &out) == 0);
  CHECK(std::string(out.intra_refresh_mode) == "fast");
  CHECK(out.intra_refresh_lines == 0);
  CHECK(out.intra_refresh_qp == 0);
  CHECK(out.ref_base == 1);
  CHECK(out.ref_enhance == 1);
  CHECK(out.ref_pred == true);
  CHECK(out.gop_overridden == true);
  CHECK(out.gop_s == 2.0);
}

TEST(fpv_preset_expands) {
  // Table row: { "fpv", "robust", ref_base=1, ref_enhance=4, gop_sec=2.0 }
  VencCfg c{};
  std::snprintf(c.resilience, sizeof c.resilience, "fpv");
  VencPresetOut out{};
  REQUIRE(venc_cfg_expand_preset(&c, &out) == 0);
  CHECK(std::string(out.intra_refresh_mode) == "robust");
  CHECK(out.ref_base == 1);
  CHECK(out.ref_enhance == 4);
  CHECK(out.ref_pred == true);
  CHECK(out.gop_overridden == true);
  CHECK(out.gop_s == 2.0);
}

TEST(off_preset_leaves_gop_untouched) {
  // Table row: { "off", "off", 0, 0, 0.0 } — gop_sec of 0 means
  // gop_overridden stays false so the caller's own gop_s config drives.
  VencCfg c{};
  std::snprintf(c.resilience, sizeof c.resilience, "off");
  VencPresetOut out{};
  REQUIRE(venc_cfg_expand_preset(&c, &out) == 0);
  CHECK(std::string(out.intra_refresh_mode) == "off");
  CHECK(out.ref_base == 0);
  CHECK(out.ref_enhance == 0);
  CHECK(out.gop_overridden == false);
}

TEST(empty_resilience_defaults_to_off) {
  VencCfg c{};  // resilience[0] == '\0'
  VencPresetOut out{};
  REQUIRE(venc_cfg_expand_preset(&c, &out) == 0);
  CHECK(std::string(out.intra_refresh_mode) == "off");
}

TEST(ltr_bare_pins_enhance_period_1) {
  VencCfg c{};
  std::snprintf(c.resilience, sizeof c.resilience, "ltr");
  VencPresetOut out{};
  REQUIRE(venc_cfg_expand_preset(&c, &out) == 0);
  CHECK(std::string(out.intra_refresh_mode) == "off");
  CHECK(out.ref_base == 1);
  CHECK(out.ref_enhance == 1);
  CHECK(out.ref_pred == false);
  CHECK(out.gop_overridden == false);  // preserves caller's gop_s
}

TEST(ltr_colon_n_pins_explicit_period) {
  VencCfg c{};
  std::snprintf(c.resilience, sizeof c.resilience, "ltr:9");
  VencPresetOut out{};
  REQUIRE(venc_cfg_expand_preset(&c, &out) == 0);
  CHECK(out.ref_enhance == 9);
}

TEST(ltr_colon_n_out_of_range_rejected) {
  VencCfg c{};
  std::snprintf(c.resilience, sizeof c.resilience, "ltr:0");
  VencPresetOut out{};
  CHECK(venc_cfg_expand_preset(&c, &out) == -1);

  std::snprintf(c.resilience, sizeof c.resilience, "ltr:256");
  CHECK(venc_cfg_expand_preset(&c, &out) == -1);

  std::snprintf(c.resilience, sizeof c.resilience, "ltr:abc");
  CHECK(venc_cfg_expand_preset(&c, &out) == -1);
}

TEST(unknown_preset_rejected) {
  VencCfg c{};
  std::snprintf(c.resilience, sizeof c.resilience, "yolo");
  VencPresetOut out{};
  CHECK(venc_cfg_expand_preset(&c, &out) == -1);
}

TEST(null_args_rejected) {
  VencCfg c{};
  VencPresetOut out{};
  CHECK(venc_cfg_expand_preset(nullptr, &out) == -1);
  CHECK(venc_cfg_expand_preset(&c, nullptr) == -1);
}

TEST(preset_known_matches_expand_preset) {
  CHECK(venc_cfg_preset_known("rally") != 0);
  CHECK(venc_cfg_preset_known("off") != 0);
  CHECK(venc_cfg_preset_known(nullptr) != 0);   // defaults to "off"
  CHECK(venc_cfg_preset_known("yolo") == 0);
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
