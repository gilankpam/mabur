/* ported-from/derived-from waybeam_venc f956a52:include/venc_config.h —
 * trimmed to the surface mabur exercises (spec 2026-08-28 §3). */
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  char sensor_bin[128];      /* venc.sensor_bin */
  uint16_t width, height;    /* venc.size "1920x1080" */
  uint16_t fps;              /* venc.fps */
  double gop_s;              /* venc.gop_s */
  int8_t qp_delta;           /* venc.qp_delta */
  uint8_t max_ipprop;        /* venc.max_ipprop: 0 (default) = leave the
                              * firmware default (0 = unbounded); 1..100 =
                              * program u32MaxIPProp at boot. 2026-08-29
                              * probe findings: enforced on star6e
                              * (dose-response measured), achieved-ratio
                              * floor ~1.4 at small values (I-QP rails);
                              * useful range 2..4 under rally where IDRs
                              * are already ~1.7x P. */
  uint8_t min_qp;            /* venc.min_qp: 0 (default) = leave the firmware
                              * u32MinQp alone; 1..51 = program the H265 CBR
                              * QP floor at boot. Bench knob for the
                              * quiet-scene-floor hypothesis in
                              * docs/handover-venc-overshoot-2026-09-03.md:
                              * a static scene rails at u32MinQp far under
                              * the command, and the first frames of motion
                              * are then sized by content, not budget. Not
                              * a shipped default until the sweep says so. */
  char resilience[16];       /* venc.resilience ("rally", ...) — preset table */
  bool roi_enabled;          /* venc.roi.enabled */
  uint8_t roi_steps;         /* venc.roi.steps */
  double roi_center;         /* venc.roi.center */
  uint16_t ae_fps, awb_fps;  /* venc.ae_fps / venc.awb_fps */
  uint8_t snapshot_quality;  /* venc.snapshot_quality */
  /* Compiled constants, NOT config: ring name VENC_RING_NAME "/mabur_f"
   * (shared with frame_source), rcMode CBR, codec H265. NO bitrate field —
   * rate arrives exclusively via venc_set_bitrate_kbps() (spec §3). */
} VencCfg;

/* ── Compiled constants ─────────────────────────────────────────────────
 * Everything below was a waybeam JSON key and is now pinned in the
 * binary.  mabur runs one camera, one transport and one codec, so a knob
 * that has exactly one legal value is dead config surface. */

/* frame-shm ring the encoder publishes whole access units into.  Shared
 * with drone/src/frame_source.cpp (config default "mabur_f"); the ring
 * layer normalises the leading '/'. */
#define VENC_RING_NAME "/mabur_f"

/* Practical VENC bitrate rails, ported verbatim from
 * waybeam_venc f956a52:include/venc_config.h.  Below ~1 Mbit/s the H.26x
 * stream collapses (decoder drops SVC-T, RC oscillates), so this is a
 * clamp on every rate that reaches the encoder, not a reject. */
#define VENC_BITRATE_MIN_KBPS 1000
#define VENC_BITRATE_MAX_KBPS 200000

/* Rate the encoder channel is CREATED at.  There is no venc.bitrate key
 * (spec §3): the real rate arrives from RcAgent via
 * star6e_controls_apply_bitrate() within the first control tick, so the
 * boot value only has to be legal, not correct. */
#define VENC_BOOT_BITRATE_KBPS VENC_BITRATE_MIN_KBPS

/* ── Resilience preset expansion ─────────────────────────────────────── */

/* Exactly the fields waybeam's apply_resilience_preset() writes into its
 * VencConfigVideo (f956a52:src/venc_config.c ~line 409).  gop_s is only
 * written when the preset pins one; `gop_overridden` says whether it did,
 * so the caller can keep its own venc.gop_s otherwise. */
typedef struct {
  char intra_refresh_mode[16];  /* "off" | "fast" | "balanced" | "robust" */
  uint16_t intra_refresh_lines; /* CTU rows refreshed per P-frame (0 = auto) */
  uint8_t intra_refresh_qp;     /* I-CTU QP override for the stripe (0 = auto) */
  uint8_t ref_base;             /* SVC-T base-layer period; 0 = off */
  uint8_t ref_enhance;          /* SVC-T reference PERIOD (see table comment) */
  bool ref_pred;                /* SVC-T enhance→base prediction */
  double gop_s;                 /* preset GOP in seconds (valid iff gop_overridden) */
  bool gop_overridden;          /* preset pinned a GOP; else keep cfg->gop_s */
} VencPresetOut;

#ifdef __cplusplus
extern "C" {
#endif

/* Seeds *cfg with the spec §3 defaults — the values an absent config key
 * falls back to. Every field EXCEPT sensor_bin gets a sane value; sensor_bin
 * is zeroed and stays REQUIRED, because it names a device-specific ISP
 * calibration blob that no default can guess (the config loader fails boot
 * when it is absent, see drone/src/config.cpp parse_venc).
 *
 * This exists so "absent key" and "explicit key" go through one table of
 * truth. Before it, VencCfg was default-initialised to all-zero, so a
 * config omitting e.g. venc.fps handed the encoder fps 0 / 0x0 / gop 0.0 —
 * not a fallback, a malformed pipeline. */
void venc_cfg_defaults(VencCfg *cfg);

/* Expands cfg->resilience into the pipeline's intra-refresh + SVC-T
 * parameters. Returns 0, or -1 on unknown preset name (boot failure). */
int venc_cfg_expand_preset(const VencCfg *cfg, VencPresetOut *out);

/* Preset-name validation authority for the config loader (drone/src/
 * config.cpp parse_venc, host build). Returns nonzero iff `name` is a
 * resilience preset venc_cfg_expand_preset() accepts (including "off",
 * NULL/empty defaulting to "off", and the "ltr"/"ltr:<N>" family) — it is
 * simply venc_cfg_expand_preset() with the output discarded, so there is
 * exactly ONE authority for "is this a known preset". */
int venc_cfg_preset_known(const char *name);

#ifdef __cplusplus
}
#endif
