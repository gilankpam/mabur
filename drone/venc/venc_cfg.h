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
                              * useful range 2..4 at the shipped GDR/SVC-T
                              * settings, where IDRs are already ~1.7x P. */
  uint16_t superframe_p_pct; /* venc.superframe_p_pct: 0 (default) = off;
                              * 100..1000 = P-frame ceiling as a percentage
                              * of the rung's per-frame budget
                              * (kbps*1024 / (fps*8)), programmed through
                              * MI_VENC_SetSuperFrameCfg REENCODE with I
                              * unlimited, re-derived on every bitrate
                              * write. The one live P-frame size bound on
                              * star6e (fork probe 2026-08-27); the RC
                              * re-plans UNDER the cap, so this is a quality
                              * lever too — bench before shipping non-zero. */
  /* ── Error-resilience structure ───────────────────────────────────────
   * These six fields WERE one string, venc.resilience, naming a row in a
   * preset table that expanded into them (deleted 2026-09-04). They are
   * now config keys in their own right, and each is a field of one of the
   * two MI structs verbatim — there is no derivation between config and
   * hardware left to surprise anyone.
   *
   * MI_VENC_IntraRefresh_t {bEnable, u32RefreshLineNum, u32ReqIQp}: */
  uint16_t intra_refresh_rows; /* venc.intra_refresh_rows: CTU rows forced
                                * intra per P-frame. 0 = rolling refresh off,
                                * and SetIntraRefresh is not called at all
                                * (the channel is created fresh every start,
                                * so there is no prior enable to clear).
                                * Otherwise 1..ceil(height/32)
                                * (H.265 CTU is 32x32; 1080 = 34 rows), and
                                * the sweep takes ceil(rows_total/rows)
                                * frames. Larger = faster self-heal, more
                                * bitrate per P and more frame-size swing. */
  uint8_t intra_refresh_qp;    /* venc.intra_refresh_qp: u32ReqIQp, the QP
                                * of the stripe. 1..51. Lower = cleaner
                                * recovery anchor, more bits. Unused while
                                * intra_refresh_rows is 0. */
  /* MI_VENC_ParamRef_t {u32Base, u32Enhance, bEnablePred} — SVC-T temporal
   * hierarchy, applied once between CreateChn and StartRecvPic: */
  uint8_t ref_base;            /* venc.ref_base: u32Base. 0 = SVC-T off,
                                * SetRefParam is not called at all and the
                                * encoder runs a flat single-ref P chain. */
  uint8_t ref_enhance;         /* venc.ref_enhance: u32Enhance, a PERIOD —
                                * the encoder emits exactly one
                                * non-referenced frame per (enhance+1), so 1
                                * is 50% droppable and the most resilient
                                * structure this SoC expresses; larger is
                                * strictly less. Must be >= 1 when ref_base
                                * is nonzero. Device-measured on Star6E
                                * 2026-08-06 by raw-ES NAL census.
                                *
                                * Costs the intra sweep: a stripe landing in
                                * a non-referenced frame never enters the
                                * DPB, so the effective self-heal window is
                                * the nominal sweep x (enhance+1), and it
                                * wants to stay under venc.gop_s or the IDR
                                * is doing the recovering, not the stripe. */
  bool ref_pred;               /* venc.ref_pred: bEnablePred, enhance->base
                                * prediction. */
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

/* Total CTU rows in a picture of `height` lines (H.265 CTU is 32x32).
 * The upper bound on venc.intra_refresh_rows, and the divisor that turns
 * a rows-per-P setting into a sweep length in frames. Pure, host-tested. */
uint16_t venc_cfg_ctu_rows(uint16_t height);

/* P-frame SuperFrame threshold in BYTES for a pct-of-budget cap at the
 * given programmed rate and frame rate: pct * (kbps*1024) / (fps*8*100).
 * kbps is the value handed to the encoder (decimal kbps, x1024 inside),
 * fps the rate the RC budgets at. 0 when pct or fps is 0 (= off). Pure,
 * host-tested (tests/test_venc_cfg.cpp). */
uint32_t venc_superframe_p_bytes(unsigned pct, unsigned kbps, unsigned fps);

#ifdef __cplusplus
}
#endif
