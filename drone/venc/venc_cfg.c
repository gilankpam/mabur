/* ported-from/derived-from waybeam_venc f956a52:src/venc_config.c — by now
 * only the absent-key defaults and two pure helpers survive; every cJSON
 * parse/save/load and live-apply grouping went with the venc fold-in
 * (spec 2026-08-28 §3), and the resilience preset table went 2026-09-04
 * when its fields became config keys of their own. */
#include "venc_cfg.h"

#include <string.h>

/* Absent-key defaults (spec 2026-08-28 venc-foldin §3).  See venc_cfg.h.
 *
 * These are the values the shipped bundle carries, so a config that omits
 * a venc key gets the flown configuration rather than a zero.  sensor_bin
 * is deliberately left EMPTY: it is the one device-specific field (the ISP
 * calibration blob path), and the config loader treats an empty sensor_bin
 * as a boot failure.
 *
 * Kept in venc_cfg.c, next to the struct it fills, rather than as C++ member
 * initialisers in config.h, so the vendored C struct owns its own defaults
 * and any future non-mabur caller of venc_core_start() gets them too. */
void venc_cfg_defaults(VencCfg *cfg)
{
	if (!cfg)
		return;
	memset(cfg, 0, sizeof(*cfg));
	/* sensor_bin: intentionally left "" — required, no default. */
	cfg->width = 1920;
	cfg->height = 1080;
	cfg->fps = 60;
	cfg->gop_s = 2.0;
	cfg->qp_delta = -4;
	cfg->max_ipprop = 0; /* leave the firmware default (unbounded) */
	cfg->min_iqp = 0;    /* leave the firmware I-QP floor */
	cfg->superframe_p_pct = 0; /* no P-frame ceiling */
	/* Error-resilience structure. These reproduce, at 1080p60, exactly
	 * what the deleted "rally" preset expanded to: its "fast" intra mode
	 * targeted a 150 ms sweep, which is ceil(34 CTU rows / 9 frames) = 4
	 * rows per P-frame at QP 36, over 1:1 SVC-T with prediction on. The
	 * rows figure is now a raw count, so unlike the preset it does NOT
	 * re-derive itself if venc.size changes — set it explicitly there. */
	cfg->intra_refresh_rows = 4;
	cfg->intra_refresh_qp = 36;
	cfg->ref_base = 1;
	cfg->ref_enhance = 1;
	cfg->ref_pred = true;
	cfg->roi_enabled = true;
	cfg->roi_steps = 2;
	cfg->roi_center = 0.4;
	cfg->ae_fps = 15;
	cfg->awb_fps = 15;
	cfg->snapshot_quality = 80;
}

/* H.265 CTU is 32x32, so a 1080-line picture is 34 CTU rows.  The upper
 * bound on venc.intra_refresh_rows (a stripe cannot be wider than the
 * picture) and the divisor for sweep length.  Shared by the config loader,
 * which rejects an out-of-range rows value at boot, and the pipeline,
 * which reports the derived sweep in its boot log. */
uint16_t venc_cfg_ctu_rows(uint16_t height)
{
	return (uint16_t)((height + 31u) / 32u);
}

uint32_t venc_superframe_p_bytes(unsigned pct, unsigned kbps, unsigned fps)
{
	uint64_t bits_per_frame;

	if (pct == 0 || fps == 0)
		return 0;
	/* Per-frame budget in bits = programmed bps / fps; programmed bps is
	 * kbps * 1024 (star6e_controls.c apply_bitrate). */
	bits_per_frame = ((uint64_t)kbps * 1024ull) / fps;
	return (uint32_t)(bits_per_frame * pct / 100ull / 8ull);
}
