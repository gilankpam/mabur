/* ported-from/derived-from waybeam_venc f956a52:src/venc_config.c — by now
 * only the absent-key defaults and two pure helpers survive; every cJSON
 * parse/save/load and live-apply grouping went with the venc fold-in, and
 * the resilience preset table went when its fields became config keys of
 * their own. */
#include "venc_cfg.h"

#include <string.h>

/* Absent-key defaults.  See venc_cfg.h.
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
	cfg->binning = true;
	cfg->gop_s = 0; /* no periodic refresh: the GS asks for an IDR when its picture is damaged */
	cfg->qp_delta = 4;
	cfg->max_ipprop = 2; /* keyframe at most twice a P-frame */
	cfg->min_iqp = 34;  /* keyframe quality floor: smaller keyframes */
	cfg->superframe_p_pct = 200; /* P-frame cap, % of the per-frame budget */
	/* Rolling intra refresh runs only at a GOP start, so it is off while
	 * gop_s is 0. */
	cfg->intra_refresh_frames = 0;
	cfg->intra_refresh_qp = 36;
	cfg->ref_base = 1;
	cfg->ref_enhance = 1;
	cfg->ref_pred = true;
	cfg->mirror = false;
	cfg->flip = false;
	cfg->slices = 4; /* row slices: truncated-AU salvage needs > 1 */
	cfg->roi_enabled = false;
	cfg->roi_steps = 2;
	cfg->roi_center = 0.4;
	cfg->ae_fps = 15;
	cfg->awb_fps = 15;
	cfg->snapshot_quality = 80;
}

/* H.265 CTU is 32x32, so a 1080-line picture is 34 CTU rows. */
uint16_t venc_cfg_ctu_rows(uint16_t height)
{
	return (uint16_t)((height + 31u) / 32u);
}

uint16_t venc_cfg_intra_rows(uint16_t height, uint16_t frames)
{
	uint16_t total = venc_cfg_ctu_rows(height);

	if (frames == 0 || total == 0)
		return 0;
	return (uint16_t)((total + frames - 1u) / frames);
}

uint16_t venc_cfg_ctb64_rows(uint16_t height)
{
	return (uint16_t)((height + 63u) / 64u);
}

uint8_t venc_cfg_slice_rows(uint16_t height, uint8_t slices)
{
	uint16_t r = venc_cfg_ctb64_rows(height);
	uint16_t k;

	if (slices <= 1 || r == 0 || slices > r)
		return 0;
	k = (uint16_t)((r + slices - 1u) / slices);
	if ((uint16_t)((r + k - 1u) / k) != slices)
		return 0;
	return (uint8_t)k;
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

uint32_t venc_cfg_gop_frames(double gop_s, uint32_t fps)
{
	uint32_t gop_fps = fps ? fps : 30;
	uint32_t frames;

	if (gop_s <= 0.0)
		return VENC_GOP_NONE;
	frames = (uint32_t)(gop_s * gop_fps + 0.5);
	return frames < 1 ? 1 : frames;
}
