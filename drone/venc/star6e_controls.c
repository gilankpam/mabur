/* ported from waybeam_venc f956a52:src/star6e_controls.c */
#include "star6e_controls.h"

#include "idr_rate_limit.h"
#include "pipeline_common.h"

#include <stdio.h>
#include <string.h>

typedef struct {
	MI_VENC_CHN venc_chn;
	MI_SYS_ChnPort_t vpe_port;
	MI_SYS_ChnPort_t venc_port;
	volatile uint32_t sensor_fps;
	/* Current VPE->VENC bind delivery rate (true frames reaching the
	 * encoder) — above STAR6E_VENC_INPUT_FPS_MAX this exceeds the RC
	 * fpsNum and the bitrate budget must be compensated (see
	 * rc_compensate_kbps). */
	volatile uint32_t delivered_fps;
	uint32_t frame_width;
	uint32_t frame_height;
	Star6ePipelineState *pipeline;
	const VencCfg *cfg;
} Star6eControlContext;

static Star6eControlContext g_star6e_control_ctx;

/* SuperFrame P-frame ceiling state (see apply_superframe_p). pct is seeded
 * from venc.superframe_p_pct at bind and changed live by the debug verb;
 * last_kbps is the rate the encoder was actually given (post-compensation,
 * post-rails), so a live pct change can re-derive the threshold without a
 * bitrate write. applied says the SDK holds a non-NONE policy, so turning
 * the knob off writes NONE once instead of leaving a stale cap. */
static uint32_t g_superframe_p_pct;
static uint32_t g_superframe_last_kbps;
static int g_superframe_applied;
static int apply_superframe_p(uint32_t kbps);

static uint32_t align_down(uint32_t value, uint32_t align)
{
	return value / align * align;
}

/* The channel is always created H.265 CBR (star6e_pipeline_start_venc), so
 * only that rate mode survives; waybeam's H.264 / VBR / AVBR arms went with
 * the video0.rcMode config key. */
static int apply_rc_qp_delta(const MI_VENC_ChnAttr_t *attr,
	MI_VENC_RcParam_t *param, int delta)
{
	if (!attr || !param || delta < -12 || delta > 12)
		return -1;

	if (attr->rate.mode != I6_VENC_RATEMODE_H265CBR)
		return -1;
	param->stParamH265Cbr.s32IPQPDelta = delta;
	return 0;
}

/* Exact-CBR compensation for >STAR6E_VENC_INPUT_FPS_MAX modes: the RC
 * budgets kbps at rc_fps (capped 120) while the bind delivers more frames,
 * so the wire rate is kbps * delivered/rc (~1.19x at 144).  Scale the
 * encoder budget down so the wire lands on the configured bitrate.  Safe
 * with rc_fps=120: the per-frame budget stays large enough that QP does
 * not saturate (the naive version was only rejected when RC wrongly
 * budgeted 30fps). */
static uint32_t rc_compensate_kbps(uint32_t kbps, uint32_t delivered_fps)
{
	if (delivered_fps > STAR6E_VENC_INPUT_FPS_MAX)
		kbps = (uint32_t)((uint64_t)kbps * STAR6E_VENC_INPUT_FPS_MAX /
			delivered_fps);
	return kbps;
}

/* Program the encoder rate.
 *
 * The bitrate-change IDR that waybeam fired here is DELETED (spec §4): the
 * encoder is a pure mechanism and does not decide to spend the largest
 * frame in the stream on a rate step.  The rate controller absorbs
 * between-IDR changes; RcAgent asks for an IDR explicitly when it wants
 * one (venc_request_idr).  The frame-shm ring-fill throttle that used
 * to scale this value is deleted too — rate policy lives in RcAgent. */
static int apply_bitrate(uint32_t kbps)
{
	MI_VENC_ChnAttr_t attr = {0};
	MI_U32 bits;

	/* Order is pinned: the >120 fps exact-CBR compensation first, then
	 * the absolute MIN/MAX rails, so the correction cannot push the
	 * programmed value outside what the encoder accepts. */
	kbps = rc_compensate_kbps(kbps, g_star6e_control_ctx.delivered_fps);
	if (kbps > VENC_BITRATE_MAX_KBPS)
		kbps = VENC_BITRATE_MAX_KBPS;
	if (kbps < VENC_BITRATE_MIN_KBPS)
		kbps = VENC_BITRATE_MIN_KBPS;
	bits = kbps * 1024;

	if (MI_VENC_GetChnAttr(g_star6e_control_ctx.venc_chn, &attr) != 0)
		return -1;

	if (attr.rate.mode != I6_VENC_RATEMODE_H265CBR)
		return -1;
	attr.rate.h265Cbr.bitrate = bits;

	if (MI_VENC_SetChnAttr(g_star6e_control_ctx.venc_chn, &attr) != 0)
		return -1;
	/* The P ceiling is a fraction of THIS rate's per-frame budget, so it
	 * follows every rung change. Non-fatal: the bitrate is programmed
	 * whether or not the SDK takes the SuperFrame policy. */
	g_superframe_last_kbps = kbps;
	if (g_superframe_p_pct || g_superframe_applied)
		(void)apply_superframe_p(kbps);
	return 0;
}

int star6e_controls_request_idr(void)
{
	int chn = g_star6e_control_ctx.venc_chn;

	if (!idr_rate_limit_allow(chn))
		return 0;  /* coalesced — not an error */
	return MI_VENC_RequestIdr(chn, 1) == 0 ? 0 : -1;
}

/* Get->modify->Set of s32IPQPDelta alone: no IDR, no log. Shared by the
 * qp_delta verb and by the SuperFrame guard below (waybeam PR #113 saw
 * SetSuperFrameCfg reset qpDelta to 0 / maxQp to 48 on its path; the
 * 2026-08-27 fork probe did not, so this is belt-and-braces and must not
 * cost a keyframe per bitrate write). */
static int restage_qp_delta(int delta)
{
	MI_VENC_ChnAttr_t attr = {0};
	MI_VENC_RcParam_t param = {0};

	if (MI_VENC_GetChnAttr(g_star6e_control_ctx.venc_chn, &attr) != 0)
		return -1;
	if (MI_VENC_GetRcParam(g_star6e_control_ctx.venc_chn, &param) != 0)
		return -1;
	if (apply_rc_qp_delta(&attr, &param, delta) != 0)
		return -1;
	return MI_VENC_SetRcParam(g_star6e_control_ctx.venc_chn, &param) == 0
		? 0 : -1;
}

static int apply_qp_delta(int delta)
{
	if (restage_qp_delta(delta) != 0)
		return -1;
	if (star6e_controls_request_idr() != 0)
		return -1;
	printf("> qpDelta changed to %d\n", delta);
	return 0;
}

/* SuperFrame REENCODE with a P-frame threshold = g_superframe_p_pct % of
 * the per-frame budget at `kbps` (the rate the encoder was just given),
 * I and B thresholds unlimited. Measured on this SoC 2026-08-27 (waybeam
 * 0.69.2 probe, ../mabur-fork stack): P frames bounded from the NEXT
 * frame, keyframe-free, no drops; an I threshold below the IDR size
 * stalls the channel permanently, hence the unconditional 0xFFFFFFFF.
 * The RC re-plans UNDER the threshold rather than clipping at it (6000 B
 * cap -> 3.2 kB frames), so the percentage is a quality lever as well as
 * a burst bound — the 2026-09-03 bench's motivating case is a scene-cut
 * P frame at 2-6x the budget (195 kB against 34 kB at rung 5). pct 0
 * writes NONE (all thresholds unlimited) if a policy was ever applied. */
static int apply_superframe_p(uint32_t kbps)
{
	MI_VENC_SuperFrameCfg_t cfg = {0};
	MI_VENC_SuperFrameCfg_t back = {0};
	uint32_t bytes = venc_superframe_p_bytes(g_superframe_p_pct, kbps,
		g_star6e_control_ctx.delivered_fps);
	int rc_qp = 0;

	cfg.eSuperFrmMode = bytes ? E_MI_VENC_SUPERFRM_REENCODE
				  : E_MI_VENC_SUPERFRM_NONE;
	cfg.u32SuperIFrmBitsThr = 0xFFFFFFFFu;
	cfg.u32SuperPFrmBitsThr = bytes ? bytes * 8u : 0xFFFFFFFFu;
	cfg.u32SuperBFrmBitsThr = 0xFFFFFFFFu;
	if (MI_VENC_SetSuperFrameCfg(g_star6e_control_ctx.venc_chn, &cfg) != 0) {
		fprintf(stderr, "WARN: MI_VENC_SetSuperFrameCfg failed (%s)\n",
			g_mi_venc.fnSetSuperFrameCfg ? "rejected" : "not exported");
		return -1;
	}
	g_superframe_applied = bytes != 0;
	(void)MI_VENC_GetSuperFrameCfg(g_star6e_control_ctx.venc_chn, &back);
	if (g_star6e_control_ctx.cfg && g_star6e_control_ctx.cfg->qp_delta != 0)
		rc_qp = restage_qp_delta(g_star6e_control_ctx.cfg->qp_delta);
	printf("> superframe P cap: %u%% of %u kbps @ %u fps = %u bytes (%s); "
		"readback mode=%d p_bits=%u qp_restage=%d\n",
		(unsigned)g_superframe_p_pct, (unsigned)kbps,
		(unsigned)g_star6e_control_ctx.delivered_fps, (unsigned)bytes,
		bytes ? "reencode" : "off", (int)back.eSuperFrmMode,
		(unsigned)back.u32SuperPFrmBitsThr, rc_qp);
	fflush(stdout);
	return 0;
}

/* Set u32MaxIPProp directly on the CBR RC params.  Legal range 1..100
 * (max I-frame size as a multiple of P-frame size).  Get->modify->Set
 * like apply_qp_delta, so every other live RcParam field (s32IPQPDelta
 * included) is preserved.  Prints the CURRENT value before overwriting
 * it, which on the first call is the firmware's compiled-in default —
 * the 2026-08-29 probe found this proportion IS enforced under H265 CBR,
 * unlike u32MaxISize/u32MaxPSize, which were proven dead on hardware. */
static int apply_max_ipprop(uint32_t prop)
{
	MI_VENC_ChnAttr_t attr = {0};
	MI_VENC_RcParam_t param = {0};

	if (prop < 1 || prop > 100)
		return -1;

	if (MI_VENC_GetChnAttr(g_star6e_control_ctx.venc_chn, &attr) != 0)
		return -1;
	if (attr.rate.mode != I6_VENC_RATEMODE_H265CBR)
		return -1;
	if (MI_VENC_GetRcParam(g_star6e_control_ctx.venc_chn, &param) != 0)
		return -1;
	printf("> max_ipprop: current (firmware default or last-set) = %u\n",
		(unsigned)param.stParamH265Cbr.u32MaxIPProp);
	/* stdout is fully buffered once the wrapper redirects it into
	 * /tmp/mabur.log (not a tty) -- unlike the stats: line (main.cpp,
	 * stderr, unbuffered), a lone control-verb printf can sit unflushed
	 * for the life of the process.  Force it out now. */
	fflush(stdout);
	param.stParamH265Cbr.u32MaxIPProp = prop;
	if (MI_VENC_SetRcParam(g_star6e_control_ctx.venc_chn, &param) != 0)
		return -1;
	printf("> max_ipprop: applied = %u\n", (unsigned)prop);
	fflush(stdout);
	return 0;
}

/* Compute one horizontal ROI band for step index of steps.
 * Full-height bands centered horizontally, tapered QP toward edges.
 * Returns 0 if valid, -1 if region should be skipped. */
static int compute_horizontal_roi(uint32_t width, uint32_t height,
	float center_frac, int qp, int steps, int index,
	MI_VENC_RoiCfg_t *roi)
{
	float frac;
	uint32_t rw, rh, rx;
	int level;

	if (!roi || index < 0 || index >= steps)
		return -1;

	level = index + 1;
	frac = center_frac + (1.0f - center_frac) *
		(float)(steps - level) / (float)steps;
	rw = align_down((uint32_t)(frac * width), 32);
	rh = align_down(height, 32);
	rx = align_down((width - rw) / 2, 32);
	if (rw == 0 || rh == 0)
		return -1;

	roi->u32Index = (uint32_t)index;
	roi->bEnable = 1;
	roi->bAbsQp = 0;
	roi->s32Qp = pipeline_common_scale_roi_qp(qp, level, steps);
	roi->stRect.u32Left = rx;
	roi->stRect.u32Top = 0;
	roi->stRect.u32Width = rw;
	roi->stRect.u32Height = rh;
	return 0;
}

static int apply_roi_qp(int qp)
{
	uint32_t width = g_star6e_control_ctx.frame_width;
	uint32_t height = g_star6e_control_ctx.frame_height;
	int ok = 1;
	uint16_t steps;
	float center_frac;

	if (width == 0 || height == 0)
		return -1;

	for (int i = 0; i < PIPELINE_ROI_MAX_STEPS; i++) {
		MI_VENC_RoiCfg_t roi = {0};
		roi.u32Index = i;
		roi.bEnable = 0;
		MI_VENC_SetRoiCfg(g_star6e_control_ctx.venc_chn, &roi);
	}

	if (!g_star6e_control_ctx.cfg ||
	    !g_star6e_control_ctx.cfg->roi_enabled || qp == 0) {
		printf("> ROI disabled (all regions cleared)\n");
		return 0;
	}

	if (qp < -30) qp = -30;
	if (qp > 30) qp = 30;

	steps = g_star6e_control_ctx.cfg->roi_steps;
	if (steps < 1) steps = 1;
	if (steps > PIPELINE_ROI_MAX_STEPS) steps = PIPELINE_ROI_MAX_STEPS;

	center_frac = (float)g_star6e_control_ctx.cfg->roi_center;
	if (center_frac < 0.1f) center_frac = 0.1f;
	if (center_frac > 0.9f) center_frac = 0.9f;

	for (int i = 0; i < steps; i++) {
		MI_VENC_RoiCfg_t roi = {0};
		MI_S32 ret;

		if (compute_horizontal_roi(width, height, center_frac, qp,
		    steps, i, &roi) != 0)
			continue;

		ret = MI_VENC_SetRoiCfg(g_star6e_control_ctx.venc_chn, &roi);
		if (ret != 0) {
			printf("> ROI[%d] set failed (ret=0x%08x) rect=(%u,%u %ux%u) qp=%+d\n",
				i, (unsigned)ret,
				roi.stRect.u32Left, roi.stRect.u32Top,
				roi.stRect.u32Width, roi.stRect.u32Height,
				roi.s32Qp);
			ok = 0;
		}
	}

	if (ok) {
		printf("> ROI horizontal: %ux%u, %u steps, center=%.0f%%, qp=%+d\n",
			width, height, steps, center_frac * 100.0f, qp);
	}

	return ok ? 0 : -1;
}

void star6e_controls_bind(Star6ePipelineState *pipeline, const VencCfg *cfg)
{
	memset(&g_star6e_control_ctx, 0, sizeof(g_star6e_control_ctx));
	if (!pipeline || !cfg)
		return;

	g_star6e_control_ctx.venc_chn = pipeline->venc_channel;
	g_star6e_control_ctx.vpe_port = pipeline->vpe_port;
	g_star6e_control_ctx.venc_port = pipeline->venc_port;
	g_star6e_control_ctx.sensor_fps = pipeline->sensor.mode.maxFps ?
		pipeline->sensor.mode.maxFps : pipeline->video.sensor_framerate;
	/* Mirror the create-path VPE->VENC bind: delivery = venc.fps
	 * clamped to the mode's true rate (0 = full rate). */
	g_star6e_control_ctx.delivered_fps = g_star6e_control_ctx.sensor_fps;
	if (cfg->fps &&
	    cfg->fps < g_star6e_control_ctx.delivered_fps)
		g_star6e_control_ctx.delivered_fps = cfg->fps;
	g_star6e_control_ctx.frame_width = pipeline->image_width;
	g_star6e_control_ctx.frame_height = pipeline->image_height;
	g_star6e_control_ctx.pipeline = pipeline;
	g_star6e_control_ctx.cfg = cfg;
	/* Seeded here, programmed by the first apply_bitrate (RcAgent's first
	 * control tick) — the boot rate is a placeholder, not a budget. */
	g_superframe_p_pct = cfg->superframe_p_pct;
	g_superframe_last_kbps = 0;
	g_superframe_applied = 0;
}

void star6e_controls_reset(void)
{
	memset(&g_star6e_control_ctx, 0, sizeof(g_star6e_control_ctx));
	g_superframe_p_pct = 0;
	g_superframe_last_kbps = 0;
	g_superframe_applied = 0;
}

int star6e_controls_set_superframe_p_pct(uint32_t pct)
{
	if (pct != 0 && (pct < 100 || pct > 1000))
		return -1;
	g_superframe_p_pct = pct;
	if (g_superframe_last_kbps == 0) {
		printf("> superframe P cap: %u%% staged, applies at the first "
			"bitrate write\n", (unsigned)pct);
		fflush(stdout);
		return 0;
	}
	return apply_superframe_p(g_superframe_last_kbps);
}

int star6e_controls_apply_bitrate(uint32_t kbps)
{
	return apply_bitrate(kbps);
}

int star6e_controls_apply_roi_qp(int qp)
{
	return apply_roi_qp(qp);
}

int star6e_controls_apply_qp_delta(int delta)
{
	return apply_qp_delta(delta);
}

int star6e_controls_set_max_ipprop(uint32_t prop)
{
	return apply_max_ipprop(prop);
}

/* Set u32MinQp on the CBR RC params.  The venc-overshoot hypothesis
 * (docs/handover-venc-overshoot-2026-09-03.md): on a quiet scene the rate
 * controller rails at the firmware's QP floor far under its command, and
 * when motion arrives the first frames are sized by content with no
 * budget pressure until the 1 s stat window catches up.  Raising the
 * floor bounds how far under the command a quiet scene can sit, i.e. the
 * deficit the RC has to climb out of.  Prints the current values first so
 * the log records what the firmware ships with.  No IDR: a QP floor is a
 * constraint on the next frames, not a stream discontinuity. */
static int apply_min_qp(uint32_t qp)
{
	MI_VENC_ChnAttr_t attr = {0};
	MI_VENC_RcParam_t param = {0};

	if (qp < 1 || qp > 51)
		return -1;

	if (MI_VENC_GetChnAttr(g_star6e_control_ctx.venc_chn, &attr) != 0)
		return -1;
	if (attr.rate.mode != I6_VENC_RATEMODE_H265CBR)
		return -1;
	if (MI_VENC_GetRcParam(g_star6e_control_ctx.venc_chn, &param) != 0)
		return -1;
	printf("> min_qp: current MinQp=%u MaxQp=%u MinIQp=%u MaxIQp=%u "
		"(firmware default or last-set)\n",
		(unsigned)param.stParamH265Cbr.u32MinQp,
		(unsigned)param.stParamH265Cbr.u32MaxQp,
		(unsigned)param.stParamH265Cbr.u32MinIQp,
		(unsigned)param.stParamH265Cbr.u32MaxIQp);
	fflush(stdout);
	if (param.stParamH265Cbr.u32MaxQp != 0 &&
	    qp > param.stParamH265Cbr.u32MaxQp)
		return -1;  /* floor above the live ceiling — refuse, don't wedge RC */
	param.stParamH265Cbr.u32MinQp = qp;
	if (MI_VENC_SetRcParam(g_star6e_control_ctx.venc_chn, &param) != 0)
		return -1;
	printf("> min_qp: applied = %u\n", (unsigned)qp);
	fflush(stdout);
	return 0;
}

int star6e_controls_set_min_qp(uint32_t qp)
{
	return apply_min_qp(qp);
}
