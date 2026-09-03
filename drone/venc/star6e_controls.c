/* ported from waybeam_venc f956a52:src/star6e_controls.c */
#include "star6e_controls.h"

#include "idr_rate_limit.h"
#include "pipeline_common.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

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
/* Last threshold the "> superframe P cap:" line reported, so the line
 * prints on CHANGE only (rung moves, knob writes) and not on every 5 s
 * RcAgent re-assert of an unchanged bitrate. */
static uint32_t g_superframe_logged_bytes;
static int g_superframe_logged;
static int apply_superframe_p(uint32_t kbps);

/* One-shot readback of what the encoder holds, taken from apply_bitrate
 * (RcAgent's thread, under venc_core's verb lock -- the ONLY thread that
 * talks to MI_VENC_*RcParam) once >= RC_READBACK_AFTER_S have passed since
 * bind, i.e. past the #255 stale window.  It was first hooked into the
 * encoder loop at t+10 s and segfaulted there on the 2026-09-03 deploy the
 * instant the line had printed (SCHED_FIFO/CPU0 thread, concurrent with
 * the agent's Get/Set on the same channel); the same Get from the agent
 * thread had already succeeded three times in that boot.  Keep every RC
 * call on one thread. */
#define RC_READBACK_AFTER_S 10.0
static struct timespec g_rc_bind_ts;
static int g_rc_readback_done;

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
	if (!g_rc_readback_done) {
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		if ((double)(now.tv_sec - g_rc_bind_ts.tv_sec) +
		    (double)(now.tv_nsec - g_rc_bind_ts.tv_nsec) / 1e9 >=
		    RC_READBACK_AFTER_S) {
			g_rc_readback_done = 1;
			star6e_controls_log_rc_readback("t+10s");
		}
	}
	return 0;
}

int star6e_controls_request_idr(void)
{
	int chn = g_star6e_control_ctx.venc_chn;

	if (!idr_rate_limit_allow(chn))
		return 0;  /* coalesced — not an error */
	return MI_VENC_RequestIdr(chn, 1) == 0 ? 0 : -1;
}

/* Everything we have asked the rate controller for, kept because
 * MI_VENC_GetRcParam cannot be trusted to give it back (upstream waybeam
 * bf8c3cb, issue #255, ported 2026-09-03).
 *
 * The SDK does not reflect a just-written s32IPQPDelta in the next Get: it
 * keeps returning driver defaults until the driver commits the pending
 * block, measured upstream on SSC338Q as somewhere between t+0 s and
 * t+5 s after StartRecvPic.  A second Get->modify->Set inside that window
 * reads a stale 0 and writes it straight back, silently reverting the
 * earlier apply -- both calls return success and both log it.  mabur's
 * startup has exactly that shape (qp_delta then max_ipprop back-to-back in
 * star6e_runtime_apply_startup_controls), and the SuperFrame guard below
 * re-stages on every bitrate write, so every RC write stages the WHOLE
 * intent rather than patching a Get.  Ordering and timing then cannot
 * matter.  star6e_controls_log_rc_readback() (encoder loop, t+10 s)
 * prints what the encoder actually holds against this intent. */
static struct {
	int      qp_delta;    /* s32IPQPDelta (0 = the firmware default) */
	uint32_t max_ipprop;  /* u32MaxIPProp; 0 = leave the firmware value */
} g_rc_intent;

/* Firmware u32MaxIPProp, captured on the first Get before any write so the
 * boot log records what the SDK ships with. */
static struct {
	int      captured;
	uint32_t max_ipprop;
} g_rc_defaults;

/* Write the whole of g_rc_intent, never a patched Get result -- see the
 * note on g_rc_intent for why the Get cannot be trusted.  No IDR, no log:
 * shared by the qp_delta / max_ipprop verbs and by the SuperFrame guard
 * (waybeam PR #113 saw SetSuperFrameCfg reset qpDelta to 0 / maxQp to 48
 * on its path; the 2026-08-27 fork probe did not, so that call is
 * belt-and-braces and must not cost a keyframe per bitrate write). */
static int rc_commit_intent(void)
{
	MI_VENC_ChnAttr_t attr = {0};
	MI_VENC_RcParam_t param = {0};

	if (MI_VENC_GetChnAttr(g_star6e_control_ctx.venc_chn, &attr) != 0)
		return -1;
	if (MI_VENC_GetRcParam(g_star6e_control_ctx.venc_chn, &param) != 0)
		return -1;
	if (apply_rc_qp_delta(&attr, &param, g_rc_intent.qp_delta) != 0)
		return -1;
	if (!g_rc_defaults.captured) {
		g_rc_defaults.max_ipprop = param.stParamH265Cbr.u32MaxIPProp;
		g_rc_defaults.captured = 1;
	}
	if (g_rc_intent.max_ipprop)
		param.stParamH265Cbr.u32MaxIPProp = g_rc_intent.max_ipprop;
	return MI_VENC_SetRcParam(g_star6e_control_ctx.venc_chn, &param) == 0
		? 0 : -1;
}

static int apply_qp_delta(int delta)
{
	int prev = g_rc_intent.qp_delta;

	if (delta < -12 || delta > 12)
		return -1;
	g_rc_intent.qp_delta = delta;
	if (rc_commit_intent() != 0) {
		g_rc_intent.qp_delta = prev;
		return -1;
	}
	if (star6e_controls_request_idr() != 0)
		return -1;
	printf("> qpDelta changed to %d\n", delta);
	fflush(stdout);
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
	rc_qp = rc_commit_intent();
	/* Print on change (or on a failed intent re-stage), like max_ipprop:
	 * with the knob on this runs on every apply_bitrate, i.e. every 5 s
	 * RcAgent re-assert, and an unchanged threshold is not news. */
	if (!g_superframe_logged || bytes != g_superframe_logged_bytes ||
	    rc_qp != 0) {
		printf("> superframe P cap: %u%% of %u kbps @ %u fps = %u bytes "
			"(%s); readback mode=%d p_bits=%u rc_restage=%d\n",
			(unsigned)g_superframe_p_pct, (unsigned)kbps,
			(unsigned)g_star6e_control_ctx.delivered_fps,
			(unsigned)bytes, bytes ? "reencode" : "off",
			(int)back.eSuperFrmMode,
			(unsigned)back.u32SuperPFrmBitsThr, rc_qp);
		fflush(stdout);
		g_superframe_logged = 1;
		g_superframe_logged_bytes = bytes;
	}
	return 0;
}

/* Set u32MaxIPProp on the CBR RC params.  Legal range 1..100 (max I-frame
 * size as a multiple of P-frame size).  Staged through g_rc_intent like
 * every other RC write, so it can neither be reverted by a later
 * qp_delta/SuperFrame write nor revert one (issue #255).  Logs the
 * firmware's own value beside it -- the 2026-08-29 probe found this
 * proportion IS enforced under H265 CBR, unlike u32MaxISize/u32MaxPSize,
 * which were proven dead on hardware. */
static int apply_max_ipprop(uint32_t prop)
{
	uint32_t prev = g_rc_intent.max_ipprop;

	if (prop < 1 || prop > 100)
		return -1;
	g_rc_intent.max_ipprop = prop;
	if (rc_commit_intent() != 0) {
		g_rc_intent.max_ipprop = prev;
		return -1;
	}
	/* stdout is fully buffered once the wrapper redirects it into
	 * /tmp/mabur.log (not a tty) -- unlike the stats: line (main.cpp,
	 * stderr, unbuffered), a lone control-verb printf can sit unflushed
	 * for the life of the process.  Force it out now. */
	printf("> max_ipprop: applied = %u (firmware default %u)\n",
		(unsigned)prop, (unsigned)g_rc_defaults.max_ipprop);
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
	g_superframe_logged = 0;
	g_superframe_logged_bytes = 0;
	memset(&g_rc_intent, 0, sizeof(g_rc_intent));
	memset(&g_rc_defaults, 0, sizeof(g_rc_defaults));
	clock_gettime(CLOCK_MONOTONIC, &g_rc_bind_ts);
	g_rc_readback_done = 0;
}

void star6e_controls_reset(void)
{
	memset(&g_star6e_control_ctx, 0, sizeof(g_star6e_control_ctx));
	g_superframe_p_pct = 0;
	g_superframe_last_kbps = 0;
	g_superframe_applied = 0;
	g_superframe_logged = 0;
	g_superframe_logged_bytes = 0;
	memset(&g_rc_intent, 0, sizeof(g_rc_intent));
	memset(&g_rc_defaults, 0, sizeof(g_rc_defaults));
	g_rc_readback_done = 0;
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

void star6e_controls_log_rc_readback(const char *when)
{
	MI_VENC_RcParam_t param = {0};
	const int chn = g_star6e_control_ctx.venc_chn;
	int held_delta, ok;
	unsigned held_prop;

	if (MI_VENC_GetRcParam(chn, &param) != 0) {
		fprintf(stderr, "WARN: rc_readback %s: MI_VENC_GetRcParam failed\n",
			when);
		return;
	}
	held_delta = param.stParamH265Cbr.s32IPQPDelta;
	held_prop = (unsigned)param.stParamH265Cbr.u32MaxIPProp;
	ok = held_delta == g_rc_intent.qp_delta &&
	     (g_rc_intent.max_ipprop == 0 ||
	      held_prop == g_rc_intent.max_ipprop);
	/* stderr: unbuffered into /tmp/mabur.log like the stats: line, so a
	 * crash a second later still leaves the proof in the log. */
	fprintf(stderr, "rc_readback %s: IPQPDelta=%d MaxIPProp=%u MinQp=%u "
		"MaxQp=%u MinIQp=%u MaxIQp=%u intent qp_delta=%d max_ipprop=%u "
		"%s\n",
		when, held_delta, held_prop,
		(unsigned)param.stParamH265Cbr.u32MinQp,
		(unsigned)param.stParamH265Cbr.u32MaxQp,
		(unsigned)param.stParamH265Cbr.u32MinIQp,
		(unsigned)param.stParamH265Cbr.u32MaxIQp,
		g_rc_intent.qp_delta, (unsigned)g_rc_intent.max_ipprop,
		ok ? "OK" : "MISMATCH -- the encoder is not holding the intent");
}
