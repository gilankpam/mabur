/* ported from waybeam_venc f956a52:src/star6e_runtime.c */
#include "star6e_runtime.h"

#include "pipeline_common.h"
#include "sdk_quiet.h"
#include "star6e.h"
#include "star6e_controls.h"
#include "star6e_cus3a.h"
#include "star6e_ipu.h"
#include "star6e_pipeline.h"

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

static SdkQuietState g_sdk_quiet = SDK_QUIET_STATE_INIT;
static volatile sig_atomic_t g_running = 1;

/* mabur addition: the encoder loop's exit reason.
 *
 * waybeam's loop broke on a process_stream failure and returned, which is
 * indistinguishable from a requested stop — the process then exited and the
 * init script's respawn was the recovery.  maburd keeps running (radio, RC,
 * telemetry) when the encoder dies, so the facade has to be able to tell
 * "operator asked us to stop" from "the MI stack failed under us".  The
 * failure sites fill this buffer; star6e_runtime_encoder_loop returns it
 * (non-NULL) on a fault and NULL on a clean stop.  Written and read on the
 * encoder thread only, plus the joiner after pthread_join. */
static char g_fault_what[96];

static MI_VENC_Pack_t *ensure_packs(MI_VENC_Pack_t **buf,
	uint32_t *cap, uint32_t need)
{
	if (need <= *cap)
		return *buf;
	free(*buf);
	*buf = malloc(need * sizeof(MI_VENC_Pack_t));
	*cap = *buf ? need : 0;
	return *buf;
}

/* Sleep for up to timeout_ms while the encoder has nothing to hand us.
 * waybeam woke early to service the RTP sidecar fd; there is no sidecar
 * here, so this is a plain sleep. */
static void idle_wait(int timeout_ms)
{
	usleep((unsigned)(timeout_ms * 1000));
}

/* ── SVC-T non-reference marking ──────────────────────────────────────── */

/* STAR6E_REFTYPE_ENHANCE_P_NOTFORREF (=5) is defined in star6e.h. Was locally
 * 4 (HiSilicon value) — wrong for the SigmaStar enum, so the TRAIL_N rewrite
 * marked referenced-enhance frames (or nothing under shallow SVC-T). */

/* Locate the NAL header byte 0 inside a payload buffer that may or may not
 * begin with a start-code prefix (00 00 01 / 00 00 00 01).  Returns the
 * index of NAL byte 0, or len on failure. */
static size_t star6e_nal_header_idx(const uint8_t *buf, size_t len)
{
	size_t i = 0;
	while (i < len && buf[i] == 0) i++;
	if (i < len && buf[i] == 0x01) i++;
	return i < len ? i : len;
}

/* If a NAL is TRAIL_R (type 1) and the SDK marked this frame as
 * ENHANCE_P_NOTFORREF, rewrite the NAL header to TRAIL_N (type 0).
 *
 * Byte 0 bit layout: forbidden_zero(1) | nal_unit_type(6) | layer_id_msb(1)
 *   TRAIL_R = 0x02   (type=1, layer_msb=0)
 *   TRAIL_N = 0x00   (type=0, layer_msb=0)
 *
 * No-op if NAL layer_id_msb != 0 (we only touch single-layer streams), if
 * the NAL is anything other than TRAIL_R, or if no slice NALs are present
 * in the pack (we never touch VPS/SPS/PPS — those are nal_type >= 32 and
 * fail the TRAIL_R check). */
static void star6e_patch_pack_to_trail_n(MI_VENC_Pack_t *pack)
{
	if (!pack || !pack->data || pack->length == 0)
		return;
	if (pack->packNum > 0) {
		const unsigned int info_cap = (unsigned int)(sizeof(pack->packetInfo) /
			sizeof(pack->packetInfo[0]));
		unsigned int n = pack->packNum > info_cap ? info_cap : pack->packNum;
		unsigned int k;
		for (k = 0; k < n; ++k) {
			MI_U32 off = pack->packetInfo[k].offset;
			MI_U32 nlen = pack->packetInfo[k].length;
			if (off >= pack->length || nlen == 0 ||
			    off + nlen > pack->length)
				continue;
			size_t hdr = star6e_nal_header_idx(pack->data + off, nlen);
			if (hdr >= nlen) continue;
			if (pack->data[off + hdr] == 0x02) {
				pack->data[off + hdr] = 0x00;
			}
		}
		return;
	}
	/* packNum == 0: single NAL */
	if (pack->offset >= pack->length)
		return;
	{
		MI_U32 off = pack->offset;
		MI_U32 nlen = pack->length - off;
		size_t hdr = star6e_nal_header_idx(pack->data + off, nlen);
		if (hdr >= nlen) return;
		if (pack->data[off + hdr] == 0x02) {
			pack->data[off + hdr] = 0x00;
		}
	}
}

static void star6e_patch_stream_to_trail_n(MI_VENC_Stream_t *s)
{
	unsigned int i;
	if (!s || !s->packet) return;
	for (i = 0; i < s->count; i++)
		star6e_patch_pack_to_trail_n(&s->packet[i]);
}

/* ── Boot-time controls ───────────────────────────────────────────────── */

/* Start the supervisory AE limit enforcer.  This is the sole AE path on
 * Star6E: the ISP firmware/bin AE does convergence and this thread re-asserts
 * the gain/shutter min/max on the exposure limit each tick.  mabur exposes
 * only the tick rate (venc.ae_fps); the gain/shutter limits keep the
 * star6e_cus3a defaults. */
static void start_ae_enforcer(const VencCfg *cfg)
{
	Star6eCus3aConfig ae_cfg;

	star6e_cus3a_config_defaults(&ae_cfg);
	if (cfg->ae_fps > 0)
		ae_cfg.ae_fps = cfg->ae_fps;
	ae_cfg.verbose = 0;
	star6e_cus3a_start(&ae_cfg);
}

static int star6e_runtime_apply_startup_controls(Star6eRunnerContext *ctx)
{
	Star6ePipelineState *ps = &ctx->ps;
	const VencCfg *cfg = &ctx->cfg;

	star6e_controls_bind(ps, cfg);

	/* AE runs in ONE mode on Star6E: the SDK firmware/bin AE converges and
	 * the supervisory thread enforces the gain/shutter limits beside it.  The
	 * thread needs aeFps>0 for a tick rate. */
	if (cfg->ae_fps > 0)
		start_ae_enforcer(cfg);

	if (cfg->qp_delta != 0)
		star6e_controls_apply_qp_delta(cfg->qp_delta);

	/* 0 = leave the firmware default (unbounded); non-fatal on failure,
	 * like every other startup control here — log and keep booting
	 * rather than kill the encoder over an RC size-cap knob. */
	if (cfg->max_ipprop > 0) {
		if (star6e_controls_set_max_ipprop(cfg->max_ipprop) != 0)
			fprintf(stderr,
				"WARN: max_ipprop %u not applied\n",
				(unsigned)cfg->max_ipprop);
	}

	/* 0 = leave the firmware u32MinQp; same non-fatal contract. */
	if (cfg->min_qp > 0) {
		if (star6e_controls_set_min_qp(cfg->min_qp) != 0)
			fprintf(stderr,
				"WARN: min_qp %u not applied\n",
				(unsigned)cfg->min_qp);
	}

	return 0;
}

/* ── Encoder loop ─────────────────────────────────────────────────────── */

static int star6e_runtime_process_stream(Star6eRunnerContext *ctx,
	struct timespec *cus3a_ts_last, unsigned int *idle_counter)
{
	Star6ePipelineState *ps = &ctx->ps;
	MI_VENC_Stat_t stat = {0};
	MI_VENC_Stream_t stream = {0};
	int ret;

	ret = MI_VENC_Query(ps->venc_channel, &stat);
	if (ret != 0) {
		if ((++(*idle_counter) % 60) == 0) {
			printf("MI_VENC_Query failed %d\n", ret);
			fflush(stdout);
		}
		star6e_pipeline_cus3a_tick(&g_sdk_quiet, cus3a_ts_last);
		idle_wait(5);
		return 0;
	}

	if (stat.curPacks == 0) {
		if ((++(*idle_counter) % 120) == 0) {
			printf("waiting for encoder data...\n");
			fflush(stdout);
		}
		star6e_pipeline_cus3a_tick(&g_sdk_quiet, cus3a_ts_last);
		idle_wait(1);
		return 0;
	}
	*idle_counter = 0;

	stream.count = stat.curPacks;
	stream.packet = ensure_packs(&ps->stream_packs,
		&ps->stream_packs_cap, stat.curPacks);
	if (!stream.packet) {
		fprintf(stderr, "ERROR: Unable to allocate stream packets\n");
		snprintf(g_fault_what, sizeof(g_fault_what),
			"stream pack alloc failed (%u packs)",
			(unsigned)stat.curPacks);
		return -1;
	}

	ret = MI_VENC_GetStream(ps->venc_channel, &stream, 40);
	if (ret != 0) {
		if (ret == -EAGAIN || ret == EAGAIN) {
			idle_wait(2);
			return 0;
		}
		fprintf(stderr, "ERROR: MI_VENC_GetStream failed %d\n", ret);
		snprintf(g_fault_what, sizeof(g_fault_what),
			"MI_VENC_GetStream(chn %d) failed %d",
			(int)ps->venc_channel, ret);
		return ret;
	}
	if (star6e_output_reject_incomplete_access_unit(&ps->output,
	    &stream)) {
		MI_VENC_ReleaseStream(ps->venc_channel, &stream);
		return 0;
	}

	/* refPred error-resilience marking — rewrite TRAIL_R → TRAIL_N for
	 * frames the SDK marked as ENHANCE_P_NOTFORREF.  The encoder's own
	 * SVC-T pyramid logic determines which frames are non-reference; we
	 * just propagate that designation into the bitstream so generic
	 * receivers can safely drop those NALs without cascade.
	 *
	 * Only active when refPred was successfully applied — otherwise the encoder
	 * produces a flat single-ref stream and every frame matters. */
	if (ps->output.svct_active &&
	    stream.h265Info.refType == STAR6E_REFTYPE_ENHANCE_P_NOTFORREF) {
		star6e_patch_stream_to_trail_n(&stream);
	}

	/* Observe ring pressure every frame.  waybeam gated this on a live
	 * RTP-sidecar subscription because the query was a SIOCOUTQ ioctl;
	 * for the frame ring it is two relaxed atomic loads out of the shm
	 * header, so the gate is gone and the counters are always fresh for
	 * the facade's telemetry.  Never skips a frame — a producer-side
	 * skip breaks the H.265 reference chain (see HISTORY 0.9.2). */
	star6e_output_observe_pressure(&ps->output);

	(void)star6e_video_send_frame(&ps->video, &ps->output, &stream);

	/* Release the encoder stream as soon as the last consumer of the
	 * stream payload is done, so the VENC output slot is free for the
	 * next frame. */
	MI_VENC_ReleaseStream(ps->venc_channel, &stream);

	/* Orientation (image.flip / image.mirror) is applied once at bring-up
	 * (sensor_select before MI_SNR_Enable + start_vpe before VPE start) and
	 * holds for the life of the stream — device-verified on IMX335 and
	 * IMX415.  The sensor driver only rewrites orientation when we set it
	 * (orien_dirty), so nothing clears it mid-stream; no per-frame re-apply
	 * is needed.  (MI_SNR_GetOrien proved unreliable under AE I2C load on
	 * IMX335 — it reads 0 while the image is plainly held — so we do not
	 * use it to second-guess the applied state.) */

	star6e_pipeline_cus3a_tick(&g_sdk_quiet, cus3a_ts_last);
	return 0;
}

void star6e_runtime_request_stop(void)
{
	g_running = 0;
}

void *star6e_runtime_encoder_loop(void *opaque)
{
	Star6eRunnerContext *ctx = opaque;
	struct timespec cus3a_ts_last = {0};
	struct timespec run_start;
	int cold_boot_fps_kick_done = 0;
	unsigned int idle_counter = 0;
	int faulted = 0;

	if (!ctx)
		return NULL;

	clock_gettime(CLOCK_MONOTONIC, &cus3a_ts_last);
	clock_gettime(CLOCK_MONOTONIC, &run_start);

	/* Pin encoder to CPU 0 and run the GetStream->publish path at an
	 * elevated SCHED_FIFO priority.  At the previous minimum priority (1)
	 * this thread was preempted mid-frame by other userspace RT threads
	 * and SCHED_OTHER work, surfacing as a periodic ~one-frame delivery
	 * stall (a single idle gap on the wire, no catch-up burst).  The SDK
	 * pipeline kernel threads run at SCHED_RR/98 and MUST keep outranking
	 * us — we depend on them to produce frames — so the priority is
	 * clamped well below 98 (~90 made timing worse, likely priority
	 * inversion).
	 *
	 * Tunable for on-device A/B without a rebuild via the VENC_RT_PRIO env
	 * var (clamped 1..80); VENC_RT_PRIO=1 reproduces the old behaviour.
	 * Silent fallback if unprivileged or single-core. */
	{
		unsigned long mask = 1UL;  /* CPU 0 */
		syscall(__NR_sched_setaffinity, 0, sizeof(mask), &mask);

		int rt_prio = 50;
		const char *env = getenv("VENC_RT_PRIO");
		if (env && *env) {
			int v = atoi(env);
			if (v < 1)
				v = 1;
			else if (v > 80)
				v = 80;
			rt_prio = v;
		}

		struct sched_param sp;
		sp.sched_priority = rt_prio;
		if (pthread_setschedparam(pthread_self(), SCHED_FIFO,
		    &sp) != 0)
			printf("> note: RT priority not available"
				" (run as root)\n");
		else
			printf("> encoder thread: SCHED_FIFO prio %d,"
				" pinned CPU0\n", rt_prio);
	}

	while (g_running) {
		if (star6e_runtime_process_stream(ctx, &cus3a_ts_last,
		    &idle_counter) != 0) {
			/* A stop racing the failure is a stop, not a fault:
			 * MI calls legitimately start failing the moment the
			 * pipeline goes down. */
			faulted = g_running ? 1 : 0;
			break;
		}

		/* One-shot cold-boot fps re-kick ~1.5s after start, once the ISP
		 * bin load + AE have settled (the init-time kick fires too early
		 * and doesn't stick on a cold boot). */
		if (!cold_boot_fps_kick_done) {
			struct timespec now;
			clock_gettime(CLOCK_MONOTONIC, &now);
			if ((now.tv_sec - run_start.tv_sec) +
			    (now.tv_nsec - run_start.tv_nsec) / 1e9 >= 1.5) {
				star6e_pipeline_cold_boot_fps_rekick(&ctx->ps);
				cold_boot_fps_kick_done = 1;
			}
		}
	}

	if (!faulted)
		return NULL;
	if (!g_fault_what[0])
		snprintf(g_fault_what, sizeof(g_fault_what),
			"encoder loop aborted");
	return (void *)g_fault_what;
}

/* ── Lifecycle ────────────────────────────────────────────────────────── */

int star6e_runtime_prepare(void)
{
	g_running = 1;
	g_fault_what[0] = '\0';
	sdk_quiet_state_init(&g_sdk_quiet);
	star6e_controls_reset();
	star6e_pipeline_cus3a_reset();
	return 0;
}

int star6e_runtime_init(Star6eRunnerContext *ctx)
{
	int ret;

	if (!ctx)
		return -1;

	if (star6e_mi_init() != 0) {
		fprintf(stderr, "ERROR: MI library load failed\n");
		return -1;
	}

	sdk_quiet_begin(&g_sdk_quiet);
	ret = MI_SYS_Init();
	sdk_quiet_end(&g_sdk_quiet);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_SYS_Init failed %d\n", ret);
		star6e_mi_deinit();
		return ret;
	}
	ctx->system_initialized = 1;

	/* Always, before any VIF/VPE/ISP bring-up: reconcile NPU driver state a
	 * predecessor may have poisoned.  A process that ran the i6e IPU
	 * detector can leave kernel-side state that permanently wedges the NEXT
	 * process's ISP CMDQ — the successor then never sees an ISP channel
	 * ("ISP channel readiness timeout" -> "MI_ISP_*CmdLoadBinFile failed -1"
	 * -> "layout type 2, bindmode 4 not sync err" -> zero frames).  A bare
	 * MI_IPU_CreateDevice+DestroyDevice cycle resets it, and nothing short
	 * of it reliably does; a scrub AFTER the ISP has wedged does not
	 * recover it, hence the placement here.  Unconditional by design — the
	 * poison survives process exit and fd release, so no flag carried from
	 * the previous instance can be trusted to know whether it is needed,
	 * and mabur's predecessor on this SoC is whatever ran before us
	 * (waybeam, an older maburd), not something we control.  No-op when
	 * /dev/mi_ipu is absent.  waybeam ran this on every start; the fold-in
	 * dropped it with the detector and lost the pipeline with it.
	 * (waybeam_venc f956a52:src/star6e_runtime.c star6e_runner_init,
	 * HISTORY 0.53.0/0.54.0.) */
	(void)star6e_ipu_scrub();

	ret = star6e_pipeline_start(&ctx->ps, &ctx->cfg, &g_sdk_quiet);
	if (ret != 0)
		return ret;
	ctx->pipeline_started = 1;

	return star6e_runtime_apply_startup_controls(ctx);
}

void star6e_runtime_teardown(Star6eRunnerContext *ctx)
{
	if (!ctx)
		return;

	star6e_cus3a_request_stop();

	if (ctx->pipeline_started) {
		star6e_controls_reset();
		star6e_pipeline_stop(&ctx->ps);
		ctx->pipeline_started = 0;
	}

	/* Now safe to join the 3A thread — pipeline is stopped so ISP
	 * calls will return errors and the thread will exit. */
	star6e_cus3a_join();

	if (ctx->system_initialized) {
		MI_SYS_Exit();
		ctx->system_initialized = 0;
		star6e_pipeline_vpe_scl_preset_shutdown();
	}
	star6e_mi_deinit();
}
