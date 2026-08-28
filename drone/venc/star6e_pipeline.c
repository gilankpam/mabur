/* ported from waybeam_venc f956a52:src/star6e_pipeline.c */
#include "star6e_pipeline.h"

#include "star6e_awb.h"

#include "codec_types.h"
#include "intra_refresh.h"
#include "isp_runtime.h"
#include "pipeline_common.h"
#include "venc_cfg.h"
#include "venc_jpeg.h"

#include <dlfcn.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* ── Compiled constants ───────────────────────────────────────────────────
 * These were waybeam JSON keys with exactly one value mabur ever uses, so
 * they are pinned here rather than carried through VencCfg (spec §3).
 * Values are waybeam's venc_config_defaults() (f956a52:src/venc_config.c). */
#define STAR6E_OVERCLOCK_LEVEL      1      /* system.overclockLevel */
#define STAR6E_KEEP_ASPECT          true   /* isp.keepAspect */
#define STAR6E_SHUTTER_RULE_180     false  /* isp.shutterRule180 */
#define STAR6E_IMAGE_MIRROR         0      /* image.mirror */
#define STAR6E_IMAGE_FLIP           0      /* image.flip */
#define STAR6E_VPE_LEVEL_3DNR       0      /* fpv.noiseLevel */
#define STAR6E_SENSOR_FORCED_PAD    (-1)   /* sensor.index: auto-detect */
#define STAR6E_SENSOR_FORCED_MODE   (-1)   /* sensor.mode: auto-select */
/* MJPEG snapshot channel; 7 is well clear of the encoder's ch0. */
#define STAR6E_SNAPSHOT_CHANNEL     7
/* Video codec is always H.265 and rate control is always CBR — the
 * intra-refresh scheduler and SVC-T refPred both assume HEVC, and the
 * ladder wants a hard rate.  rc_mode 3 is the CBR case in
 * star6e_pipeline_start_venc's switch. */
#define STAR6E_RC_MODE_CBR          3

typedef struct {
	unsigned int minShutterUs;
	unsigned int maxShutterUs;
	unsigned int minApertX10;
	unsigned int maxApertX10;
	unsigned int minSensorGain;
	unsigned int minIspGain;
	unsigned int maxSensorGain;
	unsigned int maxIspGain;
} IspExposureLimit;

typedef int (*isp_load_bin_fn_t)(int channel, char *path, unsigned int key);
typedef int (*isp_disable_userspace3a_fn_t)(int channel);
typedef int (*cus3a_fn_t)(int channel, void *params);

static void star6e_pipeline_reset(Star6ePipelineState *state)
{
	if (!state)
		return;

	star6e_video_reset(&state->video);
	memset(state, 0, sizeof(*state));
	star6e_output_reset(&state->output);
}

/* VPE SCL clock workaround — written at shutdown, effective on next start.
 *
 * Root cause: at process exit, mi_vpe_process_exit → MI_VPE_IMPL_DeInit →
 * DrvSclModuleClkDeInit disables the VPE SCL clock (fclk1) via
 * clk_disable_unprepare. On the next run, MI_VPE_IMPL_Init has a persistent
 * kernel-side "already_inited" flag that causes it to skip
 * DrvSclModuleClkInit, so fclk1 is never re-enabled.
 *
 * Writing after MI_SYS_Exit() triggers the preset path while the VPE fd is
 * closed. The preset persists in kernel memory until the next init. */
void star6e_pipeline_vpe_scl_preset_shutdown(void)
{
	int fd = open("/sys/devices/virtual/mstar/mscl/clk", O_WRONLY);

	if (fd < 0)
		return;

	(void)write(fd, "384000000\n", 10);
	close(fd);
	(void)write(STDERR_FILENO, "[venc] VPE SCL preset stored for next run\n",
		41);
}

/* Called after MI_SYS_Init() to silently tear down any pipeline state left
 * by an unclean previous exit. All errors are ignored.
 *
 * When MI libs are loaded via dlopen (no direct linking), the vendor VPE
 * module calls exit(127) on API calls before a channel is created. Guard
 * VPE teardown with a channel existence check to avoid this. */
static void star6e_pipeline_pre_init_teardown(void)
{
	MI_SYS_ChnPort_t vif_port = {
		.module = I6_SYS_MOD_VIF, .device = 0, .channel = 0, .port = 0 };
	MI_SYS_ChnPort_t vpe_port = {
		.module = I6_SYS_MOD_VPE, .device = 0, .channel = 0, .port = 0 };
	MI_SYS_ChnPort_t venc_port = {
		.module = I6_SYS_MOD_VENC, .device = 0, .channel = 0, .port = 0 };

	(void)MI_SYS_UnBindChnPort(&vpe_port, &venc_port);
	(void)MI_SYS_UnBindChnPort(&vif_port, &vpe_port);
	(void)MI_VENC_StopRecvPic(0);
	(void)MI_VENC_DestroyChn(0);

	/* VPE: probe channel existence before teardown — MI_VPE_DisablePort
	 * calls exit(127) when called on a non-existent channel under dlopen. */
	MI_VPE_ChannelAttr_t probe_attr;
	if (MI_VPE_GetChannelAttr(0, &probe_attr) == 0) {
		/* port1 too, not just port0.  An unclean exit can leave the
		 * second-scaler tap enabled and the teardown below would then
		 * run with a live port still registered. */
		MI_S32 p1 = MI_VPE_DisablePort(0, 1);
		if (p1 == 0)
			fprintf(stderr, "[venc] pre-init: stale VPE port1 was "
				"enabled — disabled\n");
		(void)MI_VPE_DisablePort(0, 0);
		(void)MI_VPE_StopChannel(0);
		(void)MI_VPE_DestroyChannel(0);
	}

	(void)MI_VIF_DisableChnPort(0, 0);
	(void)MI_VIF_DisableDev(0);
}

static int star6e_pipeline_disable_userspace3a(const IspRuntimeLib *lib,
	void *ctx)
{
	isp_disable_userspace3a_fn_t fn;

	(void)ctx;
	fn = (isp_disable_userspace3a_fn_t)lib->disable_userspace3a;
	return fn ? fn(0) : 0;
}

/* Poll MI_ISP_IQ_GetParaInitStatus until bFlag==1 or timeout (2000 ms).
 * Called standalone after VIF→VPE bind when a new VPE channel was just
 * created: the ISP channel initialises asynchronously after
 * MI_VPE_CreateChannel returns, so anything that touches the ISP (bin load,
 * exposure cap) must wait here first.  Without this poll the ISP would emit
 * "IspApiGet channel not created" kernel errors on the first probe. */
static void star6e_pipeline_wait_isp_channel(void)
{
	typedef struct { int bFlag; } IspParaInitInfoParam;
	typedef struct { IspParaInitInfoParam stParaAPI; } IspParaInitInfoType;
	typedef int (*fn_get_para_init_t)(int, IspParaInitInfoType *);
	fn_get_para_init_t fn;
	void *handle;
	int elapsed_ms = 0;

	handle = dlopen("libmi_isp.so", RTLD_LAZY | RTLD_GLOBAL);
	if (!handle) {
		usleep(100 * 1000);
		return;
	}

	fn = (fn_get_para_init_t)dlsym(handle, "MI_ISP_IQ_GetParaInitStatus");
	if (!fn) {
		usleep(100 * 1000);
		dlclose(handle);
		return;
	}

	while (elapsed_ms < 2000) {
		IspParaInitInfoType info;

		memset(&info, 0, sizeof(info));
		if (fn(0, &info) == 0 && info.stParaAPI.bFlag == 1) {
			printf("> ISP channel ready after %d ms\n", elapsed_ms);
			dlclose(handle);
			return;
		}
		usleep(1000);
		elapsed_ms++;
	}

	fprintf(stderr, "WARNING: ISP channel readiness timeout after 2000 ms\n");
	dlclose(handle);
}

static int star6e_pipeline_wait_isp_ready(const IspRuntimeLib *lib, void *ctx)
{
	typedef struct { int bFlag; } IspParaInitInfoParam;
	typedef struct { IspParaInitInfoParam stParaAPI; } IspParaInitInfoType;
	typedef int (*fn_get_para_init_t)(int, IspParaInitInfoType *);
	fn_get_para_init_t fn;
	int elapsed_ms = 0;

	(void)ctx;
	fn = (fn_get_para_init_t)dlsym(lib->handle,
		"MI_ISP_IQ_GetParaInitStatus");
	if (!fn) {
		/* Symbol not available — fall back to fixed delay */
		usleep(100 * 1000);
		return 0;
	}

	while (elapsed_ms < 2000) {
		IspParaInitInfoType info;

		memset(&info, 0, sizeof(info));
		if (fn(0, &info) == 0 && info.stParaAPI.bFlag == 1) {
			printf("> ISP ready after %d ms\n", elapsed_ms);
			return 0;
		}
		usleep(1000);
		elapsed_ms++;
	}

	fprintf(stderr, "WARNING: ISP readiness timeout after 2000 ms\n");
	return -1;
}

static int star6e_pipeline_call_load_bin(const IspRuntimeLib *lib,
	const char *path, unsigned int load_key, void *ctx)
{
	isp_load_bin_fn_t fn_api;
	isp_load_bin_fn_t fn_api_alt;
	int ret;

	(void)ctx;
	fn_api = (isp_load_bin_fn_t)lib->load_bin_api;
	fn_api_alt = (isp_load_bin_fn_t)lib->load_bin_api_alt;
	ret = -1;
	if (fn_api)
		ret = fn_api(0, (char *)path, load_key);
	if (ret != 0 && fn_api_alt && fn_api_alt != fn_api)
		ret = fn_api_alt(0, (char *)path, load_key);
	return ret;
}

/* MI_ISP_CUS3A_Enable(MI_U32 Channel, Cus3AEnable_t *data).  Each flag selects
 * who OWNS that module: 1 = custom/userspace code, 0 = the ISP-internal
 * algorithm.  It is not an "enable the algorithm" switch — setting bAWB=1
 * without supplying an AWB implementation leaves AWB driven by nothing, which
 * is exactly why this is paired with the star6e_awb loop.
 *
 * MI_BOOL is `unsigned char` on i6e, so the struct is THREE BYTES.
 * Layout from the i6e SDK header (mi_isp_datatype.h). */
typedef struct {
	unsigned char bAE;
	unsigned char bAWB;
	unsigned char bAF;
} Star6eCus3AEnable;

static void star6e_pipeline_post_load_cus3a(const IspRuntimeLib *lib,
	void *ctx)
{
	cus3a_fn_t fn_cus3a;
	Star6eCus3AEnable ae_only  = {1, 0, 0};
	Star6eCus3AEnable ae_awb   = {1, 1, 0};

	(void)ctx;
	fn_cus3a = (cus3a_fn_t)lib->cus3a_enable;
	if (!fn_cus3a)
		return;

	fn_cus3a(0, &ae_only);
	fn_cus3a(0, &ae_awb);
}

static int star6e_pipeline_load_isp_bin(const char *isp_bin_path,
	SdkQuietState *sdk_quiet)
{
	IspRuntimeLoadHooks hooks;

	if (!isp_bin_path || !*isp_bin_path)
		return 0;

	memset(&hooks, 0, sizeof(hooks));
	hooks.load_key = 1234;
	hooks.ctx = sdk_quiet;
	hooks.quiet_begin = (void (*)(void *))sdk_quiet_begin;
	hooks.quiet_end = (void (*)(void *))sdk_quiet_end;
	hooks.disable_userspace3a = star6e_pipeline_disable_userspace3a;
	hooks.wait_ready = star6e_pipeline_wait_isp_ready;
	hooks.load_bin = star6e_pipeline_call_load_bin;
	hooks.post_load = star6e_pipeline_post_load_cus3a;

	return isp_runtime_load_bin_file(isp_bin_path, &hooks);
}

static void star6e_pipeline_enable_cus3a(SdkQuietState *sdk_quiet)
{
	void *handle = dlopen("libmi_isp.so", RTLD_LAZY | RTLD_GLOBAL);
	cus3a_fn_t fn;

	if (!handle)
		return;

	fn = (cus3a_fn_t)dlsym(handle, "MI_ISP_CUS3A_Enable");
	if (fn) {
		Star6eCus3AEnable ae_only = {1, 0, 0};
		Star6eCus3AEnable ae_awb  = {1, 1, 0};
		MI_S32 ret;

		sdk_quiet_begin(sdk_quiet);
		fn(0, &ae_only);
		ret = fn(0, &ae_awb);
		sdk_quiet_end(sdk_quiet);
		if (ret != 0)
			fprintf(stderr, "WARNING: MI_ISP_CUS3A_Enable(AE+AWB) failed %d\n", ret);
	}

	dlclose(handle);
}

/* Reachable from two threads — the pipeline thread's handoff tick and the
 * facade's AWB-owner change — so the symbol lookup is done under
 * pthread_once rather than the usual lazy-static idiom, which would race on
 * first use. */
static void *g_cus3a_lib;
static int (*g_cus3a_fn)(int channel, void *params);
static pthread_once_t g_cus3a_once = PTHREAD_ONCE_INIT;

static void star6e_cus3a_bind_once(void)
{
	g_cus3a_lib = dlopen("libmi_isp.so", RTLD_LAZY | RTLD_GLOBAL);
	if (g_cus3a_lib)
		g_cus3a_fn = (int (*)(int, void *))dlsym(g_cus3a_lib,
			"MI_ISP_CUS3A_Enable");
}

static void star6e_pipeline_cus3a_apply(SdkQuietState *sdk_quiet,
	Star6eCus3AEnable *params)
{
	int (*fn)(int, void *);

	pthread_once(&g_cus3a_once, star6e_cus3a_bind_once);
	fn = g_cus3a_fn;
	if (!fn)
		return;

	sdk_quiet_begin(sdk_quiet);
	fn(0, params);
	sdk_quiet_end(sdk_quiet);
}

static int g_cus3a_handoff_done = 0;

void star6e_pipeline_cus3a_reset(void)
{
	g_cus3a_handoff_done = 0;
}

/* Delayed cold-boot fps re-kick.  The pipeline-init MI_SNR_SetFps
 * (bind_and_finalize_pipeline) fires before the ISP bin's AE has settled, so on
 * a cold boot the sensor's physical timing register can be left below the
 * configured fps (observed ~70fps @ target 90; a warm restart keeps the kernel
 * sensor state so it shows ~90).  The supervisory AE enforcer only touches the
 * ISP exposure limit, not the sensor timing register, so re-issue SetFps once
 * from the run loop ~1.5s after start, after the bin load + AE converge, to
 * force the sensor register to the target.  No-op when fps is 0. */
void star6e_pipeline_cold_boot_fps_rekick(const Star6ePipelineState *state)
{
	if (!state)
		return;
	if (state->sensor.fps == 0)
		return;
	printf("[venc] cold-boot fps re-kick: SetFps(%u)\n",
		state->sensor.fps);
	fflush(stdout);
	MI_SNR_SetFps(state->sensor.pad_id, state->sensor.fps);
}

/* Which algorithm owns AWB.  1 = handed to userspace (our star6e_awb loop
 * drives it); 0 = ISP-internal.  Read by the handoff below and re-applied
 * live on a mode change. */
static volatile int g_awb_userspace = 0;

void star6e_pipeline_set_awb_userspace(int on)
{
	Star6eCus3AEnable en = {0, 0, 0};

	g_awb_userspace = on ? 1 : 0;
	en.bAWB = (unsigned char)g_awb_userspace;
	/* NULL sdk_quiet: this can run off the pipeline thread, where
	 * suppressing stdio would race the pipeline thread's quiet windows. */
	if (g_cus3a_handoff_done)
		star6e_pipeline_cus3a_apply(NULL, &en);
}

void star6e_pipeline_cus3a_tick(SdkQuietState *sdk_quiet,
	struct timespec *ts_last)
{
	struct timespec now;
	long long elapsed_ms;
	/* Hand AE back to the ISP-internal algorithm (as before — that is what
	 * keeps AE cheap at high frame rates) but KEEP AWB in the CUS3A engine,
	 * otherwise it stops converging the moment this fires and the boot-time
	 * cast is frozen in for the life of the process. */
	Star6eCus3AEnable handoff = {0, 0, 0};

	handoff.bAWB = (unsigned char)g_awb_userspace;

	if (g_cus3a_handoff_done || !ts_last)
		return;

	clock_gettime(CLOCK_MONOTONIC, &now);
	elapsed_ms =
		((long long)(now.tv_sec - ts_last->tv_sec) * 1000LL) +
		((long long)(now.tv_nsec - ts_last->tv_nsec) / 1000000LL);
	if (elapsed_ms < 1000)
		return;

	star6e_pipeline_cus3a_apply(sdk_quiet, &handoff);
	g_cus3a_handoff_done = 1;
}

int star6e_pipeline_cap_exposure_for_fps(uint32_t fps,
	bool shutter_rule_180)
{
	return pipeline_common_cap_exposure_for_fps(fps, shutter_rule_180);
}

static void star6e_pipeline_stop_sensor(MI_SNR_PAD_ID_e pad_id)
{
	MI_SNR_Disable(pad_id);
}

static Star6ePrecropRect star6e_pipeline_compute_precrop(uint32_t sensor_w,
	uint32_t sensor_h, uint32_t image_w, uint32_t image_h,
	bool keep_aspect)
{
	PipelinePrecropRect common = pipeline_common_compute_precrop(
		sensor_w, sensor_h, image_w, image_h, keep_aspect);
	Star6ePrecropRect rect = {common.x, common.y, common.w, common.h};
	return rect;
}

static int star6e_pipeline_start_vif(const SensorSelectResult *sensor,
	const Star6ePrecropRect *precrop)
{
	MI_VIF_DevAttr_t dev = {0};
	MI_VIF_PortAttr_t port = {0};
	MI_S32 ret;

	dev.intf = sensor->pad.intf;
	dev.work = (sensor->pad.intf == I6_INTF_BT656) ? I6_VIF_WORK_1MULTIPLEX :
		I6_VIF_WORK_RGB_REALTIME;
	dev.hdr = I6_HDR_OFF;

	if (sensor->pad.intf == I6_INTF_MIPI) {
		dev.edge = I6_EDGE_DOUBLE;
		dev.input = sensor->pad.intfAttr.mipi.input;
	} else if (sensor->pad.intf == I6_INTF_BT656) {
		dev.edge = sensor->pad.intfAttr.bt656.edge;
		dev.sync = sensor->pad.intfAttr.bt656.sync;
		dev.bitswap = sensor->pad.intfAttr.bt656.bitswap;
	}

	ret = MI_VIF_SetDevAttr(0, &dev);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_VIF_SetDevAttr failed %d\n", ret);
		return ret;
	}

	ret = MI_VIF_EnableDev(0);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_VIF_EnableDev failed %d\n", ret);
		return ret;
	}

	port.capt.x = sensor->plane.capt.x + precrop->x;
	port.capt.y = sensor->plane.capt.y + precrop->y;
	port.capt.width = precrop->w;
	port.capt.height = precrop->h;
	port.dest.width = precrop->w;
	port.dest.height = precrop->h;
	port.field = 0;
	port.interlaceOn = 0;
	if (sensor->plane.bayer > I6_BAYER_END) {
		port.pixFmt = sensor->plane.pixFmt;
	} else {
		port.pixFmt = (i6_common_pixfmt)(I6_PIXFMT_RGB_BAYER +
			sensor->plane.precision * I6_BAYER_END + sensor->plane.bayer);
	}
	port.frate = I6_VIF_FRATE_FULL;
	port.frameLineCnt = 0;

	ret = MI_VIF_SetChnPortAttr(0, 0, &port);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_VIF_SetChnPortAttr failed %d\n", ret);
		MI_VIF_DisableDev(0);
		return ret;
	}

	ret = MI_VIF_EnableChnPort(0, 0);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_VIF_EnableChnPort failed %d\n", ret);
		MI_VIF_DisableChnPort(0, 0);
		MI_VIF_DisableDev(0);
		return ret;
	}

	return 0;
}

static void star6e_pipeline_stop_vif(void)
{
	MI_VIF_DisableChnPort(0, 0);
	MI_VIF_DisableDev(0);
}

static int star6e_pipeline_start_vpe(const SensorSelectResult *sensor,
	const Star6ePrecropRect *precrop, uint32_t out_width,
	uint32_t out_height, int mirror, int flip, int level_3dnr,
	SdkQuietState *sdk_quiet)
{
	MI_VPE_ChannelAttr_t channel_attr = {0};
	MI_VPE_ChannelParam_t param = {0};
	MI_VPE_PortAttr_t port = {0};
	MI_S32 ret;

	channel_attr.capt.width = precrop->w;
	channel_attr.capt.height = precrop->h;
	channel_attr.hdr = I6_HDR_OFF;
	channel_attr.sensor = (i6_vpe_sens)((int)sensor->pad_id + 1);
	channel_attr.mode = I6_VPE_MODE_REALTIME;
	if (sensor->plane.bayer > I6_BAYER_END) {
		channel_attr.pixFmt = sensor->plane.pixFmt;
	} else {
		channel_attr.pixFmt = (i6_common_pixfmt)(I6_PIXFMT_RGB_BAYER +
			sensor->plane.precision * I6_BAYER_END + sensor->plane.bayer);
	}

	sdk_quiet_begin(sdk_quiet);
	ret = MI_VPE_CreateChannel(0, &channel_attr);
	sdk_quiet_end(sdk_quiet);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_VPE_CreateChannel failed %d\n", ret);
		return ret;
	}

	param.hdr = I6_HDR_OFF;
	param.level3DNR = level_3dnr;
	param.mirror = 0;
	param.flip = 0;
	param.lensAdjOn = 0;
	ret = MI_VPE_SetChannelParam(0, &param);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_VPE_SetChannelParam failed %d\n", ret);
		MI_VPE_DestroyChannel(0);
		return ret;
	}

	/* Sensor-level orientation.  VPE digital mirror/flip is not used
	 * (unreliable on several sensor combos; param above keeps them at 0).
	 * MI_SNR_SetOrien programs the sensor's own register directly.
	 * Non-fatal: log if it fails so BSP regressions surface. */
	{
		MI_S32 orien_ret = MI_SNR_SetOrien(sensor->pad_id,
			mirror ? 1 : 0, flip ? 1 : 0);
		if (orien_ret != 0)
			fprintf(stderr, "[pipeline] WARNING: "
				"MI_SNR_SetOrien(pad=%d mirror=%d flip=%d) "
				"returned %d\n",
				(int)sensor->pad_id, mirror ? 1 : 0,
				flip ? 1 : 0, (int)orien_ret);
	}

	ret = MI_VPE_StartChannel(0);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_VPE_StartChannel failed %d\n", ret);
		MI_VPE_DestroyChannel(0);
		return ret;
	}

	port.output.width = out_width;
	port.output.height = out_height;
	port.pixFmt = I6_PIXFMT_YUV420SP;
	port.compress = I6_COMPR_NONE;

	ret = MI_VPE_SetPortMode(0, 0, &port);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_VPE_SetPortMode failed %d\n", ret);
		MI_VPE_StopChannel(0);
		MI_VPE_DestroyChannel(0);
		return ret;
	}

	ret = MI_VPE_EnablePort(0, 0);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_VPE_EnablePort failed %d\n", ret);
		MI_VPE_DisablePort(0, 0);
		MI_VPE_StopChannel(0);
		MI_VPE_DestroyChannel(0);
		return ret;
	}

	return 0;
}

static void star6e_pipeline_stop_vpe(void)
{
	MI_VPE_DisablePort(0, 0);
	MI_VPE_StopChannel(0);
	MI_VPE_DestroyChannel(0);
}

static void star6e_pipeline_fill_h26x_attr(i6_venc_attr_h26x *attr,
	uint32_t width, uint32_t height)
{
	attr->maxWidth = width;
	attr->maxHeight = height;
	attr->bufSize = width * height * 3 / 2;
	attr->profile = 0;
	attr->byFrame = 1;
	attr->width = width;
	attr->height = height;
	attr->bFrameNum = 0;
	attr->refNum = 1;
}

static int star6e_pipeline_pre_start_apply_ref_pred(MI_VENC_CHN chn,
	const VencPresetOut *preset);

static int star6e_pipeline_start_venc(uint32_t width, uint32_t height,
	uint32_t bitrate, uint32_t framerate, uint32_t gop,
	const VencPresetOut *preset, MI_VENC_CHN *chn)
{
	MI_VENC_ChnAttr_t attr = {0};
	MI_U32 bit_rate_bits;
	MI_S32 ret;

	/* Same practical bitrate bounds as the live apply_bitrate() path. */
	if (bitrate > VENC_BITRATE_MAX_KBPS)
		bitrate = VENC_BITRATE_MAX_KBPS;
	if (bitrate < VENC_BITRATE_MIN_KBPS)
		bitrate = VENC_BITRATE_MIN_KBPS;
	bit_rate_bits = bitrate * 1024;

	attr.attrib.codec = I6_VENC_CODEC_H265;
	star6e_pipeline_fill_h26x_attr(&attr.attrib.h265, width, height);

	attr.rate.mode = I6_VENC_RATEMODE_H265CBR;
	attr.rate.h265Cbr = (i6_venc_rate_h26xcbr){
		.gop = gop, .statTime = 1,
		.fpsNum = framerate, .fpsDen = 1,
		.bitrate = bit_rate_bits, .avgLvl = 1,
	};

	ret = MI_VENC_CreateChn(*chn, &attr);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_VENC_CreateChn(%d) failed %d\n",
			*chn, ret);
		return ret;
	}

	/* SDK convention: SetRefParam must be called between CreateChn and
	 * StartRecvPic.  Star6E silently no-ops the call if invoked after
	 * StartRecvPic, producing a flat single-layer stream. */
	(void)star6e_pipeline_pre_start_apply_ref_pred(*chn, preset);

	ret = MI_VENC_StartRecvPic(*chn);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_VENC_StartRecvPic failed %d\n", ret);
		MI_VENC_DestroyChn(*chn);
		return ret;
	}

	return 0;
}

static void star6e_pipeline_stop_venc(MI_VENC_CHN chn)
{
	MI_VENC_StopRecvPic(chn);
	MI_VENC_DestroyChn(chn);
}

static Star6eIntraRefreshStatus g_intra_status;
static pthread_mutex_t g_intra_status_mutex = PTHREAD_MUTEX_INITIALIZER;

void star6e_pipeline_intra_refresh_status(Star6eIntraRefreshStatus *out)
{
	if (!out)
		return;
	pthread_mutex_lock(&g_intra_status_mutex);
	*out = g_intra_status;
	pthread_mutex_unlock(&g_intra_status_mutex);
}

/* Compute IntraRefresh derived params from the expanded resilience preset.
 * Out param is always populated; mode is also returned for caller
 * convenience. */
static IntraRefreshMode star6e_pipeline_intra_refresh_derive(
	const VencPresetOut *preset, double explicit_gop_sec, uint32_t height,
	uint32_t fps, IntraRefreshDerived *out_ir)
{
	IntraRefreshMode mode = INTRA_MODE_OFF;

	memset(out_ir, 0, sizeof(*out_ir));
	if (preset) {
		mode = intra_refresh_parse_mode(preset->intra_refresh_mode);
		intra_refresh_compute(mode, height, fps,
			preset->intra_refresh_lines,
			preset->intra_refresh_qp,
			explicit_gop_sec, out_ir);
	}
	return mode;
}

static int star6e_pipeline_apply_intra_refresh(MI_VENC_CHN chn,
	const VencPresetOut *preset, double explicit_gop_sec, uint32_t height,
	uint32_t fps)
{
	MI_VENC_IntraRefresh_t cfg;
	Star6eIntraRefreshStatus snap;
	IntraRefreshDerived ir;
	IntraRefreshMode mode;
	const char *name;

	memset(&snap, 0, sizeof(snap));
	mode = star6e_pipeline_intra_refresh_derive(preset, explicit_gop_sec,
		height, fps, &ir);
	name = intra_refresh_mode_name(mode);

	snprintf(snap.mode_name, sizeof(snap.mode_name), "%s", name);
	snap.mi_supported = g_mi_venc.fnSetIntraRefresh ? 1 : 0;
	if (preset) {
		snap.requested_lines  = preset->intra_refresh_lines;
		snap.requested_qp     = preset->intra_refresh_qp;
		snap.explicit_gop_sec = explicit_gop_sec;
	}
	snap.target_ms             = ir.target_ms;
	snap.total_rows            = ir.total_rows;
	snap.effective_lines_per_p = ir.lines;
	snap.lines_clamped         = ir.lines_clamped;
	snap.effective_qp          = ir.req_iqp;
	snap.effective_gop_sec     = ir.gop_overridden ? snap.explicit_gop_sec : ir.gop_sec;
	snap.gop_auto              = ir.gop_overridden ? 0 : (ir.gop_sec > 0.0);

	if (mode == INTRA_MODE_OFF) {
		pthread_mutex_lock(&g_intra_status_mutex);
		g_intra_status = snap;
		pthread_mutex_unlock(&g_intra_status_mutex);
		return 0;
	}
	if (!g_mi_venc.fnSetIntraRefresh) {
		fprintf(stderr, "[venc] WARNING: intraRefreshMode=%s requested "
			"but libmi_venc.so does not export MI_VENC_SetIntraRefresh\n",
			name);
		pthread_mutex_lock(&g_intra_status_mutex);
		g_intra_status = snap;
		pthread_mutex_unlock(&g_intra_status_mutex);
		return -1;
	}
	if (ir.lines_clamped) {
		fprintf(stderr, "[venc] WARNING: intraRefreshLines exceeds picture "
			"LCU rows=%u, clamped\n", ir.total_rows);
	}
	if (ir.gop_overridden) {
		fprintf(stderr, "[venc] intra auto-GOP suppressed: explicit "
			"gopSize=%.2fs\n", snap.explicit_gop_sec);
	}

	memset(&cfg, 0, sizeof(cfg));
	cfg.bEnable = 1;
	cfg.u32RefreshLineNum = ir.lines;
	cfg.u32ReqIQp = ir.req_iqp;

	if (MI_VENC_SetIntraRefresh(chn, &cfg) != 0) {
		fprintf(stderr, "[venc] ERROR: MI_VENC_SetIntraRefresh(chn=%d, "
			"lines=%u, qp=%u) failed\n", chn,
			cfg.u32RefreshLineNum, cfg.u32ReqIQp);
		pthread_mutex_lock(&g_intra_status_mutex);
		g_intra_status = snap;
		pthread_mutex_unlock(&g_intra_status_mutex);
		return -1;
	}
	snap.apply_ok = 1;
	snap.active   = 1;
	pthread_mutex_lock(&g_intra_status_mutex);
	g_intra_status = snap;
	pthread_mutex_unlock(&g_intra_status_mutex);
	fprintf(stderr, "[venc] intraRefresh: mode=%s lines/P=%u qp=%u "
		"gop=%.2fs (%s)\n", name, cfg.u32RefreshLineNum, cfg.u32ReqIQp,
		snap.effective_gop_sec, snap.gop_auto ? "auto" : "explicit");
	return 0;
}

static Star6eRefPredStatus g_ref_pred_status;
static pthread_mutex_t g_ref_pred_status_mutex = PTHREAD_MUTEX_INITIALIZER;

void star6e_pipeline_ref_pred_status(Star6eRefPredStatus *out)
{
	if (!out)
		return;
	pthread_mutex_lock(&g_ref_pred_status_mutex);
	*out = g_ref_pred_status;
	pthread_mutex_unlock(&g_ref_pred_status_mutex);
}

/* SVC-T reference structure — driven by the resilience preset's refBase.
 * Disabled means SDK default single-layer reference (one P references the
 * previous P).
 *
 * Star6E SDK requires SetRefParam to land between CreateChn and
 * StartRecvPic — calling it later silently no-ops (verified with the
 * test_ref_pred harness: bitstream identical at any post-Start value).
 * Therefore this helper is invoked from star6e_pipeline_start_venc()
 * immediately after CreateChn. */
static int star6e_pipeline_pre_start_apply_ref_pred(MI_VENC_CHN chn,
	const VencPresetOut *preset)
{
	MI_VENC_ParamRef_t ref;
	Star6eRefPredStatus snap;

	memset(&snap, 0, sizeof(snap));
	snap.mi_supported = g_mi_venc.fnSetRefParam ? 1 : 0;
	if (preset) {
		snap.base    = preset->ref_base;
		snap.enhance = preset->ref_enhance;
		snap.pred    = preset->ref_pred ? 1 : 0;
	}

	if (!preset || preset->ref_base == 0)
		goto publish;
	if (!g_mi_venc.fnSetRefParam) {
		fprintf(stderr, "[venc] WARNING: refBase=%u requested but "
			"libmi_venc.so does not export MI_VENC_SetRefParam\n",
			preset->ref_base);
		goto publish;
	}

	memset(&ref, 0, sizeof(ref));
	ref.u32Base     = preset->ref_base;
	ref.u32Enhance  = preset->ref_enhance ? preset->ref_enhance : 1;
	ref.bEnablePred = preset->ref_pred ? 1 : 0;

	if (MI_VENC_SetRefParam(chn, &ref) != 0) {
		fprintf(stderr, "[venc] ERROR: MI_VENC_SetRefParam(chn=%d, "
			"base=%u, enhance=%u, pred=%u) failed\n", chn,
			ref.u32Base, ref.u32Enhance, ref.bEnablePred);
		goto publish;
	}
	snap.apply_ok = 1;
	snap.active   = 1;
	fprintf(stderr, "[venc] refPred: chn=%d base=%u enhance=%u pred=%u "
		"(applied pre-Start)\n", chn, ref.u32Base, ref.u32Enhance,
		ref.bEnablePred);
publish:
	pthread_mutex_lock(&g_ref_pred_status_mutex);
	g_ref_pred_status = snap;
	pthread_mutex_unlock(&g_ref_pred_status_mutex);
	return snap.active ? 0 : (preset && preset->ref_base > 0 ? -1 : 0);
}

static void star6e_pipeline_sysfs_write(const char *path, const char *value)
{
	FILE *handle = fopen(path, "w");

	if (!handle)
		return;

	fprintf(handle, "%s\n", value);
	fclose(handle);
}

static void star6e_pipeline_set_hw_clocks(int oc_level)
{
	static const struct {
		const char *path;
		const char *label;
	} clocks[] = {
		{ "/sys/devices/virtual/mstar/isp0/isp_clk", "ISP" },
		{ "/sys/devices/virtual/mstar/mscl/clk", "SCL" },
	};
	unsigned int i;

	for (i = 0; i < sizeof(clocks) / sizeof(clocks[0]); i++) {
		FILE *handle = fopen(clocks[i].path, "w");

		if (!handle)
			continue;

		fprintf(handle, "384000000\n");
		fclose(handle);
		printf("> Set %s clock to 384 MHz\n", clocks[i].label);
	}

	if (oc_level >= 1) {
		star6e_pipeline_sysfs_write(
			"/sys/devices/virtual/mstar/venc0/clk", "480000000");
		printf("> Set VENC clock to 480 MHz (oc-level %d)\n", oc_level);
	}

	if (oc_level >= 2) {
		star6e_pipeline_sysfs_write(
			"/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor",
			"performance");
		star6e_pipeline_sysfs_write(
			"/sys/devices/system/cpu/cpufreq/policy0/scaling_min_freq",
			"1200000");
		star6e_pipeline_sysfs_write(
			"/sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq",
			"1200000");
		printf("> Set CPU to 1200 MHz performance governor (oc-level %d)\n",
			oc_level);
	}
}

/* Aggregates all parameters derived from VencCfg before pipeline hardware
 * is touched.  Populated by prepare_pipeline_config(), consumed by the
 * remaining helpers and the main orchestrator. */
typedef struct {
	SensorSelectConfig sensor_cfg;
	SensorUnlockConfig sensor_unlock;
	SensorStrategy     sensor_strategy;
	VencPresetOut      preset;              /* expanded venc.resilience */
	char               isp_bin_path[256];   /* "" if no bin should be loaded;
	                                         * resolved by bind_and_finalize_pipeline()
	                                         * after we know the live sensor name */
	uint32_t           sensor_width;
	uint32_t           sensor_height;
	uint32_t           image_width;
	uint32_t           image_height;
	uint32_t           sensor_framerate;
	uint32_t           venc_max_rate;
	uint32_t           venc_gop_size;
	double             gop_sec;             /* preset GOP, else venc.gop_s */
	uint32_t           exposure_cap_us;
	Star6ePrecropRect  precrop;
} Star6ePipelineConfig;

/* Phase 1: validate cfg, derive all scalar parameters and the sensor
 * strategy.  No hardware is touched here. */
static int prepare_pipeline_config(const VencCfg *cfg,
	Star6ePipelineConfig *pconf)
{
	memset(pconf, 0, sizeof(*pconf));

	/* Unknown preset is a boot failure — mabur has no fall-back-to-off
	 * migration shim (CLAUDE.md: a bad config key must not boot). */
	if (venc_cfg_expand_preset(cfg, &pconf->preset) != 0) {
		fprintf(stderr, "ERROR: unknown venc.resilience preset '%s'\n",
			cfg->resilience);
		return -1;
	}
	pconf->gop_sec = pconf->preset.gop_overridden ? pconf->preset.gop_s
		: cfg->gop_s;

	pconf->sensor_width    = cfg->width;
	pconf->sensor_height   = cfg->height;
	pconf->image_width     = pconf->sensor_width;
	pconf->image_height    = pconf->sensor_height;
	pconf->sensor_framerate = cfg->fps;
	/* No venc.bitrate key (spec §3) — the channel is created at the floor
	 * and RcAgent programs the real rate on its first tick. */
	pconf->venc_max_rate   = VENC_BOOT_BITRATE_KBPS;

	/* Auto-cap exposure to frame period so the AE shutter never exceeds
	 * the frame period.  Without this, the AE converges on a long
	 * exposure that locks fps below the target. */
	pconf->exposure_cap_us = (pconf->sensor_framerate > 0) ?
		1000000 / pconf->sensor_framerate : 0;
	/* isp_bin_path is resolved later in bind_and_finalize_pipeline() once
	 * the live sensor name is known.  Leave empty here. */
	pconf->isp_bin_path[0] = '\0';

	pconf->sensor_cfg = pipeline_common_build_sensor_select_config(
		STAR6E_SENSOR_FORCED_PAD, STAR6E_SENSOR_FORCED_MODE,
		pconf->sensor_width, pconf->sensor_height, pconf->sensor_framerate);
	pconf->sensor_cfg.image_mirror = STAR6E_IMAGE_MIRROR;
	pconf->sensor_cfg.image_flip   = STAR6E_IMAGE_FLIP;
	/* IMX415/IMX335 high-FPS register hook: MI_SNR_CustFunction(pad,
	 * cmd_id=0x23, reg=0x300a, value=0x80, dir=0).  Required on cold boot
	 * for both sensors, otherwise MI_SNR_SetFps(pad, 120) returns
	 * -1608835041 and the sensor falls back to 30 fps.  Waybeam retired
	 * the JSON keys in 0.10.13 and fires it unconditionally. */
	pconf->sensor_unlock = (SensorUnlockConfig){
		.enabled = 1,
		.cmd_id  = 0x23,
		.reg     = 0x300a,
		.value   = 0x80,
		.dir     = (MI_SNR_CustDir_e)0,
	};
	pconf->sensor_strategy = sensor_unlock_strategy(&pconf->sensor_unlock);

	return 0;
}

/* Phase 2: run sensor_select(), resolve actual dimensions, compute precrop and
 * populate the relevant pconf fields.  Logs the pipeline geometry summary. */
static int select_and_configure_sensor(Star6ePipelineState *state,
	Star6ePipelineConfig *pconf, SdkQuietState *sdk_quiet)
{
	uint32_t sensor_width;
	uint32_t sensor_height;
	uint16_t overscan_x;
	uint16_t overscan_y;
	int ret;

	sdk_quiet_begin(sdk_quiet);
	star6e_pipeline_pre_init_teardown();
	sdk_quiet_end(sdk_quiet);

	ret = sensor_select(&pconf->sensor_cfg, &pconf->sensor_strategy,
		&state->sensor);
	if (ret != 0)
		return ret;

	sensor_width  = state->sensor.plane.capt.width;
	sensor_height = state->sensor.plane.capt.height;
	if (state->sensor.mode.output.width > 0 &&
	    state->sensor.mode.output.height > 0 &&
	    (state->sensor.mode.output.width  < sensor_width ||
	     state->sensor.mode.output.height < sensor_height)) {
		printf("> Note: MIPI frame %ux%u, usable output %ux%u (cropping overscan)\n",
			sensor_width, sensor_height,
			state->sensor.mode.output.width,
			state->sensor.mode.output.height);
		if (state->sensor.mode.output.width  < sensor_width)
			sensor_width  = state->sensor.mode.output.width;
		if (state->sensor.mode.output.height < sensor_height)
			sensor_height = state->sensor.mode.output.height;
	}

	pipeline_common_report_selected_fps("", pconf->sensor_framerate,
		&state->sensor);
	pconf->sensor_framerate = state->sensor.fps;
	pconf->venc_gop_size = pipeline_common_gop_frames(pconf->gop_sec,
		pconf->sensor_framerate);
	/* Auto resolution: 0x0 means use sensor native dimensions */
	if (pconf->image_width == 0 || pconf->image_height == 0) {
		pconf->image_width = sensor_width;
		pconf->image_height = sensor_height;
	}
	pipeline_common_clamp_image_size("", sensor_width, sensor_height,
		&pconf->image_width, &pconf->image_height);

	/* Precrop keeps the VIF→VPE window at the configured aspect ratio
	 * against the full sensor. */
	pconf->precrop = star6e_pipeline_compute_precrop(sensor_width,
		sensor_height, pconf->image_width, pconf->image_height,
		STAR6E_KEEP_ASPECT);

	state->image_width  = pconf->image_width;
	state->image_height = pconf->image_height;
	overscan_x = (uint16_t)(((state->sensor.plane.capt.width  - sensor_width)
		/ 2) & ~1u);
	overscan_y = (uint16_t)(((state->sensor.plane.capt.height - sensor_height)
		/ 2) & ~1u);
	pconf->precrop.x += overscan_x;
	pconf->precrop.y += overscan_y;

	printf("> Starting star6e pipeline\n");
	printf("  - Sensor: %ux%u @ %u\n", sensor_width, sensor_height,
		pconf->sensor_framerate);
	if (overscan_x || overscan_y) {
		printf("  - MIPI  : %ux%u, cropped to %ux%u (offset %u,%u)\n",
			state->sensor.plane.capt.width,
			state->sensor.plane.capt.height,
			sensor_width, sensor_height, overscan_x, overscan_y);
	}
	printf("  - Image : %ux%u\n", pconf->image_width, pconf->image_height);
	if (pconf->precrop.w != sensor_width ||
	    pconf->precrop.h != sensor_height) {
		printf("  - Precrop: %ux%u -> %ux%u (VIF offset %u,%u)\n",
			sensor_width, sensor_height,
			pconf->precrop.w, pconf->precrop.h,
			pconf->precrop.x, pconf->precrop.y);
	}
	if (pconf->image_width  != pconf->precrop.w ||
	    pconf->image_height != pconf->precrop.h) {
		printf("  - VPE scaling: %ux%u -> %ux%u\n",
			pconf->precrop.w, pconf->precrop.h,
			pconf->image_width, pconf->image_height);
	}
	printf("  - 3DNR  : level %d\n", STAR6E_VPE_LEVEL_3DNR);

	return 0;
}

/* Tracks whether CUS3A has been enabled in this MI_SYS lifetime.  Cleared
 * by star6e_pipeline_stop(), which is always followed by MI_SYS_Exit in
 * the runtime teardown — so the next process start runs a true cold sequence
 * including CUS3A enable. */
static int g_isp_initialized = 0;

/* Tracks the last-loaded ISP bin path within this MI_SYS lifetime so we
 * skip redundant reloads.  Cleared by star6e_pipeline_stop() since the
 * following MI_SYS_Exit releases ISP driver state and the next start
 * needs to reload the bin against the fresh kernel. */
static char g_last_isp_bin_path[256] = {0};

/* Phase 3: assign port structs, issue all MI_SYS bind calls, init output,
 * video, ISP bin, exposure cap, cus3a and clocks.
 * pconf is non-const because we resolve isp_bin_path in here (we need
 * the live sensor name from state->sensor, which is only populated
 * after Phase 2). */
static int bind_and_finalize_pipeline(Star6ePipelineState *state,
	const VencCfg *cfg, Star6ePipelineConfig *pconf,
	SdkQuietState *sdk_quiet)
{
	MI_U32 venc_device = 0;
	uint32_t bind_src_fps;
	uint32_t bind_dst_fps;
	int ret;

	state->vif_port = (MI_SYS_ChnPort_t){
		.module = I6_SYS_MOD_VIF, .device = 0, .channel = 0, .port = 0 };
	state->vpe_port = (MI_SYS_ChnPort_t){
		.module = I6_SYS_MOD_VPE, .device = 0, .channel = 0, .port = 0 };

	if (MI_VENC_GetChnDevid(state->venc_channel, &venc_device) != 0) {
		fprintf(stderr, "ERROR: MI_VENC_GetChnDevid failed\n");
		return -1;
	}
	state->venc_port = (MI_SYS_ChnPort_t){
		.module  = I6_SYS_MOD_VENC, .device  = venc_device,
		.channel = state->venc_channel, .port = 0 };

	if (!state->bound_vif_vpe) {
		ret = MI_SYS_BindChnPort2(&state->vif_port, &state->vpe_port,
			pconf->sensor_framerate, pconf->sensor_framerate,
			I6_SYS_LINK_REALTIME, 0);
		if (ret != 0) {
			fprintf(stderr, "ERROR: MI_SYS_Bind VIF->VPE failed %d\n", ret);
			return ret;
		}
		state->bound_vif_vpe = 1;

		/* A new VPE channel was just created. The ISP channel
		 * initialises asynchronously after MI_VPE_CreateChannel.  Poll
		 * here before the bin load and cap_exposure_for_fps touch the
		 * ISP, so the kernel ISP driver does not emit "IspApiGet
		 * channel not created" errors. */
		star6e_pipeline_wait_isp_channel();
	}

	/* Cap exposure BEFORE binding VPE→VENC.  The AE starts running as
	 * soon as VIF→VPE is bound (above).  Without an early cap the AE
	 * can converge on a shutter time longer than the frame period during
	 * the ISP bin load + CUS3A init window, locking the pipeline at a
	 * lower framerate until reinit. */
	star6e_pipeline_cap_exposure_for_fps(pconf->sensor_framerate,
		STAR6E_SHUTTER_RULE_180);

	bind_src_fps = state->sensor.mode.maxFps ?
		state->sensor.mode.maxFps : pconf->sensor_framerate;
	bind_dst_fps = cfg->fps;
	if (bind_dst_fps == 0 || bind_dst_fps > bind_src_fps)
		bind_dst_fps = bind_src_fps;
	/* Deliver the TRUE sensor rate to VENC (e.g. ~143 for the 144 mode) so
	 * the encoder actually outputs it.  The RC fpsNum is separately capped
	 * to STAR6E_VENC_INPUT_FPS_MAX — see venc_fps in pipeline_start. */

	ret = MI_SYS_BindChnPort2(&state->vpe_port, &state->venc_port,
		bind_src_fps, bind_dst_fps, I6_SYS_LINK_FRAMEBASE, 0);
	if (ret != 0) {
		fprintf(stderr, "ERROR: MI_SYS_Bind VPE->VENC failed %d\n", ret);
		MI_SYS_UnBindChnPort(&state->vif_port, &state->vpe_port);
		state->bound_vif_vpe = 0;
		return ret;
	}
	state->bound_vpe_venc = 1;
	MI_SYS_SetChnOutputPortDepth(&state->venc_port, 1, 3);

	/* Bring up the JPEG snapshot subsystem on the same VPE source port the
	 * main channel just bound to.  Failure is non-fatal — a snapshot
	 * request just fails if init fails.  Dimensions inherit the main
	 * stream; venc.snapshot_quality == 0 disables the subsystem. */
	{
		venc_jpeg_set_source(&state->vpe_port);
		VencJpegConfig jcfg = {
			.width   = state->image_width,
			.height  = state->image_height,
			.quality = cfg->snapshot_quality,
			.channel = STAR6E_SNAPSHOT_CHANNEL,
			.enabled = cfg->snapshot_quality > 0,
		};
		/* init() is always called (it sets internal state even when
		 * disabled). */
		(void)venc_jpeg_init(&jcfg);
	}

	if (star6e_output_init(&state->output, VENC_RING_NAME) != 0) {
		star6e_output_teardown(&state->output);
		MI_SYS_UnBindChnPort(&state->vpe_port, &state->venc_port);
		state->bound_vpe_venc = 0;
		MI_SYS_UnBindChnPort(&state->vif_port, &state->vpe_port);
		state->bound_vif_vpe = 0;
		return -1;
	}

	star6e_video_init(&state->video, pconf->sensor_framerate);

	/* Resolve venc.sensor_bin: configured path takes precedence; if empty
	 * or unreadable, fall back to /etc/sensors/<sensor>.bin keyed off
	 * the live sensor name. */
	pipeline_common_resolve_isp_bin(
		cfg->sensor_bin[0] ? cfg->sensor_bin : NULL,
		state->sensor.plane.sensName,
		pconf->isp_bin_path, sizeof(pconf->isp_bin_path));

	/* Load ISP bin on first start, or when the bin path changes.  Skipping
	 * redundant reloads avoids the vendor AE resetting the sensor shutter
	 * register back to its default (~10000us on IMX335), which would
	 * otherwise lock the sensor VTS at ~100 fps.  The kernel ISP driver
	 * accepts repeated loads but each one disturbs the running sensor
	 * timing. */
	if (pconf->isp_bin_path[0] &&
	    strcmp(pconf->isp_bin_path, g_last_isp_bin_path) != 0) {
		ret = star6e_pipeline_load_isp_bin(pconf->isp_bin_path, sdk_quiet);
		if (ret != 0) {
			fprintf(stderr, "WARNING: ISP bin load failed; continuing with default ISP settings\n");
		} else {
			snprintf(g_last_isp_bin_path, sizeof(g_last_isp_bin_path),
				"%s", pconf->isp_bin_path);
		}
	}
	if (!g_isp_initialized) {
		star6e_pipeline_enable_cus3a(sdk_quiet);
		g_isp_initialized = 1;
	}
	/* Userspace AWB: the ISP-internal algorithm does not converge here
	 * (see star6e_awb.h). */
	if (cfg->awb_fps > 0 && star6e_awb_start(cfg->awb_fps) == 0) {
		star6e_awb_set_paused(0);
		star6e_pipeline_set_awb_userspace(1);
	}
	/* Reapply exposure cap after ISP bin load — the bin may reset AE
	 * limits to its own defaults which could exceed the frame period. */
	star6e_pipeline_cap_exposure_for_fps(pconf->sensor_framerate,
		STAR6E_SHUTTER_RULE_180);

	/* Cold-boot fix: the ISP bin's AE may initialize the sensor at a shutter
	 * exceeding the frame period.  SetExposureLimit only constrains the AE
	 * algorithm, not the physical sensor register.  MI_SNR_SetFps forces the
	 * sensor driver to reconfigure timing, resetting the shutter to fit the
	 * frame period.  The supervisory AE enforcer only touches the exposure
	 * limit, so the pipeline owns this kick. */
	if (pconf->exposure_cap_us > 0 &&
	    pconf->sensor_framerate > 0) {
		MI_SNR_SetFps(state->sensor.pad_id, pconf->sensor_framerate);
	}

	star6e_pipeline_set_hw_clocks(STAR6E_OVERCLOCK_LEVEL);

	return 0;
}

/* Drain buffered frames from a VENC channel (non-blocking).
 * Relieves VPE backpressure before StopRecvPic to prevent D-state hangs.
 * Drains until no frames remain or max_ms elapsed — time-bounded to
 * handle continuous 120fps production during teardown. */
static void drain_venc_channel(MI_VENC_CHN chn, int max_ms,
	const char *label)
{
	struct timespec start;
	int drained = 0;

	clock_gettime(CLOCK_MONOTONIC, &start);

	for (;;) {
		MI_VENC_Stat_t stat = {0};
		MI_VENC_Stream_t stream = {0};
		struct timespec now;
		long long elapsed_ms;

		if (MI_VENC_Query(chn, &stat) != 0 || stat.curPacks == 0)
			break;

		stream.count = stat.curPacks;
		stream.packet = calloc(stat.curPacks, sizeof(MI_VENC_Pack_t));
		if (!stream.packet)
			break;

		if (MI_VENC_GetStream(chn, &stream, 0) != 0) {
			free(stream.packet);
			break;
		}

		MI_VENC_ReleaseStream(chn, &stream);
		free(stream.packet);
		drained++;

		clock_gettime(CLOCK_MONOTONIC, &now);
		elapsed_ms = (long long)(now.tv_sec - start.tv_sec) * 1000LL +
			(long long)(now.tv_nsec - start.tv_nsec) / 1000000LL;
		if (elapsed_ms >= max_ms)
			break;
	}

	if (drained > 0)
		printf("> Drained %d frames from VENC %s before stop\n",
			drained, label);
}

void star6e_pipeline_stop(Star6ePipelineState *state)
{
	if (!state)
		return;

	star6e_awb_stop();

	/* Clear userspace persist flags.  The runtime teardown follows with
	 * MI_SYS_Exit, so the kernel ISP/CUS3A state is genuinely cold on the
	 * next start.  Skipping these clears would leave stale "already
	 * initialised" flags that bypass the very work the fresh kernel state
	 * expects us to redo. */
	g_isp_initialized = 0;
	g_last_isp_bin_path[0] = '\0';
	g_cus3a_handoff_done = 0;

	/* Clear the IntraRefresh status snapshot — the channel it described is
	 * about to be destroyed. */
	pthread_mutex_lock(&g_intra_status_mutex);
	memset(&g_intra_status, 0, sizeof(g_intra_status));
	pthread_mutex_unlock(&g_intra_status_mutex);

	star6e_output_teardown(&state->output);

	/* Tear down the JPEG snapshot channel first — it's bound to the same
	 * VPE port we're about to unbind, and its UnBindChnPort/DestroyChn
	 * must run while the SDK still holds a consistent view of the VPE
	 * source.  Idempotent; safe even if init was skipped or failed. */
	venc_jpeg_shutdown();

	/* MI teardown order: StopRecvPic the VENC consumer BEFORE unbinding
	 * its input port.  The previous Star6E order unbound VPE→VENC first and
	 * only then stopped VENC, leaving the kernel SDK still encoding/flushing
	 * a buffered frame out of a port userspace had just ripped out — VENC
	 * (MMU client 0x15) then reads a freed VPE buffer:
	 * `_MI_SYS_MMU_Callback Status=0x2 IsWrite=0` storms into a hardware
	 * watchdog reset on the ~2nd rapid respawn.  StopRecvPic is a soft pause
	 * and does not deadlock while still bound.  Sequence: StopRecvPic →
	 * drain output → unbind VPE→VENC → unbind VIF→VPE → destroy VENC →
	 * stop VPE/VIF/sensor. */
	MI_VENC_StopRecvPic(state->venc_channel);

	/* Drain the last buffered frames that StopRecvPic let flow out. */
	drain_venc_channel(state->venc_channel, 150, "ch0");

	/* Unbind VPE→VENC now that the consumer is stopped. */
	if (state->bound_vpe_venc) {
		MI_SYS_UnBindChnPort(&state->vpe_port, &state->venc_port);
		state->bound_vpe_venc = 0;
	}

	if (state->bound_vif_vpe) {
		MI_SYS_UnBindChnPort(&state->vif_port, &state->vpe_port);
		state->bound_vif_vpe = 0;
	}

	MI_VENC_DestroyChn(state->venc_channel);
	free(state->stream_packs);
	state->stream_packs = NULL;
	state->stream_packs_cap = 0;
	star6e_pipeline_stop_vpe();
	star6e_pipeline_stop_vif();
	star6e_pipeline_stop_sensor(state->sensor.pad_id);
}

/* flatten: force GCC to inline all static callees into this function.
 * The SigmaStar I6E ISP driver depends on the monolithic stack layout
 * that results from inlining bind_and_finalize_pipeline() and
 * prepare_pipeline_config().  When these are emitted as separate functions
 * (as happens with -Os when they have multiple call-sites), the VPE→ISP
 * channel init fails (MI_ISP_IQ_GetParaInitStatus returns error 6). */
__attribute__((flatten))
int star6e_pipeline_start(Star6ePipelineState *state, const VencCfg *cfg,
	SdkQuietState *sdk_quiet)
{
	Star6ePipelineConfig pconf;
	uint32_t venc_fps;
	int ret;

	if (!state || !cfg)
		return -1;

	star6e_pipeline_reset(state);

	if (prepare_pipeline_config(cfg, &pconf) != 0)
		return -1;

	ret = select_and_configure_sensor(state, &pconf, sdk_quiet);
	if (ret != 0)
		return ret;

	state->active_precrop = pconf.precrop;

	ret = star6e_pipeline_start_vif(&state->sensor, &pconf.precrop);
	if (ret != 0)
		goto fail_sensor;

	ret = star6e_pipeline_start_vpe(&state->sensor, &pconf.precrop,
		pconf.image_width, pconf.image_height,
		STAR6E_IMAGE_MIRROR, STAR6E_IMAGE_FLIP,
		STAR6E_VPE_LEVEL_3DNR, sdk_quiet);
	if (ret != 0)
		goto fail_vif;

	state->image_width = pconf.image_width;
	state->image_height = pconf.image_height;

	state->venc_channel = 0;
	/* Encoder rate control must budget for the frame rate that actually
	 * reaches VENC — the VPE->VENC framebase bind's delivered rate =
	 * min(mode.maxFps, venc.fps), then capped to the VENC input ceiling.
	 * Do NOT clamp to pconf.sensor_framerate: a SetFps fallback can record
	 * state->sensor.fps well below the mode's real (fixed-timing) rate.  And
	 * do NOT let it exceed STAR6E_VENC_INPUT_FPS_MAX: the VENC rejects >120
	 * and resets to 30, so the 1600x900@144 mode must encode at 120 (the bind
	 * drops the sensor's ~143 down to match).  Both mismatches otherwise
	 * make CBR overshoot the configured bitrate several-fold. */
	{
		uint32_t delivered = state->sensor.mode.maxFps ?
			state->sensor.mode.maxFps : pconf.sensor_framerate;
		uint32_t delivered_true = delivered;
		if (delivered > STAR6E_VENC_INPUT_FPS_MAX)
			delivered = STAR6E_VENC_INPUT_FPS_MAX;
		venc_fps = cfg->fps;
		if (venc_fps == 0 || venc_fps > delivered)
			venc_fps = delivered;
		/* Exact-CBR: the bind delivers delivered_true frames while the
		 * RC budgets at venc_fps (capped 120) — scale the encoder budget
		 * by venc_fps/delivered_true so the wire rate lands on the
		 * configured bitrate (x120/144 for the 144 mode).  Mirrors
		 * rc_compensate_kbps() in star6e_controls.c for the live paths. */
		if (cfg->fps && cfg->fps < delivered_true)
			delivered_true = cfg->fps;
		if (delivered_true > venc_fps && venc_fps > 0)
			pconf.venc_max_rate = (uint32_t)((uint64_t)
				pconf.venc_max_rate * venc_fps / delivered_true);
	}

	/* GOP frame count must track the encoder's real fps (venc_fps), not
	 * pconf.sensor_framerate which may hold the stale SetFps-fallback value;
	 * otherwise the I-frame interval desyncs on modes whose delivered rate
	 * exceeds the recorded sensor.fps (e.g. GOP=60 instead of 288 @144). */
	pconf.venc_gop_size = pipeline_common_gop_frames(pconf.gop_sec, venc_fps);

	/* IntraRefresh auto-GOP: when the preset's intra mode != off and it did
	 * not pin a GOP, override the GOP frame count so each IDR aligns with
	 * one full GDR pass. */
	{
		IntraRefreshDerived ir;
		IntraRefreshMode mode = star6e_pipeline_intra_refresh_derive(
			&pconf.preset, pconf.gop_sec, pconf.image_height,
			venc_fps, &ir);
		if (mode != INTRA_MODE_OFF && !ir.gop_overridden && ir.gop_frames > 0)
			pconf.venc_gop_size = ir.gop_frames;
	}

	ret = star6e_pipeline_start_venc(pconf.image_width, pconf.image_height,
		pconf.venc_max_rate, venc_fps, pconf.venc_gop_size,
		&pconf.preset, &state->venc_channel);
	if (ret != 0)
		goto fail_vpe;

	/* IntraRefresh — driven by the resilience preset.  Failure is logged
	 * but not fatal: stream still works without rolling refresh. */
	(void)star6e_pipeline_apply_intra_refresh(state->venc_channel,
		&pconf.preset, pconf.gop_sec, pconf.image_height, venc_fps);

	/* SVC-T reference pyramid (refPred) is applied inside
	 * star6e_pipeline_start_venc() before StartRecvPic — the SDK
	 * requires that ordering or the call silently no-ops. */

	ret = bind_and_finalize_pipeline(state, cfg, &pconf, sdk_quiet);
	if (ret != 0)
		goto fail_venc;

	/* Must run AFTER bind_and_finalize_pipeline(): it calls
	 * star6e_output_init() -> star6e_output_reset(), which memsets the
	 * whole output struct and would otherwise zero these GDR/SVC-T
	 * tagging fields. */
	{
		Star6eIntraRefreshStatus ir_status;
		Star6eRefPredStatus ref_status;
		uint32_t clen;

		star6e_pipeline_intra_refresh_status(&ir_status);
		star6e_pipeline_ref_pred_status(&ref_status);
		state->output.gdr_active = ir_status.active && ir_status.apply_ok;
		state->output.svct_active = ref_status.active && ref_status.apply_ok;
		clen = (state->output.gdr_active && ir_status.total_rows &&
			ir_status.effective_lines_per_p)
			? (ir_status.total_rows + ir_status.effective_lines_per_p - 1) /
				ir_status.effective_lines_per_p
			: 0;
		state->output.gdr_cycle_len = clen > 255 ? 255 : (uint8_t)clen;
		state->output.gdr_counter = 0;
	}

	return 0;

fail_venc:
	star6e_pipeline_stop_venc(state->venc_channel);
fail_vpe:
	star6e_pipeline_stop_vpe();
fail_vif:
	star6e_pipeline_stop_vif();
fail_sensor:
	star6e_pipeline_stop_sensor(state->sensor.pad_id);
	return ret ? ret : -1;
}
