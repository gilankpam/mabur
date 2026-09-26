/* venc_record.c -- the VTX onboard recorder's VENC channel (spec
 * docs/superpowers/specs/2026-09-26-vtx-recorder-design.md).
 *
 * One H.265 CBR channel (STAR6E_RECORD_CHANNEL) bound FRAMEBASE to the VPE
 * port the link channel taps, created and bound BEFORE the link channel so
 * the link is the newest peer and its encode runs first (bench R1 vs R2-R4,
 * docs/sd-record-findings-2026-09-26.md).
 *
 * Idle (no StartRecvPic) until venc_record_start(). A drain thread waits on
 * the channel fd (poll, 20 ms cap -- correctness never depends on the fd
 * firing: every wake Queries), concatenates a frame's packs into one Annex-B
 * AU and hands it to the sink. Record-channel MI calls take g_mx, never
 * venc_core's g_verb_lock: the link's verbs and the recorder never wait on
 * each other.
 */
#include "venc_record.h"
#include "venc_core.h"
#include "star6e_pipeline.h"   /* STAR6E_VENC_INPUT_FPS_MAX */

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define REC_MAX_PACKS 8
#define REC_STOP_EMPTY_WAKES 3   /* after StopRecvPic: this many empty wakes = drained */

static VencRecordConfig g_cfg;
static VencRecordSink g_sink;
static void *g_sink_user;

static pthread_mutex_t g_mx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_cv = PTHREAD_COND_INITIALIZER;
static int g_attached, g_bound, g_created;
static int g_active, g_stopping, g_empty, g_quit;
static int g_in_drain;   /* drain thread is inside drain_one (may call the sink) */
static int g_want;   /* start() called and not stopped: survives a pipeline re-attach */
static MI_SYS_ChnPort_t g_src, g_port;
static pthread_t g_thr;
static int g_thr_started;
static uint8_t *g_buf;
static size_t g_buf_cap;

void venc_record_configure(const VencRecordConfig *cfg, VencRecordSink sink, void *user)
{
	if (cfg)
		g_cfg = *cfg;
	g_sink = sink;
	g_sink_user = user;
}

/* 1 = delivered an AU, 0 = nothing ready, -1 = out of memory. */
static int drain_one(int fd)
{
	MI_VENC_Pack_t packs[REC_MAX_PACKS];
	MI_VENC_Stream_t stream;
	MI_VENC_Stat_t stat;
	size_t cap = 0, len = 0;
	uint32_t pts;
	int key = 0;
	unsigned int i;

	if (fd >= 0) {
		struct pollfd p = { .fd = fd, .events = POLLIN };
		(void)poll(&p, 1, 20);
	} else {
		struct timespec ts = { 0, 2 * 1000 * 1000 };
		nanosleep(&ts, NULL);
	}
	memset(&stat, 0, sizeof(stat));
	if (MI_VENC_Query(STAR6E_RECORD_CHANNEL, &stat) != 0 || stat.curPacks == 0)
		return 0;
	memset(&stream, 0, sizeof(stream));
	memset(packs, 0, sizeof(packs));
	stream.count = stat.curPacks > REC_MAX_PACKS ? REC_MAX_PACKS : stat.curPacks;
	stream.packet = packs;
	if (MI_VENC_GetStream(STAR6E_RECORD_CHANNEL, &stream, 40) != 0)
		return 0;

	for (i = 0; i < stream.count; ++i)
		cap += packs[i].length;
	if (cap > g_buf_cap) {
		uint8_t *nb = realloc(g_buf, cap);
		if (!nb) {
			MI_VENC_ReleaseStream(STAR6E_RECORD_CHANNEL, &stream);
			return -1;
		}
		g_buf = nb;
		g_buf_cap = cap;
	}
	for (i = 0; i < stream.count; ++i) {
		const MI_VENC_Pack_t *pk = &packs[i];
		if (!pk->data)
			continue;
		if (pk->packNum > 0) {
			unsigned int k;
			for (k = 0; k < pk->packNum && k < 8; ++k) {
				unsigned int off = pk->packetInfo[k].offset;
				unsigned int n = pk->packetInfo[k].length;
				uint8_t nt = (uint8_t)pk->packetInfo[k].packType.h265Nalu;
				if (n == 0 || off >= pk->length || n > pk->length - off)
					continue;
				if (n > cap - len)   /* overlapping packetInfo: never overrun g_buf */
					continue;
				memcpy(g_buf + len, pk->data + off, n);
				len += n;
				if (nt == 19 || nt == 20)
					key = 1;
			}
		} else if (pk->length > pk->offset &&
		           pk->length - pk->offset <= cap - len) {
			memcpy(g_buf + len, pk->data + pk->offset, pk->length - pk->offset);
			len += pk->length - pk->offset;
		}
	}
	pts = (uint32_t)packs[0].timestamp;
	MI_VENC_ReleaseStream(STAR6E_RECORD_CHANNEL, &stream);
	if (len && g_sink)
		g_sink(g_sink_user, g_buf, len, pts, key);
	return 1;
}

static void *drain_fn(void *arg)
{
	int fd = MI_VENC_GetFd(STAR6E_RECORD_CHANNEL);
	(void)arg;
	pthread_mutex_lock(&g_mx);
	while (!g_quit) {
		if (!g_active) {
			pthread_cond_broadcast(&g_cv);   /* wake a stop() waiting for the park */
			while (!g_active && !g_quit)
				pthread_cond_wait(&g_cv, &g_mx);
			continue;
		}
		g_in_drain = 1;
		pthread_mutex_unlock(&g_mx);
		int got = drain_one(fd);
		pthread_mutex_lock(&g_mx);
		g_in_drain = 0;
		pthread_cond_broadcast(&g_cv);   /* a timed-out stop() waits for this */
		if (got <= 0 && g_stopping && ++g_empty >= REC_STOP_EMPTY_WAKES) {
			g_active = 0;       /* everything in flight reached the sink: park */
			g_stopping = 0;
		} else if (got > 0) {
			g_empty = 0;
		}
	}
	pthread_cond_broadcast(&g_cv);
	pthread_mutex_unlock(&g_mx);
	if (fd >= 0)
		MI_VENC_CloseFd(STAR6E_RECORD_CHANNEL);
	return NULL;
}

static int start_locked(void)
{
	if (MI_VENC_StartRecvPic(STAR6E_RECORD_CHANNEL) != 0) {
		fprintf(stderr, "[record] StartRecvPic failed\n");
		return -1;
	}
	(void)MI_VENC_RequestIdr(STAR6E_RECORD_CHANNEL, 1);
	g_active = 1;
	g_stopping = 0;
	g_empty = 0;
	pthread_cond_broadcast(&g_cv);
	return 0;
}

int venc_record_attach(const MI_SYS_ChnPort_t *vpe_port, uint32_t width,
	uint32_t height, uint32_t src_fps)
{
	MI_VENC_ChnAttr_t attr;
	MI_U32 dev = 0;
	uint32_t fps;
	MI_S32 ret;

	if (!g_cfg.enabled || !vpe_port || g_attached)
		return 0;
	fps = g_cfg.fps;
	if (src_fps && fps > src_fps)
		fps = src_fps;
	if (fps > STAR6E_VENC_INPUT_FPS_MAX)
		fps = STAR6E_VENC_INPUT_FPS_MAX;
	if (fps == 0)
		fps = 30;

	memset(&attr, 0, sizeof(attr));
	attr.attrib.codec = I6_VENC_CODEC_H265;
	attr.attrib.h265.maxWidth = width;
	attr.attrib.h265.maxHeight = height;
	attr.attrib.h265.bufSize = width * height * 3 / 2;
	attr.attrib.h265.profile = 0;
	attr.attrib.h265.byFrame = 1;
	attr.attrib.h265.width = width;
	attr.attrib.h265.height = height;
	attr.attrib.h265.bFrameNum = 0;
	attr.attrib.h265.refNum = 1;
	attr.rate.mode = I6_VENC_RATEMODE_H265CBR;
	attr.rate.h265Cbr = (i6_venc_rate_h26xcbr){
		.gop = fps, .statTime = 1,
		.fpsNum = fps, .fpsDen = 1,
		.bitrate = g_cfg.bitrate_kbps * 1024, .avgLvl = 1,
	};

	ret = MI_VENC_CreateChn(STAR6E_RECORD_CHANNEL, &attr);
	if (ret != 0) {
		fprintf(stderr, "[record] CreateChn(%d) failed %d -- recorder disabled\n",
			STAR6E_RECORD_CHANNEL, ret);
		return -1;
	}
	g_created = 1;
	if (MI_VENC_GetChnDevid(STAR6E_RECORD_CHANNEL, &dev) != 0) {
		fprintf(stderr, "[record] GetChnDevid failed -- recorder disabled\n");
		goto fail;
	}
	g_src = *vpe_port;
	g_port = (MI_SYS_ChnPort_t){ .module = I6_SYS_MOD_VENC, .device = dev,
		.channel = STAR6E_RECORD_CHANNEL, .port = 0 };
	ret = MI_SYS_BindChnPort2(&g_src, &g_port, src_fps ? src_fps : fps, fps,
		I6_SYS_LINK_FRAMEBASE, 0);
	if (ret != 0) {
		fprintf(stderr, "[record] Bind VPE->ch%d failed %d -- recorder disabled\n",
			STAR6E_RECORD_CHANNEL, ret);
		goto fail;
	}
	g_bound = 1;
	/* Deep output queue: an SD stall backs up here, never into VPE. */
	MI_SYS_SetChnOutputPortDepth(&g_port, 8, 56);

	g_quit = 0;
	g_active = 0;
	if (pthread_create(&g_thr, NULL, drain_fn, NULL) != 0) {
		fprintf(stderr, "[record] pthread_create failed -- recorder disabled\n");
		goto fail;
	}
	g_thr_started = 1;
	(void)pthread_setname_np(g_thr, "mbr-recdrain");  /* top/perf, next to mbr-rec */
	g_attached = 1;
	fprintf(stderr, "[record] ch%d %ux%u %u fps CBR %u kbps bound before the link (idle)\n",
		STAR6E_RECORD_CHANNEL, width, height, fps, g_cfg.bitrate_kbps);
	/* A pipeline re-attach while the operator wanted a recording: resume. */
	pthread_mutex_lock(&g_mx);
	if (g_want)
		(void)start_locked();
	pthread_mutex_unlock(&g_mx);
	return 0;
fail:
	venc_record_detach();
	return -1;
}

int venc_record_start(void)
{
	int rc = -1;
	pthread_mutex_lock(&g_mx);
	if (g_attached)
		rc = g_active ? 0 : start_locked();
	if (rc == 0)
		g_want = 1;
	pthread_mutex_unlock(&g_mx);
	return rc;
}

void venc_record_stop(void)
{
	struct timespec dl;
	pthread_mutex_lock(&g_mx);
	g_want = 0;
	if (!g_attached || !g_active) {
		pthread_mutex_unlock(&g_mx);
		return;
	}
	(void)MI_VENC_StopRecvPic(STAR6E_RECORD_CHANNEL);
	g_stopping = 1;
	g_empty = 0;
	clock_gettime(CLOCK_REALTIME, &dl);
	dl.tv_nsec += 500L * 1000 * 1000;
	if (dl.tv_nsec >= 1000000000L) {
		dl.tv_sec += 1;
		dl.tv_nsec -= 1000000000L;
	}
	while (g_active && !g_quit) {
		if (pthread_cond_timedwait(&g_cv, &g_mx, &dl) == ETIMEDOUT) {
			fprintf(stderr, "[record] stop: drain did not settle in 500 ms\n");
			g_active = 0;
			g_stopping = 0;
			break;
		}
	}
	/* RecordChannel::stop() is synchronous: no sink call after it returns.
	 * On the timeout path the thread may still be inside drain_one (bounded:
	 * 20 ms poll + 40 ms GetStream + the sink's copy) -- wait it out. */
	while (g_in_drain)
		pthread_cond_wait(&g_cv, &g_mx);
	pthread_mutex_unlock(&g_mx);
}

void venc_record_request_idr(void)
{
	pthread_mutex_lock(&g_mx);
	if (g_attached && g_active)
		(void)MI_VENC_RequestIdr(STAR6E_RECORD_CHANNEL, 1);
	pthread_mutex_unlock(&g_mx);
}

void venc_record_detach(void)
{
	const int want = g_want;
	venc_record_stop();
	g_want = want;   /* a detach is not the operator's stop: re-attach resumes */
	pthread_mutex_lock(&g_mx);
	g_quit = 1;
	pthread_cond_broadcast(&g_cv);
	pthread_mutex_unlock(&g_mx);
	if (g_thr_started) {
		pthread_join(g_thr, NULL);
		g_thr_started = 0;
	}
	if (g_bound) {
		MI_SYS_UnBindChnPort(&g_src, &g_port);
		g_bound = 0;
	}
	if (g_created) {
		MI_VENC_DestroyChn(STAR6E_RECORD_CHANNEL);
		g_created = 0;
	}
	g_attached = 0;
	free(g_buf);
	g_buf = NULL;
	g_buf_cap = 0;
}
