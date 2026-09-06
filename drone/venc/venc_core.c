/* venc_core.c — mabur-local facade over the ported waybeam_venc star6e
 * encoder core.  No 1:1 waybeam counterpart: upstream's equivalent is
 * main.c + backend_execute(), which also owned the httpd, the mDNS beacon,
 * the config file and the fork+exec respawn.  All of that is deleted; what
 * is left is the bring-up ORDER (ported faithfully) behind four verbs and
 * two signals.
 *
 * Threading contract:
 *   - venc_core_start / venc_core_stop are called by maburd's main thread
 *     and must not overlap each other or the verbs.
 *   - The verbs and venc_get_stats / venc_snapshot_jpeg are callable from
 *     the agent thread while the encoder runs.
 *   - on_chain_break / on_fault fire on the encoder thread.  Consumers
 *     marshal (RcAgent sets an atomic and acts on its own tick).
 */
#include "venc_core.h"

#include "star6e_controls.h"
#include "star6e_output.h"
#include "star6e_runtime.h"
#include "venc_core_internal.h"
#include "venc_frame_ring.h"
#include "venc_jpeg.h"

#include <dirent.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── State ────────────────────────────────────────────────────────────── */

static Star6eRunnerContext g_ctx;
static VencCallbacks g_cb;          /* written before any thread exists */
static pthread_t g_enc_thread;
static int g_enc_thread_started;

/* Set with release AFTER the pipeline is up, cleared with release BEFORE
 * teardown starts.  Every verb loads it with acquire and bails when clear:
 * the MI entry points are raw dlopen'd function pointers (star6e.h's
 * MI_VENC_* macros do not null-check), so a verb arriving before start or
 * after stop would jump through a NULL vtable slot. */
static int g_started;

/* Requested rate of the last successful venc_set_bitrate_kbps (-1 before
 * the first).  Not what the encoder was programmed with: apply_bitrate
 * clamps to the VENC_BITRATE_* rails and compensates >120 fps modes. */
static int g_cur_bitrate_kbps = -1;

/* Serialises the read-modify-write verbs.  apply_bitrate and apply_qp_delta
 * are each a GetChnAttr/GetRcParam → mutate → Set pair on one channel, so
 * two of them interleaving would lose one of the two edits. */
static pthread_mutex_t g_verb_lock = PTHREAD_MUTEX_INITIALIZER;

static int venc_core_running(void)
{
	return __atomic_load_n(&g_started, __ATOMIC_ACQUIRE);
}

/* ── Chain-break signal (venc_core_internal.h) ────────────────────────── */

/* Called on the encoder thread by star6e_output when a ring-full drop threw
 * away a REFERENCE frame.  Deliberately unpaced: this is a report of a
 * fact, and spec §4's 1 s chain-break holdoff is policy, which lives in
 * RcAgent (task B6).  A burst of drops therefore produces a burst of
 * callbacks; the consumer coalesces. */
void venc_core_signal_chain_break(void)
{
	if (g_cb.on_chain_break)
		g_cb.on_chain_break(g_cb.user);
}

/* ── Stale MI kernel worker guard ─────────────────────────────────────── */

/* PPid from /proc/PID/status, or -1 if it cannot be read (process gone,
 * or a /proc without the field).  Parsed line-wise rather than with a
 * scanf format over the whole file because the field order in
 * /proc/PID/status is not a stable ABI. */
static long venc_proc_ppid(long pid)
{
	char path[64];
	char line[128];
	FILE *f;
	long ppid = -1;

	snprintf(path, sizeof(path), "/proc/%ld/status", pid);
	f = fopen(path, "r");
	if (!f)
		return -1;
	while (fgets(line, sizeof(line), f)) {
		if (strncmp(line, "PPid:", 5) == 0) {
			ppid = strtol(line + 5, NULL, 10);
			break;
		}
	}
	fclose(f);
	return ppid;
}

/* ported from waybeam_venc f956a52:src/main.c (is_another_waybeam_running),
 * inverted to the case mabur actually cares about.
 *
 * Upstream scanned for a second USERSPACE waybeam and skipped kernel
 * threads.  maburd cannot be started twice by its wrapper, but it CAN be
 * SIGKILLed / OOM-killed / panic out of MI_SYS_Exit, and that leaves a
 * "[maburd]" MI_VENC kernel worker behind holding the VENC hardware.  The
 * next pipeline_start then fails deep in the SDK with an unhelpful error.
 * Kernel threads have an EMPTY /proc/PID/cmdline; that is the first
 * filter.  Nothing in userspace clears such a worker — say so and refuse to
 * boot rather than burn a flight discovering it.
 *
 * But an empty cmdline is NOT unique to kernel threads: a ZOMBIE or an
 * exiting process has one too, and the wrapper respawns maburd at 2 s, so
 * the overwhelmingly common "empty cmdline + comm == maburd" hit is our own
 * dying predecessor still being reaped — a condition that resolves itself
 * in milliseconds.  Refusing boot on that would turn every ordinary restart
 * into a "REBOOT the drone" instruction, which is worse than the bug.
 *
 * Discriminator: /proc/PID/status PPid.  Kernel threads are children of
 * kthreadd (PPid 2), or PPid 0 for the few spawned before it; every
 * userspace process — zombie included, since the parent has not reaped it
 * yet, which is precisely why it is still a zombie — has PPid > 2.  So
 * refuse boot only for PPid <= 2 and log-and-skip everything else.
 * (readlink /proc/PID/exe was considered and rejected: it fails with ENOENT
 * for BOTH a kernel thread and a zombie, so it cannot tell them apart.)
 *
 * The scan continues after a skip rather than stopping, so a real kernel
 * worker sitting behind a zombie in readdir order is still found. */
static int venc_stale_kernel_thread_check(void)
{
	DIR *proc = opendir("/proc");
	struct dirent *ent;
	int found = 0;

	if (!proc)
		return 0;  /* no /proc to consult — do not block boot */

	while ((ent = readdir(proc)) != NULL) {
		char path[64];
		char comm[32] = {0};
		FILE *f;
		long pid;
		int cmdline_empty;

		if (ent->d_name[0] < '0' || ent->d_name[0] > '9')
			continue;
		pid = strtol(ent->d_name, NULL, 10);
		if (pid <= 0)
			continue;

		snprintf(path, sizeof(path), "/proc/%ld/cmdline", pid);
		f = fopen(path, "r");
		if (!f)
			continue;
		cmdline_empty = (fgetc(f) == EOF);
		fclose(f);
		if (!cmdline_empty)
			continue;  /* a userspace process, not a kernel worker */

		snprintf(path, sizeof(path), "/proc/%ld/comm", pid);
		f = fopen(path, "r");
		if (!f)
			continue;
		if (fgets(comm, sizeof(comm), f)) {
			size_t len = strlen(comm);
			if (len > 0 && comm[len - 1] == '\n')
				comm[len - 1] = '\0';
			if (strcmp(comm, "maburd") == 0) {
				long ppid = venc_proc_ppid(pid);

				/* ppid < 0 = /proc/PID/status vanished
				 * mid-scan, i.e. the process exited while we
				 * looked at it: by definition not a stale
				 * kernel worker. */
				if (ppid >= 0 && ppid <= 2) {
					fprintf(stderr,
						"ERROR: stale [maburd] MI kernel"
						" worker (pid %ld, ppid %ld)"
						" still holds the encoder;"
						" no userspace action clears"
						" it — REBOOT the drone\n",
						pid, ppid);
					found = 1;
				} else {
					fprintf(stderr,
						"venc: ignoring pid %ld"
						" (comm maburd, empty cmdline,"
						" ppid %ld) — a zombie/exiting"
						" userspace predecessor, not a"
						" kernel worker\n",
						pid, ppid);
				}
			}
		}
		fclose(f);
		if (found)
			break;
	}

	closedir(proc);
	return found ? -1 : 0;
}

/* ── Encoder thread ───────────────────────────────────────────────────── */

/* Thin wrapper so the loop's exit reason becomes a signal.  The SCHED_FIFO
 * 50 / CPU0 affinity block is inside star6e_runtime_encoder_loop where
 * waybeam put it (it must run ON the encoder thread, and it carries the
 * VENC_RT_PRIO override) — it is not duplicated here as a pthread_attr. */
static void *venc_encoder_thread(void *arg)
{
	const char *fault = (const char *)star6e_runtime_encoder_loop(arg);

	if (!fault)
		return NULL;

	fprintf(stderr, "ERROR: venc encoder loop died: %s\n", fault);
	fflush(stderr);
	if (g_cb.on_fault)
		g_cb.on_fault(g_cb.user, fault);
	return NULL;
}

/* ── Lifecycle ────────────────────────────────────────────────────────── */

int venc_core_start(const VencCfg *cfg, const VencCallbacks *cb)
{
	int rc;

	if (!cfg)
		return -1;
	if (venc_core_running()) {
		fprintf(stderr, "ERROR: venc_core_start: already started\n");
		return -1;
	}

	/* Published before any thread that reads them exists; the
	 * pthread_create below is the barrier. */
	memset(&g_cb, 0, sizeof(g_cb));
	if (cb)
		g_cb = *cb;
	g_cur_bitrate_kbps = -1;

	if (venc_stale_kernel_thread_check() != 0)
		return -1;

	memset(&g_ctx, 0, sizeof(g_ctx));
	g_ctx.cfg = *cfg;

	if (star6e_runtime_prepare() != 0) {
		fprintf(stderr, "ERROR: venc runtime prepare failed\n");
		return -1;
	}

	/* One call, because the ported runtime already performs waybeam's
	 * whole prepare→init order in it: MI dlopen (star6e_mi_init) →
	 * MI_SYS_Init → star6e_pipeline_start, which is sensor select +
	 * unlock → VIF/VPE → VENC create at VENC_BOOT_BITRATE_KBPS →
	 * refPred/intra-refresh →
	 * JPEG chn7 → frame ring create (VENC_RING_NAME) → AWB thread; then
	 * apply_startup_controls → controls_bind → CUS3A/AE thread →
	 * qp_delta.  Splitting it here would only re-order the SDK calls
	 * away from the sequence that is device-proven. */
	rc = star6e_runtime_init(&g_ctx);
	if (rc != 0) {
		fprintf(stderr, "ERROR: venc runtime init failed (%d)\n", rc);
		star6e_runtime_teardown(&g_ctx);
		return -1;
	}

	/* ROI needs no boot-time setup: star6e_controls_apply_roi_qp()
	 * clears all PIPELINE_ROI_MAX_STEPS regions and re-derives every
	 * region it wants from the bound VencCfg (roi_enabled / roi_steps /
	 * roi_center) plus the bound frame geometry on EVERY call, so the
	 * first RcAgent call is self-sufficient.  A freshly created channel
	 * has no regions programmed, so nothing stale survives either. */

	__atomic_store_n(&g_started, 1, __ATOMIC_RELEASE);

	if (pthread_create(&g_enc_thread, NULL, venc_encoder_thread,
	    &g_ctx) != 0) {
		fprintf(stderr, "ERROR: venc encoder thread create failed\n");
		__atomic_store_n(&g_started, 0, __ATOMIC_RELEASE);
		star6e_runtime_request_stop();
		star6e_runtime_teardown(&g_ctx);
		return -1;
	}
	g_enc_thread_started = 1;
	pthread_setname_np(g_enc_thread, "venc-enc");

	printf("> venc_core: started (%ux%u@%u, ring %s)\n",
		(unsigned)cfg->width, (unsigned)cfg->height,
		(unsigned)cfg->fps, VENC_RING_NAME);
	fflush(stdout);
	return 0;
}

void venc_core_stop(void)
{
	if (!venc_core_running() && !g_enc_thread_started)
		return;

	/* Close the verb window first: teardown destroys the channel and
	 * dlcloses the MI libs out from under any late caller. */
	__atomic_store_n(&g_started, 0, __ATOMIC_RELEASE);

	star6e_runtime_request_stop();
	if (g_enc_thread_started) {
		/* waybeam's runner_teardown forked a child here that killed
		 * the parent after ~3 s and wrote 'b' to /proc/sysrq-trigger
		 * if it was stuck in D-state inside StopRecvPic.  NOT ported
		 * — fold-in ruling 2026-08-29: an autonomous mid-flight
		 * reboot takes the RC link with it, while a D-state-wedged
		 * encoder thread leaves maburd's radio and control threads
		 * alive (video lost, aircraft still commandable).  That is
		 * the failure we prefer, so this join is allowed to hang.
		 * The escalation, if it is ever wanted, belongs in maburd's
		 * own shutdown path, not in the encoder library. */
		pthread_join(g_enc_thread, NULL);
		g_enc_thread_started = 0;
	}

	star6e_runtime_teardown(&g_ctx);
	memset(&g_cb, 0, sizeof(g_cb));  /* drop the consumer's user ptr */
	g_cur_bitrate_kbps = -1;
}

/* ── Verbs ────────────────────────────────────────────────────────────── */

int venc_set_bitrate_kbps(int kbps)
{
	int rc;

	if (kbps <= 0 || !venc_core_running())
		return -1;

	pthread_mutex_lock(&g_verb_lock);
	rc = star6e_controls_apply_bitrate((uint32_t)kbps);
	if (rc == 0)
		__atomic_store_n(&g_cur_bitrate_kbps, kbps, __ATOMIC_RELAXED);
	pthread_mutex_unlock(&g_verb_lock);
	return rc == 0 ? 0 : -1;
}

int venc_set_roi_qp(int qp)
{
	int rc;

	if (!venc_core_running())
		return -1;

	pthread_mutex_lock(&g_verb_lock);
	rc = star6e_controls_apply_roi_qp(qp);
	pthread_mutex_unlock(&g_verb_lock);
	return rc == 0 ? 0 : -1;
}

int venc_request_idr(void)
{
	int rc;

	if (!venc_core_running())
		return -1;

	/* Under the verb lock too: apply_qp_delta ends in an IDR request, so
	 * without it a concurrent IDR could land between that Set and its
	 * request. */
	pthread_mutex_lock(&g_verb_lock);
	rc = star6e_controls_request_idr();
	pthread_mutex_unlock(&g_verb_lock);
	return rc == 0 ? 0 : -1;
}

int venc_set_qp_delta(int qp_delta)
{
	int rc;

	if (!venc_core_running())
		return -1;

	pthread_mutex_lock(&g_verb_lock);
	rc = star6e_controls_apply_qp_delta(qp_delta);
	pthread_mutex_unlock(&g_verb_lock);
	return rc == 0 ? 0 : -1;
}

int venc_set_max_ipprop(int prop)
{
	int rc;

	if (!venc_core_running())
		return -1;
	if (prop < 1 || prop > 100)
		return -1;

	pthread_mutex_lock(&g_verb_lock);
	rc = star6e_controls_set_max_ipprop((uint32_t)prop);
	pthread_mutex_unlock(&g_verb_lock);
	return rc == 0 ? 0 : -1;
}

int venc_set_min_iqp(int qp)
{
	int rc;

	if (!venc_core_running())
		return -1;
	if (qp < 1 || qp > 51)
		return -1;

	pthread_mutex_lock(&g_verb_lock);
	rc = star6e_controls_set_iqp_bound(1, (uint32_t)qp);
	pthread_mutex_unlock(&g_verb_lock);
	return rc == 0 ? 0 : -1;
}

int venc_set_max_iqp(int qp)
{
	int rc;

	if (!venc_core_running())
		return -1;
	if (qp < 1 || qp > 51)
		return -1;

	pthread_mutex_lock(&g_verb_lock);
	rc = star6e_controls_set_iqp_bound(0, (uint32_t)qp);
	pthread_mutex_unlock(&g_verb_lock);
	return rc == 0 ? 0 : -1;
}

int venc_set_superframe_p_pct(int pct)
{
	int rc;

	if (!venc_core_running())
		return -1;
	if (pct < 0 || pct > 1000 || (pct > 0 && pct < 100))
		return -1;

	pthread_mutex_lock(&g_verb_lock);
	rc = star6e_controls_set_superframe_p_pct((uint32_t)pct);
	pthread_mutex_unlock(&g_verb_lock);
	return rc == 0 ? 0 : -1;
}

/* ── Signals ──────────────────────────────────────────────────────────── */

uint64_t venc_cur_pts_us(void)
{
	uint64_t cur = 0;

	if (!venc_core_running())
		return 0;
	if (MI_SYS_GetCurPts(&cur) != 0)
		return 0;
	return cur;
}

void venc_get_stats(VencStats *out)
{
	venc_frame_ring_fill_t fill;

	if (!out)
		return;
	memset(out, 0, sizeof(*out));
	out->cur_bitrate_kbps = __atomic_load_n(&g_cur_bitrate_kbps,
		__ATOMIC_RELAXED);

	if (!venc_core_running())
		return;

	/* Producer-side counter, bumped on the encoder thread
	 * (star6e_video_send_frame). It counts encoded AUs INCLUDING ones
	 * the full ring then drops: the increment happens pre-publish, so
	 * frames_encoded is "what the encoder produced", not "what reached
	 * the ring". full_drops below is the difference. */
	out->frames_encoded = (uint32_t)__atomic_load_n(
		&g_ctx.ps.video.frame_counter, __ATOMIC_RELAXED);

	/* Straight from the shm ring header rather than the output's cached
	 * copy, so a stalled encoder cannot serve a stale fill to RcAgent. */
	if (star6e_output_frame_ring_fill(&g_ctx.ps.output, &fill) == 0) {
		out->full_drops = fill.full_drops;
		out->ring_fill_pct = fill.fill_pct;
	}
}

int venc_snapshot_jpeg(uint8_t **out, size_t *out_len, int quality)
{
	if (!out || !out_len)
		return -1;
	*out = NULL;
	*out_len = 0;

	if (!venc_core_running())
		return -1;

	/* The MJPEG channel (chn 7) is created at pipeline start when
	 * venc.snapshot_quality > 0 and kept idle between requests; capture
	 * pulse-encodes one frame.  venc_jpeg serialises internally, so no
	 * verb lock — and it must not take one, since a capture waits on a
	 * frame while the ladder keeps steering the bitrate. */
	if (quality > 0)
		(void)venc_jpeg_set_quality((uint32_t)quality);

	/* 1 s: chn7 must be started, deliver one frame and be stopped again;
	 * at 60 fps that is milliseconds, so this only bounds a wedge. */
	if (venc_jpeg_capture(out, out_len, 1000) != 0) {
		*out = NULL;
		*out_len = 0;
		return -1;
	}
	return 0;
}
