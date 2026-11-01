/* drone/venc/venc_core.h — the ONLY interface mabur C++ sees. */
#pragma once
#include <stddef.h> /* size_t — venc_cfg.h pulls in stdint/stdbool only */
#include "venc_cfg.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  /* Encoder thread context. All callbacks fire on venc-internal threads;
   * marshal to your own thread (RcAgent polls atomics). */
  void (*on_chain_break)(void *user); /* ring-full drop killed a ref frame */
  void (*on_fault)(void *user, const char *what); /* unrecoverable MI error */
  void *user;
} VencCallbacks;

/* Boot the sensor→ISP→VENC pipeline and the encoder/AE/AWB threads.
 * Frames appear in the shared /mabur_f ring. Returns 0 or -1 (boot
 * failure — caller logs and exits; the wrapper respawn is the retry).
 * Refused while a session is open: venc_core_stop first. */
int venc_core_start(const VencCfg *cfg, const VencCallbacks *cb);
/* Ends the MI_SYS session whatever state the core is in — running, or left
 * not running by a failed venc_core_restart — so a venc_core_start after it
 * always opens a fresh session. A no-op when no session is open. */
void venc_core_stop(void);

/* Rebuild the pipeline with a new cfg — stop the encoder thread, rebuild
 * under the same MI_SYS session (star6e_runtime_pipeline_restart), restart
 * the encoder thread. Not callable from the encoder thread itself (it joins
 * that thread, which would deadlock). Holds g_verb_lock for the whole call,
 * same as the verbs below, so it serialises against them; since it clears
 * the running flag before doing anything else, a verb that arrives while a
 * restart is in flight sees the core as not running and returns -1 rather
 * than acting on a pipeline mid-rebuild. Like venc_core_start/stop it must
 * never run concurrently with them; that is caller discipline.
 *
 * The contract the rebuild failure ladder relies on:
 *   - Returns 0 with the core running on `cfg`.
 *   - Returns -1 with the core NOT running and the MI_SYS session still
 *     open. Another venc_core_restart (with any cfg) is allowed and may
 *     succeed: the restart is gated on the open session, not on the core
 *     running. venc_core_stop then venc_core_start is the cold fallback;
 *     the stop really closes the session.
 *   - Returns -1 at once, touching nothing, when no session is open (never
 *     started, or stopped).
 * MABUR_VENC_FAIL_RESTARTS=N in maburd's environment at its first
 * venc_core_start fails the next N restarts after the old pipeline is
 * stopped (a bench knob for the ladder; docs/vtx-config-mavlink.md). */
int venc_core_restart(const VencCfg *cfg);

/* Optional, before venc_core_start: dlopen the SigmaStar MI libraries
 * (~0.4 s of page-in from squashfs on a cold boot). Touches no MI device --
 * the kernel modules need not be loaded yet -- so a boot path can run it on
 * a side thread under the radio's USB port reset. Idempotent; venc_core_start
 * does it anyway if it was not called. Returns 0 or -1 (logged). */
int venc_core_preload(void);

/* Verbs — thread-safe, callable from the agent thread. */
int venc_set_bitrate_kbps(int kbps);
int venc_set_roi_qp(int qp);
int venc_set_fps(int fps);           /* live frame rate: rebind + RC fps + GOP; agent thread */
int venc_request_idr(void);          /* goes through idr_rate_limit */
int venc_set_qp_delta(int qp_delta); /* boot + debug endpoint only */
int venc_set_max_ipprop(int prop);   /* boot + debug endpoint only; u32MaxIPProp */
int venc_set_superframe_p_pct(int pct); /* debug endpoint; 0 = off, 100..1000 (SuperFrame P cap) */
int venc_set_min_iqp(int qp);          /* debug endpoint; u32MinIQp 1..51 (I-frame QP floor = IDR size cap) */
int venc_set_max_iqp(int qp);          /* debug endpoint; u32MaxIQp 1..51 */
int venc_set_sensor_offset(int mfps);  /* RcAgent: phase-lock offset on the trim request (mfps); stored, applied by the encoder thread once the trim has locked */

/* Signals — read on demand (agent tick / telemetry / debug endpoint). */
typedef struct {
  uint64_t full_drops;      /* lifetime ring-full drops */
  uint32_t ring_fill_pct;   /* 0..100 */
  uint32_t frames_encoded;  /* lifetime */
  int cur_bitrate_kbps;     /* last applied via venc_set_bitrate_kbps, -1 before first */
  uint32_t sensor_mhz;      /* measured sensor frame rate, milli-Hz; 0 until the
                             * boot-time trim's first window (sensor_cal.h) */
  int32_t sensor_off_mfps;  /* phase-lock offset now on the sensor, milli-fps
                             * on the trim request; 0 = the trim's own rate */
  /* No encoder QP: MI_VENC_Stream_t h265Info.startQual reads 0 on every
   * frame this firmware emits and there is no GetChnStat QP. */
} VencStats;
void venc_get_stats(VencStats *out);

/* link-rtt: current pts-domain clock (MI_SYS_GetCurPts, µs) — the t3 of the
 * GS's telem-time offset estimate, same MI timebase as frame pts and the
 * enc_us probe. Returns 0 when the SDK symbol is unresolved or the core is
 * not running; telem then ships pts_at_build 0 and the GS skips the offset
 * sample (0 is the wire's "unavailable" sentinel, never a real clock). */
uint64_t venc_cur_pts_us(void);

/* One-shot JPEG snapshot (chn7). Returns malloc'd buffer via *out (caller
 * frees) and its size, or -1. Debug endpoint only. */
int venc_snapshot_jpeg(uint8_t **out, size_t *out_len, int quality);

/* VTX onboard recorder channel (docs/vtx-recorder.md). A second
 * H.265 CBR channel bound to the link's VPE port BEFORE the link channel:
 * the single-task H.265 engine serves the most recently bound peer first,
 * so the link keeps encoding ahead of it (docs/findings/2026-09-26-sd-record.md).
 * Idle until venc_record_start(). */
/* prefixed = 1: the AU is in MP4 layout (4-byte big-endian length before
 * each NAL) and is_key came from the encoder's NAL table. prefixed = 0: an
 * Annex-B fallback AU (no usable NAL table); the consumer should re-check
 * is_key against the bitstream. */
typedef void (*VencRecordSink)(void *user, const uint8_t *au, size_t len,
	uint32_t pts_us, int is_key, int prefixed);
typedef struct {
	int enabled;            /* 0 = no channel is created */
	uint32_t bitrate_kbps;  /* CBR */
	uint32_t fps;           /* clamped to the sensor rate */
	uint32_t width, height; /* 0 = the link's size; else VPE port 1 */
} VencRecordConfig;
/* Before venc_core_start. The sink runs on the record drain thread and must copy. */
void venc_record_configure(const VencRecordConfig *cfg, VencRecordSink sink, void *user);
int venc_record_start(void);        /* 0 ok, -1 no record channel */
void venc_record_stop(void);        /* returns once the last AU reached the sink (<= 500 ms) */
void venc_record_request_idr(void);

#ifdef __cplusplus
}
#endif
