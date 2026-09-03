/* drone/venc/venc_core.h — the ONLY interface mabur C++ sees. */
#pragma once
#include <stddef.h> /* size_t — venc_cfg.h pulls in stdint/stdbool only */
#include "venc_cfg.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  /* Encoder thread context. All callbacks fire on venc-internal threads;
   * marshal to your own thread (RcAgent polls atomics — Task B6). */
  void (*on_chain_break)(void *user); /* ring-full drop killed a ref frame */
  void (*on_fault)(void *user, const char *what); /* unrecoverable MI error */
  void *user;
} VencCallbacks;

/* Boot the sensor→ISP→VENC pipeline and the encoder/AE/AWB threads.
 * Frames appear in the shared /mabur_f ring. Returns 0 or -1 (boot
 * failure — caller logs and exits; the wrapper respawn is the retry). */
int venc_core_start(const VencCfg *cfg, const VencCallbacks *cb);
void venc_core_stop(void);

/* Verbs — thread-safe, callable from the agent thread. */
int venc_set_bitrate_kbps(int kbps);
int venc_set_roi_qp(int qp);
int venc_request_idr(void);          /* goes through idr_rate_limit */
int venc_set_qp_delta(int qp_delta); /* boot + debug endpoint only */
int venc_set_max_ipprop(int prop);   /* boot + debug endpoint only; u32MaxIPProp */
int venc_set_superframe_p_pct(int pct); /* debug endpoint; 0 = off, 100..1000 (SuperFrame P cap) */

/* Signals — read on demand (agent tick / telemetry / debug endpoint). */
typedef struct {
  uint64_t full_drops;      /* lifetime ring-full drops */
  uint32_t ring_fill_pct;   /* 0..100 */
  uint32_t frames_encoded;  /* lifetime */
  int cur_bitrate_kbps;     /* last applied via venc_set_bitrate_kbps, -1 before first */
  /* No encoder QP: MI_VENC_Stream_t h265Info.startQual reads 0 on every
   * frame this firmware emits and there is no GetChnStat QP (2026-09-03). */
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

#ifdef __cplusplus
}
#endif
