/* ported from waybeam_venc f956a52:include/star6e_controls.h */
#ifndef STAR6E_CONTROLS_H
#define STAR6E_CONTROLS_H

#include "star6e_pipeline.h"
#include "venc_cfg.h"

/* waybeam's VencApplyCallbacks table (the HTTP /api/v1/set live-apply glue)
 * is deleted along with httpd; the surviving controls are called directly.
 * Everything here is pure mechanism — no encoder-local policy (spec §4). */

/** Bind pipeline and config to runtime control state. */
void star6e_controls_bind(Star6ePipelineState *pipeline, const VencCfg *cfg);

/** Reset control state to defaults (called between pipeline restarts). */
void star6e_controls_reset(void);

/** Program the encoder rate.  The ONLY rate source is RcAgent — there is
 *  no venc.bitrate key and no encoder-local clamp policy beyond the
 *  VENC_BITRATE_{MIN,MAX}_KBPS rails and the >120 fps exact-CBR
 *  compensation.  Does NOT request an IDR (spec §4). */
int star6e_controls_apply_bitrate(uint32_t kbps);

/** Apply ROI-based QP adjustment for FPV center emphasis. */
int star6e_controls_apply_roi_qp(int qp);

/** Apply relative I/P QP delta to the running encoder (boot-time). */
int star6e_controls_apply_qp_delta(int delta);

/** Set u32MaxIPProp on the CBR RC params (boot startup control and debug
 *  endpoint).  Rejects prop outside [1,100].  Staged through the RC intent
 *  (see g_rc_intent in star6e_controls.c): every RC write re-writes both
 *  s32IPQPDelta and u32MaxIPProp, because MI_VENC_GetRcParam returns stale
 *  driver defaults for ~5 s after StartRecvPic and a Get->modify->Set in
 *  that window silently reverts the other field (waybeam #255). */
int star6e_controls_set_max_ipprop(uint32_t prop);

/** Log what the encoder's RC params actually hold (IPQPDelta, MaxIPProp,
 *  the QP bounds) against the staged intent, tagged with `when`
 *  (e.g. "t+10s").  Read-only; called once from the encoder loop after
 *  the #255 stale-Get window has closed so /tmp/mabur.log proves the
 *  boot-time qp_delta / max_ipprop reached the encoder. */
void star6e_controls_log_rc_readback(const char *when);

/** SuperFrame P-frame ceiling as a percentage of the per-frame budget
 *  (0 = off, 100..1000).  Re-derived and re-programmed on every bitrate
 *  write (apply_bitrate) from the rate the encoder was actually given;
 *  this call changes the percentage live and re-applies at the last
 *  programmed rate.  I threshold always unlimited (a low one stalls the
 *  channel — see MI_VENC_SuperFrameCfg_t in star6e.h). */
int star6e_controls_set_superframe_p_pct(uint32_t pct);

/** Explicit IDR request.  Goes through the shared per-channel 100 ms
 *  rate limiter (idr_rate_limit.h); a coalesced request returns 0, not an
 *  error.  This is the only place in venc_core that calls
 *  MI_VENC_RequestIdr on behalf of a caller.
 *
 *  Unguarded: it dereferences the dlopen'd MI vtable, so it is only legal
 *  between pipeline start and stop.  The public verb venc_request_idr()
 *  (venc_core.h) is this call plus the started guard — external callers
 *  want that one. */
int star6e_controls_request_idr(void);

#endif /* STAR6E_CONTROLS_H */
